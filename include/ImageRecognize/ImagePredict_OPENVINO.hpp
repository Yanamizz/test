/**
 * @file    include/ImageRecognize/ImagePredict_OPENVINO.hpp
 * @brief   OpenVINO 推理入口：加载 IR/ONNX 模型并执行单张图像推理。
 *
 * @details
 * - 复用 `ImagePreprocess` 执行图像预处理（NCHW float）。
 * - 使用 OpenVINO `ov::Core` + `CompiledModel` + `InferRequest` 执行推理。
 * - 在本文件内完成与 `OutputDataProcess` 等价的后处理，返回同一 `PredictResult`。
 */

#pragma once

#include "ImageRecognize/ImagePreprocess.hpp"
#include "ImageRecognize/OutputDataProcess.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if __has_include(<openvino/openvino.hpp>)
#include <openvino/openvino.hpp>
#define IMAGE_RECOGNIZE_HAS_OPENVINO 1
#else
#define IMAGE_RECOGNIZE_HAS_OPENVINO 0
#endif

namespace ImageRecognize {

using PredictResult = ImageRecognize::DataProcessResult;

#if IMAGE_RECOGNIZE_HAS_OPENVINO
class ImagePredict {
 public:
  ImagePredict() = default;

  // 与 ONNX 版本保持同名构造：默认使用 AUTO 设备（可自动选 GPU/CPU）
  explicit ImagePredict(const std::string &model_path) { init(model_path, "AUTO"); }

  // 可选构造：允许显式指定设备，如 "CPU" / "GPU" / "AUTO"
  ImagePredict(const std::string &model_path, const std::string &device_name) { init(model_path, device_name); }

  // 与 ONNX 版本保持一致：传图 + 模型路径（每次临时加载）
  PredictResult run(const cv::Mat &origin_image_, std::string model_path_) const {
    ImagePredict tmp_predictor(model_path_, device_name_);
    return tmp_predictor.run(origin_image_);
  }

  // 复用已加载模型进行推理（推荐实时场景）
  PredictResult run(const cv::Mat &origin_image_) {
    if (!initialized_) {
      throw std::runtime_error("OpenVINO session not initialized. Use ImagePredict(model_path) constructor.");
    }

    ImageRecognize::ImagePreprocess preprocessor_{cv::Size(width_, height_)};
    auto pre_image_ = preprocessor_.run(origin_image_);

    ov::Tensor input_tensor(ov::element::f32, {1, 3, static_cast<size_t>(height_), static_cast<size_t>(width_)},
                            pre_image_.data.data());
    infer_request_.set_tensor(input_name_, input_tensor);
    infer_request_.infer();

    const ov::Tensor output_tensor = infer_request_.get_tensor(output_name_);
    return postprocess_(output_tensor, origin_image_.size());
  }

 private:
  void init(const std::string &model_path, const std::string &device_name) {
    if (model_path.empty()) {
      throw std::runtime_error("Model path is empty.");
    }

    model_path_ = model_path;
    device_name_ = device_name.empty() ? std::string("AUTO") : device_name;

    core_ = std::make_unique<ov::Core>();
    auto model = core_->read_model(model_path_);

    // 从模型自动读取输入 H/W（shape = [1,3,H,W]），动态维度则保持默认 640
    const auto &pshape = model->input().get_partial_shape();
    if (pshape.rank().is_static() && pshape.rank().get_length() == 4) {
      if (pshape[2].is_static()) height_ = static_cast<int>(pshape[2].get_length());
      if (pshape[3].is_static()) width_ = static_cast<int>(pshape[3].get_length());
    }

    compiled_model_ = core_->compile_model(model, device_name_);
    infer_request_ = compiled_model_.create_infer_request();

    if (compiled_model_.inputs().size() != 1 || compiled_model_.outputs().size() != 1) {
      throw std::runtime_error("Model must have exactly one input and one output");
    }

    input_name_ = compiled_model_.input().get_any_name();
    output_name_ = compiled_model_.output().get_any_name();
    initialized_ = true;
  }

  // 与原 OutputDataProcess 等价的后处理（输入支持 [1,5,N] / [1,N,5]）
  PredictResult postprocess_(const ov::Tensor &output_tensor_, const cv::Size &original_image_size_) const {
    PredictResult result_{};

    const auto &shape = output_tensor_.get_shape();
    if (shape.size() != 3) {
      throw std::runtime_error("Unexpected output rank. Expect 3D tensor like [1,5,N] or [1,N,5].");
    }

    const float *data = output_tensor_.data<const float>();
    if (!data) {
      throw std::runtime_error("Output tensor data is null.");
    }

    bool channel_first = false;  // [1,5,N]
    int num_detections = 0;

    if (shape[1] == 5) {
      channel_first = true;
      num_detections = static_cast<int>(shape[2]);
    } else if (shape[2] == 5) {
      channel_first = false;  // [1,N,5]
      num_detections = static_cast<int>(shape[1]);
    } else {
      throw std::runtime_error("Unexpected output shape. Need channel dim = 5.");
    }

    auto fetch = [&](int det_idx, int ch_idx) -> float {
      if (channel_first) {
        return data[ch_idx * num_detections + det_idx];
      }
      return data[det_idx * 5 + ch_idx];
    };

    const float scale_x = static_cast<float>(original_image_size_.width) / static_cast<float>(width_);
    const float scale_y = static_cast<float>(original_image_size_.height) / static_cast<float>(height_);

    for (int i = 0; i < num_detections; ++i) {
      const float cx = fetch(i, 0);
      const float cy = fetch(i, 1);
      const float w = fetch(i, 2);
      const float h = fetch(i, 3);
      const float score = fetch(i, 4);

      if (score <= score_thresh_) continue;

      float x1 = (cx - w * 0.5f) * scale_x;
      float y1 = (cy - h * 0.5f) * scale_y;
      float x2 = (cx + w * 0.5f) * scale_x;
      float y2 = (cy + h * 0.5f) * scale_y;

      result_.boxes.push_back({x1, y1, x2, y2, score});
    }

    result_.boxes = nms_(result_.boxes, nms_iou_thresh_);
    return result_;
  }

  static float iou_(const std::array<float, 5> &a, const std::array<float, 5> &b) {
    const float xx1 = std::max(a[0], b[0]);
    const float yy1 = std::max(a[1], b[1]);
    const float xx2 = std::min(a[2], b[2]);
    const float yy2 = std::min(a[3], b[3]);
    const float w = std::max(0.0f, xx2 - xx1);
    const float h = std::max(0.0f, yy2 - yy1);
    const float inter = w * h;
    const float area_a = std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
    const float area_b = std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
    const float uni = area_a + area_b - inter;
    return (uni <= 0.0f) ? 0.0f : (inter / uni);
  }

  static std::vector<std::array<float, 5>> nms_(const std::vector<std::array<float, 5>> &boxes, float iou_thresh) {
    if (boxes.empty()) return {};

    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return boxes[a][4] > boxes[b][4]; });

    std::vector<char> removed(boxes.size(), 0);
    std::vector<std::array<float, 5>> keep;
    keep.reserve(boxes.size());

    for (size_t oi = 0; oi < order.size(); ++oi) {
      const int i = order[oi];
      if (removed[i]) continue;

      keep.push_back(boxes[i]);

      for (size_t oj = oi + 1; oj < order.size(); ++oj) {
        const int j = order[oj];
        if (removed[j]) continue;
        if (iou_(boxes[i], boxes[j]) > iou_thresh) removed[j] = 1;
      }
    }

    return keep;
  }

 private:
  int width_ = 640;   ///< 从模型自动读取的输入宽
  int height_ = 640;  ///< 从模型自动读取的输入高

  std::unique_ptr<ov::Core> core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  bool initialized_ = false;
  std::string model_path_;
  std::string device_name_ = "AUTO";
  std::string input_name_;
  std::string output_name_;

  float score_thresh_ = 0.5f;
  float nms_iou_thresh_ = 0.5f;
};
#else
class ImagePredict {
 public:
  ImagePredict() = default;

  explicit ImagePredict(const std::string &) { throwNotAvailable_(); }

  ImagePredict(const std::string &, const std::string &) { throwNotAvailable_(); }

  PredictResult run(const cv::Mat &, std::string) const {
    throwNotAvailable_();
    return {};
  }

  PredictResult run(const cv::Mat &) {
    throwNotAvailable_();
    return {};
  }

 private:
  [[noreturn]] static void throwNotAvailable_() {
    throw std::runtime_error(
        "OpenVINO headers not found. Please install OpenVINO and configure include/library paths before using "
        "ImagePredict_OPENVINO.hpp");
  }
};
#endif

}  // namespace ImageRecognize
