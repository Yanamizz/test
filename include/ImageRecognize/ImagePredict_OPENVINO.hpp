/**
 * @file    include/ImageRecognize/ImagePredict_OPENVINO.hpp
 * @brief   提供基于 OpenVINO 的目标检测推理、异步推理与结果后处理能力。
 */

#pragma once

#include "ImageRecognize/ImagePreprocess.hpp"
#include "ImageRecognize/OutputDataProcess.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
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

#if IMAGE_RECOGNIZE_HAS_OPENVINO
class ImagePredict {
public:
  ImagePredict() = default;

  explicit ImagePredict(const std::string &model_path) {
    init_(model_path, "AUTO");
  }

  ImagePredict(const std::string &model_path, const std::string &device_name) {
    init_(model_path, device_name);
  }

  PredictResult run(const cv::Mat &origin_image_) {
    if (!initialized_) {
      throw std::runtime_error("OpenVINO session not initialized. Use "
                               "ImagePredict(model_path) constructor.");
    }

    PreprocessResult preprocessed{};
    preprocessor_.run(origin_image_, &preprocessed);
    ov::Tensor input_tensor(
        ov::element::f32,
        {1, 3, static_cast<size_t>(height_), static_cast<size_t>(width_)},
        preprocessed.data.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    const ov::Tensor output_tensor = infer_request_.get_output_tensor(0);
    return postprocess_(output_tensor, origin_image_.size(), preprocessed);
  }

  void startAsync(const cv::Mat &origin_image_) {
    if (!initialized_) {
      throw std::runtime_error("OpenVINO session not initialized. Use "
                               "ImagePredict(model_path) constructor.");
    }
    if (async_inflight_) {
      throw std::runtime_error("Async inference already in flight.");
    }

    preprocessor_.run(origin_image_, &async_preprocessed_);
    async_original_size_ = origin_image_.size();
    ov::Tensor input_tensor(
        ov::element::f32,
        {1, 3, static_cast<size_t>(height_), static_cast<size_t>(width_)},
        async_preprocessed_.data.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.start_async();
    async_inflight_ = true;
  }

  PredictResult getAsyncResult() {
    if (!async_inflight_) {
      throw std::runtime_error("No async inference result is pending.");
    }
    infer_request_.wait();
    const ov::Tensor output_tensor = infer_request_.get_output_tensor(0);
    async_inflight_ = false;
    return postprocess_(output_tensor, async_original_size_,
                        async_preprocessed_);
  }

private:
  enum class OutputLayoutKind {
    Unknown,
    YoloRawCxCyWhCls,
    NmsBoxes,
  };

  struct TunableParams {
    int latency_threads_cap;
    int streams_num;
    std::size_t pre_merge_top_k;
    float score_thresh;
    int min_channel_dim;
    float merge_iou_thresh;
    float suppress_iou_thresh;
    std::size_t max_output_boxes;
  };

  struct OutputSpec {
    OutputLayoutKind layout = OutputLayoutKind::Unknown;
    bool channel_first = false;
    std::size_t rank = 0;
    std::vector<std::int64_t> dims;
  };

  static constexpr int kDefaultInputSide = 640;

  static const TunableParams &Params();

  void init_(const std::string &model_path, const std::string &device_name) {
    if (model_path.empty()) {
      throw std::runtime_error("Model path is empty.");
    }

    namespace fs = std::filesystem;
    const fs::path abs_input_path = fs::absolute(fs::path(model_path));
    model_path_ = abs_input_path.string();
    device_name_ = device_name.empty() ? std::string("AUTO") : device_name;
    core_ = std::make_unique<ov::Core>();

    auto model = loadModel_(abs_input_path);
    validateModelIo_(*model);
    resolveInputSpec_(*model);
    preprocessor_ = ImageRecognize::ImagePreprocess(cv::Size(width_, height_));
    compiled_model_ = compileModel_(model);
    createInferRequest_();
    resolveOutputSpec_(*model);

    std::cerr << "[OpenVINO] model=" << model_path_
              << " device=" << device_name_ << " input=" << width_ << "x"
              << height_ << " output=" << describeOutputSpec_(output_spec_)
              << std::endl;
    initialized_ = true;
  }

  std::shared_ptr<ov::Model>
  loadModel_(const std::filesystem::path &model_path) {
    namespace fs = std::filesystem;
    if (!fs::exists(model_path)) {
      throw std::runtime_error("Model file not found: " + model_path.string());
    }

    const bool is_ir_xml =
        model_path.has_extension() && model_path.extension() == ".xml";
    if (!is_ir_xml) {
      return core_->read_model(model_path.string());
    }

    fs::path bin_path = model_path;
    bin_path.replace_extension(".bin");
    if (!fs::exists(bin_path)) {
      throw std::runtime_error("OpenVINO BIN weights not found: " +
                               bin_path.string());
    }
    if (fs::file_size(bin_path) == 0) {
      throw std::runtime_error("OpenVINO BIN weights file is empty: " +
                               bin_path.string());
    }
    return core_->read_model(model_path.string(), bin_path.string());
  }

  void validateModelIo_(const ov::Model &model) const {
    if (model.inputs().size() != 1 || model.outputs().size() != 1) {
      std::ostringstream oss;
      oss << "Model must have exactly one input and one output, but got inputs="
          << model.inputs().size() << " outputs=" << model.outputs().size();
      throw std::runtime_error(oss.str());
    }
  }

  void resolveInputSpec_(const ov::Model &model) {
    const auto input = model.input();
    input_name_ = input.get_any_name();
    input_element_type_ = input.get_element_type();

    const auto pshape = input.get_partial_shape();
    if (!pshape.rank().is_static() || pshape.rank().get_length() != 4) {
      throw std::runtime_error("Input tensor must be rank-4 NCHW, got: " +
                               partialShapeToString_(pshape));
    }

    if (pshape[1].is_static() && pshape[1].get_length() != 3) {
      throw std::runtime_error("Input channel count must be 3, got: " +
                               partialShapeToString_(pshape));
    }

    if (pshape[2].is_static()) {
      height_ = static_cast<int>(pshape[2].get_length());
    } else {
      height_ = kDefaultInputSide;
    }

    if (pshape[3].is_static()) {
      width_ = static_cast<int>(pshape[3].get_length());
    } else {
      width_ = kDefaultInputSide;
    }

    if (height_ <= 0 || width_ <= 0) {
      throw std::runtime_error("Input H/W must be positive, got: " +
                               partialShapeToString_(pshape));
    }

    if (!pshape[2].is_static() || !pshape[3].is_static()) {
      std::cerr << "[OpenVINO] dynamic input shape detected "
                << partialShapeToString_(pshape) << ", fallback to " << width_
                << "x" << height_ << std::endl;
    }
  }

  ov::CompiledModel compileModel_(const std::shared_ptr<ov::Model> &model) {
    const unsigned int hw_threads =
        std::max(1u, std::thread::hardware_concurrency());
    const unsigned int latency_threads_cap =
        static_cast<unsigned int>(std::max(1, Params().latency_threads_cap));
    const unsigned int infer_threads =
        std::min(hw_threads, latency_threads_cap);

    std::vector<std::string> available_devices;
    try {
      available_devices = core_->get_available_devices();
    } catch (const std::exception &e) {
      std::cerr << "[OpenVINO] Failed to query available devices: " << e.what()
                << std::endl;
    }

    if (available_devices.empty()) {
      std::cerr << "[OpenVINO] available devices: <none>" << std::endl;
    } else {
      std::cerr << "[OpenVINO] available devices:";
      for (const auto &device : available_devices) {
        std::cerr << ' ' << device;
      }
      std::cerr << std::endl;
    }

    auto has_device_prefix = [&](const std::string &prefix) {
      return std::any_of(available_devices.begin(), available_devices.end(),
                         [&](const std::string &device) {
                           return device.rfind(prefix, 0) == 0;
                         });
    };

    const bool gpu_available = has_device_prefix("GPU");
    std::vector<std::string> candidate_devices;
    if (device_name_ == "GPU") {
      candidate_devices = gpu_available
                              ? std::vector<std::string>{"GPU", "AUTO", "CPU"}
                              : std::vector<std::string>{"AUTO", "CPU"};
    } else if (device_name_ == "AUTO") {
      candidate_devices = gpu_available
                              ? std::vector<std::string>{"AUTO", "GPU", "CPU"}
                              : std::vector<std::string>{"AUTO", "CPU"};
    } else {
      candidate_devices = {device_name_, "AUTO", "CPU"};
    }

    auto try_compile = [&](const std::string &target_device)
        -> std::optional<ov::CompiledModel> {
      try {
        ov::AnyMap properties;
        properties.emplace(ov::hint::performance_mode.name(),
                           ov::Any(ov::hint::PerformanceMode::LATENCY));
        properties.emplace(ov::streams::num.name(),
                           ov::Any(Params().streams_num));
        properties.emplace(ov::inference_num_threads.name(),
                           ov::Any(infer_threads));
        return core_->compile_model(model, target_device, properties);
      } catch (const std::exception &e) {
        std::cerr << "[OpenVINO] device '" << target_device
                  << "' compile failed: " << e.what() << std::endl;
        return std::nullopt;
      }
    };

    for (const auto &candidate_device : candidate_devices) {
      if (auto compiled = try_compile(candidate_device)) {
        device_name_ = candidate_device;
        return std::move(*compiled);
      }
    }

    throw std::runtime_error("Failed to compile OpenVINO model on "
                             "requested/AUTO/CPU fallback chain.");
  }

  void createInferRequest_() {
    infer_request_ = compiled_model_.create_infer_request();
  }

  void resolveOutputSpec_(const ov::Model &model) {
    const auto output = model.output();
    output_name_ = output.get_any_name();
    output_spec_.rank =
        output.get_partial_shape().rank().is_static()
            ? static_cast<std::size_t>(
                  output.get_partial_shape().rank().get_length())
            : 0u;
    output_spec_.dims = partialShapeToDims_(output.get_partial_shape());

    if (output_spec_.rank == 3 && output_spec_.dims.size() == 3 &&
        output_spec_.dims[1] > 0 && output_spec_.dims[2] > 0) {
      const auto inferred = inferOutputSpecFromDims_(output_spec_.dims);
      output_spec_.layout = inferred.layout;
      output_spec_.channel_first = inferred.channel_first;
    }
  }

  PredictResult postprocess_(const ov::Tensor &output_tensor,
                             const cv::Size &original_image_size,
                             const PreprocessResult &preprocess_meta) {
    const auto runtime_spec =
        inferOutputSpecFromShape_(output_tensor.get_shape());
    const OutputSpec effective_spec =
        runtime_spec.layout == OutputLayoutKind::Unknown ? output_spec_
                                                         : runtime_spec;

    if (effective_spec.layout == OutputLayoutKind::Unknown) {
      throw std::runtime_error("Unsupported output tensor shape: " +
                               shapeToString_(output_tensor.get_shape()));
    }
    if (output_tensor.get_element_type() != ov::element::f32) {
      throw std::runtime_error("Output tensor element type must be f32.");
    }

    const float *data = output_tensor.data<const float>();
    if (data == nullptr) {
      throw std::runtime_error("Output tensor data is null.");
    }

    const std::size_t dim1 = output_tensor.get_shape()[1];
    const std::size_t dim2 = output_tensor.get_shape()[2];
    const std::size_t channel_count =
        effective_spec.channel_first ? dim1 : dim2;
    const std::size_t num_detections =
        effective_spec.channel_first ? dim2 : dim1;

    auto fetch = [&](std::size_t det_idx, std::size_t ch_idx) -> float {
      if (effective_spec.channel_first) {
        return data[ch_idx * num_detections + det_idx];
      }
      return data[det_idx * channel_count + ch_idx];
    };

    PredictResult result;
    result.boxes.reserve(num_detections);

    for (std::size_t i = 0; i < num_detections; ++i) {
      if (effective_spec.layout == OutputLayoutKind::NmsBoxes) {
        appendNmsBox_(result, fetch(i, 0), fetch(i, 1), fetch(i, 2),
                      fetch(i, 3), fetch(i, 4), static_cast<int>(fetch(i, 5)),
                      preprocess_meta, original_image_size);
        continue;
      }

      const float cx = fetch(i, 0);
      const float cy = fetch(i, 1);
      const float w = fetch(i, 2);
      const float h = fetch(i, 3);

      int class_id = 0;
      float best_score = -1.0f;
      for (std::size_t c = 4; c < channel_count; ++c) {
        const float cls_score = fetch(i, c);
        if (cls_score > best_score) {
          best_score = cls_score;
          class_id = static_cast<int>(c - 4);
        }
      }

      appendNmsBox_(result, cx - 0.5f * w, cy - 0.5f * h, cx + 0.5f * w,
                    cy + 0.5f * h, best_score, class_id, preprocess_meta,
                    original_image_size);
    }

    if (!result.boxes.empty()) {
      const std::size_t top_k =
          std::max<std::size_t>(1, Params().pre_merge_top_k);
      if (result.boxes.size() > top_k) {
        std::partial_sort(
            result.boxes.begin(),
            result.boxes.begin() + static_cast<std::ptrdiff_t>(top_k),
            result.boxes.end(),
            [](const std::array<float, 6> &lhs,
               const std::array<float, 6> &rhs) { return lhs[4] > rhs[4]; });
        result.boxes.resize(top_k);
      }
    }

    result.boxes = mergeAndFilterBoxes_(result.boxes);
    return result;
  }

  static float boxIou_(const std::array<float, 6> &a,
                       const std::array<float, 6> &b) {
    const float xx1 = std::max(a[0], b[0]);
    const float yy1 = std::max(a[1], b[1]);
    const float xx2 = std::min(a[2], b[2]);
    const float yy2 = std::min(a[3], b[3]);
    const float inter_w = std::max(0.0f, xx2 - xx1);
    const float inter_h = std::max(0.0f, yy2 - yy1);
    const float inter = inter_w * inter_h;
    const float area_a =
        std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
    const float area_b =
        std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
    const float uni = area_a + area_b - inter;
    return uni > 0.0f ? (inter / uni) : 0.0f;
  }

  std::vector<std::array<float, 6>>
  mergeAndFilterBoxes_(const std::vector<std::array<float, 6>> &boxes) const {
    if (boxes.size() <= 1) {
      return boxes;
    }

    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      return boxes[static_cast<std::size_t>(lhs)][4] >
             boxes[static_cast<std::size_t>(rhs)][4];
    });

    std::vector<char> consumed(boxes.size(), 0);
    std::vector<std::array<float, 6>> merged;
    merged.reserve(boxes.size());

    for (std::size_t order_idx = 0; order_idx < order.size(); ++order_idx) {
      const std::size_t seed_pos = static_cast<std::size_t>(order[order_idx]);
      if (consumed[seed_pos]) {
        continue;
      }

      const auto &seed_box = boxes[seed_pos];
      const int seed_class = static_cast<int>(seed_box[5]);
      float weight_sum = 0.0f;
      float x1_sum = 0.0f;
      float y1_sum = 0.0f;
      float x2_sum = 0.0f;
      float y2_sum = 0.0f;
      float best_score = seed_box[4];

      for (std::size_t candidate_order_idx = order_idx;
           candidate_order_idx < order.size(); ++candidate_order_idx) {
        const std::size_t candidate_pos =
            static_cast<std::size_t>(order[candidate_order_idx]);
        if (consumed[candidate_pos]) {
          continue;
        }

        const auto &candidate_box = boxes[candidate_pos];
        if (static_cast<int>(candidate_box[5]) != seed_class) {
          continue;
        }
        if (boxIou_(seed_box, candidate_box) < Params().merge_iou_thresh) {
          continue;
        }

        const float weight = std::max(candidate_box[4], 1e-6f);
        weight_sum += weight;
        x1_sum += candidate_box[0] * weight;
        y1_sum += candidate_box[1] * weight;
        x2_sum += candidate_box[2] * weight;
        y2_sum += candidate_box[3] * weight;
        best_score = std::max(best_score, candidate_box[4]);
        consumed[candidate_pos] = 1;
      }

      if (weight_sum <= 0.0f) {
        continue;
      }

      merged.push_back({x1_sum / weight_sum, y1_sum / weight_sum,
                        x2_sum / weight_sum, y2_sum / weight_sum, best_score,
                        static_cast<float>(seed_class)});
    }

    std::sort(merged.begin(), merged.end(),
              [](const std::array<float, 6> &lhs,
                 const std::array<float, 6> &rhs) { return lhs[4] > rhs[4]; });

    std::vector<std::array<float, 6>> filtered;
    filtered.reserve(std::min(merged.size(), Params().max_output_boxes));
    for (const auto &candidate_box : merged) {
      bool overlaps_kept = false;
      for (const auto &kept_box : filtered) {
        if (static_cast<int>(candidate_box[5]) ==
                static_cast<int>(kept_box[5]) &&
            boxIou_(candidate_box, kept_box) >= Params().suppress_iou_thresh) {
          overlaps_kept = true;
          break;
        }
      }
      if (overlaps_kept) {
        continue;
      }

      filtered.push_back(candidate_box);
      if (filtered.size() >= Params().max_output_boxes) {
        break;
      }
    }

    return filtered;
  }

  void appendNmsBox_(PredictResult &result, float x1, float y1, float x2,
                     float y2, float score, int class_id,
                     const PreprocessResult &preprocess_meta,
                     const cv::Size &original_image_size) const {
    if (score < Params().score_thresh) {
      return;
    }

    const float safe_scale = std::max(preprocess_meta.scale, 1e-6f);
    float scaled_x1 =
        (x1 - static_cast<float>(preprocess_meta.pad_x)) / safe_scale;
    float scaled_y1 =
        (y1 - static_cast<float>(preprocess_meta.pad_y)) / safe_scale;
    float scaled_x2 =
        (x2 - static_cast<float>(preprocess_meta.pad_x)) / safe_scale;
    float scaled_y2 =
        (y2 - static_cast<float>(preprocess_meta.pad_y)) / safe_scale;

    scaled_x1 = std::clamp(scaled_x1, 0.0f,
                           static_cast<float>(original_image_size.width));
    scaled_y1 = std::clamp(scaled_y1, 0.0f,
                           static_cast<float>(original_image_size.height));
    scaled_x2 = std::clamp(scaled_x2, 0.0f,
                           static_cast<float>(original_image_size.width));
    scaled_y2 = std::clamp(scaled_y2, 0.0f,
                           static_cast<float>(original_image_size.height));

    if (scaled_x2 <= scaled_x1 || scaled_y2 <= scaled_y1) {
      return;
    }

    result.boxes.push_back({scaled_x1, scaled_y1, scaled_x2, scaled_y2, score,
                            static_cast<float>(class_id)});
  }

  OutputSpec inferOutputSpecFromShape_(const ov::Shape &shape) const {
    OutputSpec spec;
    spec.rank = shape.size();
    spec.dims.reserve(shape.size());
    for (const auto dim : shape) {
      spec.dims.push_back(static_cast<std::int64_t>(dim));
    }

    if (shape.size() != 3) {
      return spec;
    }
    return inferOutputSpecFromDims_(spec.dims);
  }

  OutputSpec
  inferOutputSpecFromDims_(const std::vector<std::int64_t> &dims) const {
    OutputSpec spec;
    spec.rank = dims.size();
    spec.dims = dims;

    if (dims.size() != 3 || dims[1] <= 0 || dims[2] <= 0) {
      return spec;
    }

    const std::int64_t dim1 = dims[1];
    const std::int64_t dim2 = dims[2];
    if (dim1 == 6 && dim2 != 6) {
      spec.layout = OutputLayoutKind::NmsBoxes;
      spec.channel_first = true;
      return spec;
    }
    if (dim2 == 6 && dim1 != 6) {
      spec.layout = OutputLayoutKind::NmsBoxes;
      spec.channel_first = false;
      return spec;
    }

    const std::int64_t channel_dim = std::min(dim1, dim2);
    const std::int64_t detection_dim = std::max(dim1, dim2);
    if (channel_dim < Params().min_channel_dim ||
        detection_dim <= channel_dim) {
      return spec;
    }

    spec.layout = OutputLayoutKind::YoloRawCxCyWhCls;
    spec.channel_first = (dim1 == channel_dim);
    return spec;
  }

  static std::vector<std::int64_t>
  partialShapeToDims_(const ov::PartialShape &shape) {
    std::vector<std::int64_t> dims;
    if (!shape.rank().is_static()) {
      return dims;
    }
    dims.reserve(static_cast<std::size_t>(shape.rank().get_length()));
    for (const auto &dim : shape) {
      dims.push_back(dim.is_static() ? dim.get_length() : -1);
    }
    return dims;
  }

  static std::string partialShapeToString_(const ov::PartialShape &shape) {
    if (!shape.rank().is_static()) {
      return "{?}";
    }

    std::ostringstream oss;
    oss << '{';
    for (std::size_t i = 0;
         i < static_cast<std::size_t>(shape.rank().get_length()); ++i) {
      if (i != 0) {
        oss << ',';
      }
      if (shape[i].is_static()) {
        oss << shape[i].get_length();
      } else {
        oss << '?';
      }
    }
    oss << '}';
    return oss.str();
  }

  static std::string shapeToString_(const ov::Shape &shape) {
    std::ostringstream oss;
    oss << '{';
    for (std::size_t i = 0; i < shape.size(); ++i) {
      if (i != 0) {
        oss << ',';
      }
      oss << shape[i];
    }
    oss << '}';
    return oss.str();
  }

  static std::string describeOutputSpec_(const OutputSpec &spec) {
    std::ostringstream oss;
    switch (spec.layout) {
    case OutputLayoutKind::YoloRawCxCyWhCls:
      oss << "yolo_raw";
      break;
    case OutputLayoutKind::NmsBoxes:
      oss << "nms_boxes";
      break;
    default:
      oss << "unknown";
      break;
    }

    oss << " rank=" << spec.rank << " dims=";
    if (spec.dims.empty()) {
      oss << "{?}";
    } else {
      oss << '{';
      for (std::size_t i = 0; i < spec.dims.size(); ++i) {
        if (i != 0) {
          oss << ',';
        }
        oss << spec.dims[i];
      }
      oss << '}';
    }
    if (spec.layout != OutputLayoutKind::Unknown) {
      oss << " layout="
          << (spec.channel_first ? "channel_first" : "channel_last");
    }
    return oss.str();
  }

private:
  int width_ = kDefaultInputSide;
  int height_ = kDefaultInputSide;

  std::unique_ptr<ov::Core> core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::element::Type input_element_type_;

  bool initialized_ = false;
  bool async_inflight_ = false;
  std::string model_path_;
  std::string device_name_ = "AUTO";
  std::string input_name_;
  std::string output_name_;
  OutputSpec output_spec_;

  ImageRecognize::ImagePreprocess preprocessor_{};
  ImageRecognize::PreprocessResult async_preprocessed_{};
  cv::Size async_original_size_{};
};

inline const ImagePredict::TunableParams &ImagePredict::Params() {
  static const TunableParams p{
      3, // latency_threads_cap: 低延迟优先，12900H 建议先从 2~4 线程 A/B
      1, // streams_num: OpenVINO 推理流数量（低延迟建议 1）
      32, // pre_merge_top_k: 融合前仅保留高分候选，降低 O(n^2) 开销
      0.1f,  // score_thresh: 置信度阈值
      5,     // min_channel_dim: 原始检测头最少应为 cx,cy,w,h,cls0
      0.45f, // merge_iou_thresh: 同类高重叠框做融合
      0.35f, // suppress_iou_thresh: 融合后再次抑制重叠框
      1      // max_output_boxes: 最终只保留 1 个框，保证唯一目标
  };
  return p;
}
#else
class ImagePredict {
public:
  ImagePredict() = default;

  explicit ImagePredict(const std::string &) { throwNotAvailable_(); }

  ImagePredict(const std::string &, const std::string &) {
    throwNotAvailable_();
  }

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
        "OpenVINO headers not found. Please install OpenVINO and configure "
        "include/library paths before using "
        "ImagePredict_OPENVINO.hpp");
  }
};
#endif

} // namespace ImageRecognize
