/**
 * @file    include/ImageRecognize/OutputDataProcess.hpp
 * @brief   处理检测模型输出张量，生成用于绘制的边框与分数。
 *
 * @details
 * - 解析输出形状为 [1, 5, N] 的张量，其中 5 对应 (cx, cy, w, h, score)，N 为候选数。
 * - 将中心点宽高 (cx, cy, w, h) 转换为角点坐标 (x1, y1, x2, y2)。
 * - 执行分数阈值过滤与 NMS 抑制重叠框。
 */

#pragma once
#include <iostream>
#include <algorithm>
#include <numeric>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace OutputDataProcess {

/**
 * @brief 后处理结果结构体。
 * @note  `boxes` 中每个元素为 {x1, y1, x2, y2, score}。
 */
struct DataProcessResult {
  std::vector<std::array<float, 5>> boxes;  ///< 检测框列表：{x1,y1,x2,y2,score}
};

/**
 * @brief 输出后处理器：解析模型输出并生成最终检测框。
 * @note  支持阈值过滤与 NMS 去重。
 */
class OutputDataProcess {
 public:
  OutputDataProcess() = default;

  /**
   * @brief 解析模型输出张量，过滤并抑制重叠框，返回最终检测结果。
   * @param[in] output_tensor_         ONNX Runtime 输出张量，形状期望为 [1, 5, N]
   * @param[in] original_image_size_   原图尺寸，用于将 640×640 空间坐标缩放回原图坐标
   * @returns   DataProcessResult      包含最终用于绘制的边框与分数
   */
  DataProcessResult run(Ort::Value &output_tensor_, cv::Size original_image_size_) const {
    DataProcessResult result_{};

    float *output_data_ = output_tensor_.GetTensorMutableData<float>();
    Ort::TensorTypeAndShapeInfo output_info_ = output_tensor_.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_shape_ = output_info_.GetShape();
    // 期望形状为 [1, 5, 8400]，其中 5 为 (cx, cy, w, h, score)，8400 为候选数

    const int num_detections_ = static_cast<int>(output_shape_[2]);  // 8400

    float scale_x_ = static_cast<float>(original_image_size_.width) / static_cast<float>(cut_size_.width);
    float scale_y_ = static_cast<float>(original_image_size_.height) / static_cast<float>(cut_size_.height);
    float max = -1e9, min = 1e9;
    // 通道优先：每个通道连续存放所有候选的该通道值
    for (int i = 0; i < num_detections_; ++i) {
      float cx = output_data_[0 * num_detections_ + i];
      float cy = output_data_[1 * num_detections_ + i];
      float w = output_data_[2 * num_detections_ + i];
      float h = output_data_[3 * num_detections_ + i];
      float score = output_data_[4 * num_detections_ + i];
      if (score > max) max = score;
      if (score < min) min = score;

      if (score > set_score_) {
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        float x2 = cx + w / 2.0f;
        float y2 = cy + h / 2.0f;
        x1 *= scale_x_;
        y1 *= scale_y_;
        x2 *= scale_x_;
        y2 *= scale_y_;

        float w_h_ratio_ = w / h;
        if (w_h_ratio_ < w_h_scale_ || (1.0f / w_h_ratio_) < w_h_scale_) {
          continue;
        }
        result_.boxes.push_back({x1, y1, x2, y2, score});
      }
    }

    // 应用 NMS 过滤重叠框
    result_.boxes = nms_(result_.boxes, nms_iou_thresh_);
    return result_;
  }

 private:
  static float iou_(const std::array<float, 5> &a, const std::array<float, 5> &b) {
    float xx1 = std::max(a[0], b[0]);
    float yy1 = std::max(a[1], b[1]);
    float xx2 = std::min(a[2], b[2]);
    float yy2 = std::min(a[3], b[3]);
    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;
    float areaA = std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
    float areaB = std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
    float uni = areaA + areaB - inter;
    if (uni <= 0.0f) return 0.0f;
    return inter / uni;
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
      int i = order[oi];
      if (removed[i]) continue;
      keep.push_back(boxes[i]);
      for (size_t oj = oi + 1; oj < order.size(); ++oj) {
        int j = order[oj];
        if (removed[j]) continue;
        if (iou_(boxes[i], boxes[j]) > iou_thresh) {
          if (boxes[i][4] >= boxes[j][4]) {
            removed[j] = 1;
          } else {
            removed[i] = 1;
            keep.pop_back();
            break;
          }
        }
      }
    }
    return keep;
  }
  cv::Size cut_size_{640, 640};  ///< 预处理/模型输入的基准尺寸（用于反缩放）
  float set_score_{0.85f};       ///< 分数阈值，低于该值的候选将被丢弃
  float nms_iou_thresh_{0.5f};   ///< NMS 的 IoU 阈值
  float w_h_scale_{0.5f};        ///< 长宽比阈值
};
}  // namespace OutputDataProcess