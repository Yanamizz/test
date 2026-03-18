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
#include <opencv2/opencv.hpp>

namespace ImageRecognize {

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
  cv::Size cut_size_{480, 300};  ///< 预处理/模型输入的基准尺寸（用于反缩放）
  float set_score_{0.5f};        ///< 分数阈值，低于该值的候选将被丢弃
  float nms_iou_thresh_{0.5f};   ///< NMS 的 IoU 阈值
  float w_h_scale_{0.5f};        ///< 长宽比阈值
};
}  // namespace ImageRecognize