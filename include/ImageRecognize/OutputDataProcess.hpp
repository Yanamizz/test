/**
 * @file    include/ImageRecognize/OutputDataProcess.hpp
 * @brief   定义图像识别模块共享的检测结果数据结构。
 *
 * 该头文件保存检测框数组、类别、置信度和后处理输出结构，是推理、跟踪、
 * 显示和主流程之间传递识别结果的公共数据契约。它只定义轻量数据类型和
 * 辅助访问函数，不承担模型推理或控制决策。
 */

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ImageRecognize {

using DetectionBox = std::array<float, 6>;

struct BoxCenterPoint {
  float x = 0.0f;
  float y = 0.0f;
};

inline constexpr std::size_t kBoxX1Index = 0;
inline constexpr std::size_t kBoxY1Index = 1;
inline constexpr std::size_t kBoxX2Index = 2;
inline constexpr std::size_t kBoxY2Index = 3;
inline constexpr std::size_t kBoxScoreIndex = 4;
inline constexpr std::size_t kBoxClassIdIndex = 5;

inline float BoxX1(const DetectionBox &box) { return box[kBoxX1Index]; }
inline float BoxY1(const DetectionBox &box) { return box[kBoxY1Index]; }
inline float BoxX2(const DetectionBox &box) { return box[kBoxX2Index]; }
inline float BoxY2(const DetectionBox &box) { return box[kBoxY2Index]; }
inline float BoxScore(const DetectionBox &box) { return box[kBoxScoreIndex]; }
inline int BoxClassId(const DetectionBox &box) {
  return static_cast<int>(box[kBoxClassIdIndex]);
}
inline void SetBoxClassId(DetectionBox *box, int class_id) {
  if (box != nullptr) {
    (*box)[kBoxClassIdIndex] = static_cast<float>(class_id);
  }
}
inline DetectionBox MakeDetectionBox(float x1, float y1, float x2, float y2,
                                     float score, int class_id) {
  return {x1, y1, x2, y2, score, static_cast<float>(class_id)};
}
inline float BoxWidth(const DetectionBox &box) {
  return std::max(0.0f, BoxX2(box) - BoxX1(box));
}
inline float BoxHeight(const DetectionBox &box) {
  return std::max(0.0f, BoxY2(box) - BoxY1(box));
}
inline BoxCenterPoint BoxCenter(const DetectionBox &box) {
  return {0.5f * (BoxX1(box) + BoxX2(box)),
          0.5f * (BoxY1(box) + BoxY2(box))};
}
inline float BoxArea(const DetectionBox &box) {
  return BoxWidth(box) * BoxHeight(box);
}
inline float BoxCenterDistanceRatio(const DetectionBox &reference,
                                    const DetectionBox &candidate) {
  const BoxCenterPoint reference_center = BoxCenter(reference);
  const BoxCenterPoint candidate_center = BoxCenter(candidate);
  const float dx = candidate_center.x - reference_center.x;
  const float dy = candidate_center.y - reference_center.y;
  const float center_distance = std::sqrt(dx * dx + dy * dy);

  const float reference_width = std::max(1.0f, BoxWidth(reference));
  const float reference_height = std::max(1.0f, BoxHeight(reference));
  const float reference_diag =
      std::sqrt(reference_width * reference_width +
                reference_height * reference_height);
  return center_distance / std::max(1.0f, 0.5f * reference_diag);
}
inline float BoxCenterSimilarity(const DetectionBox &reference,
                                 const DetectionBox &candidate) {
  return 1.0f / (1.0f + BoxCenterDistanceRatio(reference, candidate));
}
inline float BoxAreaRatio(const DetectionBox &reference,
                          const DetectionBox &candidate) {
  const float reference_area = std::max(1.0f, BoxArea(reference));
  const float candidate_area = std::max(1.0f, BoxArea(candidate));
  return candidate_area / reference_area;
}
inline float BoxIoU(const DetectionBox &a, const DetectionBox &b) {
  const float xx1 = std::max(BoxX1(a), BoxX1(b));
  const float yy1 = std::max(BoxY1(a), BoxY1(b));
  const float xx2 = std::min(BoxX2(a), BoxX2(b));
  const float yy2 = std::min(BoxY2(a), BoxY2(b));
  const float inter_width = std::max(0.0f, xx2 - xx1);
  const float inter_height = std::max(0.0f, yy2 - yy1);
  const float intersection = inter_width * inter_height;
  const float union_area = BoxArea(a) + BoxArea(b) - intersection;
  return union_area > 0.0f ? intersection / union_area : 0.0f;
}

/**
 * @brief 后处理结果结构体。
 * @note  `boxes` 中每个元素为 {x1, y1, x2, y2, score, class_id}。
 */
struct DataProcessResult {
  std::vector<DetectionBox> boxes;  ///< 检测框列表：{x1,y1,x2,y2,score,class_id}
};

using PredictResult = DataProcessResult;

}  // namespace ImageRecognize
