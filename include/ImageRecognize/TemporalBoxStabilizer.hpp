/**
 * @file    include/ImageRecognize/TemporalBoxStabilizer.hpp
 * @brief   提供跨帧检测框稳定能力，用于抑制高频框抖动。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace ImageRecognize {

struct TemporalBoxStabilizerParams {
  float min_iou_to_stabilize;         // 低于该 IoU 直接放弃平滑，避免错平滑到别的目标
  float max_center_distance_ratio;    // 中心点相对位移超过该比例时不做平滑
  float max_area_ratio;               // 面积变化过大时不做平滑
  float base_measurement_gain;        // 当前帧测量的基础权重，越大越跟手
  float min_measurement_gain;         // 平滑时当前帧的最小权重
  float max_measurement_gain;         // 平滑时当前帧的最大权重

  TemporalBoxStabilizerParams();
};

class TemporalBoxStabilizer {
 public:
  explicit TemporalBoxStabilizer(
      const TemporalBoxStabilizerParams &params = TemporalBoxStabilizerParams{})
      : params_(params) {}

  void Reset() {
    initialized_ = false;
    stable_box_ = {};
  }

  std::array<float, 6> Update(const std::array<float, 6> &raw_box,
                              bool matched_history) {
    if (!initialized_ || !matched_history ||
        static_cast<int>(stable_box_[5]) != static_cast<int>(raw_box[5])) {
      stable_box_ = raw_box;
      initialized_ = true;
      return stable_box_;
    }

    const float iou = BoxIou_(stable_box_, raw_box);
    const float center_distance_ratio =
        CenterDistanceRatio_(stable_box_, raw_box);
    const float area_ratio = AreaRatio_(stable_box_, raw_box);
    const bool area_ratio_ok =
        area_ratio >= (1.0f / std::max(params_.max_area_ratio, 1.01f)) &&
        area_ratio <= std::max(params_.max_area_ratio, 1.01f);

    if (iou < params_.min_iou_to_stabilize ||
        center_distance_ratio > params_.max_center_distance_ratio ||
        !area_ratio_ok) {
      stable_box_ = raw_box;
      initialized_ = true;
      return stable_box_;
    }

    const float iou_score =
        Saturate_((iou - params_.min_iou_to_stabilize) /
                  std::max(1e-3f, 1.0f - params_.min_iou_to_stabilize));
    const float motion_score =
        Saturate_(1.0f - center_distance_ratio /
                             std::max(params_.max_center_distance_ratio, 1e-3f));
    const float confidence_score = Saturate_(raw_box[4]);
    const float area_score = AreaScore_(area_ratio);

    float measurement_gain =
        params_.base_measurement_gain +
        0.18f * (1.0f - iou_score) +
        0.16f * (1.0f - motion_score) +
        0.08f * (1.0f - confidence_score) +
        0.08f * (1.0f - area_score);
    measurement_gain =
        std::clamp(measurement_gain, params_.min_measurement_gain,
                   params_.max_measurement_gain);

    for (int i = 0; i < 4; ++i) {
      stable_box_[i] =
          stable_box_[i] +
          (raw_box[i] - stable_box_[i]) * measurement_gain;
    }
    stable_box_[4] = raw_box[4];
    stable_box_[5] = raw_box[5];
    initialized_ = true;
    return stable_box_;
  }

 private:
  static float Saturate_(float value) {
    return std::clamp(value, 0.0f, 1.0f);
  }

  static float BoxWidth_(const std::array<float, 6> &box) {
    return std::max(0.0f, box[2] - box[0]);
  }

  static float BoxHeight_(const std::array<float, 6> &box) {
    return std::max(0.0f, box[3] - box[1]);
  }

  static float BoxArea_(const std::array<float, 6> &box) {
    return BoxWidth_(box) * BoxHeight_(box);
  }

  static float BoxIou_(const std::array<float, 6> &a,
                       const std::array<float, 6> &b) {
    const float xx1 = std::max(a[0], b[0]);
    const float yy1 = std::max(a[1], b[1]);
    const float xx2 = std::min(a[2], b[2]);
    const float yy2 = std::min(a[3], b[3]);
    const float inter_w = std::max(0.0f, xx2 - xx1);
    const float inter_h = std::max(0.0f, yy2 - yy1);
    const float inter = inter_w * inter_h;
    const float uni = BoxArea_(a) + BoxArea_(b) - inter;
    return uni > 0.0f ? (inter / uni) : 0.0f;
  }

  static float CenterDistanceRatio_(const std::array<float, 6> &reference,
                                    const std::array<float, 6> &candidate) {
    const float ref_cx = 0.5f * (reference[0] + reference[2]);
    const float ref_cy = 0.5f * (reference[1] + reference[3]);
    const float cand_cx = 0.5f * (candidate[0] + candidate[2]);
    const float cand_cy = 0.5f * (candidate[1] + candidate[3]);
    const float dx = cand_cx - ref_cx;
    const float dy = cand_cy - ref_cy;
    const float center_distance = std::sqrt(dx * dx + dy * dy);

    const float ref_w = std::max(1.0f, BoxWidth_(reference));
    const float ref_h = std::max(1.0f, BoxHeight_(reference));
    const float ref_diag = std::sqrt(ref_w * ref_w + ref_h * ref_h);
    return center_distance / std::max(1.0f, 0.5f * ref_diag);
  }

  static float AreaRatio_(const std::array<float, 6> &reference,
                          const std::array<float, 6> &candidate) {
    const float ref_area = std::max(1.0f, BoxArea_(reference));
    const float cand_area = std::max(1.0f, BoxArea_(candidate));
    return cand_area / ref_area;
  }

  float AreaScore_(float area_ratio) const {
    const float safe_ratio = std::max(area_ratio, 1e-6f);
    const float safe_limit = std::max(params_.max_area_ratio, 1.01f);
    const float normalized_log_distance =
        std::abs(std::log(safe_ratio)) / std::log(safe_limit);
    return Saturate_(1.0f - normalized_log_distance);
  }

  TemporalBoxStabilizerParams params_{};
  bool initialized_ = false;
  std::array<float, 6> stable_box_{};
};

inline TemporalBoxStabilizerParams::TemporalBoxStabilizerParams()
    : min_iou_to_stabilize(0.30f),      // 低于该 IoU 视为明显跳框，不做平滑
      max_center_distance_ratio(0.65f), // 位移过大时优先保留响应速度
      max_area_ratio(1.8f),             // 面积变化超过 1.8x 不做平滑
      base_measurement_gain(0.62f),     // 默认更偏向当前观测，优先保精度
      min_measurement_gain(0.45f),      // 平滑时至少保留 45% 当前观测
      max_measurement_gain(0.88f) {}    // 跳动较大时几乎直接跟随当前观测

} // namespace ImageRecognize
