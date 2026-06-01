/**
 * @file    include/ImageRecognize/TemporalBoxStabilizer.hpp
 * @brief   提供跨帧检测框稳定能力，用于抑制高频框抖动。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "KalmanFilter/OneEuroFilter.hpp"

namespace ImageRecognize {

struct TemporalBoxStabilizerParams {
  float min_iou_to_stabilize;         // 低于该 IoU 直接放弃平滑，避免错平滑到别的目标
  float max_center_distance_ratio;    // 中心点相对位移超过该比例时不做平滑
  float max_area_ratio;               // 面积变化过大时不做平滑
  float max_height_expand_ratio;      // 单帧高度上跳允许的最大比例
  double center_min_cutoff_hz;        // 框中心点 One Euro 最小截止频率
  double center_beta;                 // 框中心点 One Euro 速度增益
  double size_min_cutoff_hz;          // 框尺寸 One Euro 最小截止频率
  double size_beta;                   // 框尺寸 One Euro 速度增益
  double derivative_cutoff_hz;        // One Euro 导数低通截止频率
  double center_x_min_cutoff_hz;      // X 方向中心点单独截止频率；<=0 时回退到 center_min_cutoff_hz
  double center_y_min_cutoff_hz;      // Y 方向中心点单独截止频率；<=0 时回退到 center_min_cutoff_hz
  double center_x_beta;               // X 方向中心点单独速度增益；<0 时回退到 center_beta
  double center_y_beta;               // Y 方向中心点单独速度增益；<0 时回退到 center_beta
  double width_min_cutoff_hz;         // 宽度单独截止频率；<=0 时回退到 size_min_cutoff_hz
  double height_min_cutoff_hz;        // 高度单独截止频率；<=0 时回退到 size_min_cutoff_hz
  double width_beta;                  // 宽度单独速度增益；<0 时回退到 size_beta
  double height_beta;                 // 高度单独速度增益；<0 时回退到 size_beta

  TemporalBoxStabilizerParams();

  double EffectiveCenterXMinCutoffHz() const {
    return center_x_min_cutoff_hz > 0.0 ? center_x_min_cutoff_hz
                                        : center_min_cutoff_hz;
  }

  double EffectiveCenterYMinCutoffHz() const {
    return center_y_min_cutoff_hz > 0.0 ? center_y_min_cutoff_hz
                                        : center_min_cutoff_hz;
  }

  double EffectiveCenterXBeta() const {
    return center_x_beta >= 0.0 ? center_x_beta : center_beta;
  }

  double EffectiveCenterYBeta() const {
    return center_y_beta >= 0.0 ? center_y_beta : center_beta;
  }

  double EffectiveWidthMinCutoffHz() const {
    return width_min_cutoff_hz > 0.0 ? width_min_cutoff_hz
                                     : size_min_cutoff_hz;
  }

  double EffectiveHeightMinCutoffHz() const {
    return height_min_cutoff_hz > 0.0 ? height_min_cutoff_hz
                                      : size_min_cutoff_hz;
  }

  double EffectiveWidthBeta() const {
    return width_beta >= 0.0 ? width_beta : size_beta;
  }

  double EffectiveHeightBeta() const {
    return height_beta >= 0.0 ? height_beta : size_beta;
  }
};

class TemporalBoxStabilizer {
 public:
  explicit TemporalBoxStabilizer(
      const TemporalBoxStabilizerParams &params = TemporalBoxStabilizerParams{})
      : params_(params),
        center_x_filter_(120.0, params.EffectiveCenterXMinCutoffHz(),
                         params.EffectiveCenterXBeta(),
                         params.derivative_cutoff_hz),
        center_y_filter_(120.0, params.EffectiveCenterYMinCutoffHz(),
                         params.EffectiveCenterYBeta(),
                         params.derivative_cutoff_hz),
        width_filter_(120.0, params.EffectiveWidthMinCutoffHz(),
                      params.EffectiveWidthBeta(),
                      params.derivative_cutoff_hz),
        height_filter_(120.0, params.EffectiveHeightMinCutoffHz(),
                       params.EffectiveHeightBeta(),
                       params.derivative_cutoff_hz) {}

  void Reset() {
    initialized_ = false;
    stable_box_ = {};
    ResetFilters_();
  }

  std::array<float, 6> Update(const std::array<float, 6> &raw_box,
                              bool matched_history, double dt = -1.0) {
    if (!initialized_ || !matched_history ||
        static_cast<int>(stable_box_[5]) != static_cast<int>(raw_box[5])) {
      stable_box_ = raw_box;
      initialized_ = true;
      SeedFiltersFromBox_(raw_box, dt);
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
      SeedFiltersFromBox_(raw_box, dt);
      return stable_box_;
    }

    const BoxState raw_state = ToBoxState_(raw_box);
    const BoxState stable_state = ToBoxState_(stable_box_);
    BoxState filtered_state{};
    filtered_state.cx =
        static_cast<float>(center_x_filter_.filter(raw_state.cx, dt));
    filtered_state.cy =
        static_cast<float>(center_y_filter_.filter(raw_state.cy, dt));
    filtered_state.width =
        std::max(1.0f,
                 static_cast<float>(width_filter_.filter(raw_state.width, dt)));
    const float clamped_height_measurement =
        ClampHeightMeasurement_(stable_state.height, raw_state.height);
    filtered_state.height = std::max(
        1.0f,
        static_cast<float>(height_filter_.filter(clamped_height_measurement, dt)));

    stable_box_ = FromBoxState_(filtered_state, raw_box[4], raw_box[5]);
    initialized_ = true;
    return stable_box_;
  }

 private:
  struct BoxState {
    float cx = 0.0f;
    float cy = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
  };

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

  static BoxState ToBoxState_(const std::array<float, 6> &box) {
    const float width = BoxWidth_(box);
    const float height = BoxHeight_(box);
    return {0.5f * (box[0] + box[2]), 0.5f * (box[1] + box[3]), width, height};
  }

  static std::array<float, 6> FromBoxState_(const BoxState &state, float score,
                                            float class_id) {
    const float half_width = 0.5f * state.width;
    const float half_height = 0.5f * state.height;
    return {state.cx - half_width, state.cy - half_height,
            state.cx + half_width, state.cy + half_height, score, class_id};
  }

  float ClampHeightMeasurement_(float stable_height, float raw_height) const {
    const float safe_stable_height = std::max(1.0f, stable_height);
    if (raw_height <= safe_stable_height) {
      return std::max(1.0f, raw_height);
    }

    const float expand_ratio =
        std::max(params_.max_height_expand_ratio, 1.0f);
    return std::min(raw_height, safe_stable_height * expand_ratio);
  }

  void ResetFilters_() {
    center_x_filter_.reset();
    center_y_filter_.reset();
    width_filter_.reset();
    height_filter_.reset();
  }

  void SeedFiltersFromBox_(const std::array<float, 6> &box, double dt) {
    ResetFilters_();
    const BoxState state = ToBoxState_(box);
    center_x_filter_.filter(state.cx, dt);
    center_y_filter_.filter(state.cy, dt);
    width_filter_.filter(state.width, dt);
    height_filter_.filter(state.height, dt);
  }

  TemporalBoxStabilizerParams params_{};
  Tools::OneEuroFilter center_x_filter_;
  Tools::OneEuroFilter center_y_filter_;
  Tools::OneEuroFilter width_filter_;
  Tools::OneEuroFilter height_filter_;
  bool initialized_ = false;
  std::array<float, 6> stable_box_{};
};

inline TemporalBoxStabilizerParams::TemporalBoxStabilizerParams()
    : min_iou_to_stabilize(0.30f),      // 低于该 IoU 视为明显跳框，不做平滑
      max_center_distance_ratio(0.65f), // 位移过大时优先保留响应速度
      max_area_ratio(1.8f),             // 面积变化超过 1.8x 不做平滑
      max_height_expand_ratio(1.12f),   // 默认允许高度单帧上跳 12%
      center_min_cutoff_hz(1.6),        // 低速时更稳，高速时交给 beta 拉高带宽
      center_beta(0.28),                // 中心移动加快时主动减小滞后
      size_min_cutoff_hz(1.2),          // 尺寸变化通常比中心更抖，适当更稳
      size_beta(0.16),                  // 尺寸运动时仍保留一定响应
      derivative_cutoff_hz(1.0),
      center_x_min_cutoff_hz(0.0),
      center_y_min_cutoff_hz(0.0),
      center_x_beta(-1.0),
      center_y_beta(-1.0),
      width_min_cutoff_hz(0.0),
      height_min_cutoff_hz(0.0),
      width_beta(-1.0),
      height_beta(-1.0) {}

} // namespace ImageRecognize
