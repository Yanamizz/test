/**
 * @file    include/ImageRecognize/TemporalBoxStabilizer.hpp
 * @brief   提供跨帧检测框稳定能力，用于抑制高频框抖动。
 *
 * TemporalBoxStabilizer 通过门控和 One Euro Filter 平滑检测框中心与尺寸，
 * 仅在确认仍是同一目标时更新滤波状态。当前主要用于 stage3 小画幅场景，
 * 目标是在不明显增加跟随滞后的前提下降低框抖动导致的角度和距离波动。
 */

#pragma once

#include <algorithm>

#include "ImageRecognize/OutputDataProcess.hpp"
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

  DetectionBox Update(const DetectionBox &raw_box, bool matched_history,
                      double dt = -1.0) {
    if (!initialized_ || !matched_history ||
        BoxClassId(stable_box_) != BoxClassId(raw_box)) {
      stable_box_ = raw_box;
      initialized_ = true;
      SeedFiltersFromBox_(raw_box, dt);
      return stable_box_;
    }

    const float iou = BoxIoU(stable_box_, raw_box);
    const float center_distance_ratio =
        BoxCenterDistanceRatio(stable_box_, raw_box);
    const float area_ratio = BoxAreaRatio(stable_box_, raw_box);
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

    stable_box_ =
        FromBoxState_(filtered_state, BoxScore(raw_box), BoxClassId(raw_box));
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

  static BoxState ToBoxState_(const DetectionBox &box) {
    const BoxCenterPoint center = BoxCenter(box);
    return {center.x, center.y, BoxWidth(box), BoxHeight(box)};
  }

  static DetectionBox FromBoxState_(const BoxState &state, float score,
                                    int class_id) {
    const float half_width = 0.5f * state.width;
    const float half_height = 0.5f * state.height;
    return MakeDetectionBox(state.cx - half_width, state.cy - half_height,
                            state.cx + half_width, state.cy + half_height,
                            score, class_id);
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

  void SeedFiltersFromBox_(const DetectionBox &box, double dt) {
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
  DetectionBox stable_box_{};
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
