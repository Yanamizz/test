/**
 * @file    include/Tools/Stage12PitchPostprocess.hpp
 * @brief   Pitch 控制后处理工具，包含微死区和激光补偿平滑。
 *
 * 该文件提供两个轻量后处理能力：ApplySoftDeadband() 用于在最终发送前
 * 压制极小 pitch 抖动，LaserPitchCompStabilizer 用 One Euro Filter 与
 * 单帧变化率限制平滑 LaserPc 激光 pitch 补偿角。激光补偿平滑器不绑定
 * 特定 stage，stage1/2 与 stage3 使用同一条处理链路，只由
 * LaserAngleCalculate.hpp 中的阶段参数决定补偿数值。
 */

#pragma once

#include <algorithm>
#include <cmath>

#include "KalmanFilter/OneEuroFilter.hpp"

namespace Tools {

inline float ApplySoftDeadband(float value, float deadband) {
  const float safe_deadband = std::max(0.0f, deadband);
  if (safe_deadband <= 0.0f || !std::isfinite(value)) {
    return value;
  }

  const float magnitude = std::abs(value);
  if (magnitude >= safe_deadband) {
    return value;
  }

  const float scaled_magnitude =
      (magnitude * magnitude) / std::max(safe_deadband, 1e-6f);
  return std::copysign(scaled_magnitude, value);
}

class LaserPitchCompStabilizer {
public:
  float Filter(float raw_comp_deg, double dt_sec) {
    if (!std::isfinite(raw_comp_deg)) {
      Reset();
      return 0.0f;
    }

    const double filter_dt =
        (std::isfinite(dt_sec) && dt_sec > 0.0) ? dt_sec : -1.0;
    const float filtered_comp =
        static_cast<float>(filter_.filter(raw_comp_deg, filter_dt));

    if (!has_last_output_) {
      last_output_deg_ = filtered_comp;
      has_last_output_ = true;
      return filtered_comp;
    }

    const double safe_dt =
        (std::isfinite(dt_sec) && dt_sec > 0.0 && dt_sec <= 1.0)
            ? dt_sec
            : kFallbackDtSec;
    const float max_step_deg =
        kMaxCompDeltaDegPerSec * static_cast<float>(safe_dt);
    const float stabilized_comp = std::clamp(
        filtered_comp, last_output_deg_ - max_step_deg,
        last_output_deg_ + max_step_deg);
    last_output_deg_ = stabilized_comp;
    return stabilized_comp;
  }

  void Reset() {
    filter_.reset();
    has_last_output_ = false;
    last_output_deg_ = 0.0f;
  }

private:
  static constexpr double kFilterFrequencyHz = 120.0;
  static constexpr double kMinCutoffHz = 1.2;
  static constexpr double kBeta = 0.02;
  static constexpr double kDerivativeCutoffHz = 1.0;
  static constexpr double kFallbackDtSec = 1.0 / 120.0;
  static constexpr float kMaxCompDeltaDegPerSec = 6.0f;

  OneEuroFilter filter_{kFilterFrequencyHz, kMinCutoffHz, kBeta,
                        kDerivativeCutoffHz};
  bool has_last_output_ = false;
  float last_output_deg_ = 0.0f;
};

} // namespace Tools
