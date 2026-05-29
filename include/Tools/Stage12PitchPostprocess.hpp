/**
 * @file    include/Tools/Stage12PitchPostprocess.hpp
 * @brief   收口 stage1/2 pitch 微抖抑制与激光补偿平滑后处理。
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

class Stage12LaserPitchCompStabilizer {
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
