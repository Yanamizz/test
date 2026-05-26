/**
 * @file    include/Tools/ScanController.hpp
 * @brief   生成沿固定 yaw 边界往返、pitch 服从正弦轨迹的扫描指令。
 *
 * 参数语义：
 * - A 百分比：以 pitch 半扫描高度为 100%
 * - lambda 百分比：以“单次去程恰好走完一个完整正弦周期”为 100%
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "Tools/RuntimeParams.hpp"
namespace Tools {

struct ScanCommand {
  float absolute_yaw_deg = 0.0f;
  float absolute_pitch_deg = 0.0f;
  float offset_yaw_deg = 0.0f;
  float offset_pitch_deg = 0.0f;
  float yaw_velocity_deg_per_sec = 0.0f;
  float pitch_velocity_deg_per_sec = 0.0f;
  uint8_t aimbot_state = 0x01;
};

class ScanController {
public:
  struct Config {
    float min_pitch_deg;
    float max_pitch_deg;
    float min_yaw_deg;
    float max_yaw_deg;
    float yaw_step_deg_per_tick;
    float tick_rate_hz;
    float pitch_wavelength_percent;
    float pitch_amplitude_percent;

    Config();
  };

  ScanController() { Reset(); }

  explicit ScanController(const Config &config) : config_(config) { Reset(); }

  void SetConfig(const Config &config) {
    config_ = config;
    Reset();
  }

  void Reset() {
    current_yaw_deg_ = std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    yaw_direction_sign_ = 1.0f;
  }

  ScanCommand BuildOriginCommand(float imu_yaw_deg, float imu_pitch_deg) const {
    ScanCommand command{};
    const float origin_yaw_deg = OriginYawDeg_();
    const float origin_pitch_deg = OriginPitchDeg_();
    command.absolute_yaw_deg = origin_yaw_deg;
    command.absolute_pitch_deg = origin_pitch_deg;
    command.offset_yaw_deg = origin_yaw_deg - imu_yaw_deg;
    command.offset_pitch_deg = origin_pitch_deg - imu_pitch_deg;
    command.yaw_velocity_deg_per_sec = 0.0f;
    command.pitch_velocity_deg_per_sec = 0.0f;
    command.aimbot_state = 0x01;
    return command;
  }

  ScanCommand BuildCommand(float imu_yaw_deg, float imu_pitch_deg) {
    ScanCommand command{};
    const float min_yaw_deg =
        std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    const float max_yaw_deg =
        std::max(config_.min_yaw_deg, config_.max_yaw_deg);
    const float yaw_span_deg = std::max(max_yaw_deg - min_yaw_deg, 1e-3f);
    const float min_pitch_deg =
        std::min(config_.min_pitch_deg, config_.max_pitch_deg);
    const float max_pitch_deg =
        std::max(config_.min_pitch_deg, config_.max_pitch_deg);
    const float pitch_mid_deg =
        0.5f * (min_pitch_deg + max_pitch_deg);
    const float pitch_span_deg = std::max(max_pitch_deg - min_pitch_deg, 1e-3f);
    const float pitch_amplitude_ratio =
        std::clamp(config_.pitch_amplitude_percent, 0.0f, 1000.0f) / 100.0f;
    const float pitch_amplitude_deg = 0.5f * pitch_span_deg * pitch_amplitude_ratio;
    const float wavelength_ratio =
        std::max(config_.pitch_wavelength_percent, 1e-3f) / 100.0f;
    const float effective_lambda_deg = yaw_span_deg * wavelength_ratio;

    AdvanceAlongYaw_();

    const float yaw_travel_deg = current_yaw_deg_ - min_yaw_deg;
    const float pitch_phase_rad =
        (2.0f * kPiF_ * yaw_travel_deg) / effective_lambda_deg;
    const float pitch_phase_sign =
        yaw_direction_sign_ >= 0.0f ? 1.0f : -1.0f;
    const float scan_yaw_deg = current_yaw_deg_;
    const float scan_pitch_deg =
        pitch_mid_deg + pitch_phase_sign * pitch_amplitude_deg *
                            std::sin(pitch_phase_rad);
    const float scan_yaw_velocity_deg_per_sec =
        yaw_direction_sign_ * std::max(config_.yaw_step_deg_per_tick, 1e-4f) *
        std::max(config_.tick_rate_hz, 1.0f);
    const float scan_pitch_velocity_deg_per_sec =
        pitch_phase_sign * pitch_amplitude_deg * std::cos(pitch_phase_rad) *
        ((2.0f * kPiF_) / effective_lambda_deg) *
        scan_yaw_velocity_deg_per_sec;

    command.absolute_yaw_deg = scan_yaw_deg;
    command.absolute_pitch_deg =
        std::clamp(scan_pitch_deg, min_pitch_deg, max_pitch_deg);
    command.offset_yaw_deg = scan_yaw_deg - imu_yaw_deg;
    command.offset_pitch_deg = command.absolute_pitch_deg - imu_pitch_deg;
    command.yaw_velocity_deg_per_sec = scan_yaw_velocity_deg_per_sec;
    command.pitch_velocity_deg_per_sec = scan_pitch_velocity_deg_per_sec;
    command.aimbot_state = 0x01;
    return command;
  }

private:
  static constexpr float kPiF_ = 3.1415926f;

  float OriginYawDeg_() const {
    return std::min(config_.min_yaw_deg, config_.max_yaw_deg);
  }

  float OriginPitchDeg_() const {
    return 0.5f * (config_.min_pitch_deg + config_.max_pitch_deg);
  }

  void AdvanceAlongYaw_() {
    const float min_yaw_deg =
        std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    const float max_yaw_deg =
        std::max(config_.min_yaw_deg, config_.max_yaw_deg);
    const float yaw_span_deg = max_yaw_deg - min_yaw_deg;
    if (yaw_span_deg <= 1e-4f) {
      current_yaw_deg_ = min_yaw_deg;
      yaw_direction_sign_ = 1.0f;
      return;
    }

    float remaining_step_deg = std::max(config_.yaw_step_deg_per_tick, 1e-4f);
    while (remaining_step_deg > 0.0f) {
      const float boundary_yaw_deg =
          yaw_direction_sign_ > 0.0f ? max_yaw_deg : min_yaw_deg;
      const float available_step_deg =
          std::abs(boundary_yaw_deg - current_yaw_deg_);
      if (remaining_step_deg <= available_step_deg + 1e-6f) {
        current_yaw_deg_ += yaw_direction_sign_ * remaining_step_deg;
        break;
      }

      current_yaw_deg_ = boundary_yaw_deg;
      remaining_step_deg -= available_step_deg;
      yaw_direction_sign_ *= -1.0f;
    }
  }

  Config config_{};
  float current_yaw_deg_ = 0.0f;
  float yaw_direction_sign_ = 1.0f;
};

inline ScanController::Config::Config()
    : min_pitch_deg(70.9f),         // pitch 扫描范围下限（度）
      max_pitch_deg(83.1f),         // pitch 扫描起始上限（度）
      min_yaw_deg(60.62f),          // yaw 扫描起始下限（度）
      max_yaw_deg(85.1f),           // yaw 扫描起始上限（度）
      yaw_step_deg_per_tick(0.08f),    // 每次发送沿 yaw 轨迹前进的角度
      tick_rate_hz(200.0f),            // 当前扫描发送频率（用于速度估计）
      pitch_wavelength_percent(100.0f), // lambda 百分比，100% 表示单次去程恰好一周期
      pitch_amplitude_percent(100.0f) {} // A 百分比，100% 表示占满 pitch 半量程

inline ScanController::Config MakeDefaultScanControllerConfig() {
  ScanController::Config config;
  config.pitch_wavelength_percent =
      static_cast<float>(Params().scan_pitch_wavelength_percent);
  config.pitch_amplitude_percent =
      static_cast<float>(Params().scan_pitch_amplitude_percent);
  return config;
}

inline ScanController::Config MakeStage3ScanControllerConfig() {
  ScanController::Config config;
  config.pitch_wavelength_percent =
      static_cast<float>(Params().stage3_scan_pitch_wavelength_percent);
  config.pitch_amplitude_percent =
      static_cast<float>(Params().stage3_scan_pitch_amplitude_percent);
  return config;
}

} // namespace Tools
