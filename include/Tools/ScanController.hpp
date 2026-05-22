/**
 * @file    include/Tools/ScanController.hpp
 * @brief   生成云台扫描模式下的绝对角度与相对偏移控制指令。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "Tools/AngleCalculate.hpp"
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
    float yaw_speed_deg_per_sec;

    Config();
  };

  ScanController() { Reset(); }

  explicit ScanController(const Config &config) : config_(config) { Reset(); }

  void Reset() {
    scan_phase_rad_ = -0.5f * static_cast<float>(kPi);
    last_update_time_ = Clock::now();
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
    const float yaw_mid_deg = 0.5f * (min_yaw_deg + max_yaw_deg);
    const float yaw_amplitude_deg = 0.5f * yaw_span_deg;
    const float pitch_mid_deg =
        0.5f * (config_.min_pitch_deg + config_.max_pitch_deg);
    const float pitch_amplitude_deg =
        0.25f * std::abs(config_.max_pitch_deg - config_.min_pitch_deg);
    const auto now = Clock::now();
    const float dt_sec =
        std::max(0.0f, std::chrono::duration_cast<std::chrono::duration<float>>(
                           now - last_update_time_)
                           .count());
    last_update_time_ = now;

    const float omega_rad_per_sec =
        std::max(config_.yaw_speed_deg_per_sec, 1e-4f) /
        std::max(yaw_amplitude_deg, 1e-3f);
    scan_phase_rad_ += omega_rad_per_sec * dt_sec;
    scan_phase_rad_ =
        std::fmod(scan_phase_rad_, 2.0f * static_cast<float>(kPi));

    const float sin_phase = std::sin(scan_phase_rad_);
    const float cos_phase = std::cos(scan_phase_rad_);

    const float scan_yaw_deg = yaw_mid_deg + yaw_amplitude_deg * sin_phase;
    const float scan_pitch_deg =
        pitch_mid_deg +
        pitch_amplitude_deg *
            std::sin(scan_phase_rad_ + 0.5f * static_cast<float>(kPi));
    const float scan_yaw_velocity_deg_per_sec =
        yaw_amplitude_deg * omega_rad_per_sec * cos_phase;
    const float scan_pitch_velocity_deg_per_sec =
        pitch_amplitude_deg * omega_rad_per_sec *
        std::cos(scan_phase_rad_ + 0.5f * static_cast<float>(kPi));

    command.absolute_yaw_deg = scan_yaw_deg;
    command.absolute_pitch_deg = scan_pitch_deg;
    command.offset_yaw_deg = scan_yaw_deg - imu_yaw_deg;
    command.offset_pitch_deg = scan_pitch_deg - imu_pitch_deg;
    command.yaw_velocity_deg_per_sec = scan_yaw_velocity_deg_per_sec;
    command.pitch_velocity_deg_per_sec = scan_pitch_velocity_deg_per_sec;
    command.aimbot_state = 0x01;
    return command;
  }

private:
  using Clock = std::chrono::steady_clock;

  float OriginYawDeg_() const {
    return std::min(config_.min_yaw_deg, config_.max_yaw_deg);
  }

  float OriginPitchDeg_() const {
    return 0.5f * (config_.min_pitch_deg + config_.max_pitch_deg);
  }

  Config config_{};
  float scan_phase_rad_ = -0.5f * static_cast<float>(kPi);
  Clock::time_point last_update_time_{Clock::now()};
};

inline ScanController::Config::Config()
    : min_pitch_deg(90.5f),           // pitch 扫描范围下限（度）
      max_pitch_deg(75.1f),           // pitch 扫描起始上限（度）
      min_yaw_deg(60.62f),            // yaw 扫描起始下限（度）
      max_yaw_deg(85.1f),             // yaw 扫描起始上限（度）
      yaw_speed_deg_per_sec(16.0f) {} // yaw 扫描角速度（度/秒）

} // namespace Tools
