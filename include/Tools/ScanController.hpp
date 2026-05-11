#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace Tools {

enum class ScanAxis {
  Yaw,
  Pitch,
};

inline const char *ToString(ScanAxis axis) {
  switch (axis) {
  case ScanAxis::Yaw:
    return "yaw";
  case ScanAxis::Pitch:
    return "pitch";
  default:
    return "unknown";
  }
}

struct ScanCommand {
  float absolute_yaw_deg = 0.0f;
  float absolute_pitch_deg = 0.0f;
  float offset_yaw_deg = 0.0f;
  float offset_pitch_deg = 0.0f;
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
    current_yaw_deg_ = std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    scan_forward_ = true;
    last_update_time_ = Clock::now();
  }

  ScanCommand BuildOriginCommand(float imu_yaw_deg, float imu_pitch_deg) const {
    ScanCommand command{};
    const float origin_yaw_deg =
        std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    const float origin_pitch_deg =
        0.5f * (config_.min_pitch_deg + config_.max_pitch_deg);
    command.absolute_yaw_deg = origin_yaw_deg;
    command.absolute_pitch_deg = origin_pitch_deg;
    command.offset_yaw_deg = origin_yaw_deg - imu_yaw_deg;
    command.offset_pitch_deg = origin_pitch_deg - imu_pitch_deg;
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
    const float pitch_mid_deg =
        0.5f * (config_.min_pitch_deg + config_.max_pitch_deg);
    const float pitch_amplitude_deg =
        0.125f * std::abs(config_.max_pitch_deg - config_.min_pitch_deg);
    const auto now = Clock::now();
    const float dt_sec =
        std::max(0.0f, std::chrono::duration_cast<std::chrono::duration<float>>(
                           now - last_update_time_)
                           .count());
    last_update_time_ = now;

    const float yaw_delta =
        std::max(config_.yaw_speed_deg_per_sec, 1e-4f) * dt_sec;
    if (scan_forward_) {
      current_yaw_deg_ += yaw_delta;
      if (current_yaw_deg_ >= max_yaw_deg) {
        const float overshoot = current_yaw_deg_ - max_yaw_deg;
        current_yaw_deg_ = max_yaw_deg - overshoot;
        scan_forward_ = false;
      }
    } else {
      current_yaw_deg_ -= yaw_delta;
      if (current_yaw_deg_ <= min_yaw_deg) {
        const float overshoot = min_yaw_deg - current_yaw_deg_;
        current_yaw_deg_ = min_yaw_deg + overshoot;
        scan_forward_ = true;
      }
    }
    current_yaw_deg_ = std::clamp(current_yaw_deg_, min_yaw_deg, max_yaw_deg);

    const float normalized_progress =
        (current_yaw_deg_ - min_yaw_deg) / yaw_span_deg;
    const float phase_offset = scan_forward_ ? 0.0f : kPi;
    const float scan_pitch_deg =
        pitch_mid_deg +
        pitch_amplitude_deg *
            std::sin(2.0f * kPi * normalized_progress + phase_offset);

    command.absolute_yaw_deg = current_yaw_deg_;
    command.absolute_pitch_deg = scan_pitch_deg;
    command.offset_yaw_deg = current_yaw_deg_ - imu_yaw_deg;
    command.offset_pitch_deg = scan_pitch_deg - imu_pitch_deg;
    command.aimbot_state = 0x01;
    return command;
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr float kPi = 3.14159265358979323846f;

  Config config_{};
  float current_yaw_deg_ = 0.0f;
  bool scan_forward_ = true;
  Clock::time_point last_update_time_{Clock::now()};
};

inline ScanController::Config::Config()
    : min_pitch_deg(-268.0f),        // pitch 扫描范围下限（度）
      max_pitch_deg(-247.0f),        // pitch 扫描起始上限（度）
      min_yaw_deg(36.0f),            // yaw 扫描起始下限（度）
      max_yaw_deg(57.0f),            // yaw 扫描起始上限（度）
      yaw_speed_deg_per_sec(4.0f) {} // yaw 扫描角速度（度/秒）

} // namespace Tools
