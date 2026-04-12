#pragma once

#include <algorithm>
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

struct YawScanCommand {
  float absolute_yaw_deg = 0.0f;
  float absolute_pitch_deg = 0.0f;
  float offset_yaw_deg = 0.0f;
  float offset_pitch_deg = 0.0f;
  uint8_t aimbot_state = 0x01;
};

class YawScanController {
 public:
  struct Config {
    ScanAxis axis;
    float min_pitch_deg;
    float max_pitch_deg;
    float min_yaw_deg;
    float max_yaw_deg;
    float step_deg;

    Config();
  };

  YawScanController() { Reset(); }

  explicit YawScanController(const Config &config) : config_(config) { Reset(); }

  void Reset() {
    current_scan_deg_ = ActiveMinDeg_();
    scan_forward_ = true;
  }

  YawScanCommand BuildCommand(float imu_yaw_deg, float imu_pitch_deg) {
    const float scan_min_deg = ActiveMinDeg_();
    const float scan_max_deg = ActiveMaxDeg_();
    const float scan_step_deg = config_.step_deg;

    if (current_scan_deg_ < scan_min_deg) current_scan_deg_ = scan_min_deg;
    if (current_scan_deg_ > scan_max_deg) current_scan_deg_ = scan_max_deg;

    const float scan_deg = current_scan_deg_;
    if (scan_forward_) {
      current_scan_deg_ += scan_step_deg;
      if (current_scan_deg_ >= scan_max_deg) {
        current_scan_deg_ = scan_max_deg;
        scan_forward_ = false;
      }
    } else {
      current_scan_deg_ -= scan_step_deg;
      if (current_scan_deg_ <= scan_min_deg) {
        current_scan_deg_ = scan_min_deg;
        scan_forward_ = true;
      }
    }

    YawScanCommand command{};
    if (config_.axis == ScanAxis::Yaw) {
      command.absolute_yaw_deg = scan_deg;
      command.absolute_pitch_deg = imu_pitch_deg;
      command.offset_yaw_deg = scan_deg - imu_yaw_deg;
      command.offset_pitch_deg = 0.0f;
    } else {
      command.absolute_yaw_deg = imu_yaw_deg;
      command.absolute_pitch_deg = scan_deg;
      command.offset_yaw_deg = 0.0f;
      command.offset_pitch_deg = scan_deg - imu_pitch_deg;
    }
    command.aimbot_state = 0x01;
    return command;
  }

 private:
  float ActiveMinDeg_() const {
    if (config_.axis == ScanAxis::Yaw) {
      return std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    }
    return std::min(config_.min_pitch_deg, config_.max_pitch_deg);
  }

  float ActiveMaxDeg_() const {
    if (config_.axis == ScanAxis::Yaw) {
      return std::max(config_.min_yaw_deg, config_.max_yaw_deg);
    }
    return std::max(config_.min_pitch_deg, config_.max_pitch_deg);
  }

  Config config_{};
  float current_scan_deg_ = 0.0f;
  bool scan_forward_ = true;
};

inline YawScanController::Config::Config()
    : axis(ScanAxis::Pitch),   // 扫描默认轴向：Pitch
      min_pitch_deg(-268.0f),  // pitch 扫描起始下限（度）
      max_pitch_deg(-247.0f),  // pitch 扫描起始上限（度）
      min_yaw_deg(36.0f),      // yaw 扫描起始下限（度）
      max_yaw_deg(57.0f),      // yaw 扫描起始上限（度）
      step_deg(0.1f) {}        // 每次扫描的偏移步长（度）

}  // namespace Tools