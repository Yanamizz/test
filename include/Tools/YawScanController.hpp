#pragma once

#include <algorithm>
#include <cstdint>

namespace Tools {

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
    float min_yaw_deg = 53.0f;  // 暂定下限
    float max_yaw_deg = 76.0f;  // 暂定上限
    float step_deg = 0.5f;      // 暂定步进
  };

  explicit YawScanController(Config config = Config{}) : config_(config) { Reset(); }

  void Reset() {
    current_yaw_deg_ = std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    scan_forward_ = true;
  }

  YawScanCommand BuildCommand(float imu_yaw_deg, float imu_pitch_deg) {
    const float scan_min_deg = std::min(config_.min_yaw_deg, config_.max_yaw_deg);
    const float scan_max_deg = std::max(config_.min_yaw_deg, config_.max_yaw_deg);
    const float scan_step_deg = std::max(0.1f, config_.step_deg);

    if (current_yaw_deg_ < scan_min_deg) current_yaw_deg_ = scan_min_deg;
    if (current_yaw_deg_ > scan_max_deg) current_yaw_deg_ = scan_max_deg;

    const float scan_yaw_deg = current_yaw_deg_;
    if (scan_forward_) {
      current_yaw_deg_ += scan_step_deg;
      if (current_yaw_deg_ >= scan_max_deg) {
        current_yaw_deg_ = scan_max_deg;
        scan_forward_ = false;
      }
    } else {
      current_yaw_deg_ -= scan_step_deg;
      if (current_yaw_deg_ <= scan_min_deg) {
        current_yaw_deg_ = scan_min_deg;
        scan_forward_ = true;
      }
    }

    YawScanCommand command{};
    command.absolute_yaw_deg = scan_yaw_deg;
    command.absolute_pitch_deg = imu_pitch_deg;
    command.offset_yaw_deg = scan_yaw_deg - imu_yaw_deg;
    command.offset_pitch_deg = 0.0f;
    command.aimbot_state = 0x01;
    return command;
  }

 private:
  Config config_{};
  float current_yaw_deg_ = 53.0f;
  bool scan_forward_ = true;
};

}  // namespace Tools