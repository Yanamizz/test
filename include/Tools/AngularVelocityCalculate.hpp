#pragma once

#include <chrono>
#include "SerialTask/SerialRead.hpp"

namespace Tools {

class AngularVelocityCalculator {
 public:
  AngularVelocityCalculator() : has_prev_(false) {}

  // 传入当前 IMU（角度，单位度）和时间戳，返回是否能够计算速率
  // 若返回 true，则 yawRate 和 pitchRate 被设置为 deg/s
  bool computeRates(const SerialTask::EulerAngles &imu, const std::chrono::steady_clock::time_point &ts, float &yawRate,
                    float &pitchRate) {
    yawRate = 0.0f;
    pitchRate = 0.0f;
    if (!has_prev_) {
      prev_imu_ = imu;
      prev_ts_ = ts;
      has_prev_ = true;
      return false;
    }

    float dt = std::chrono::duration<float>(ts - prev_ts_).count();
    if (dt <= 1e-6f) {
      // 时间间隔太小，认为无法计算
      return false;
    }

    // 使用通用无状态函数计算并更新内部状态
    if (!computeRatesBetween(prev_imu_, prev_ts_, imu, ts, yawRate, pitchRate)) return false;

    prev_imu_ = imu;
    prev_ts_ = ts;
    return true;
  }

  void reset() { has_prev_ = false; }

  // 无状态的通用计算接口：传入前后两次 IMU（含时间戳）并计算角速度（deg/s）
  static bool computeRatesBetween(const SerialTask::EulerAngles &prev_imu,
                                  const std::chrono::steady_clock::time_point &prev_ts,
                                  const SerialTask::EulerAngles &curr_imu,
                                  const std::chrono::steady_clock::time_point &curr_ts, float &yawRate,
                                  float &pitchRate) {
    yawRate = 0.0f;
    pitchRate = 0.0f;
    float dt = std::chrono::duration<float>(curr_ts - prev_ts).count();
    if (dt <= 1e-6f) return false;

    auto angleDiff = [](float a, float b) {
      float d = a - b;
      while (d > 180.0f) d -= 360.0f;
      while (d < -180.0f) d += 360.0f;
      return d;
    };

    float dyaw = angleDiff(curr_imu.yaw, prev_imu.yaw);
    float dpitch = angleDiff(curr_imu.pitch, prev_imu.pitch);

    yawRate = dyaw / dt;
    pitchRate = dpitch / dt;
    return true;
  }

 private:
  SerialTask::EulerAngles prev_imu_{};
  std::chrono::steady_clock::time_point prev_ts_{};
  bool has_prev_;
};

}  // namespace Tools
