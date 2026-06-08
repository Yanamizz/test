/**
 * @file    include/SerialTask/ImuBuffer.hpp
 * @brief   提供 IMU 姿态序列缓存、插值匹配与速度估计能力。
 *
 * ImuBuffer 保存串口读取线程产生的 yaw/pitch 时间序列，支持按图像帧时间戳
 * 查询最近姿态、限制最大匹配年龄，并估计角速度供控制前馈使用。它是图像
 * 时间与 IMU 时间对齐的关键缓冲层，不负责串口解析或控制命令发送。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <iterator>
#include <mutex>
#include <utility>

#include "SerialTask/SerialRead.hpp"
#include "Tools/AngleUtils.hpp"

namespace SerialTask {

class ImuBuffer {
public:
  using Clock = std::chrono::steady_clock;

  void Add(Clock::time_point timestamp, const EulerAngles &angles,
           std::chrono::milliseconds max_age) {
    std::lock_guard<std::mutex> lk(mutex_);
    entries_.emplace_back(timestamp, angles);

    while (!entries_.empty() && timestamp - entries_.front().first > max_age) {
      entries_.pop_front();
    }
  }

  bool MatchForFrame(Clock::time_point frame_ts, EulerAngles *matched_imu,
                     std::chrono::milliseconds max_match_age =
                         std::chrono::milliseconds::max()) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (entries_.empty()) {
      return false;
    }

    auto upper_it = std::lower_bound(
        entries_.begin(), entries_.end(), frame_ts,
        [](const auto &entry, const auto &ts) { return entry.first < ts; });

    if (entries_.size() == 1 || upper_it == entries_.begin()) {
      if (!IsWithinMaxAge(entries_.front().first, frame_ts, max_match_age)) {
        return false;
      }
      *matched_imu = entries_.front().second;
      return true;
    }
    if (upper_it == entries_.end()) {
      if (!IsWithinMaxAge(entries_.back().first, frame_ts, max_match_age)) {
        return false;
      }
      *matched_imu = entries_.back().second;
      return true;
    }

    const auto lower_it = std::prev(upper_it);
    const bool lower_within_age =
        IsWithinMaxAge(lower_it->first, frame_ts, max_match_age);
    const bool upper_within_age =
        IsWithinMaxAge(upper_it->first, frame_ts, max_match_age);
    if (!lower_within_age && !upper_within_age) {
      return false;
    }
    if (!lower_within_age || !upper_within_age) {
      *matched_imu = lower_within_age ? lower_it->second : upper_it->second;
      return true;
    }

    const auto span_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             upper_it->first - lower_it->first)
                             .count();
    if (span_ns <= 0) {
      *matched_imu = upper_it->second;
      return true;
    }

    const auto elapsed_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(frame_ts -
                                                             lower_it->first)
            .count();
    const float alpha =
        static_cast<float>(elapsed_ns) / static_cast<float>(span_ns);
    *matched_imu =
        InterpolateEulerAngles(lower_it->second, upper_it->second, alpha);
    return true;
  }

  bool GetLatest(EulerAngles *latest_imu) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (entries_.empty()) {
      return false;
    }

    *latest_imu = entries_.back().second;
    return true;
  }

  bool GetLatestVelocity(float *pitch_velocity_deg_per_sec,
                         float *yaw_velocity_deg_per_sec) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (entries_.size() < 2) {
      return false;
    }

    const auto &prev = entries_[entries_.size() - 2];
    const auto &curr = entries_.back();
    const auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           curr.first - prev.first)
                           .count();
    if (dt_ns <= 0) {
      return false;
    }

    const float dt_sec = static_cast<float>(dt_ns) * 1e-9f;
    *pitch_velocity_deg_per_sec =
        Tools::NormalizeDeltaDeg(curr.second.pitch - prev.second.pitch) / dt_sec;
    *yaw_velocity_deg_per_sec =
        Tools::NormalizeDeltaDeg(curr.second.yaw - prev.second.yaw) / dt_sec;
    return true;
  }

private:
  static bool IsWithinMaxAge(Clock::time_point sample_ts,
                             Clock::time_point frame_ts,
                             std::chrono::milliseconds max_match_age) {
    if (max_match_age == std::chrono::milliseconds::max()) {
      return true;
    }

    const auto delta = sample_ts > frame_ts ? sample_ts - frame_ts
                                            : frame_ts - sample_ts;
    return delta <= max_match_age;
  }

  static float InterpolateAngleDeg(float from_deg, float to_deg, float alpha) {
    if (alpha < 0.0f) {
      alpha = 0.0f;
    }
    if (alpha > 1.0f) {
      alpha = 1.0f;
    }
    return from_deg + Tools::NormalizeDeltaDeg(to_deg - from_deg) * alpha;
  }

  static EulerAngles InterpolateEulerAngles(const EulerAngles &lower,
                                            const EulerAngles &upper,
                                            float alpha) {
    EulerAngles result{};
    result.roll = InterpolateAngleDeg(lower.roll, upper.roll, alpha);
    result.pitch = InterpolateAngleDeg(lower.pitch, upper.pitch, alpha);
    result.yaw = InterpolateAngleDeg(lower.yaw, upper.yaw, alpha);
    return result;
  }

  mutable std::mutex mutex_;
  std::deque<std::pair<Clock::time_point, EulerAngles>> entries_;
};

} // namespace SerialTask
