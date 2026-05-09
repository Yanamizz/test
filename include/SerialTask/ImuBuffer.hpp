#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <iterator>
#include <mutex>
#include <utility>

#include "SerialTask/SerialRead.hpp"

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

  bool MatchForFrame(Clock::time_point frame_ts,
                     EulerAngles *matched_imu) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (entries_.empty()) {
      return false;
    }

    auto upper_it = std::lower_bound(
        entries_.begin(), entries_.end(), frame_ts,
        [](const auto &entry, const auto &ts) { return entry.first < ts; });

    if (entries_.size() == 1 || upper_it == entries_.begin()) {
      *matched_imu = entries_.front().second;
      return true;
    }
    if (upper_it == entries_.end()) {
      *matched_imu = entries_.back().second;
      return true;
    }

    const auto lower_it = std::prev(upper_it);
    const auto span_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             upper_it->first - lower_it->first)
                             .count();
    if (span_ns <= 0) {
      *matched_imu = upper_it->second;
      return true;
    }

    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                frame_ts - lower_it->first)
                                .count();
    const float alpha =
        static_cast<float>(elapsed_ns) / static_cast<float>(span_ns);
    *matched_imu = InterpolateEulerAngles(lower_it->second, upper_it->second,
                                          alpha);
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

private:
  static float NormalizeDeltaDeg(float delta) {
    while (delta > 180.0f) {
      delta -= 360.0f;
    }
    while (delta < -180.0f) {
      delta += 360.0f;
    }
    return delta;
  }

  static float InterpolateAngleDeg(float from_deg, float to_deg, float alpha) {
    if (alpha < 0.0f) {
      alpha = 0.0f;
    }
    if (alpha > 1.0f) {
      alpha = 1.0f;
    }
    return from_deg + NormalizeDeltaDeg(to_deg - from_deg) * alpha;
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
