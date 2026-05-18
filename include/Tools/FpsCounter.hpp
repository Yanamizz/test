/**
 * @file    include/Tools/FpsCounter.hpp
 * @brief   提供固定时间窗口下的实时帧率统计能力。
 */

#pragma once

#include <chrono>

/**
 * @brief 统计帧率，在固定时间窗口更新一次显示值。
 * @brief FPS 计数器模块
 * @brief 说明：每经过一段时间（默认 500 ms）更新一次 FPS 值。
 * @brief 用法：在主循环每帧调用 `tick()`，并通过 `get()` 获取当前 FPS。
 */
class FPSCounter {
 public:
  explicit FPSCounter(int interval_ms = 500)
      : interval_ms_ms_(interval_ms), frame_count_(0), fps_(0.0), last_time_(std::chrono::steady_clock::now()) {}

  // 每帧调用一次
  void tick() {
    ++frame_count_;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count();
    if (elapsed >= interval_ms_ms_) {
      fps_ = static_cast<double>(frame_count_) * 1000.0 / static_cast<double>(elapsed);
      frame_count_ = 0;
      last_time_ = now;
    }
  }

  double get() const { return fps_; }

 private:
  int interval_ms_ms_;
  int frame_count_;
  double fps_;
  std::chrono::steady_clock::time_point last_time_;
};
