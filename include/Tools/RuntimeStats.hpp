#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace Tools {

struct LatencyStats {
  std::uint64_t frames = 0;
  std::uint64_t infer_ns = 0;
  std::uint64_t imu_match_ns = 0;
  std::uint64_t select_box_ns = 0;
  std::uint64_t motion_predict_ns = 0;
  std::uint64_t angle_calc_ns = 0;
  std::uint64_t control_calc_ns = 0;
  std::uint64_t render_ns = 0;
  std::uint64_t loop_ns = 0;

  void Add(std::uint64_t &bucket,
           const std::chrono::steady_clock::time_point &t0,
           const std::chrono::steady_clock::time_point &t1) {
    bucket += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  }

  void AddFrame() { ++frames; }
};

struct PixelHeightStats {
  std::uint64_t samples = 0;
  double sum = 0.0;
  double min = 0.0;
  double max = 0.0;

  void Add(double height_px) {
    if (height_px <= 0.0) {
      return;
    }

    if (samples == 0) {
      min = height_px;
      max = height_px;
    } else {
      min = std::min(min, height_px);
      max = std::max(max, height_px);
    }

    sum += height_px;
    ++samples;
  }

  double Average() const {
    return samples > 0 ? sum / static_cast<double>(samples) : 0.0;
  }
};

inline void PrintLatencyStats(const LatencyStats &s, const char *tag) {
  if (s.frames == 0) {
    return;
  }

  const double inv = 1.0 / static_cast<double>(s.frames);
  auto ns2ms = [&](std::uint64_t ns) {
    return static_cast<double>(ns) * inv / 1e6;
  };
  auto ns2us = [&](std::uint64_t ns) {
    return static_cast<double>(ns) * inv / 1e3;
  };

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[延迟][" << tag << "] 帧数=" << s.frames << " 平均毫秒"
            << " 推理=" << ns2ms(s.infer_ns)
            << " IMU匹配=" << ns2ms(s.imu_match_ns)
            << " 选框=" << ns2ms(s.select_box_ns)
            << " 运动预测=" << ns2ms(s.motion_predict_ns)
            << " 角度=" << ns2ms(s.angle_calc_ns)
            << " 控制=" << ns2ms(s.control_calc_ns)
            << " 渲染=" << ns2ms(s.render_ns) << " 循环=" << ns2ms(s.loop_ns)
            << " | 微秒 IMU匹配=" << ns2us(s.imu_match_ns)
            << " 选框=" << ns2us(s.select_box_ns)
            << " 运动预测=" << ns2us(s.motion_predict_ns)
            << " 控制=" << ns2us(s.control_calc_ns) << std::endl;
}

inline void PrintPixelHeightStats(const PixelHeightStats &s) {
  if (s.samples == 0) {
    std::cout << "[像素高度] 无有效样本" << std::endl;
    return;
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[像素高度] 样本数=" << s.samples
            << " 平均=" << s.Average() << " px"
            << " 最小=" << s.min << " px"
            << " 最大=" << s.max << " px" << std::endl;
}

} // namespace Tools
