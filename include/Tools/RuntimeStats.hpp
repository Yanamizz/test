/**
 * @file    include/Tools/RuntimeStats.hpp
 * @brief   定义主循环延迟与像素高度统计数据及其打印辅助函数。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace Tools {

struct LatencyBucket {
  std::uint64_t samples = 0;
  std::uint64_t total_ns = 0;

  void AddDuration(std::uint64_t duration_ns) {
    total_ns += duration_ns;
    ++samples;
  }
};

struct LatencyStats {
  std::uint64_t frames = 0;
  LatencyBucket infer_ns;
  LatencyBucket imu_match_ns;
  LatencyBucket select_box_ns;
  LatencyBucket angle_calc_ns;
  LatencyBucket control_calc_ns;
  LatencyBucket render_ns;
  LatencyBucket loop_ns;
  LatencyBucket capture_to_snapshot_ns;
  LatencyBucket submit_wait_ns;
  LatencyBucket submit_prepare_ns;
  LatencyBucket submit_stage3_preprocess_ns;
  LatencyBucket submit_async_ns;
  LatencyBucket capture_to_submit_ns;
  LatencyBucket capture_to_result_ns;
  LatencyBucket result_to_control_ns;
  LatencyBucket queue_to_serial_ns;
  LatencyBucket capture_to_serial_ns;

  void Add(LatencyBucket &bucket,
           const std::chrono::steady_clock::time_point &t0,
           const std::chrono::steady_clock::time_point &t1) {
    if (t1 <= t0) {
      return;
    }
    const auto duration_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    bucket.AddDuration(duration_ns);
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
  const auto has_any_samples = [&](const LatencyBucket &bucket) {
    return bucket.samples > 0;
  };
  if (s.frames == 0 && !has_any_samples(s.infer_ns) &&
      !has_any_samples(s.capture_to_serial_ns)) {
    return;
  }

  auto avg_ms = [&](const LatencyBucket &bucket) {
    if (bucket.samples == 0) {
      return 0.0;
    }
    return static_cast<double>(bucket.total_ns) /
           static_cast<double>(bucket.samples) / 1e6;
  };
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[延迟][" << tag << "] 循环帧数=" << s.frames << " | 平均毫秒"
            << " 推理=" << avg_ms(s.infer_ns) << " 渲染=" << avg_ms(s.render_ns)
            << " 循环=" << avg_ms(s.loop_ns)
            << " 采集到取帧=" << avg_ms(s.capture_to_snapshot_ns)
            << " stage3原图增强=" << avg_ms(s.submit_stage3_preprocess_ns)
            << " async提交=" << avg_ms(s.submit_async_ns)
            << " 采集到提交=" << avg_ms(s.capture_to_submit_ns)
            << " 采集到结果=" << avg_ms(s.capture_to_result_ns)
            << " 结果到控制=" << avg_ms(s.result_to_control_ns) << std::endl;
}

inline void PrintSerialLatencyStats(const LatencyStats &s, const char *tag) {
  const auto has_serial_samples =
      s.queue_to_serial_ns.samples > 0 || s.capture_to_serial_ns.samples > 0;
  if (s.frames == 0 && !has_serial_samples) {
    return;
  }

  auto avg_ms = [&](const LatencyBucket &bucket) {
    if (bucket.samples == 0) {
      return 0.0;
    }
    return static_cast<double>(bucket.total_ns) /
           static_cast<double>(bucket.samples) / 1e6;
  };

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[延迟][" << tag << "] 发送帧数=" << s.frames << " | 平均毫秒"
            << " 入队到串口发送=" << avg_ms(s.queue_to_serial_ns)
            << " 采集到串口发送=" << avg_ms(s.capture_to_serial_ns)
            << std::endl;
}

inline void PrintPixelHeightStats(const PixelHeightStats &s) {
  if (s.samples == 0) {
    std::cout << "[像素高度] 无有效样本" << std::endl;
    return;
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[像素高度] 样本数=" << s.samples << " 平均=" << s.Average()
            << " px"
            << " 最小=" << s.min << " px"
            << " 最大=" << s.max << " px" << std::endl;
}

} // namespace Tools
