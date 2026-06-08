/**
 * @file    include/Tools/RuntimeStats.hpp
 * @brief   定义主循环延迟与像素尺寸统计数据及其打印辅助函数。
 *
 * RuntimeStats 保存推理、跟踪、角度计算、控制发送等阶段的耗时采样，并提供窗口
 * 统计输出。PixelSizeStats 用于记录目标框宽高分布，辅助实机距离标定和抖动排查。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

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

struct PixelSizeStats {
  std::uint64_t samples = 0;
  double width_sum = 0.0;
  double width_min = 0.0;
  double width_max = 0.0;
  double height_sum = 0.0;
  double height_min = 0.0;
  double height_max = 0.0;
  std::vector<double> width_samples;
  std::vector<double> height_samples;

  void Add(double width_px, double height_px) {
    if (width_px <= 0.0 || height_px <= 0.0) {
      return;
    }

    if (samples == 0) {
      width_min = width_px;
      width_max = width_px;
      height_min = height_px;
      height_max = height_px;
    } else {
      width_min = std::min(width_min, width_px);
      width_max = std::max(width_max, width_px);
      height_min = std::min(height_min, height_px);
      height_max = std::max(height_max, height_px);
    }

    width_sum += width_px;
    height_sum += height_px;
    width_samples.push_back(width_px);
    height_samples.push_back(height_px);
    ++samples;
  }

  double AverageWidth() const {
    return samples > 0 ? width_sum / static_cast<double>(samples) : 0.0;
  }

  double AverageHeight() const {
    return samples > 0 ? height_sum / static_cast<double>(samples) : 0.0;
  }

  double WidthMedian() const { return Percentile(width_samples, 0.5); }

  double HeightMedian() const { return Percentile(height_samples, 0.5); }

  double WidthP25() const { return Percentile(width_samples, 0.25); }

  double HeightP25() const { return Percentile(height_samples, 0.25); }

private:
  static double Percentile(const std::vector<double> &values, double q) {
    if (values.empty()) {
      return 0.0;
    }

    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    const double clamped_q = std::clamp(q, 0.0, 1.0);
    const std::size_t index = static_cast<std::size_t>(
        clamped_q * static_cast<double>(sorted.size() - 1));
    return sorted[index];
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

inline void PrintPixelSizeStats(const PixelSizeStats &s) {
  if (s.samples == 0) {
    std::cout << "[像素尺寸] 无有效样本" << std::endl;
    return;
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[像素尺寸] 样本数=" << s.samples
            << " 宽平均=" << s.AverageWidth() << " px"
            << " 宽P25=" << s.WidthP25() << " px"
            << " 宽中位=" << s.WidthMedian() << " px"
            << " 宽最小=" << s.width_min << " px"
            << " 宽最大=" << s.width_max << " px"
            << " 高平均=" << s.AverageHeight() << " px"
            << " 高P25=" << s.HeightP25() << " px"
            << " 高中位=" << s.HeightMedian() << " px"
            << " 高最小=" << s.height_min << " px"
            << " 高最大=" << s.height_max << " px" << std::endl;
}

} // namespace Tools
