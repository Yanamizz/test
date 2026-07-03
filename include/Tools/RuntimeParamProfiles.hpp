/**
 * @file    include/Tools/RuntimeParamProfiles.hpp
 * @brief   将 RuntimeParams 中常一起使用的参数组装成小配置快照。
 *
 * 该文件把默认参数中属于同一功能面的字段整理成更小的只读 profile，例如
 * 阶段切换、扫描、时序容差或保存策略。这样调用侧可以依赖语义明确的配置块，
 * 减少到处直接访问 RuntimeParams 大结构带来的耦合。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>

#include "Tools/RuntimeParams.hpp"

namespace Tools {

struct ControlRuntimeConfig {
  std::chrono::milliseconds imu_buffer_max_age{0};
  double dt_max_sec = 0.0;
  float minimum_angle_deg = 0.0f;
  float max_send_delta_deg = 0.0f;
  float pitch_abs_limit = 0.0f;
  float stage12_pitch_micro_deadband_deg = 0.0f;
  float yaw_velocity_feedforward_error_threshold_deg = 0.0f;
  float pitch_velocity_feedforward_error_threshold_deg = 0.0f;
  float yaw_error_feedforward_gain_deg_per_sec_per_deg = 0.0f;
  float pitch_error_feedforward_gain_deg_per_sec_per_deg = 0.0f;
  double yaw_velocity_abs_limit_deg_per_sec = 0.0;
  double pitch_velocity_abs_limit_deg_per_sec = 0.0;
};

struct OutputRuntimeConfig {
  std::uint64_t display_every_n_frames = 1;
  std::uint64_t gui_poll_every_n_frames = 1;
  std::uint64_t latency_print_interval_frames = 1;
};

inline ControlRuntimeConfig MakeControlRuntimeConfig() {
  const auto &params = Params();
  return ControlRuntimeConfig{
      std::chrono::milliseconds(params.imu_buffer_max_age_ms),
      params.dt_max_sec,
      params.minimum_angle_deg,
      params.max_send_delta_deg,
      params.pitch_abs_limit,
      std::max(0.0f, params.stage12_pitch_micro_deadband_deg),
      std::max(0.0f, params.yaw_velocity_feedforward_error_threshold_deg),
      std::max(0.0f, params.pitch_velocity_feedforward_error_threshold_deg),
      std::max(0.0f, params.yaw_error_feedforward_gain_deg_per_sec_per_deg),
      std::max(0.0f, params.pitch_error_feedforward_gain_deg_per_sec_per_deg),
      std::max(0.0, params.angle_velocity_yaw_abs_limit_deg_per_sec),
      std::max(0.0, params.angle_velocity_pitch_abs_limit_deg_per_sec)};
}

inline OutputRuntimeConfig MakeOutputRuntimeConfig() {
  const auto &params = Params();
  return OutputRuntimeConfig{
      static_cast<std::uint64_t>(std::max(1, params.display_every_n_frames)),
      static_cast<std::uint64_t>(std::max(1, params.gui_poll_every_n_frames)),
      static_cast<std::uint64_t>(
          std::max(1, params.latency_print_interval_frames))};
}

} // namespace Tools
