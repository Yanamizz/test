/**
 * @file    include/Tools/AimbotCommand.hpp
 * @brief   定义图像主流程与串口发送线程之间传递的云台控制命令。
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace Tools {

struct AimbotSendCommand {
  float absolute_pitch = 0.0f;
  float absolute_yaw = 0.0f;
  float offset_pitch = 0.0f;
  float offset_yaw = 0.0f;
  float pitch_velocity = 0.0f;
  float yaw_velocity = 0.0f;
  uint8_t aimbot_state = 0x00;
  std::chrono::steady_clock::time_point source_frame_ts{};
  std::chrono::steady_clock::time_point enqueue_ts{};
};

} // namespace Tools
