/**
 * @file include/SerialTask/SerialSend.hpp
 *
 * @brief 串口 发送 云台需要转动的欧拉角（Pitch, Yaw）数据
 *
 *
 */
#pragma once

#include "Common.hpp"
#include "SerialRead.hpp"
#include <serial/serial.h>
#include <thread>
#include <chrono>

#define RAD_TO_DEG 57.29577951308232  // 180 / PI

namespace SerialTask {
// 将度数归一化到 [-180, 180]
inline float NormalizeDegTo180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

/**
 * @brief 发送目标侧欧拉角帧（Pitch, Yaw）到串口
 * @param serial_port 已配置好的串口对象
 * @param pitch_relative_angle 目标侧 Pitch 相对角度（单位：度）
 * @param yaw_relative_angle 目标侧 Yaw 相对角度（单位：度）
 */
SerialTask::EulerAngles GetAnglesNow(serial::Serial& serial_port);
void SendAimbotFrame(serial::Serial& serial_port, float pitch_relative_angle, float yaw_relative_angle);

// Inline overload defined below; remove duplicate non-inline definition.

// 重载：使用已知的当前角度发送，不再从串口读取（适用于接收线程在外部运行时）
inline void SerialSend(serial::Serial& serial_port, float absolute_pitch, float absolute_yaw) {
  float pitch_relative_angle = (absolute_pitch);
  float yaw_relative_angle = (absolute_yaw);

  pitch_relative_angle = NormalizeDegTo180(pitch_relative_angle);
  yaw_relative_angle = NormalizeDegTo180(yaw_relative_angle);

  // 在转换为弧度前保存度值以便打印

  pitch_relative_angle = pitch_relative_angle / RAD_TO_DEG;
  yaw_relative_angle = yaw_relative_angle / RAD_TO_DEG;

  SerialTask::SendAimbotFrame(serial_port, pitch_relative_angle, yaw_relative_angle);
}

inline void SendAimbotFrame(serial::Serial& serial_port, float pitch_relative_angle, float yaw_relative_angle) {
  AimbotFrame_SCM_t aimbot_frame;
  aimbot_frame._SOF = 0x55;  // 包头
  aimbot_frame.ID = 0x02;    // 发送 ID
  aimbot_frame.PitchRelativeAngle = pitch_relative_angle;
  aimbot_frame.YawRelativeAngle = yaw_relative_angle;
  aimbot_frame._EOF = 0xFF;  // 包尾
  serial_port.write(reinterpret_cast<uint8_t*>(&aimbot_frame), sizeof(AimbotFrame_SCM_t));
}

}  // namespace SerialTask