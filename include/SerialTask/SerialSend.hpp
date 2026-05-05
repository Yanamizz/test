/**
 * @file include/SerialTask/SerialSend.hpp
 *
 * @brief 串口 发送 云台需要转动的欧拉角（Pitch, Yaw）和角速度数据
 *
 *
 */
#pragma once

#include "Common.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <serial/serial.h>

inline constexpr float kDegToRad = 0.01745329251994329577f;

namespace SerialTask {
// 将度数归一化到 [-180, 180]
inline float NormalizeDegTo180(float d) {
  while (d > 180.0f)
    d -= 360.0f;
  while (d < -180.0f)
    d += 360.0f;
  return d;
}

inline float NormalizeDegTo360(float d) {
  while (d > 0.0f)
    d -= 360.0f;
  while (d < -360.0f)
    d += 360.0f;
  return d;
}

inline void SendAimbotFrame(serial::Serial &serial_port,
                            float pitch_relative_angle,
                            float yaw_relative_angle, float pitch_offset,
                            float yaw_offset, float pitch_velocity,
                            float yaw_velocity, uint8_t AimbotState);

// 重载：使用已知的当前角度发送，不再从串口读取（适用于接收线程在外部运行时）
inline void SerialSend(serial::Serial &serial_port, float absolute_pitch,
                       float absolute_yaw, float pitch_offset, float yaw_offset,
                       float pitch_velocity, float yaw_velocity,
                       uint8_t AimbotState) {
  float pitch_relative_angle = absolute_pitch;
  float yaw_relative_angle = absolute_yaw;

  pitch_relative_angle = NormalizeDegTo360(pitch_relative_angle);
  yaw_relative_angle = NormalizeDegTo180(yaw_relative_angle);

  // 在转换为弧度前保存度值以便打印
  pitch_relative_angle *= kDegToRad;
  yaw_relative_angle *= kDegToRad;
  pitch_velocity *= kDegToRad;
  yaw_velocity *= kDegToRad;

  SerialTask::SendAimbotFrame(serial_port, pitch_relative_angle,
                              yaw_relative_angle, pitch_offset, yaw_offset,
                              pitch_velocity, yaw_velocity, AimbotState);
}

inline void SendAimbotFrame(serial::Serial &serial_port,
                            float pitch_relative_angle,
                            float yaw_relative_angle, float pitch_offset,
                            float yaw_offset, float pitch_velocity,
                            float yaw_velocity, uint8_t AimbotState) {
  AimbotFrame_SCM_t aimbot_frame;
  aimbot_frame._SOF = 0x55;               // 包头
  aimbot_frame.ID = 0x02;                 // 发送 ID
  aimbot_frame.AimbotState = AimbotState; ///< 0x00 无目标，0x01 有目标
  aimbot_frame.AimbotTarget = 0x00;       // 目前未使用，默认为 0
  aimbot_frame.PitchRelativeAngle = pitch_relative_angle;
  aimbot_frame.YawRelativeAngle = yaw_relative_angle;
  aimbot_frame.PitchOffset = pitch_offset;
  aimbot_frame.YawOffset = yaw_offset;
  aimbot_frame.PitchVelocity = pitch_velocity;
  aimbot_frame.YawVelocity = yaw_velocity;
  aimbot_frame.SystemTimer = 0.0f; ///< 时间戳
  aimbot_frame._EOF = 0xFF;        // 包尾
  serial_port.write(reinterpret_cast<uint8_t *>(&aimbot_frame),
                    sizeof(AimbotFrame_SCM_t));
}

} // namespace SerialTask