/**
 * @file include/SerialTask/SerialSend.hpp
 *
 * @brief 串口 发送 云台需要转动的欧拉角（Pitch, Yaw）数据
 *
 *
 */
#pragma once

#include "SerialTask/Common.hpp"
#include "SerialTask/SerialRead.hpp"
#include "ImageRecognize/AngleCalculate.hpp"
#include <serial/serial.h>
#include <thread>
#include <chrono>

#define RAD_TO_DEG 57.29577951308232  // 180 / PI

namespace SerialTask {
/**
 * @brief 发送目标侧欧拉角帧（Pitch, Yaw）到串口
 * @param serial_port 已配置好的串口对象
 * @param pitch_relative_angle 目标侧 Pitch 相对角度（单位：度）
 * @param yaw_relative_angle 目标侧 Yaw 相对角度（单位：度）
 */
SerialTask::EulerAngles GetAnglesNow(serial::Serial& serial_port);
void SendAimbotFrame(serial::Serial& serial_port, float pitch_relative_angle, float yaw_relative_angle);

void SerialSend(serial::Serial& serial_port, float pitch_offset, float yaw_offset) {
  EulerAngles angles_now = SerialTask::GetAnglesNow(serial_port);

  float pitch_relative_angle = (angles_now.pitch + pitch_offset);
  float yaw_relative_angle = (angles_now.yaw + yaw_offset);

  if (pitch_relative_angle > 180.0f) {
    pitch_relative_angle -= 360.0f;
  } else if (pitch_relative_angle < -180.0f) {
    pitch_relative_angle += 360.0f;
  }

  if (yaw_relative_angle > 180.0f) {
    yaw_relative_angle -= 360.0f;
  } else if (yaw_relative_angle < -180.0f) {
    yaw_relative_angle += 360.0f;
  }

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
  std::cout << "已发送目标侧欧拉角帧: Pitch = " << pitch_relative_angle * RAD_TO_DEG
            << ", Yaw = " << yaw_relative_angle * RAD_TO_DEG << std::endl;
}

SerialTask::EulerAngles GetAnglesNow(serial::Serial& serial_port) {
  SerialTask::EulerAngles angles;
  if (SerialTask::ReadIMUData(serial_port, angles)) {
    return angles;
  } else {
    throw std::runtime_error("无法读取当前欧拉角数据");
  }
}
}  // namespace SerialTask