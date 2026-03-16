/**
 * @file    include\SerialTask\SerialRead.hpp
 * @brief   串口 IMU 数据读取与四元数转欧拉角
 * @brief   输入串口字节流，输出目标侧欧拉角帧（Roll，Pitch, Yaw）
 *
 * @date    2026-01-23
 */

#pragma once

#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/Common.hpp"
#include <serial/serial.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>  // For std::cerr
#include <vector>    // For std::vector
#include <cstring>
#include <Eigen/Geometry>

namespace SerialTask {

typedef struct {
  float roll;   // 绕X轴旋转角度（单位：度）
  float pitch;  // 绕Y轴旋转角度（单位：度）
  float yaw;    // 绕Z轴旋转角度（单位：度）
} EulerAngles;

inline bool ReadIMUFrame(serial::Serial& serial_port, GimbalImuFrame_SCM_t& out_frame);

/**
 * @brief 从串口读取字节流，解析 GimbalImuFrame_SCM_t 数据，并将四元数转换为欧拉角
 *        策略：读取缓冲区所有数据，寻找并使用最后一个有效帧，以确保数据最新。
 * @param serial_port 已配置好的串口对象
 * @param angles 输出的欧拉角结构体（Roll, Pitch, Yaw）
 */
inline bool ReadIMUData(serial::Serial& serial_port, EulerAngles& angles) {
  // 现在 ReadIMUData 重用 ReadIMUFrame：先解析出最新帧再转换为欧拉角
  GimbalImuFrame_SCM_t latest_frame;
  if (!ReadIMUFrame(serial_port, latest_frame)) return false;

  float q0 = latest_frame.q0;  // w
  float q1 = latest_frame.q1;  // x
  float q2 = latest_frame.q2;  // y
  float q3 = latest_frame.q3;  // z

  angles.roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);
  angles.pitch = -std::asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0f / M_PI) - 180.0f;
  angles.yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI) + 180.0f;

  return true;
}

/**
 * @brief 解析串口缓冲区并返回最新的 GimbalImuFrame_SCM_t（不做四元数->欧拉转换）
 * @param serial_port 已打开的串口
 * @param out_frame 输出的最新帧（若找到返回 true）
 */
inline bool ReadIMUFrame(serial::Serial& serial_port, GimbalImuFrame_SCM_t& out_frame) {
  constexpr size_t kFrameSize = sizeof(GimbalImuFrame_SCM_t);
  constexpr size_t kIdOffset = 1;
  constexpr size_t kTimeStampOffset = 2;
  constexpr size_t kQ0Offset = kTimeStampOffset + sizeof(uint32_t);
  constexpr size_t kQ1Offset = kQ0Offset + sizeof(float);
  constexpr size_t kQ2Offset = kQ1Offset + sizeof(float);
  constexpr size_t kQ3Offset = kQ2Offset + sizeof(float);
  constexpr size_t kRobotIdOffset = kQ3Offset + sizeof(float);
  constexpr size_t kAimModeOffset = kRobotIdOffset + sizeof(uint8_t);
  constexpr size_t kEofOffset = kAimModeOffset + sizeof(uint8_t);

  if (!serial_port.isOpen()) {
    std::cerr << "[ERROR] 串口未打开，无法读取数据。" << std::endl;
    return false;
  }

  size_t available_bytes = serial_port.available();
  if (available_bytes < kFrameSize) {
    return false;
  }

  std::vector<uint8_t> buffer(available_bytes);
  size_t read_count = serial_port.read(buffer.data(), available_bytes);
  if (read_count < kFrameSize) return false;

  bool found = false;
  GimbalImuFrame_SCM_t latest_frame{};
  for (size_t i = 0; i + kFrameSize <= read_count; ++i) {
    if (buffer[i] == 0x55) {
      if (buffer[i + kIdOffset] == IMU_DATA_SEND_ID && buffer[i + kEofOffset] == 0xFF) {
        GimbalImuFrame_SCM_t tmp{};
        tmp._SOF = 0x55;
        tmp.ID = IMU_DATA_SEND_ID;

        // 发送方新增字段不参与业务逻辑，直接跳过，仅提取四元数。
        std::memcpy(&tmp.q0, &buffer[i + kQ0Offset], sizeof(float));
        std::memcpy(&tmp.q1, &buffer[i + kQ1Offset], sizeof(float));
        std::memcpy(&tmp.q2, &buffer[i + kQ2Offset], sizeof(float));
        std::memcpy(&tmp.q3, &buffer[i + kQ3Offset], sizeof(float));

        tmp.TimeStamp = 0;
        tmp.robot_id = 0;
        tmp.aim_mode = 0;
        tmp._EOF = 0xFF;

        latest_frame = tmp;
        found = true;
        i += kFrameSize - 1;
      }
    }
  }
  if (!found) return false;
  out_frame = latest_frame;
  return true;
}
}  // namespace SerialTask