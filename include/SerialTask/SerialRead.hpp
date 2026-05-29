/**
 * @file    include/SerialTask/SerialRead.hpp
 * @brief   提供 IMU 串口帧解析、四元数转欧拉角与姿态读取能力。
 */

#pragma once

#include "SerialTask/Common.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream> // 提供 std::cerr
#include <serial/serial.h>
#include <vector> // 提供 std::vector

namespace SerialTask {

struct EulerAngles {
  float roll;  // 绕X轴旋转角度（单位：度）
  float pitch; // 绕Y轴旋转角度（单位：度）
  float yaw;   // 绕Z轴旋转角度（单位：度）
};

inline bool TryToEulerAngles(const GimbalImuFrame_SCM_t &frame,
                             EulerAngles &angles) {
  float q0 = frame.q0; // w
  float q1 = frame.q1; // x
  float q2 = frame.q2; // y
  float q3 = frame.q3; // z
  constexpr float kRadToDeg = 57.29577951308232f;

  const float norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (!std::isfinite(norm_sq) || norm_sq < 1e-6f) {
    return false;
  }

  const float inv_norm = 1.0f / std::sqrt(norm_sq);
  q0 *= inv_norm;
  q1 *= inv_norm;
  q2 *= inv_norm;
  q3 *= inv_norm;

  const float sin_pitch =
      std::clamp(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
  angles.roll = std::atan2(2.0f * (q0 * q1 + q2 * q3),
                           1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
                kRadToDeg;
  angles.pitch = std::asin(sin_pitch) * kRadToDeg;
  angles.yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                          1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
               kRadToDeg;
  return true;
}

inline bool ParseLatestIMUFrame(const uint8_t *data, size_t data_size,
                                GimbalImuFrame_SCM_t &out_frame) {
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

  if (data_size < kFrameSize)
    return false;

  bool found = false;
  GimbalImuFrame_SCM_t latest_frame{};
  for (size_t i = 0; i + kFrameSize <= data_size; ++i) {
    if (data[i] == 0x55) {
      if (data[i + kIdOffset] == IMU_DATA_SEND_ID &&
          data[i + kEofOffset] == 0xFF) {
        GimbalImuFrame_SCM_t tmp{};
        tmp._SOF = 0x55;
        tmp.ID = IMU_DATA_SEND_ID;

        // 发送方新增字段不参与业务逻辑，直接跳过，仅提取四元数。
        std::memcpy(&tmp.q0, &data[i + kQ0Offset], sizeof(float));
        std::memcpy(&tmp.q1, &data[i + kQ1Offset], sizeof(float));
        std::memcpy(&tmp.q2, &data[i + kQ2Offset], sizeof(float));
        std::memcpy(&tmp.q3, &data[i + kQ3Offset], sizeof(float));

        tmp._EOF = 0xFF;

        latest_frame = tmp;
        found = true;
        i += kFrameSize - 1;
      }
    }
  }
  if (!found)
    return false;
  out_frame = latest_frame;
  return true;
}

inline size_t ReadAvailableIMUBytes(serial::Serial &serial_port,
                                    std::vector<uint8_t> &buffer) {
  constexpr size_t kFrameSize = sizeof(GimbalImuFrame_SCM_t);

  if (!serial_port.isOpen()) {
    std::cerr << "[ERROR] 串口未打开，无法读取数据。" << std::endl;
    return 0;
  }

  size_t available_bytes = serial_port.available();
  if (available_bytes < kFrameSize) {
    return 0;
  }

  buffer.resize(available_bytes);
  return serial_port.read(buffer.data(), available_bytes);
}

/**
 * @brief 解析串口缓冲区并返回最新的 GimbalImuFrame_SCM_t，
 *        不执行四元数到欧拉角的转换。
 * @param serial_port 已打开的串口
 * @param out_frame 输出的最新帧（若找到返回 true）
 */
inline bool ReadIMUFrame(serial::Serial &serial_port,
                         GimbalImuFrame_SCM_t &out_frame) {
  thread_local std::vector<uint8_t> buffer;
  const size_t read_count = ReadAvailableIMUBytes(serial_port, buffer);
  return ParseLatestIMUFrame(buffer.data(), read_count, out_frame);
}

/**
 * @brief 从串口读取字节流，解析 GimbalImuFrame_SCM_t 数据，
 *        并将四元数转换为欧拉角。
 *        策略：读取缓冲区所有数据，寻找并使用最后一个有效帧，以确保数据最新。
 * @param serial_port 已配置好的串口对象
 * @param angles 输出的欧拉角结构体（Roll, Pitch, Yaw）
 */
inline bool ReadIMUData(serial::Serial &serial_port, EulerAngles &angles) {
  // 这里复用 ReadIMUFrame：先解析出最新帧，再转换为欧拉角。
  GimbalImuFrame_SCM_t latest_frame;
  if (!ReadIMUFrame(serial_port, latest_frame))
    return false;

  return TryToEulerAngles(latest_frame, angles);
}
} // namespace SerialTask
