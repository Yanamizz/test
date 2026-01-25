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

namespace SerialTask {

typedef struct {
  float roll;   // 绕X轴旋转角度（单位：度）
  float pitch;  // 绕Y轴旋转角度（单位：度）
  float yaw;    // 绕Z轴旋转角度（单位：度）
} EulerAngles;

/**
 * @brief 从串口读取字节流，解析 GimbalImuFrame_SCM_t 数据，并将四元数转换为欧拉角
 *        策略：读取缓冲区所有数据，寻找并使用最后一个有效帧，以确保数据最新。
 * @param serial_port 已配置好的串口对象
 * @param angles 输出的欧拉角结构体（Roll, Pitch, Yaw）
 */
inline bool ReadIMUData(serial::Serial& serial_port, EulerAngles& angles) {
  size_t frame_size = sizeof(GimbalImuFrame_SCM_t);

  // 检查串口是否已打开
  if (!serial_port.isOpen()) {
    std::cerr << "[ERROR] 串口未打开，无法读取数据。" << std::endl;
    return false;
  }

  // 1. 检查缓冲区是否有至少一帧的数据
  size_t available_bytes = serial_port.available();
  if (available_bytes < frame_size) {
    // 只有在调试时才打开此输出，避免刷屏
    // std::cerr << "[DEBUG] 数据不足: 可用 " << available_bytes << " 字节" << std::endl;
    return false;
  }

  // 2. 读取缓冲区中的所有数据
  std::vector<uint8_t> buffer(available_bytes);
  size_t read_count = serial_port.read(buffer.data(), available_bytes);

  if (read_count < frame_size) {
    return false;
  }

  // 3. 在读取的数据中寻找最后一个有效的帧
  bool found_valid_frame = false;
  GimbalImuFrame_SCM_t latest_frame;

  // 遍历缓冲区寻找有效帧
  // 注意：我们从前向后扫描，每次找到有效帧 update 'latest_frame'，
  // 这样循环结束时 'latest_frame' 就是最新的那一帧。
  for (size_t i = 0; i <= read_count - frame_size; ++i) {
    // 检查包头 (SOF)
    if (buffer[i] == 0x55) {
      // 尝试解析
      GimbalImuFrame_SCM_t temp_frame;
      memcpy(&temp_frame, &buffer[i], frame_size);

      // 校验包尾 (EOF) 和 ID
      if (temp_frame._EOF == 0xFF && temp_frame.ID == IMU_DATA_SEND_ID) {
        latest_frame = temp_frame;
        found_valid_frame = true;

        // 优化：跳过当前帧长度，继续寻找更新的帧
        // (i 在循环末尾会 ++，所以这里加 frame_size - 1)
        i += (frame_size - 1);
      }
    }
  }

  if (found_valid_frame) {
    // 提取四元数
    float q0 = latest_frame.q0;  // w
    float q1 = latest_frame.q1;  // x
    float q2 = latest_frame.q2;  // y
    float q3 = latest_frame.q3;  // z

    // 四元数转欧拉角 (转为角度值)
    angles.roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);  // Roll
    angles.pitch = -std::asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0f / M_PI);                                    // Pitch
    angles.yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);   // Yaw

    // 输出调试信息
    std::cerr << "[DEBUG] 最新欧拉角: Roll=" << angles.roll << ", Pitch=" << angles.pitch << ", Yaw=" << angles.yaw
              << std::endl;

    return true;  // 成功解析并转换
  }

  std::cerr << "[DEBUG] 在 " << read_count << " 字节中未找到有效帧。" << std::endl;
  return false;
}
}  // namespace SerialTask
