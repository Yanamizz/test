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

namespace SerialTask {

typedef struct {
  float roll;   // 绕X轴旋转角度（单位：度）
  float pitch;  // 绕Y轴旋转角度（单位：度）
  float yaw;    // 绕Z轴旋转角度（单位：度）
} EulerAngles;

/**
 * @brief 从串口读取字节流，解析 GimbalImuFrame_SCM_t 数据，并将四元数转换为欧拉角
 * @param serial_port 已配置好的串口对象
 * @param angles 输出的欧拉角结构体（Roll, Pitch, Yaw）
 */
inline bool ReadIMUData(serial::Serial& serial_port, EulerAngles& angles) {
  GimbalImuFrame_SCM_t imu_frame;
  size_t frame_size = sizeof(GimbalImuFrame_SCM_t);

  // 检查串口是否已打开
  if (!serial_port.isOpen()) {
    std::cerr << "[ERROR] 串口未打开，无法读取数据。" << std::endl;
    return false;
  }

  // 检查是否有足够的数据可供读取
  if (serial_port.available() < frame_size) {
    std::cerr << "[DEBUG] 数据不足: 可用数据 = " << serial_port.available() << ", 需要数据 = " << frame_size
              << std::endl;
    return false;  // 数据不足，直接返回
  }

  // 初始化 buffer
  uint8_t buffer[frame_size] = {0};

  // 读取数据
  serial_port.read(buffer, frame_size);

  // 将字节流解析为 GimbalImuFrame_SCM_t 结构体
  memcpy(&imu_frame, buffer, frame_size);

  // 检查包头和包尾是否正确
  if (imu_frame._SOF == 0x55 && imu_frame._EOF == 0xFF && imu_frame.ID == IMU_DATA_SEND_ID) {
    // 提取四元数
    float q0 = imu_frame.q0;  // w
    float q1 = imu_frame.q1;  // x
    float q2 = imu_frame.q2;  // y
    float q3 = imu_frame.q3;  // z

    // 四元数转欧拉角 (转为角度值)
    angles.roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);  // Roll
    angles.pitch = -std::asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0f / M_PI);                                    // Pitch
    angles.yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);   // Yaw

    // 输出调试信息
    std::cerr << "[DEBUG] 转换后的欧拉角: Roll=" << angles.roll << ", Pitch=" << angles.pitch << ", Yaw=" << angles.yaw
              << std::endl;

    return true;  // 成功解析并转换
  }

  std::cerr << "[DEBUG] 数据包无效: SOF=" << imu_frame._SOF << ", EOF=" << imu_frame._EOF << ", ID=" << imu_frame.ID
            << std::endl;
  return false;  // 数据无效或解析失败
}
}  // namespace SerialTask
