/**
 * @file    include\SerialTask\SerialRead.hpp
 * @brief   串口 IMU 数据读取与四元数转欧拉角
 * @brief   输入串口字节流，输出目标侧欧拉角帧（Roll，Pitch, Yaw）
 * 
 * @date    2026-01-19
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <Eigen/Dense>

/**
 * @brief 目标侧欧拉角帧（Pitch, Yaw）
 */
typedef struct __attribute__((packed)) {
  uint8_t _SOF;              ///< 包头 0x55
  uint8_t ID;                ///< 接收 id 0x02
  float PitchRelativeAngle;  ///< Pitch (°)
  float YawRelativeAngle;    ///< Yaw   (°)
  uint8_t _EOF;              ///< 包尾 0xff
} AimbotFrame_SCM_t;

/**
 * @brief 上位机发送的 IMU 四元数帧
 */
typedef struct __attribute__((packed)) {
  uint8_t _SOF;  ///< 包头 0x55
  uint8_t ID;    ///< 接收 id 0x01
  float q0;      ///< 四元数 w
  float q1;      ///< 四元数 x
  float q2;      ///< 四元数 y
  float q3;      ///< 四元数 z
  uint8_t _EOF;  ///< 包尾 0xff
} GimbalImuFrame_SCM_t;

#define IMU_DATA_SEND_ID 0x1

/**
 * @brief 解析串口字节流为 IMU 四元数帧
 * @param[in]  data   输入字节流
 * @param[in]  len    字节流长度
 * @param[out] frame  解析出的 IMU 帧
 * @return true 解析成功，false 失败
 */
inline bool DecodeImuFrame(const uint8_t *data, size_t len, GimbalImuFrame_SCM_t &frame) {
  constexpr size_t kFrameSize = sizeof(GimbalImuFrame_SCM_t);
  if (!data || len < kFrameSize) return false;
  // 边界检查
  if (data[0] != 0x55 || data[1] != IMU_DATA_SEND_ID || data[kFrameSize - 1] != 0xff) return false;
  std::memcpy(&frame, data, kFrameSize);
  return true;
}

/**
 * @brief 将四元数帧转换为欧拉角并填充 Aimbot 帧
 * @param[in]  imu_frame   IMU 四元数帧
 * @param[out] aimbot      欧拉角输出帧（Pitch, Yaw）
 * @param[out] euler_rpy   返回 [roll, pitch, yaw]（角度）
 */
inline void QuaternionToEulerAngles(const GimbalImuFrame_SCM_t &imu_frame, AimbotFrame_SCM_t &aimbot,
                                    Eigen::Vector3f *euler_rpy = nullptr) {
  Eigen::Quaternionf q(imu_frame.q0, imu_frame.q1, imu_frame.q2, imu_frame.q3);  // w, x, y, z
  const Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);       // roll, pitch, yaw
  if (euler_rpy) {
    *euler_rpy = euler;
  }
  aimbot._SOF = 0x55;
  aimbot.ID = 0x02;
  aimbot.PitchRelativeAngle = euler.y();
  aimbot.YawRelativeAngle = euler.z();
  aimbot._EOF = 0xff;
}