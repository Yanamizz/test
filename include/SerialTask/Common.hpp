/**
 * @file Common.hpp
 * @brief 公共结构体定义
 */
#pragma once

#include <cstdint>
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
