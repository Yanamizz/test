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
  uint8_t _SOF;         ///< 包头 0x55
  uint8_t ID;           ///< 接收 id 0x02
  uint8_t AimbotState;  ///< 0x00 无目标，0x01 有目标
  uint8_t AimbotTarget;
  float PitchRelativeAngle;  ///< Pitch (°)
  float YawRelativeAngle;    ///< Yaw   (°)
  float SystemTimer;         ///< 时间戳
  uint8_t _EOF;              ///< 包尾 0xff
} AimbotFrame_SCM_t;

typedef struct __attribute__((packed)) {
  uint8_t _SOF;  ///< 包头 0x55
  uint8_t ID;    ///< 接收 id 0x03
  uint32_t TimeStamp;
  float q0;  ///< 四元数 w
  float q1;  ///< 四元数 x
  float q2;  ///< 四元数 y
  float q3;  ///< 四元数 z
  uint8_t robot_id;
  uint8_t aim_mode;
  uint8_t _EOF;  ///< 包尾 0xff
} GimbalImuFrame_SCM_t;

inline constexpr uint8_t IMU_DATA_SEND_ID = 0x03;
