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
  uint8_t _SOF;        ///< 包头 0x55
  uint8_t ID;          ///< 接收 id 0x02
  uint8_t AimbotState; ///< 0x00 无目标，0x01 有目标
  uint8_t AimbotTarget;
  float PitchRelativeAngle; ///< Pitch (rad)
  float YawRelativeAngle;   ///< Yaw   (rad)
  float PitchOffset;        ///< Pitch 偏差角 (rad)
  float YawOffset;          ///< Yaw 偏差角 (rad)
  float PitchVelocity;      ///< Pitch 角速度 (rad/s)
  float YawVelocity;        ///< Yaw   角速度 (rad/s)
  float SystemTimer;        ///< 时间戳
  uint8_t _EOF;             ///< 包尾 0xff
} AimbotFrame_SCM_t;

static_assert(sizeof(float) == 4, "Serial protocol requires 32-bit float.");
static_assert(sizeof(AimbotFrame_SCM_t) == 33,
              "AimbotFrame_SCM_t wire size changed.");

typedef struct __attribute__((packed)) {
  uint8_t _SOF; ///< 包头 0x55
  uint8_t ID;   ///< 接收 id 0x03
  uint32_t TimeStamp;
  float q0; ///< 四元数 w
  float q1; ///< 四元数 x
  float q2; ///< 四元数 y
  float q3; ///< 四元数 z
  uint8_t robot_id;
  uint8_t aim_mode;
  uint8_t _EOF; ///< 包尾 0xff
} GimbalImuFrame_SCM_t;

static_assert(sizeof(GimbalImuFrame_SCM_t) == 25,
              "GimbalImuFrame_SCM_t wire size changed.");

inline constexpr uint8_t IMU_DATA_SEND_ID = 0x03;
