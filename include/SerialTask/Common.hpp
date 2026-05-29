/**
 * @file    include/SerialTask/Common.hpp
 * @brief   定义串口通信协议中使用的公共帧结构与常量。
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
  uint8_t AimbotTarget; ///< 串口激光开关：0x00 关闭，0x01 开启
  float PitchRelativeAngle; ///< Pitch 角（弧度）
  float YawRelativeAngle;   ///< Yaw 角（弧度）
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

namespace SerialTask {

inline constexpr uint8_t IMU_DATA_SEND_ID = 0x03;

// AimbotTarget 线协议值：0x00 关激光，0x01 开激光。
// 主程序不再通过 TCP 维护 AimbotTarget 计数；线值由运行时激光状态决定。
inline constexpr uint8_t kAimbotTargetMin = 0x00;
inline constexpr uint8_t kAimbotTargetActiveThreshold = 0x01;

inline uint8_t ToWireAimbotTarget(uint8_t value) {
  return (value >= kAimbotTargetActiveThreshold)
             ? kAimbotTargetActiveThreshold
             : kAimbotTargetMin;
}

} // namespace SerialTask
