/**
 * @file    include/SerialTask/Common.hpp
 * @brief   定义串口通信协议中使用的公共帧结构与常量。
 */
#pragma once

#include <atomic>
#include <cstdint>
/**
 * @brief 目标侧欧拉角帧（Pitch, Yaw）
 */
typedef struct __attribute__((packed)) {
  uint8_t _SOF;        ///< 包头 0x55
  uint8_t ID;          ///< 接收 id 0x02
  uint8_t AimbotState; ///< 0x00 无目标，0x01 有目标
  uint8_t AimbotTarget; ///< 串口激光开关：0x00 关闭，0x01 开启
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

namespace SerialTask {

inline constexpr uint8_t IMU_DATA_SEND_ID = 0x03;

// AimbotTarget 内部计数语义：
// - 计数范围 [kAimbotTargetMin, kAimbotTargetMax]
// - TCP 接收与 stage 切换仍维护该计数
// - 串口发送时按运行时激光窗口二值化为 0x00/0x01
inline constexpr uint8_t kAimbotTargetMin = 0x00;
inline constexpr uint8_t kAimbotTargetActiveThreshold = 0x01;
inline constexpr uint8_t kAimbotTargetMax = 0x03;

inline uint8_t ToWireAimbotTarget(uint8_t counter_value) {
  return (counter_value >= kAimbotTargetActiveThreshold)
             ? kAimbotTargetActiveThreshold
             : kAimbotTargetMin;
}

inline void SaturatingIncrementAimbotTarget(std::atomic<uint8_t> &counter) {
  uint8_t old_value = counter.load(std::memory_order_acquire);
  while (old_value < kAimbotTargetMax) {
    if (counter.compare_exchange_weak(old_value,
                                      static_cast<uint8_t>(old_value + 1),
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire)) {
      return;
    }
  }
}

inline void SaturatingDecrementAimbotTarget(std::atomic<uint8_t> &counter) {
  uint8_t old_value = counter.load(std::memory_order_acquire);
  while (old_value > kAimbotTargetMin) {
    if (counter.compare_exchange_weak(old_value,
                                      static_cast<uint8_t>(old_value - 1),
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire)) {
      return;
    }
  }
}

} // namespace SerialTask
