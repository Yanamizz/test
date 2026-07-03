/**
 * @file    include/Tools/AimbotLaserStateController.hpp
 * @brief   收口激光开启阈值与阶段初始化逻辑。
 *
 * AimbotLaserStateController 根据首次有效目标距离触发激光开启 flag，并维护
 * 阶段判断初始化状态与当前锁定阶段。当前业务约定是触发后 AimbotTarget 保持
 * 开启，距离不再直接关闭激光；该控制器只管理这些语义，不发送串口帧。
 */

#pragma once

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "SerialTask/Common.hpp"
#include "Tools/RuntimeParams.hpp"

namespace Tools {

class AimbotLaserStateController {
public:
  int CurrentStage() const {
    return current_stage_.load(std::memory_order_acquire);
  }

  void SetCurrentStage(int stage) {
    const int clamped_stage =
        std::clamp(stage, ImageRecognize::AerialRobotLaserLockJudge::kInitialStage,
                   ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage);
    const int previous_stage =
        current_stage_.exchange(clamped_stage, std::memory_order_acq_rel);
    if (previous_stage != clamped_stage) {
      stage_version_.fetch_add(1, std::memory_order_acq_rel);
    }
  }

  bool ProcessTcpSignalLevel(uint8_t signal_value) {
    const bool signal_high = signal_value != 0x00;
    const bool previous_signal_high =
        last_tcp_signal_high_.exchange(signal_high, std::memory_order_acq_rel);
    if (previous_signal_high || !signal_high) {
      return false;
    }

    const int previous_stage = CurrentStage();
    const int next_stage = std::min(
        previous_stage + 1, ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage);
    SetCurrentStage(next_stage);
    return next_stage != previous_stage;
  }

  std::uint64_t StageVersion() const {
    return stage_version_.load(std::memory_order_acquire);
  }

  bool UsesStage3Resources() const {
    return CurrentStage() >=
           ImageRecognize::AerialRobotLaserLockJudge::kStage3FirstStage;
  }

  bool DistanceFlagTriggered() const {
    return distance_flag_triggered_.load(std::memory_order_acquire);
  }

  bool IsValidTargetDistance(float distance_m) const {
    const float max_distance_m = Params().aimbot_target_laser_max_distance_m;
    return std::isfinite(distance_m) && distance_m > 0.0f &&
           std::isfinite(max_distance_m) && max_distance_m > 0.0f &&
           distance_m <= max_distance_m;
  }

  void UpdateDistanceFlags(
      float distance_m,
      ImageRecognize::AerialRobotLaserLockJudge *stage_judge) {
    if (!IsValidTargetDistance(distance_m)) {
      return;
    }

    bool expected_flag = false;
    if (distance_flag_triggered_.compare_exchange_strong(
            expected_flag, true, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      std::cout << "[LaserStage] 首次获得有效目标距离 " << distance_m
                << "m，触发激光开启 flag" << std::endl;
    }
    if (stage_judge != nullptr) {
      stage_judge->SetStage(CurrentStage(), false);
    }
  }

  uint8_t CurrentWireTarget() {
    if (UsesStage3Resources()) {
      return SerialTask::kAimbotTargetActiveThreshold;
    }

    if (!distance_flag_triggered_.load(std::memory_order_acquire)) {
      return SerialTask::kAimbotTargetMin;
    }

    return SerialTask::kAimbotTargetActiveThreshold;
  }

private:
  std::atomic<bool> distance_flag_triggered_{false};
  std::atomic<int> current_stage_{
      ImageRecognize::AerialRobotLaserLockJudge::kInitialStage};
  std::atomic<bool> last_tcp_signal_high_{false};
  std::atomic<std::uint64_t> stage_version_{0};
};

} // namespace Tools
