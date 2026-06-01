/**
 * @file    include/Tools/AimbotLaserStateController.hpp
 * @brief   收口激光开启阈值与阶段初始化逻辑。
 */

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "SerialTask/Common.hpp"
#include "Tools/RuntimeParams.hpp"

namespace Tools {

class AimbotLaserStateController {
public:
  bool IsStageJudgeInitialized() const {
    return stage_judge_initialized_.load(std::memory_order_acquire);
  }

  int CurrentStage() const {
    return current_stage_.load(std::memory_order_acquire);
  }

  void SetCurrentStage(int stage) {
    current_stage_.store(stage, std::memory_order_release);
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

    if (stage_judge == nullptr ||
        !distance_flag_triggered_.load(std::memory_order_acquire)) {
      return;
    }

    bool expected_init = false;
    if (!stage_judge_initialized_.compare_exchange_strong(
            expected_init, true, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return;
    }

    stage_judge->Reset();
    current_stage_.store(ImageRecognize::AerialRobotLaserLockJudge::kInitialStage,
                         std::memory_order_release);
    std::cout << "[LaserStage] 首次获得有效目标距离 " << distance_m
              << "m，初始化阶段判断" << std::endl;
  }

  uint8_t CurrentWireTarget() {
    if (current_stage_.load(std::memory_order_acquire) >=
        ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
      return SerialTask::kAimbotTargetActiveThreshold;
    }

    if (!distance_flag_triggered_.load(std::memory_order_acquire)) {
      return SerialTask::kAimbotTargetMin;
    }

    return SerialTask::kAimbotTargetActiveThreshold;
  }

private:
  std::atomic<bool> distance_flag_triggered_{false};
  std::atomic<bool> stage_judge_initialized_{false};
  std::atomic<int> current_stage_{
      ImageRecognize::AerialRobotLaserLockJudge::kInitialStage};
};

} // namespace Tools
