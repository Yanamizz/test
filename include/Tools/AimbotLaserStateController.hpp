/**
 * @file    include/Tools/AimbotLaserStateController.hpp
 * @brief   收口激光开关阈值、阶段初始化与 stage1->stage2 关闭窗口逻辑。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "SerialTask/Common.hpp"
#include "Tools/RuntimeParams.hpp"

namespace Tools {

class AimbotLaserStateController {
public:
  using Clock = std::chrono::steady_clock;

  void OnStage1ToStage2Locked(const Clock::time_point &now) {
    const auto suppressed_until = now + kSuppressAfterLockDuration_;
    suppressed_until_ticks_.store(
        suppressed_until.time_since_epoch().count(), std::memory_order_release);
    suppressed_after_lock_.store(true, std::memory_order_release);
    std::cout << "[AimbotTarget] stage1->stage2 完成，关闭激光 "
              << std::chrono::duration_cast<std::chrono::seconds>(
                     kSuppressAfterLockDuration_)
                     .count()
              << "s" << std::endl;
  }

  void ClearSuppressWindow() {
    suppressed_after_lock_.store(false, std::memory_order_release);
    suppressed_until_ticks_.store(0, std::memory_order_release);
  }

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
    ClearSuppressWindow();
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

    if (suppressed_after_lock_.load(std::memory_order_acquire)) {
      const auto now = Clock::now();
      const auto suppressed_until = Clock::time_point(
          Clock::duration(suppressed_until_ticks_.load(std::memory_order_acquire)));
      if (now < suppressed_until) {
        return SerialTask::kAimbotTargetMin;
      }
      suppressed_after_lock_.store(false, std::memory_order_release);
      std::cout << "[AimbotTarget] 55s 关闭窗口结束，恢复开激光" << std::endl;
    }

    return SerialTask::kAimbotTargetActiveThreshold;
  }

private:
  static constexpr auto kSuppressAfterLockDuration_ = std::chrono::seconds(0);

  std::atomic<bool> suppressed_after_lock_{false};
  std::atomic<Clock::time_point::rep> suppressed_until_ticks_{0};
  std::atomic<bool> distance_flag_triggered_{false};
  std::atomic<bool> stage_judge_initialized_{false};
  std::atomic<int> current_stage_{
      ImageRecognize::AerialRobotLaserLockJudge::kInitialStage};
};

} // namespace Tools
