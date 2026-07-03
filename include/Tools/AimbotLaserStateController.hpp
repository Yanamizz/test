/**
 * @file    include/Tools/AimbotLaserStateController.hpp
 * @brief   收口激光开启阈值、TCP 阶段状态与阶段初始化逻辑。
 *
 * AimbotLaserStateController 根据首次有效目标距离触发激光开启 flag，并维护
 * 当前锁定阶段、`game_progress`、`stage_remain_time` 和 0x92 反制位的
 * 上升沿推进。当前业务约定是触发后 AimbotTarget 保持开启，距离不再直接
 * 关闭激光；该控制器只管理这些语义，不发送串口帧。
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
#include "Tools/TcpStageProtocol.hpp"

namespace Tools {

class AimbotLaserStateController {
public:
  struct TcpStageCommandApplyResult {
    TcpStageCommand command{};
    int previous_stage = ImageRecognize::AerialRobotLaserLockJudge::kInitialStage;
    int current_stage = ImageRecognize::AerialRobotLaserLockJudge::kInitialStage;
    bool stage_advanced = false;
  };

  int CurrentStage() const {
    return current_stage_.load(std::memory_order_acquire);
  }

  std::uint8_t CurrentGameProgress() const {
    return game_progress_.load(std::memory_order_acquire);
  }

  std::uint16_t CurrentStageRemainTime() const {
    return stage_remain_time_.load(std::memory_order_acquire);
  }

  bool CurrentCounteredState() const {
    return last_tcp_countered_high_.load(std::memory_order_acquire);
  }

  static bool UsesStage3ResourcesForStage(int stage) {
    return stage >= ImageRecognize::AerialRobotLaserLockJudge::kStage3FirstStage;
  }

  static const char *ResourceGroupNameForStage(int stage) {
    return UsesStage3ResourcesForStage(stage) ? "stage3" : "stage1/2";
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

  void UpdateGameState(std::uint8_t game_progress,
                       std::uint16_t stage_remain_time) {
    game_progress_.store(static_cast<std::uint8_t>(game_progress & 0x0F),
                         std::memory_order_release);
    stage_remain_time_.store(stage_remain_time, std::memory_order_release);
  }

  bool ProcessTcpCounteredState(bool countered) {
    const bool previous_countered =
        last_tcp_countered_high_.exchange(countered, std::memory_order_acq_rel);
    if (previous_countered || !countered) {
      return false;
    }

    const int previous_stage = CurrentStage();
    const int next_stage = std::min(
        previous_stage + 1, ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage);
    SetCurrentStage(next_stage);
    return next_stage != previous_stage;
  }

  TcpStageCommandApplyResult ApplyTcpCommand(const TcpStageCommand &command) {
    TcpStageCommandApplyResult result{};
    result.command = command;
    result.previous_stage = CurrentStage();

    if (command.type == TcpStageCommandType::GameState91) {
      UpdateGameState(command.game_progress, command.stage_remain_time);
      result.current_stage = result.previous_stage;
      return result;
    }

    result.stage_advanced = ProcessTcpCounteredState(command.countered);
    result.current_stage = CurrentStage();
    return result;
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
  std::atomic<std::uint8_t> game_progress_{0};
  std::atomic<std::uint16_t> stage_remain_time_{0};
  std::atomic<bool> last_tcp_countered_high_{false};
  std::atomic<std::uint64_t> stage_version_{0};
};

} // namespace Tools
