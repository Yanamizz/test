/**
 * @file    include/ImageRecognize/TcpStageRuntimeSynchronizer.hpp
 * @brief   收口 TCP 阶段状态到主预测运行时的同步逻辑。
 *
 * 该模块负责监听 `AimbotLaserStateController` 中已经落库的 TCP 阶段状态，
 * 并把“阶段号 -> 锁定显示状态 / 资源组切换 / 切换副作用 / 主流程重置”
 * 这一套同步动作从 `ImagePredict.cc` 中拿出来。主程序只需要在循环里调用
 * `SyncIfNeeded()`，不再自己维护 stage version 与资源组切换判断。
 */

#pragma once

#include <cstdint>
#include <functional>
#include <iostream>

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "ImageRecognize/StagePredictorController.hpp"
#include "Tools/AimbotLaserStateController.hpp"

namespace ImageRecognize {

class TcpStageRuntimeSynchronizer {
public:
  struct Hooks {
    std::function<void()> on_stage_synced;
    std::function<void()> on_resource_group_changed;
  };

  TcpStageRuntimeSynchronizer(
      Tools::AimbotLaserStateController *stage_state_controller,
      ImageRecognize::AerialRobotLaserLockJudge *stage_judge,
      ImageRecognize::StagePredictorController *stage_predictor_controller,
      Hooks hooks)
      : stage_state_controller_(stage_state_controller),
        stage_judge_(stage_judge),
        stage_predictor_controller_(stage_predictor_controller),
        hooks_(std::move(hooks)) {
    if (stage_state_controller_ != nullptr) {
      last_stage_version_ = stage_state_controller_->StageVersion();
      last_stage_ = stage_state_controller_->CurrentStage();
    }
  }

  void InitializeCurrentStage() {
    if (stage_judge_ != nullptr) {
      stage_judge_->SetStage(last_stage_);
    }
    EnsureRuntimeStage_("startup_stage_sync");
  }

  void SyncIfNeeded() {
    if (stage_state_controller_ == nullptr) {
      return;
    }

    const std::uint64_t stage_version =
        stage_state_controller_->StageVersion();
    if (stage_version == last_stage_version_) {
      return;
    }

    const int current_stage = stage_state_controller_->CurrentStage();
    const bool resource_group_changed =
        Tools::AimbotLaserStateController::UsesStage3ResourcesForStage(
            last_stage_) !=
        Tools::AimbotLaserStateController::UsesStage3ResourcesForStage(
            current_stage);

    if (stage_judge_ != nullptr) {
      stage_judge_->SetStage(current_stage);
    }
    if (hooks_.on_stage_synced) {
      hooks_.on_stage_synced();
    }

    if (resource_group_changed) {
      EnsureRuntimeStage_("tcp_stage_edge");
      if (hooks_.on_resource_group_changed) {
        hooks_.on_resource_group_changed();
      }
    }

    std::cout << "[空中机器人阶段] TCP 推进到 stage" << current_stage
              << "，资源组="
              << Tools::AimbotLaserStateController::ResourceGroupNameForStage(
                     current_stage)
              << " game_progress="
              << static_cast<int>(
                     stage_state_controller_->CurrentGameProgress())
              << " stage_remain_time="
              << stage_state_controller_->CurrentStageRemainTime()
              << std::endl;

    last_stage_ = current_stage;
    last_stage_version_ = stage_version;
  }

private:
  void EnsureRuntimeStage_(const char *reason) {
    if (stage_state_controller_ == nullptr ||
        stage_predictor_controller_ == nullptr) {
      return;
    }

    stage_predictor_controller_->EnsureRuntimeStage(
        stage_state_controller_->UsesStage3Resources()
            ? Tools::RuntimeStage::Stage3
            : Tools::RuntimeStage::Stage12,
        reason);
  }

  Tools::AimbotLaserStateController *stage_state_controller_ = nullptr;
  ImageRecognize::AerialRobotLaserLockJudge *stage_judge_ = nullptr;
  ImageRecognize::StagePredictorController *stage_predictor_controller_ =
      nullptr;
  Hooks hooks_{};
  std::uint64_t last_stage_version_ = 0;
  int last_stage_ = ImageRecognize::AerialRobotLaserLockJudge::kInitialStage;
};

} // namespace ImageRecognize
