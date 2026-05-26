/**
 * @file    include/ImageRecognize/StagePredictorController.hpp
 * @brief   收口阶段预测器切换、副作用应用与待切换状态。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "CameraTask/ExposureHotkeyController.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/StageRuntimeProfile.hpp"

namespace ImageRecognize {

class StagePredictorController {
public:
  struct Hooks {
    CameraTask::ExposureHotkeyController *exposure_controller = nullptr;
    std::atomic<bool> *stage3_roi_mode_flag = nullptr;
    std::atomic<bool> *scan_stage3_mode_flag = nullptr;
    Tools::ScanController *scan_controller = nullptr;
    std::mutex *scan_controller_mutex = nullptr;
    std::function<void()> on_stage_changed;
    std::function<void()> on_stage3_switch_failure;
  };

  StagePredictorController(ImageRecognize::ImagePredict *stage12_predictor,
                           std::string device_name, double scan_send_hz_cap,
                           Hooks hooks)
      : stage12_predictor_(stage12_predictor),
        active_predictor_(stage12_predictor),
        device_name_(std::move(device_name)),
        stage12_profile_(Tools::MakeStageRuntimeProfile(
            Tools::RuntimeStage::Stage12, scan_send_hz_cap)),
        stage3_profile_(Tools::MakeStageRuntimeProfile(
            Tools::RuntimeStage::Stage3, scan_send_hz_cap)),
        stage3_switch_target_lost_delay_(
            std::chrono::milliseconds(stage3_profile_.switch_target_lost_delay_ms)),
        hooks_(std::move(hooks)) {}

  ImageRecognize::ImagePredict *ActivePredictor() const {
    return active_predictor_;
  }

  bool UsingStage3Predictor() const { return using_stage3_predictor_; }

  bool PendingStage3Switch() const { return pending_stage3_switch_; }

  const Tools::StageRuntimeProfile &
  ProfileFor(Tools::RuntimeStage stage) const {
    return stage == Tools::RuntimeStage::Stage3 ? stage3_profile_
                                                : stage12_profile_;
  }

  void RequestStage3SwitchAfterTargetLoss() {
    if (using_stage3_predictor_ || pending_stage3_switch_) {
      return;
    }
    pending_stage3_switch_ = true;
    std::cout << "[空中机器人阶段] 达到 stage=3，等待目标丢失持续 "
              << stage3_switch_target_lost_delay_.count() << "ms 后切换模型"
              << std::endl;
  }

  bool ShouldSwitchToStage3AfterLostTarget(
      bool target_lost_since_initialized,
      const std::chrono::steady_clock::time_point &now,
      const std::chrono::steady_clock::time_point &target_lost_since) const {
    return pending_stage3_switch_ && !using_stage3_predictor_ &&
           target_lost_since_initialized &&
           (now - target_lost_since) >= stage3_switch_target_lost_delay_;
  }

  bool SwitchToStage3(const char *reason) {
    if (using_stage3_predictor_) {
      return true;
    }

    try {
      ApplyStageSideEffects_(stage3_profile_);
      if (!stage3_predictor_) {
        stage3_predictor_ = std::make_unique<ImageRecognize::ImagePredict>(
            *stage3_profile_.model_path, device_name_,
            stage3_profile_.enable_light_preprocess);
      }
      active_predictor_ = stage3_predictor_.get();
      using_stage3_predictor_ = true;
      pending_stage3_switch_ = false;
      if (hooks_.on_stage_changed) {
        hooks_.on_stage_changed();
      }
      std::cout << "[空中机器人阶段] 切换到 " << stage3_profile_.DisplayName()
                << "，模型=" << *stage3_profile_.model_path << " 曝光(us)="
                << stage3_profile_.exposure_time_us << " 原因=" << reason
                << std::endl;
      return true;
    } catch (const std::exception &e) {
      std::cerr << "切换到 stage3 模型失败：" << e.what() << std::endl;
      if (hooks_.on_stage3_switch_failure) {
        hooks_.on_stage3_switch_failure();
      }
      return false;
    }
  }

  void SwitchToStage12(const char *reason) {
    if (!using_stage3_predictor_) {
      return;
    }

    ApplyStageSideEffects_(stage12_profile_);
    active_predictor_ = stage12_predictor_;
    using_stage3_predictor_ = false;
    pending_stage3_switch_ = false;
    if (hooks_.on_stage_changed) {
      hooks_.on_stage_changed();
    }
    std::cout << "[空中机器人阶段] 切换到 " << stage12_profile_.DisplayName()
              << "，模型=" << *stage12_profile_.model_path << " 曝光(us)="
              << stage12_profile_.exposure_time_us << " 原因=" << reason
              << std::endl;
  }

private:
  static CameraTask::ExposureHotkeyController::ExposureMode ToExposureMode_(
      Tools::RuntimeStage stage) {
    return stage == Tools::RuntimeStage::Stage3
               ? CameraTask::ExposureHotkeyController::ExposureMode::Stage3
               : CameraTask::ExposureHotkeyController::ExposureMode::Stage12;
  }

  static Tools::CalibrationStage ToCalibrationStage_(
      Tools::RuntimeStage stage) {
    return stage == Tools::RuntimeStage::Stage3
               ? Tools::CalibrationStage::Stage3
               : Tools::CalibrationStage::Stage12;
  }

  void ApplyStageSideEffects_(const Tools::StageRuntimeProfile &profile) {
    const bool use_stage3_resources = profile.UsesStage3Resources();
    if (hooks_.exposure_controller) {
      hooks_.exposure_controller->SetActiveMode(ToExposureMode_(profile.stage));
    }
    if (hooks_.stage3_roi_mode_flag) {
      hooks_.stage3_roi_mode_flag->store(use_stage3_resources,
                                         std::memory_order_release);
    }
    if (hooks_.scan_stage3_mode_flag) {
      hooks_.scan_stage3_mode_flag->store(use_stage3_resources,
                                          std::memory_order_release);
    }
    Tools::DistanceCalculator::SetActiveStage(
        ToCalibrationStage_(profile.stage));
    Tools::LaserAngleCalculator::SetActiveStage(
        ToCalibrationStage_(profile.stage));
    if (hooks_.scan_controller && hooks_.scan_controller_mutex) {
      std::lock_guard<std::mutex> lk(*hooks_.scan_controller_mutex);
      hooks_.scan_controller->SetConfig(profile.scan.controller_config);
    }
  }

  ImageRecognize::ImagePredict *stage12_predictor_ = nullptr;
  ImageRecognize::ImagePredict *active_predictor_ = nullptr;
  std::unique_ptr<ImageRecognize::ImagePredict> stage3_predictor_;
  std::string device_name_;
  Tools::StageRuntimeProfile stage12_profile_{};
  Tools::StageRuntimeProfile stage3_profile_{};
  std::chrono::milliseconds stage3_switch_target_lost_delay_{0};
  Hooks hooks_{};
  bool using_stage3_predictor_ = false;
  bool pending_stage3_switch_ = false;
};

} // namespace ImageRecognize
