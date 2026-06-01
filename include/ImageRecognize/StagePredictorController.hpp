/**
 * @file    include/ImageRecognize/StagePredictorController.hpp
 * @brief   收口阶段预测器切换、副作用应用与待切换状态。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "CameraTask/ExposureHotkeyController.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/StageRuntimeProfile.hpp"

namespace ImageRecognize {

class StagePredictorController {
public:
  struct Stage3WarmupResult {
    std::unique_ptr<ImageRecognize::ImagePredict> predictor;
    double elapsed_ms = 0.0;
  };

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

  void RequestStage3SwitchAfterTargetLoss(const char *reason = "strict_progress",
                                          bool probe_switch = false) {
    if (using_stage3_predictor_ || pending_stage3_switch_) {
      return;
    }
    StartStage3WarmupIfNeeded_();
    pending_stage3_switch_ = true;
    pending_stage3_probe_switch_ = probe_switch;
    std::cout << "[空中机器人阶段] 请求 stage3，原因="
              << (reason == nullptr ? "unknown" : reason)
              << "，probe=" << (probe_switch ? "true" : "false")
              << "，等待目标丢失持续 "
              << stage3_switch_target_lost_delay_.count() << "ms 后切换模型"
              << std::endl;
  }

  bool Stage3ProbeActive() const {
    return using_stage3_predictor_ && stage3_probe_active_;
  }

  void ConfirmStage3Probe() {
    if (!stage3_probe_active_) {
      return;
    }
    stage3_probe_active_ = false;
    std::cout << "[空中机器人阶段] stage3 probe 已识别到目标，确认保持 stage3"
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
      EnsureStage3PredictorReady_();
      const auto switch_start = std::chrono::steady_clock::now();
      active_predictor_ = stage3_predictor_.get();
      using_stage3_predictor_ = true;
      pending_stage3_switch_ = false;
      stage3_probe_active_ = pending_stage3_probe_switch_;
      pending_stage3_probe_switch_ = false;
      if (hooks_.on_stage_changed) {
        hooks_.on_stage_changed();
      }
      const auto switch_end = std::chrono::steady_clock::now();
      const double switch_elapsed_ms = std::chrono::duration<double, std::milli>(
                                           switch_end - switch_start)
                                           .count();
      std::cout << "[空中机器人阶段] 切换到 " << stage3_profile_.DisplayName()
                << "，模型=" << *stage3_profile_.model_path << " 曝光(us)="
                << LoggedExposureTimeUs_(stage3_profile_) << " 原因="
                << reason << " 切换接管耗时=" << switch_elapsed_ms << "ms"
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
    pending_stage3_probe_switch_ = false;
    stage3_probe_active_ = false;
    if (hooks_.on_stage_changed) {
      hooks_.on_stage_changed();
    }
    std::cout << "[空中机器人阶段] 切换到 " << stage12_profile_.DisplayName()
              << "，模型=" << *stage12_profile_.model_path << " 曝光(us)="
              << LoggedExposureTimeUs_(stage12_profile_) << " 原因="
              << reason << std::endl;
  }

private:
  static double ElapsedMilliseconds_(
      const std::chrono::steady_clock::time_point &start,
      const std::chrono::steady_clock::time_point &end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

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

  double LoggedExposureTimeUs_(const Tools::StageRuntimeProfile &profile) const {
    if (hooks_.exposure_controller) {
      return hooks_.exposure_controller->GetExposureTime();
    }
    return profile.exposure_time_us;
  }

  void ApplyStageSideEffects_(const Tools::StageRuntimeProfile &profile) {
    const auto total_start = std::chrono::steady_clock::now();
    const bool use_stage3_resources = profile.UsesStage3Resources();
    double exposure_ms = 0.0;
    double roi_flag_ms = 0.0;
    double calibration_ms = 0.0;
    double scan_config_ms = 0.0;

    if (hooks_.exposure_controller) {
      const auto step_start = std::chrono::steady_clock::now();
      hooks_.exposure_controller->SetActiveMode(ToExposureMode_(profile.stage));
      const auto step_end = std::chrono::steady_clock::now();
      exposure_ms = ElapsedMilliseconds_(step_start, step_end);
    }

    const auto roi_flag_start = std::chrono::steady_clock::now();
    if (hooks_.stage3_roi_mode_flag) {
      hooks_.stage3_roi_mode_flag->store(use_stage3_resources,
                                         std::memory_order_release);
    }
    if (hooks_.scan_stage3_mode_flag) {
      hooks_.scan_stage3_mode_flag->store(use_stage3_resources,
                                          std::memory_order_release);
    }
    const auto roi_flag_end = std::chrono::steady_clock::now();
    roi_flag_ms = ElapsedMilliseconds_(roi_flag_start, roi_flag_end);

    const auto calibration_start = std::chrono::steady_clock::now();
    Tools::DistanceCalculator::SetActiveStage(
        ToCalibrationStage_(profile.stage));
    const auto calibration_end = std::chrono::steady_clock::now();
    calibration_ms = ElapsedMilliseconds_(calibration_start, calibration_end);

    if (hooks_.scan_controller && hooks_.scan_controller_mutex) {
      const auto scan_config_start = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lk(*hooks_.scan_controller_mutex);
      hooks_.scan_controller->SetConfig(profile.scan.controller_config);
      const auto scan_config_end = std::chrono::steady_clock::now();
      scan_config_ms = ElapsedMilliseconds_(scan_config_start, scan_config_end);
    }

    const auto total_end = std::chrono::steady_clock::now();
    std::cout << "[空中机器人阶段] " << profile.DisplayName()
              << " 副作用耗时 total="
              << ElapsedMilliseconds_(total_start, total_end)
              << "ms exposure=" << exposure_ms << "ms roi_flag="
              << roi_flag_ms << "ms calibration=" << calibration_ms
              << "ms scan_config=" << scan_config_ms << "ms" << std::endl;
  }

  void StartStage3WarmupIfNeeded_() {
    if (stage3_predictor_ || stage3_warmup_future_.valid()) {
      return;
    }

    const std::string model_path = *stage3_profile_.model_path;
    const std::string device_name = device_name_;
    const bool enable_light_preprocess = stage3_profile_.enable_light_preprocess;
    std::cout << "[空中机器人阶段] stage3 模型后台预热启动，模型="
              << model_path << std::endl;
    stage3_warmup_future_ = std::async(
        std::launch::async,
        [model_path, device_name, enable_light_preprocess]() mutable {
          Tools::BindCurrentThreadToAuxCores();
          const auto warmup_start = std::chrono::steady_clock::now();
          auto predictor = std::make_unique<ImageRecognize::ImagePredict>(
              model_path, device_name, enable_light_preprocess);
          const auto warmup_end = std::chrono::steady_clock::now();
          Stage3WarmupResult result{};
          result.predictor = std::move(predictor);
          result.elapsed_ms =
              std::chrono::duration<double, std::milli>(warmup_end - warmup_start)
                  .count();
          return result;
        });
  }

  void EnsureStage3PredictorReady_() {
    if (stage3_predictor_) {
      return;
    }

    if (!stage3_warmup_future_.valid()) {
      const auto load_start = std::chrono::steady_clock::now();
      stage3_predictor_ = std::make_unique<ImageRecognize::ImagePredict>(
          *stage3_profile_.model_path, device_name_,
          stage3_profile_.enable_light_preprocess);
      const auto load_end = std::chrono::steady_clock::now();
      std::cout << "[空中机器人阶段] stage3 模型同步加载完成，耗时="
                << std::chrono::duration<double, std::milli>(load_end - load_start)
                       .count()
                << "ms" << std::endl;
      return;
    }

    const auto future_status =
        stage3_warmup_future_.wait_for(std::chrono::milliseconds(0));
    if (future_status != std::future_status::ready) {
      std::cout << "[空中机器人阶段] stage3 模型预热未完成，切换时等待后台预热收尾"
                << std::endl;
    }

    try {
      Stage3WarmupResult result = stage3_warmup_future_.get();
      stage3_predictor_ = std::move(result.predictor);
      std::cout << "[空中机器人阶段] stage3 模型后台预热完成，耗时="
                << result.elapsed_ms << "ms" << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[空中机器人阶段] stage3 模型后台预热失败，回退同步加载："
                << e.what() << std::endl;
      const auto load_start = std::chrono::steady_clock::now();
      stage3_predictor_ = std::make_unique<ImageRecognize::ImagePredict>(
          *stage3_profile_.model_path, device_name_,
          stage3_profile_.enable_light_preprocess);
      const auto load_end = std::chrono::steady_clock::now();
      std::cout << "[空中机器人阶段] stage3 模型同步回退加载完成，耗时="
                << std::chrono::duration<double, std::milli>(load_end - load_start)
                       .count()
                << "ms" << std::endl;
    }
  }

  ImageRecognize::ImagePredict *stage12_predictor_ = nullptr;
  ImageRecognize::ImagePredict *active_predictor_ = nullptr;
  std::unique_ptr<ImageRecognize::ImagePredict> stage3_predictor_;
  std::future<Stage3WarmupResult> stage3_warmup_future_;
  std::string device_name_;
  Tools::StageRuntimeProfile stage12_profile_{};
  Tools::StageRuntimeProfile stage3_profile_{};
  std::chrono::milliseconds stage3_switch_target_lost_delay_{0};
  Hooks hooks_{};
  bool using_stage3_predictor_ = false;
  bool pending_stage3_switch_ = false;
  bool pending_stage3_probe_switch_ = false;
  bool stage3_probe_active_ = false;
};

} // namespace ImageRecognize
