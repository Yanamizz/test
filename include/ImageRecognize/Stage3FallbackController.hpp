/**
 * @file    include/ImageRecognize/Stage3FallbackController.hpp
 * @brief   收口 stage2 异常无目标兜底、stage3 probe 与电机响应探测。
 *
 * Stage3FallbackController 跟踪 stage2 进度、无目标持续时间、近期紫色观测
 * 和电机探测反馈，用于决定是否进入 stage3 候选/探测路径。它输出阶段兜底
 * 意图和探测动作，具体模型切换、ROI、曝光和扫描副作用由主流程其它模块执行。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "SerialTask/SerialRead.hpp"
#include "Tools/AimbotCommand.hpp"
#include "Tools/AngleUtils.hpp"

namespace ImageRecognize {

class Stage3FallbackController {
public:
  using Clock = std::chrono::steady_clock;
  using TriggerReason = Stage3FallbackSwitchGuard::TriggerReason;

  struct Config {
    Stage3FallbackSwitchGuard::Config guard{};
    float motor_probe_yaw_offset_deg = 0.0f;
    float motor_probe_min_imu_delta_deg = 0.0f;
    std::chrono::milliseconds motor_probe_wait{0};
  };

  struct SwitchRequest {
    bool valid = false;
    TriggerReason reason = TriggerReason::None;
    double max_progress = 0.0;
    Clock::time_point trigger_time{};

    const char *ReasonTag() const {
      return Stage3FallbackSwitchGuard::TriggerReasonTag(reason);
    }

    bool IsProbeSwitch() const {
      return reason == TriggerReason::HighProgressRecentPurpleLostTarget ||
             reason == TriggerReason::HighProgressLongNoTarget;
    }
  };

  struct JudgeTickInput {
    bool stage_judge_initialized = false;
    bool using_stage3_predictor = false;
    int current_stage = AerialRobotLaserLockJudge::kInitialStage;
    int laser_judge_class_id = -1;
    bool has_boxes = false;
    double progress = 0.0;
    Clock::time_point now{};
  };

  struct MotorProbeInput {
    bool has_tracked_box = false;
    bool has_current_imu = false;
    SerialTask::EulerAngles current_imu{};
    Clock::time_point now{};
  };

  struct MotorProbeResult {
    bool active = false;
    bool has_command = false;
    bool clear_pending_send = false;
    Tools::AimbotSendCommand command{};
    SwitchRequest switch_request{};
  };

  void SetConfig(const Config &config) { config_ = config; }

  void Reset() {
    guard_.Reset();
    pending_switch_ = SwitchRequest{};
    motor_probe_ = MotorProbeState{};
  }

  void ResetNoTargetTimers() { guard_.ResetNoTargetTimers(); }

  double MaxStage2Progress() const { return guard_.MaxStage2Progress(); }

  bool UpdateGuardOnJudgeTick(const JudgeTickInput &input) {
    const bool should_fallback = guard_.Update(
        Stage3FallbackSwitchGuard::UpdateInput{
            input.stage_judge_initialized, input.using_stage3_predictor,
            input.current_stage, input.laser_judge_class_id, input.has_boxes,
            input.progress, config_.guard, input.now});
    if (!should_fallback || pending_switch_.valid) {
      return false;
    }

    pending_switch_.valid = true;
    pending_switch_.reason = guard_.LastTriggerReason();
    pending_switch_.max_progress = guard_.MaxStage2Progress();
    pending_switch_.trigger_time = input.now;
    std::cout << "[空中机器人阶段] 异常兜底等待电机响应确认：原因="
              << pending_switch_.ReasonTag()
              << "，stage2 最高P=" << pending_switch_.max_progress
              << std::endl;
    return true;
  }

  MotorProbeResult UpdateMotorProbe(const MotorProbeInput &input) {
    MotorProbeResult result{};
    if (!pending_switch_.valid) {
      motor_probe_ = MotorProbeState{};
      return result;
    }

    if (input.has_tracked_box) {
      std::cout << "[空中机器人阶段] 异常兜底电机探测取消：重新识别到目标"
                << std::endl;
      pending_switch_ = SwitchRequest{};
      motor_probe_ = MotorProbeState{};
      return result;
    }

    if (!input.has_current_imu) {
      std::cout << "[空中机器人阶段] 异常兜底电机探测无法执行：没有可用 IMU，重置异常无目标计时"
                << std::endl;
      guard_.ResetNoTargetTimers();
      pending_switch_ = SwitchRequest{};
      motor_probe_ = MotorProbeState{};
      result.active = true;
      return result;
    }

    const float safe_probe_yaw_offset =
        std::isfinite(config_.motor_probe_yaw_offset_deg)
            ? config_.motor_probe_yaw_offset_deg
            : 0.0f;
    const float safe_min_imu_delta = std::max(
        0.0f, std::isfinite(config_.motor_probe_min_imu_delta_deg)
                  ? config_.motor_probe_min_imu_delta_deg
                  : 0.0f);

    if (!motor_probe_.active) {
      motor_probe_.active = true;
      motor_probe_.start_time = input.now;
      motor_probe_.start_imu = input.current_imu;
      const auto command_enqueue_time = std::chrono::steady_clock::now();
      result.has_command = true;
      result.active = true;
      result.command = Tools::AimbotSendCommand{
          input.current_imu.pitch,
          input.current_imu.yaw + safe_probe_yaw_offset,
          0.0f,
          safe_probe_yaw_offset,
          0.0f,
          0.0f,
          0x01,
          input.now,
          command_enqueue_time};
      std::cout << "[空中机器人阶段] 异常兜底电机探测：发送 yaw 小偏角 "
                << safe_probe_yaw_offset << "deg，等待 "
                << config_.motor_probe_wait.count() << "ms" << std::endl;
      return result;
    }

    const float imu_delta =
        MaxImuDeltaDeg_(motor_probe_.start_imu, input.current_imu);
    result.active = true;
    if (imu_delta >= safe_min_imu_delta) {
      std::cout << "[空中机器人阶段] 异常兜底电机探测通过：IMU 变化 "
                << imu_delta << "deg >= " << safe_min_imu_delta << "deg"
                << std::endl;
      result.switch_request = pending_switch_;
      pending_switch_ = SwitchRequest{};
      motor_probe_ = MotorProbeState{};
      return result;
    }

    if (config_.motor_probe_wait.count() <= 0 ||
        (input.now - motor_probe_.start_time) >= config_.motor_probe_wait) {
      std::cout << "[空中机器人阶段] 异常兜底电机探测未通过：IMU 变化 "
                << imu_delta << "deg < " << safe_min_imu_delta
                << "deg，判断对方可能未发起空中支援，重置异常无目标计时"
                << std::endl;
      guard_.ResetNoTargetTimers();
      pending_switch_ = SwitchRequest{};
      motor_probe_ = MotorProbeState{};
      result.clear_pending_send = true;
      return result;
    }

    return result;
  }

  static void LogSwitchTrigger(const SwitchRequest &request,
                               const Config &config) {
    if (!request.valid) {
      return;
    }

    std::cout << "[空中机器人阶段] stage3 保守兜底触发：原因="
              << request.ReasonTag() << "，stage2 最高P="
              << request.max_progress;
    if (request.reason == TriggerReason::Stage2LongNoTarget) {
      std::cout << "，stage2 连续空框达到 "
                << config.guard.stage2_no_target_force_duration.count()
                << "ms，直接兜底进入 stage3";
    } else if (request.reason == TriggerReason::HighProgressLongNoTarget) {
      std::cout << "，P 曾达到阈值后连续空框达到 "
                << config.guard.high_progress_no_target_probe_duration.count()
                << "ms，进入 stage3 probe";
    } else {
      std::cout << "，连续空框达到 "
                << config.guard.no_target_duration.count()
                << "ms，最近紫色窗口="
                << config.guard.recent_purple_duration.count()
                << "ms，进入 stage3 probe";
    }
    std::cout << std::endl;
  }

private:
  struct MotorProbeState {
    bool active = false;
    Clock::time_point start_time{};
    SerialTask::EulerAngles start_imu{};
  };

  static float MaxImuDeltaDeg_(const SerialTask::EulerAngles &from,
                               const SerialTask::EulerAngles &to) {
    const float yaw_delta =
        std::abs(Tools::NormalizeDeltaDeg(to.yaw - from.yaw));
    const float pitch_delta =
        std::abs(Tools::NormalizeDeltaDeg(to.pitch - from.pitch));
    return std::max(yaw_delta, pitch_delta);
  }

  Config config_{};
  Stage3FallbackSwitchGuard guard_{};
  SwitchRequest pending_switch_{};
  MotorProbeState motor_probe_{};
};

} // namespace ImageRecognize
