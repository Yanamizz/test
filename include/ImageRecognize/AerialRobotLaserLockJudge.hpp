/**
 * @file    include/ImageRecognize/AerialRobotLaserLockJudge.hpp
 * @brief   根据目标类别连续观测结果判断空中机器人激光锁定阶段。
 *
 * AerialRobotLaserLockJudge 将逐帧检测类别转换为阶段进度，处理红/蓝/紫等
 * 目标状态的连续确认、阈值推进和阶段输出。主流程依赖它判断激光阶段和
 * stage3 切换条件；该模块只消费识别结果，不直接控制串口或扫描状态。
 */

#pragma once

#include <algorithm>
#include <chrono>

namespace ImageRecognize {

class AerialRobotLaserLockJudge {
 public:
  static constexpr int kPurpleClassId = 2;
  static constexpr int kInitialStage = 1;
  static constexpr int kFinishedStage = 3;
  static constexpr double kJudgePeriodSeconds = 0.1;
  static constexpr double kProgressDecayPerSecond = 0.5;

  struct TickResult {
    int previous_stage = kInitialStage;
    int stage = kInitialStage;
    double previous_progress = 0.0;
    double progress = 0.0;
    bool progress_increased = false;
    bool advanced_stage = false;
    bool reached_finished = false;
  };

  TickResult Tick100ms(bool illuminated) {
    TickResult result{};
    result.previous_stage = stage_;
    result.stage = stage_;
    result.previous_progress = progress_;
    result.progress = progress_;

    if (IsFinished()) {
      return result;
    }

    if (!illuminated) {
      progress_ = std::max(
          0.0, progress_ - kProgressDecayPerSecond * kJudgePeriodSeconds);
      ResetContinuousIllumination();
      result.stage = stage_;
      result.progress = progress_;
      return result;
    }

    ++continuous_judge_count_;
    progress_ = std::min(
        100.0, progress_ + static_cast<double>(continuous_judge_count_));
    result.progress_increased = true;

    if (progress_ >= CurrentThreshold()) {
      AdvanceStage();
    }

    result.stage = stage_;
    result.progress = progress_;
    result.advanced_stage = result.stage != result.previous_stage;
    result.reached_finished =
        result.previous_stage < kFinishedStage && result.stage >= kFinishedStage;
    return result;
  }

  void Reset() {
    stage_ = kInitialStage;
    progress_ = 0.0;
    ResetContinuousIllumination();
  }

  void ForceFinished() {
    stage_ = kFinishedStage;
    progress_ = 0.0;
    ResetContinuousIllumination();
  }

  void ForceStage2() {
    stage_ = kInitialStage + 1;
    progress_ = 0.0;
    ResetContinuousIllumination();
  }

  static bool IsPurpleClassId(int class_id) {
    return class_id == kPurpleClassId;
  }

  int Stage() const { return stage_; }
  bool IsFinished() const { return stage_ >= kFinishedStage; }
  double Progress() const { return progress_; }

  int CurrentThreshold() const {
    return stage_ == kInitialStage ? kFirstLockThreshold : kLaterLockThreshold;
  }

 private:
  static constexpr int kFirstLockThreshold = 50;
  static constexpr int kLaterLockThreshold = 100;

  void AdvanceStage() {
    progress_ = 0.0;
    stage_ = std::min(stage_ + 1, kFinishedStage);
  }

  void ResetContinuousIllumination() {
    continuous_judge_count_ = 0;
  }

  int stage_ = kInitialStage;
  double progress_ = 0.0;

  int continuous_judge_count_ = 0;
};

class Stage3FallbackSwitchGuard {
 public:
  using Clock = std::chrono::steady_clock;

  struct Config {
    double min_stage2_progress = 68.0;
    std::chrono::milliseconds no_target_duration{300};
    std::chrono::milliseconds recent_purple_duration{500};
    std::chrono::milliseconds high_progress_no_target_probe_duration{60000};
    std::chrono::milliseconds stage2_no_target_force_duration{90000};
  };

  enum class TriggerReason {
    None,
    HighProgressRecentPurpleLostTarget,
    HighProgressLongNoTarget,
    Stage2LongNoTarget,
  };

  static const char *TriggerReasonTag(TriggerReason reason) {
    switch (reason) {
    case TriggerReason::HighProgressRecentPurpleLostTarget:
      return "stage3_fallback_high_progress_lost_target";
    case TriggerReason::HighProgressLongNoTarget:
      return "stage3_fallback_high_progress_long_no_target";
    case TriggerReason::Stage2LongNoTarget:
      return "stage3_fallback_stage2_long_no_target";
    case TriggerReason::None:
    default:
      return "none";
    }
  }

  struct UpdateInput {
    bool stage_judge_initialized = false;
    bool using_stage3_predictor = false;
    int current_stage = AerialRobotLaserLockJudge::kInitialStage;
    int laser_judge_class_id = -1;
    bool has_boxes = false;
    double progress = 0.0;
    Config config{};
    Clock::time_point now{};
  };

  bool Update(const UpdateInput &input) {
    if (!CanConsiderFallback_(input)) {
      Reset();
      return false;
    }

    UpdateProgressState_(input);

    const bool purple_observed =
        AerialRobotLaserLockJudge::IsPurpleClassId(input.laser_judge_class_id);
    if (purple_observed) {
      last_purple_seen_ = input.now;
      has_last_purple_seen_ = true;
      no_target_since_initialized_ = false;
      stage2_no_target_since_initialized_ = false;
      triggered_ = false;
      last_trigger_reason_ = TriggerReason::None;
      return false;
    }

    if (input.has_boxes) {
      // 最后一次有框识别不是紫色时，不能继续沿用此前的“最近紫色”。
      has_last_purple_seen_ = false;
      no_target_since_initialized_ = false;
      stage2_no_target_since_initialized_ = false;
      triggered_ = false;
      last_trigger_reason_ = TriggerReason::None;
      return false;
    }

    if (!stage2_no_target_since_initialized_) {
      stage2_no_target_since_ = input.now;
      stage2_no_target_since_initialized_ = true;
    }

    if (!no_target_since_initialized_) {
      no_target_since_ = input.now;
      no_target_since_initialized_ = true;
    }

    if (triggered_) {
      return false;
    }

    if (stage2_progress_reached_min_ &&
        HasRecentPurple_(input.now, input.config.recent_purple_duration) &&
        (input.now - no_target_since_) >= input.config.no_target_duration) {
      triggered_ = true;
      last_trigger_reason_ = TriggerReason::HighProgressRecentPurpleLostTarget;
      return true;
    }

    if (stage2_progress_reached_min_ &&
        input.config.high_progress_no_target_probe_duration.count() > 0 &&
        (input.now - stage2_no_target_since_) >=
            input.config.high_progress_no_target_probe_duration) {
      triggered_ = true;
      last_trigger_reason_ = TriggerReason::HighProgressLongNoTarget;
      return true;
    }

    if (!stage2_progress_reached_min_ &&
        input.config.stage2_no_target_force_duration.count() > 0 &&
        (input.now - stage2_no_target_since_) >=
            input.config.stage2_no_target_force_duration) {
      triggered_ = true;
      last_trigger_reason_ = TriggerReason::Stage2LongNoTarget;
      return true;
    }

    return false;
  }

  void Reset() {
    has_last_purple_seen_ = false;
    no_target_since_initialized_ = false;
    stage2_no_target_since_initialized_ = false;
    triggered_ = false;
    last_trigger_reason_ = TriggerReason::None;
    max_stage2_progress_ = 0.0;
    stage2_progress_reached_min_ = false;
  }

  void ResetNoTargetTimers() {
    has_last_purple_seen_ = false;
    no_target_since_initialized_ = false;
    stage2_no_target_since_initialized_ = false;
    triggered_ = false;
    last_trigger_reason_ = TriggerReason::None;
  }

  double MaxStage2Progress() const { return max_stage2_progress_; }
  TriggerReason LastTriggerReason() const { return last_trigger_reason_; }

 private:
  static bool CanConsiderFallback_(const UpdateInput &input) {
    return input.stage_judge_initialized && !input.using_stage3_predictor &&
           input.current_stage ==
               AerialRobotLaserLockJudge::kInitialStage + 1;
  }

  bool HasRecentPurple_(
      const Clock::time_point &now,
      const std::chrono::milliseconds &recent_purple_duration) const {
    return has_last_purple_seen_ &&
           (now - last_purple_seen_) <= recent_purple_duration;
  }

  void UpdateProgressState_(const UpdateInput &input) {
    max_stage2_progress_ = std::max(max_stage2_progress_, input.progress);
    if (max_stage2_progress_ >= input.config.min_stage2_progress) {
      stage2_progress_reached_min_ = true;
    }
  }

  bool has_last_purple_seen_ = false;
  Clock::time_point last_purple_seen_{};
  bool no_target_since_initialized_ = false;
  Clock::time_point no_target_since_{};
  bool stage2_no_target_since_initialized_ = false;
  Clock::time_point stage2_no_target_since_{};
  bool triggered_ = false;
  TriggerReason last_trigger_reason_ = TriggerReason::None;
  double max_stage2_progress_ = 0.0;
  bool stage2_progress_reached_min_ = false;
};

}  // namespace ImageRecognize
