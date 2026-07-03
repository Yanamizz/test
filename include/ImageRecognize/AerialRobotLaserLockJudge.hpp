/**
 * @file    include/ImageRecognize/AerialRobotLaserLockJudge.hpp
 * @brief   根据目标类别连续观测结果判断空中机器人激光锁定阶段。
 *
 * AerialRobotLaserLockJudge 将逐帧紫色照射观测转换为锁定进度显示，当前业务
 * 阶段由外部 TCP 边沿驱动。该模块只消费识别结果，不直接控制串口或扫描状态。
 */

#pragma once

#include <algorithm>
namespace ImageRecognize {

class AerialRobotLaserLockJudge {
 public:
  static constexpr int kPurpleClassId = 2;
  static constexpr int kInitialStage = 1;
  static constexpr int kStage12LastStage = 3;
  static constexpr int kStage3FirstStage = 4;
  static constexpr int kFinishedStage = 5;
  static constexpr double kJudgePeriodSeconds = 0.1;
  static constexpr double kProgressDecayPerSecond = 0.5;

  struct TickResult {
    int stage = kInitialStage;
    double progress = 0.0;
  };

  TickResult Tick100ms(bool illuminated) {
    TickResult result{};
    result.stage = stage_;
    result.progress = progress_;

    if (IsFinished()) {
      return result;
    }

    if (!illuminated) {
      progress_ = std::max(0.0, progress_ - kProgressDecayPerSecond * kJudgePeriodSeconds);
      ResetContinuousIllumination();
      result.stage = stage_;
      result.progress = progress_;
      return result;
    }

    ++continuous_judge_count_;
    const double progress_step =
        kProgressFirstTerm + static_cast<double>(continuous_judge_count_ - 1) * kProgressCommonDifference;
    progress_ = std::min(100.0, progress_ + progress_step);

    result.stage = stage_;
    result.progress = progress_;
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

  void SetStage(int stage, bool reset_progress = true) {
    const int clamped_stage = std::clamp(stage, kInitialStage, kFinishedStage);
    const bool changed = stage_ != clamped_stage;
    stage_ = clamped_stage;
    if (reset_progress || changed) {
      progress_ = 0.0;
      ResetContinuousIllumination();
    }
  }

  static bool IsPurpleClassId(int class_id) { return class_id == kPurpleClassId; }

  int Stage() const { return stage_; }
  bool IsFinished() const { return stage_ >= kFinishedStage; }
  bool UsesStage3Resources() const { return stage_ >= kStage3FirstStage; }
  double Progress() const { return progress_; }

  int CurrentThreshold() const { return stage_ == kInitialStage ? kFirstLockThreshold : kLaterLockThreshold; }

 private:
  static constexpr int kFirstLockThreshold = 50;
  static constexpr int kLaterLockThreshold = 100;
  static constexpr double kProgressFirstTerm = 0.6;
  static constexpr double kProgressCommonDifference = 0.6;

  void ResetContinuousIllumination() { continuous_judge_count_ = 0; }

  int stage_ = kInitialStage;
  double progress_ = 0.0;

  int continuous_judge_count_ = 0;
};

}  // namespace ImageRecognize
