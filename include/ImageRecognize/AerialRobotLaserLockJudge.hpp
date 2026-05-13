#pragma once

#include <algorithm>

namespace ImageRecognize {

class AerialRobotLaserLockJudge {
 public:
  static constexpr int kPurpleClassId = 2;
  static constexpr int kInitialStage = 1;
  static constexpr int kFinishedStage = 3;
  static constexpr double kDefaultDeltaSeconds = 0.1;
  static constexpr double kJudgePeriodSeconds = 0.1;
  static constexpr double kProgressDecayPerSecond = 0.5;

  int Update(int class_id) { return Update(class_id, kDefaultDeltaSeconds); }

  int Update(int class_id, double delta_seconds) {
    if (delta_seconds <= 0.0) {
      return stage_;
    }

    const bool illuminated = IsPurpleClassId(class_id);
    double remaining_seconds = delta_seconds;

    while (remaining_seconds > 0.0) {
      if (IsFinished()) {
        break;
      }

      if (!illuminated) {
        progress_ = std::max(
            0.0, progress_ - kProgressDecayPerSecond * remaining_seconds);
        ResetContinuousIllumination();
        break;
      }

      const double need_seconds = kJudgePeriodSeconds - judge_accumulator_;
      const double used_seconds = std::min(remaining_seconds, need_seconds);
      judge_accumulator_ += used_seconds;
      remaining_seconds -= used_seconds;

      if (judge_accumulator_ < kJudgePeriodSeconds) {
        continue;
      }

      judge_accumulator_ = 0.0;
      ++continuous_judge_count_;
      progress_ = std::min(
          100.0, progress_ + static_cast<double>(continuous_judge_count_));

      if (progress_ >= CurrentThreshold()) {
        AdvanceStage();
      }
    }

    return stage_;
  }

  void Reset() {
    stage_ = kInitialStage;
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
    judge_accumulator_ = 0.0;
    continuous_judge_count_ = 0;
  }

  int stage_ = kInitialStage;
  double progress_ = 0.0;

  double judge_accumulator_ = 0.0;
  int continuous_judge_count_ = 0;
};

}  // namespace ImageRecognize
