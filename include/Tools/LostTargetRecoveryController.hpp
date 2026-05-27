/**
 * @file    include/Tools/LostTargetRecoveryController.hpp
 * @brief   收口丢目标后的 stage3 续行与回退扫描恢复流程。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "SerialTask/SerialRead.hpp"
#include "Tools/AngleUtils.hpp"
#include "Tools/StageRuntimeProfile.hpp"

namespace Tools {

struct LostTargetRecoveryCommand {
  float absolute_pitch = 0.0f;
  float absolute_yaw = 0.0f;
  float offset_pitch = 0.0f;
  float offset_yaw = 0.0f;
  float pitch_velocity = 0.0f;
  float yaw_velocity = 0.0f;
};

class LostTargetRecoveryController {
public:
  enum class PendingAction {
    None,
    ClearPendingSend,
    StartScanMode,
  };

  struct UpdateInput {
    bool using_stage3_predictor = false;
    bool has_matched_imu = false;
    SerialTask::EulerAngles matched_imu{};
    bool track_alive = false;
    bool track_has_box = false;
    bool enable_scan_mode = false;
    bool target_lost_since_initialized = false;
    std::chrono::steady_clock::time_point now{};
    std::chrono::steady_clock::time_point target_lost_since{};
    std::chrono::milliseconds scan_trigger_delay{0};
    StageLostTargetCoastProfile coast_profile{};
    float max_send_delta_deg = 0.0f;
    float pitch_abs_limit = 0.0f;
  };

  struct UpdateResult {
    bool coast_active = false;
    bool has_command = false;
    LostTargetRecoveryCommand command{};
    PendingAction pending_action = PendingAction::None;
  };

  struct ReacquireCheckInput {
    bool using_stage3_predictor = false;
    bool has_tracked_box = false;
    bool has_reacquire_angles = false;
    float tracked_absolute_pitch = 0.0f;
    float tracked_absolute_yaw = 0.0f;
    std::chrono::steady_clock::time_point now{};
    StageLostTargetCoastProfile coast_profile{};
  };

  void Reset() { state_ = State{}; }

  bool ShouldAcceptTrackedTarget(const ReacquireCheckInput &input) {
    if (!input.using_stage3_predictor || !state_.active) {
      return input.has_tracked_box;
    }
    if (!input.has_tracked_box) {
      state_.reacquire_confirm_started = false;
      return false;
    }
    if (!ReacquireGatePassed_(input)) {
      state_.reacquire_confirm_started = false;
      return false;
    }
    if (input.coast_profile.reacquire_confirm_ms <= 0) {
      FinishCurrentLoss_();
      return true;
    }
    if (!state_.reacquire_confirm_started) {
      state_.reacquire_confirm_started = true;
      state_.reacquire_confirm_start_ts = input.now;
      return false;
    }

    const auto reacquire_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            input.now - state_.reacquire_confirm_start_ts);
    if (reacquire_elapsed >=
        std::chrono::milliseconds(input.coast_profile.reacquire_confirm_ms)) {
      FinishCurrentLoss_();
      return true;
    }
    return false;
  }

  void PrepareStage3Coast(bool using_stage3_predictor, float absolute_pitch,
                          float absolute_yaw,
                          float measured_pitch_velocity_deg_per_sec,
                          float measured_yaw_velocity_deg_per_sec,
                          const StageLostTargetCoastProfile &coast_profile) {
    if (!using_stage3_predictor) {
      return;
    }

    state_.active = false;
    state_.prepared_for_current_loss = true;
    state_.coast_consumed_for_current_loss = false;
    state_.loss_confirm_started = false;
    state_.reacquire_confirm_started = false;
    state_.absolute_pitch = absolute_pitch;
    state_.absolute_yaw = absolute_yaw;

    const float yaw_direction =
        measured_yaw_velocity_deg_per_sec >= 0.0f ? 1.0f : -1.0f;
    const float pitch_direction =
        measured_pitch_velocity_deg_per_sec >= 0.0f ? 1.0f : -1.0f;
    state_.yaw_velocity =
        yaw_direction * static_cast<float>(coast_profile.yaw_speed_deg_per_sec);
    state_.pitch_velocity = pitch_direction *
                            static_cast<float>(
                                coast_profile.pitch_speed_deg_per_sec);
  }

  UpdateResult Update(const UpdateInput &input) {
    UpdateResult result{};

    const bool track_missing = input.using_stage3_predictor && !input.track_has_box;
    if (track_missing) {
      if (!state_.loss_confirm_started) {
        state_.loss_confirm_started = true;
        state_.loss_confirm_start_ts = input.now;
      }
    } else {
      state_.loss_confirm_started = false;
    }

    const bool loss_confirmed =
        track_missing &&
        (input.coast_profile.trigger_delay_ms <= 0 ||
         (state_.loss_confirm_started &&
          std::chrono::duration_cast<std::chrono::milliseconds>(
              input.now - state_.loss_confirm_start_ts) >=
              std::chrono::milliseconds(input.coast_profile.trigger_delay_ms)));

    const bool stage3_can_coast =
        input.using_stage3_predictor && input.has_matched_imu &&
        state_.prepared_for_current_loss && !state_.active &&
        !state_.coast_consumed_for_current_loss && loss_confirmed &&
        input.coast_profile.duration_ms > 0;
    if (stage3_can_coast) {
      state_.active = true;
      state_.coast_consumed_for_current_loss = true;
      state_.start_ts = input.now;
      state_.reacquire_confirm_started = false;
    }

    if (state_.active && !input.track_has_box) {
      state_.reacquire_confirm_started = false;
    }

    if (state_.active && input.has_matched_imu) {
      const auto coast_elapsed =
          std::chrono::duration<double>(input.now - state_.start_ts);
      const double coast_limit_sec =
          static_cast<double>(input.coast_profile.duration_ms) / 1000.0;
      if (coast_elapsed.count() < coast_limit_sec) {
        const float coast_abs_yaw =
            state_.absolute_yaw +
            state_.yaw_velocity * static_cast<float>(coast_elapsed.count());
        const float coast_abs_pitch =
            state_.absolute_pitch +
            state_.pitch_velocity * static_cast<float>(coast_elapsed.count());
        const float coast_offset_yaw =
            NormalizeDeltaDeg(coast_abs_yaw - input.matched_imu.yaw);
        const float coast_offset_pitch =
            NormalizeDeltaDeg(coast_abs_pitch - input.matched_imu.pitch);

        result.has_command = true;
        result.command.offset_yaw =
            std::clamp(coast_offset_yaw, -input.max_send_delta_deg,
                       input.max_send_delta_deg);
        result.command.offset_pitch =
            std::clamp(coast_offset_pitch, -input.pitch_abs_limit,
                       input.pitch_abs_limit);
        result.command.absolute_yaw =
            input.matched_imu.yaw + result.command.offset_yaw;
        result.command.absolute_pitch =
            input.matched_imu.pitch + result.command.offset_pitch;
        result.command.yaw_velocity = state_.yaw_velocity;
        result.command.pitch_velocity = state_.pitch_velocity;
      } else {
        state_.active = false;
      }
    }

    result.coast_active = state_.active;
    if (!state_.active) {
      if (input.enable_scan_mode && input.has_matched_imu && !input.track_alive &&
          input.target_lost_since_initialized &&
          (input.now - input.target_lost_since) >= input.scan_trigger_delay) {
        result.pending_action = PendingAction::StartScanMode;
      } else {
        result.pending_action = PendingAction::ClearPendingSend;
      }
    }

    return result;
  }

private:
  struct State {
    bool active = false;
    bool prepared_for_current_loss = false;
    bool coast_consumed_for_current_loss = false;
    bool loss_confirm_started = false;
    bool reacquire_confirm_started = false;
    float absolute_pitch = 0.0f;
    float absolute_yaw = 0.0f;
    float pitch_velocity = 0.0f;
    float yaw_velocity = 0.0f;
    std::chrono::steady_clock::time_point start_ts{};
    std::chrono::steady_clock::time_point loss_confirm_start_ts{};
    std::chrono::steady_clock::time_point reacquire_confirm_start_ts{};
  };

  bool ReacquireGatePassed_(const ReacquireCheckInput &input) const {
    if (input.coast_profile.reacquire_gate_deg <= 0.0) {
      return true;
    }
    if (!input.has_reacquire_angles) {
      return true;
    }

    const auto [predicted_pitch, predicted_yaw] =
        PredictedAbsoluteAnglesAt_(input.now);
    const float yaw_delta =
        NormalizeDeltaDeg(input.tracked_absolute_yaw - predicted_yaw);
    const float pitch_delta =
        NormalizeDeltaDeg(input.tracked_absolute_pitch - predicted_pitch);
    const float angle_distance_deg =
        std::sqrt(yaw_delta * yaw_delta + pitch_delta * pitch_delta);
    return angle_distance_deg <=
           static_cast<float>(input.coast_profile.reacquire_gate_deg);
  }

  std::pair<float, float>
  PredictedAbsoluteAnglesAt_(const std::chrono::steady_clock::time_point &now) const {
    if (!state_.active) {
      return {state_.absolute_pitch, state_.absolute_yaw};
    }

    const double elapsed_sec = std::max(
        0.0, std::chrono::duration<double>(now - state_.start_ts).count());
    return {state_.absolute_pitch +
                state_.pitch_velocity * static_cast<float>(elapsed_sec),
            state_.absolute_yaw +
                state_.yaw_velocity * static_cast<float>(elapsed_sec)};
  }

  void FinishCurrentLoss_() {
    state_.active = false;
    state_.prepared_for_current_loss = false;
    state_.coast_consumed_for_current_loss = false;
    state_.loss_confirm_started = false;
    state_.reacquire_confirm_started = false;
  }

  State state_{};
};

} // namespace Tools
