/**
 * @file    include/Tools/ScanSendController.hpp
 * @brief   收口 IMU 发送线程中的扫描发送状态机。
 *
 * ScanSendController 管理进入扫描、原点保持、下一次扫描发送时间、扫描命令构造
 * 和退出扫描时的状态清理。它把发送线程中的扫描时序从主循环剥离出来，但不负责
 * 生成具体 yaw/pitch 轨迹，轨迹由 ScanController 提供。
 */

#pragma once

#include <chrono>

#include "SerialTask/SerialRead.hpp"
#include "Tools/ScanController.hpp"

namespace Tools {

class ScanSendController {
public:
  using Clock = std::chrono::steady_clock;

  struct TickConfig {
    double scan_send_hz = 1.0;
    std::chrono::milliseconds scan_origin_hold_duration{0};
    ScanController::Config controller_config{};
  };

  enum class StepStatus {
    ReadyToSend,
    WaitForNextSend,
    NeedLatestImu,
  };

  struct StepResult {
    StepStatus status = StepStatus::NeedLatestImu;
    Clock::time_point wait_until{};
  };

  void Reset() {
    last_scan_mode_ = false;
    scan_waiting_at_origin_ = false;
    next_scan_send_time_ = Clock::now();
    scan_origin_deadline_ = Clock::now();
    has_active_tick_config_ = false;
    resume_from_last_scan_position_ = false;
  }

  bool EnterOrStayScanMode(const TickConfig &config, ScanController *controller,
                           std::mutex *controller_mutex,
                           const Clock::time_point &now) {
    if (last_scan_mode_) {
      if (controller != nullptr && controller_mutex != nullptr &&
          (!has_active_tick_config_ ||
           !SameTickConfig_(active_tick_config_, config))) {
        std::lock_guard<std::mutex> lk(*controller_mutex);
        controller->SetConfigPreserveProgress(config.controller_config);
        active_tick_config_ = config;
        has_active_tick_config_ = true;
      }
      return false;
    }

    next_scan_send_time_ = now;
    last_scan_mode_ = true;
    scan_waiting_at_origin_ = !resume_from_last_scan_position_;
    scan_origin_deadline_ = now + config.scan_origin_hold_duration;
    if (controller != nullptr && controller_mutex != nullptr) {
      std::lock_guard<std::mutex> lk(*controller_mutex);
      if (resume_from_last_scan_position_) {
        controller->SetConfigPreserveProgress(config.controller_config);
      } else {
        controller->SetConfig(config.controller_config);
      }
    }
    resume_from_last_scan_position_ = false;
    active_tick_config_ = config;
    has_active_tick_config_ = true;
    return true;
  }

  StepResult Step(const TickConfig &config, const Clock::time_point &now) const {
    if (now < next_scan_send_time_) {
      return {StepStatus::WaitForNextSend, next_scan_send_time_};
    }
    return {StepStatus::ReadyToSend, now};
  }

  ScanCommand BuildCommand(ScanController *controller, std::mutex *controller_mutex,
                           const SerialTask::EulerAngles &latest_imu) const {
    ScanCommand scan_command{};
    if (controller == nullptr || controller_mutex == nullptr) {
      return scan_command;
    }

    std::lock_guard<std::mutex> lk(*controller_mutex);
    if (scan_waiting_at_origin_) {
      return controller->BuildOriginCommand(latest_imu.yaw, latest_imu.pitch);
    }
    return controller->BuildCommand(latest_imu.yaw, latest_imu.pitch);
  }

  void FinishSend(const TickConfig &config, ScanController *controller,
                  std::mutex *controller_mutex,
                  const Clock::time_point &now) {
    if (scan_waiting_at_origin_ && now >= scan_origin_deadline_) {
      scan_waiting_at_origin_ = false;
      if (controller != nullptr && controller_mutex != nullptr) {
        std::lock_guard<std::mutex> lk(*controller_mutex);
        controller->Reset();
      }
    }
    const auto scan_send_interval =
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / scan_send_hz_(config)));
    next_scan_send_time_ = now + scan_send_interval;
  }

  void ExitScanMode() {
    last_scan_mode_ = false;
    scan_waiting_at_origin_ = false;
    next_scan_send_time_ = Clock::now();
    has_active_tick_config_ = false;
  }

  void ExitScanModeAndResumeNextEntry() {
    ExitScanMode();
    resume_from_last_scan_position_ = true;
  }

  void ClearResumeNextEntry() { resume_from_last_scan_position_ = false; }

private:
  static bool SameControllerConfig_(const ScanController::Config &a,
                                    const ScanController::Config &b) {
    return a.min_pitch_deg == b.min_pitch_deg &&
           a.max_pitch_deg == b.max_pitch_deg &&
           a.min_yaw_deg == b.min_yaw_deg && a.max_yaw_deg == b.max_yaw_deg &&
           a.yaw_step_deg_per_tick == b.yaw_step_deg_per_tick &&
           a.tick_rate_hz == b.tick_rate_hz &&
           a.pitch_wavelength_percent == b.pitch_wavelength_percent &&
           a.pitch_amplitude_percent == b.pitch_amplitude_percent;
  }

  static bool SameTickConfig_(const TickConfig &a, const TickConfig &b) {
    return a.scan_send_hz == b.scan_send_hz &&
           a.scan_origin_hold_duration == b.scan_origin_hold_duration &&
           SameControllerConfig_(a.controller_config, b.controller_config);
  }

  static double scan_send_hz_(const TickConfig &config) {
    return config.scan_send_hz > 0.0 ? config.scan_send_hz : 1.0;
  }

  bool last_scan_mode_ = false;
  bool scan_waiting_at_origin_ = false;
  bool has_active_tick_config_ = false;
  bool resume_from_last_scan_position_ = false;
  TickConfig active_tick_config_{};
  Clock::time_point next_scan_send_time_ = Clock::now();
  Clock::time_point scan_origin_deadline_ = Clock::now();
};

} // namespace Tools
