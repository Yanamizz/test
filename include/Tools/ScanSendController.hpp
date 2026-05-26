/**
 * @file    include/Tools/ScanSendController.hpp
 * @brief   收口 IMU 发送线程中的扫描发送状态机。
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
  }

  bool EnterOrStayScanMode(const TickConfig &config, ScanController *controller,
                           std::mutex *controller_mutex,
                           const Clock::time_point &now) {
    if (last_scan_mode_) {
      return false;
    }

    next_scan_send_time_ = now;
    last_scan_mode_ = true;
    scan_waiting_at_origin_ = true;
    scan_origin_deadline_ = now + config.scan_origin_hold_duration;
    if (controller != nullptr && controller_mutex != nullptr) {
      std::lock_guard<std::mutex> lk(*controller_mutex);
      controller->SetConfig(config.controller_config);
    }
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
  }

private:
  static double scan_send_hz_(const TickConfig &config) {
    return config.scan_send_hz > 0.0 ? config.scan_send_hz : 1.0;
  }

  bool last_scan_mode_ = false;
  bool scan_waiting_at_origin_ = false;
  Clock::time_point next_scan_send_time_ = Clock::now();
  Clock::time_point scan_origin_deadline_ = Clock::now();
};

} // namespace Tools
