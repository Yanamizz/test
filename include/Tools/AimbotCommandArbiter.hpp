/**
 * @file    include/Tools/AimbotCommandArbiter.hpp
 * @brief   收口目标控制、扫描和清空发送状态之间的线程仲裁。
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "Tools/AimbotCommand.hpp"

namespace Tools {

class AimbotCommandArbiter {
public:
  void NotifyAll() { cv_.notify_all(); }

  void ClearPendingSend() {
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      should_notify =
          has_pending_send_ || send_is_scan_.load(std::memory_order_acquire);
      send_is_scan_.store(false, std::memory_order_release);
      has_pending_send_ = false;
    }
    if (should_notify) {
      cv_.notify_one();
    }
  }

  void StopScanModeKeepPendingSend() {
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      should_notify = send_is_scan_.load(std::memory_order_acquire);
      send_is_scan_.store(false, std::memory_order_release);
    }
    if (should_notify) {
      cv_.notify_one();
    }
  }

  void StorePendingSend(const AimbotSendCommand &command) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      pending_send_ = command;
      send_is_scan_.store(false, std::memory_order_release);
      has_pending_send_ = true;
    }
    cv_.notify_one();
  }

  bool TakePendingSend(AimbotSendCommand *command) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_pending_send_) {
      return false;
    }

    if (command != nullptr) {
      *command = pending_send_;
    }
    has_pending_send_ = false;
    return true;
  }

  void StartScanMode() {
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      should_notify =
          has_pending_send_ || !send_is_scan_.load(std::memory_order_acquire);
      has_pending_send_ = false;
      send_is_scan_.store(true, std::memory_order_release);
    }
    if (should_notify) {
      cv_.notify_one();
    }
  }

  bool ScanMode() const {
    return send_is_scan_.load(std::memory_order_acquire);
  }

  void WaitForNormalWork(const std::atomic<bool> &running,
                         std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_for(lk, timeout, [&]() {
      return !running.load(std::memory_order_acquire) || has_pending_send_ ||
             send_is_scan_.load(std::memory_order_acquire);
    });
  }

  void WaitForScanStateChangeFor(const std::atomic<bool> &running,
                                 const std::atomic<bool> &target_visible,
                                 std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_for(lk, timeout, [&]() {
      return !running.load(std::memory_order_acquire) ||
             !send_is_scan_.load(std::memory_order_acquire) ||
             has_pending_send_ ||
             target_visible.load(std::memory_order_acquire);
    });
  }

  void WaitUntilNextScanSend(
      const std::atomic<bool> &running,
      const std::atomic<bool> &target_visible,
      std::chrono::steady_clock::time_point time) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_until(lk, time, [&]() {
      return !running.load(std::memory_order_acquire) ||
             !send_is_scan_.load(std::memory_order_acquire) ||
             has_pending_send_ ||
             target_visible.load(std::memory_order_acquire);
    });
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  AimbotSendCommand pending_send_{};
  bool has_pending_send_ = false;
  std::atomic<bool> send_is_scan_{false};
};

} // namespace Tools
