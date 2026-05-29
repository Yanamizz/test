/**
 * @file    include/Tools/Stage3AutoScanBoundsController.hpp
 * @brief   收口 stage3 auto 扫描上下限学习、钳制与版本追踪。
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include "Tools/ScanController.hpp"

namespace Tools {

class Stage3AutoScanBoundsController {
public:
  struct AngleBounds {
    bool initialized = false;
    float min_yaw_deg = 0.0f;
    float max_yaw_deg = 0.0f;
    float min_pitch_deg = 0.0f;
    float max_pitch_deg = 0.0f;
  };

  bool IsAutoMode() const {
    static const bool auto_mode = []() {
      std::string mode = Params().stage3_scan_bounds_mode;
      std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });
      return mode == "AUTO";
    }();
    return auto_mode;
  }

  void UpdateFromStage12Target(float yaw_deg, float pitch_deg) {
    UpdateBounds_(yaw_deg, pitch_deg);
  }

  void ExpandForStage3Target(float yaw_deg, float pitch_deg) {
    if (!IsAutoMode() || !std::isfinite(yaw_deg) || !std::isfinite(pitch_deg)) {
      return;
    }

    AngleBounds bounds;
    if (!Snapshot(&bounds)) {
      return;
    }
    bounds = ClampToManual_(bounds);

    const auto limits = ManualBoundsLimits_();
    const float clamped_yaw =
        std::clamp(yaw_deg, limits.min_yaw_deg, limits.max_yaw_deg);
    const float clamped_pitch =
        std::clamp(pitch_deg, limits.min_pitch_deg, limits.max_pitch_deg);
    if (clamped_yaw < bounds.min_yaw_deg || clamped_yaw > bounds.max_yaw_deg ||
        clamped_pitch < bounds.min_pitch_deg ||
        clamped_pitch > bounds.max_pitch_deg) {
      UpdateBounds_(clamped_yaw, clamped_pitch);
    }
  }

  bool Snapshot(AngleBounds *bounds) const {
    if (bounds == nullptr) {
      return false;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    if (!bounds_.initialized) {
      return false;
    }
    *bounds = bounds_;
    return true;
  }

  std::uint64_t Version() const {
    return version_.load(std::memory_order_acquire);
  }

  ScanController::Config EffectiveControllerConfig() const {
    auto controller_config = MakeStage3ScanControllerConfig();
    if (!IsAutoMode()) {
      return controller_config;
    }

    AngleBounds bounds;
    if (!Snapshot(&bounds)) {
      return controller_config;
    }

    bounds = ClampToManual_(bounds);
    controller_config.min_yaw_deg = bounds.min_yaw_deg;
    controller_config.max_yaw_deg = bounds.max_yaw_deg;
    controller_config.min_pitch_deg = bounds.min_pitch_deg;
    controller_config.max_pitch_deg = bounds.max_pitch_deg;
    return controller_config;
  }

private:
  struct Limits {
    float min_yaw_deg = 0.0f;
    float max_yaw_deg = 0.0f;
    float min_pitch_deg = 0.0f;
    float max_pitch_deg = 0.0f;
  };

  static Limits ManualBoundsLimits_() {
    const auto controller_config = MakeStage3ScanControllerConfig();
    return {
        std::min(controller_config.min_yaw_deg, controller_config.max_yaw_deg),
        std::max(controller_config.min_yaw_deg, controller_config.max_yaw_deg),
        std::min(controller_config.min_pitch_deg,
                 controller_config.max_pitch_deg),
        std::max(controller_config.min_pitch_deg,
                 controller_config.max_pitch_deg)};
  }

  static AngleBounds ClampToManual_(AngleBounds bounds) {
    const auto limits = ManualBoundsLimits_();
    bounds.min_yaw_deg =
        std::clamp(bounds.min_yaw_deg, limits.min_yaw_deg, limits.max_yaw_deg);
    bounds.max_yaw_deg =
        std::clamp(bounds.max_yaw_deg, limits.min_yaw_deg, limits.max_yaw_deg);
    bounds.min_pitch_deg = std::clamp(bounds.min_pitch_deg, limits.min_pitch_deg,
                                      limits.max_pitch_deg);
    bounds.max_pitch_deg = std::clamp(bounds.max_pitch_deg, limits.min_pitch_deg,
                                      limits.max_pitch_deg);
    return bounds;
  }

  void UpdateBounds_(float yaw_deg, float pitch_deg) {
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg)) {
      return;
    }

    const auto limits = ManualBoundsLimits_();
    yaw_deg = std::clamp(yaw_deg, limits.min_yaw_deg, limits.max_yaw_deg);
    pitch_deg =
        std::clamp(pitch_deg, limits.min_pitch_deg, limits.max_pitch_deg);

    bool changed = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!bounds_.initialized) {
        bounds_.initialized = true;
        bounds_.min_yaw_deg = yaw_deg;
        bounds_.max_yaw_deg = yaw_deg;
        bounds_.min_pitch_deg = pitch_deg;
        bounds_.max_pitch_deg = pitch_deg;
        changed = true;
      } else {
        if (yaw_deg < bounds_.min_yaw_deg) {
          bounds_.min_yaw_deg = yaw_deg;
          changed = true;
        }
        if (yaw_deg > bounds_.max_yaw_deg) {
          bounds_.max_yaw_deg = yaw_deg;
          changed = true;
        }
        if (pitch_deg < bounds_.min_pitch_deg) {
          bounds_.min_pitch_deg = pitch_deg;
          changed = true;
        }
        if (pitch_deg > bounds_.max_pitch_deg) {
          bounds_.max_pitch_deg = pitch_deg;
          changed = true;
        }
      }
    }

    if (changed) {
      version_.fetch_add(1, std::memory_order_release);
    }
  }

  mutable std::mutex mutex_;
  AngleBounds bounds_{};
  std::atomic<std::uint64_t> version_{0};
};

} // namespace Tools
