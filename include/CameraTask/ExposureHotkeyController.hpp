#pragma once

#include <algorithm>
#include <cctype>
#include <iostream>
#include <mutex>

#include "CameraTask/GetImage.hpp"
#include "ImageRecognize/ImageShow.hpp"

namespace CameraTask {

class ExposureHotkeyController {
public:
  explicit ExposureHotkeyController(double initial_exposure_time_us = 1000.0)
      : requested_exposure_time_us_(std::max(1.0, initial_exposure_time_us)) {}

  void SetExposureTime(double exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    requested_exposure_time_us_ = std::max(1.0, exposure_time_us);
    update_pending_ = false;
  }

  bool HandleGuiKey(int key) {
    if (key < 0) {
      return false;
    }

    const int normalized_key = key & 0xff;
    const char ch = static_cast<char>(
        std::tolower(static_cast<unsigned char>(normalized_key)));
    if (ch == 'w' || ch == 's') {
      RequestDelta(ch == 'w' ? kExposureStepUs : -kExposureStepUs);
      return false;
    }

    return ImageRecognize::ImageShow::IsExitKey(key);
  }

  void ApplyPendingChange(GalaxyCamera *camera) {
    double requested_exposure_time_us = 0.0;
    if (!TakePendingExposureTime(&requested_exposure_time_us)) {
      return;
    }

    if (!camera->applyExposureTime(requested_exposure_time_us)) {
      std::cerr << "[Camera] failed to apply exposure_time_us="
                << requested_exposure_time_us << std::endl;
      return;
    }

    ConfirmAppliedExposureTime(camera->getExposureTime());
  }

private:
  static constexpr double kExposureStepUs = 100.0;

  void RequestDelta(double delta_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    requested_exposure_time_us_ =
        std::max(1.0, requested_exposure_time_us_ + delta_us);
    update_pending_ = true;
  }

  bool TakePendingExposureTime(double *exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!update_pending_) {
      return false;
    }

    *exposure_time_us = requested_exposure_time_us_;
    update_pending_ = false;
    return true;
  }

  void ConfirmAppliedExposureTime(double exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!update_pending_) {
      requested_exposure_time_us_ = exposure_time_us;
    }
  }

  std::mutex mutex_;
  double requested_exposure_time_us_;
  bool update_pending_ = false;
};

} // namespace CameraTask
