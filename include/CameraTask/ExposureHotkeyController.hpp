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
  enum class ExposureMode {
    Stage12,
    Stage3,
  };

  explicit ExposureHotkeyController(double initial_exposure_time_us = 1000.0)
      : stage12_exposure_time_us_(NormalizeExposure(initial_exposure_time_us)),
        stage3_exposure_time_us_(NormalizeExposure(initial_exposure_time_us)) {}

  void SetExposureTimes(double stage12_exposure_time_us,
                        double stage3_exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    stage12_exposure_time_us_ = NormalizeExposure(stage12_exposure_time_us);
    stage3_exposure_time_us_ = NormalizeExposure(stage3_exposure_time_us);
    active_mode_ = ExposureMode::Stage12;
    edit_mode_ = ExposureMode::Stage12;
    update_pending_ = false;
  }

  void SetExposureTime(double exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    ExposureForMode(active_mode_) = NormalizeExposure(exposure_time_us);
    update_pending_ = false;
  }

  double GetExposureTime() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return ExposureForMode(active_mode_);
  }

  double GetEditingExposureTime() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return ExposureForMode(edit_mode_);
  }

  double GetStage12ExposureTime() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stage12_exposure_time_us_;
  }

  double GetStage3ExposureTime() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stage3_exposure_time_us_;
  }

  ExposureMode GetActiveMode() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return active_mode_;
  }

  ExposureMode GetEditMode() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return edit_mode_;
  }

  static const char *ModeName(ExposureMode mode) {
    return mode == ExposureMode::Stage12 ? "stage1/2" : "stage3";
  }

  void RequestExposureTime(double exposure_time_us) {
    RequestEditingExposureTime(exposure_time_us);
  }

  void RequestEditingExposureTime(double exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    ExposureForMode(edit_mode_) = NormalizeExposure(exposure_time_us);
    if (edit_mode_ == active_mode_) {
      update_pending_ = true;
    }
  }

  void SetActiveMode(ExposureMode mode) {
    std::lock_guard<std::mutex> lk(mutex_);
    active_mode_ = mode;
    edit_mode_ = mode;
    update_pending_ = true;
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
    if (ch == 'd') {
      ToggleEditMode();
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

  static double NormalizeExposure(double exposure_time_us) {
    return std::max(1.0, exposure_time_us);
  }

  double &ExposureForMode(ExposureMode mode) {
    return mode == ExposureMode::Stage12 ? stage12_exposure_time_us_
                                         : stage3_exposure_time_us_;
  }

  double ExposureForMode(ExposureMode mode) const {
    return mode == ExposureMode::Stage12 ? stage12_exposure_time_us_
                                         : stage3_exposure_time_us_;
  }

  void RequestDelta(double delta_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    double &editing_exposure_time_us = ExposureForMode(edit_mode_);
    editing_exposure_time_us =
        NormalizeExposure(editing_exposure_time_us + delta_us);
    if (edit_mode_ == active_mode_) {
      update_pending_ = true;
    }
  }

  void ToggleEditMode() {
    std::lock_guard<std::mutex> lk(mutex_);
    edit_mode_ = edit_mode_ == ExposureMode::Stage12 ? ExposureMode::Stage3
                                                     : ExposureMode::Stage12;
    std::cout << "[Camera] exposure edit mode: " << ModeName(edit_mode_)
              << " exposure_time_us=" << ExposureForMode(edit_mode_)
              << std::endl;
  }

  bool TakePendingExposureTime(double *exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!update_pending_) {
      return false;
    }

    *exposure_time_us = ExposureForMode(active_mode_);
    update_pending_ = false;
    return true;
  }

  void ConfirmAppliedExposureTime(double exposure_time_us) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!update_pending_) {
      ExposureForMode(active_mode_) = NormalizeExposure(exposure_time_us);
    }
  }

  mutable std::mutex mutex_;
  double stage12_exposure_time_us_;
  double stage3_exposure_time_us_;
  ExposureMode active_mode_ = ExposureMode::Stage12;
  ExposureMode edit_mode_ = ExposureMode::Stage12;
  bool update_pending_ = false;
};

} // namespace CameraTask
