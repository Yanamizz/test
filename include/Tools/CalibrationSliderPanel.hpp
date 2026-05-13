#pragma once

#include <algorithm>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraTask/ExposureHotkeyController.hpp"
#include "Tools/LaserAngleCalculate.hpp"

namespace Tools {

class CalibrationSliderPanel {
public:
  static void Show(CameraTask::ExposureHotkeyController *exposure_controller) {
    exposure_controller_ = exposure_controller;
    EnsureInitialized();
    DrawPanel();
  }

private:
  static constexpr const char *kWindowName = "Calibration Sliders";
  static constexpr const char *kNearHeightTrackbar = "near target height";
  static constexpr const char *kFarHeightTrackbar = "far target height";
  static constexpr const char *kFixedYawTrackbar = "fixed yaw x0.01";
  static constexpr const char *kFixedPitchTrackbar = "fixed pitch x0.01";
  static constexpr const char *kExposureTrackbar = "exposure x100us";
  static constexpr int kHeightScale = 100000;
  static constexpr int kMinHeightTicks = 5500;
  static constexpr int kMaxHeightTicks = 7000;
  static constexpr int kHeightSliderMax = kMaxHeightTicks - kMinHeightTicks;
  static constexpr int kOffsetScale = 100;
  static constexpr int kMinOffsetTicks = -100;
  static constexpr int kMaxOffsetTicks = 100;
  static constexpr int kOffsetSliderMax = kMaxOffsetTicks - kMinOffsetTicks;
  static constexpr int kExposureMinUs = 100;
  static constexpr int kExposureMaxUs = 10000;
  static constexpr int kExposureStepUs = 100;
  static constexpr int kExposureSliderMax =
      (kExposureMaxUs - kExposureMinUs) / kExposureStepUs;

  static int ToSliderValue(float height_m) {
    const int ticks = static_cast<int>(height_m * kHeightScale + 0.5f);
    return std::clamp(ticks - kMinHeightTicks, 0, kHeightSliderMax);
  }

  static float ToHeightMeters(int slider_value) {
    const int ticks =
        kMinHeightTicks + std::clamp(slider_value, 0, kHeightSliderMax);
    return static_cast<float>(ticks) / static_cast<float>(kHeightScale);
  }

  static int ToOffsetSliderValue(float offset_deg) {
    const int ticks = static_cast<int>(offset_deg * kOffsetScale +
                                      (offset_deg >= 0.0f ? 0.5f : -0.5f));
    return std::clamp(ticks - kMinOffsetTicks, 0, kOffsetSliderMax);
  }

  static float ToOffsetDeg(int slider_value) {
    const int ticks =
        kMinOffsetTicks + std::clamp(slider_value, 0, kOffsetSliderMax);
    return static_cast<float>(ticks) / static_cast<float>(kOffsetScale);
  }

  static int ToExposureSliderValue(double exposure_us) {
    const int ticks =
        static_cast<int>((exposure_us - kExposureMinUs) / kExposureStepUs +
                         0.5);
    return std::clamp(ticks, 0, kExposureSliderMax);
  }

  static double ToExposureUs(int slider_value) {
    return static_cast<double>(
        kExposureMinUs +
        std::clamp(slider_value, 0, kExposureSliderMax) * kExposureStepUs);
  }

  static void EnsureInitialized() {
    if (initialized_) {
      return;
    }

    const auto heights = DistanceCalculator::GetCalibrationTargetHeights();
    near_height_slider_ =
        ToSliderValue(heights.near_calibration_target_height);
    far_height_slider_ = ToSliderValue(heights.far_calibration_target_height);
    const auto offsets = LaserAngleCalculator::GetFixedOffsets();
    fixed_offset_yaw_slider_ =
        ToOffsetSliderValue(offsets.fixed_offset_yaw);
    fixed_offset_pitch_slider_ =
        ToOffsetSliderValue(offsets.fixed_offset_pitch);
    if (exposure_controller_ != nullptr) {
      exposure_slider_ =
          ToExposureSliderValue(exposure_controller_->GetEditingExposureTime());
      exposure_slider_mode_ = exposure_controller_->GetEditMode();
    }

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar(kNearHeightTrackbar, kWindowName, nullptr,
                       kHeightSliderMax, OnTrackbar);
    cv::createTrackbar(kFarHeightTrackbar, kWindowName, nullptr,
                       kHeightSliderMax, OnTrackbar);
    cv::createTrackbar(kFixedYawTrackbar, kWindowName, nullptr,
                       kOffsetSliderMax, OnTrackbar);
    cv::createTrackbar(kFixedPitchTrackbar, kWindowName, nullptr,
                       kOffsetSliderMax, OnTrackbar);
    cv::createTrackbar(kExposureTrackbar, kWindowName, nullptr,
                       kExposureSliderMax, OnTrackbar);
    cv::setTrackbarPos(kNearHeightTrackbar, kWindowName, near_height_slider_);
    cv::setTrackbarPos(kFarHeightTrackbar, kWindowName, far_height_slider_);
    cv::setTrackbarPos(kFixedYawTrackbar, kWindowName,
                       fixed_offset_yaw_slider_);
    cv::setTrackbarPos(kFixedPitchTrackbar, kWindowName,
                       fixed_offset_pitch_slider_);
    cv::setTrackbarPos(kExposureTrackbar, kWindowName, exposure_slider_);

    initialized_ = true;
    ApplySliderValues(false);
  }

  static void OnTrackbar(int, void *) {
    if (!initialized_) {
      return;
    }

    ApplySliderValues(true);
  }

  static void ApplySliderValues(bool allow_exposure_update) {
    ReadSliderValues();
    DistanceCalculator::SetCalibrationTargetHeights(
        ToHeightMeters(near_height_slider_), ToHeightMeters(far_height_slider_));
    LaserAngleCalculator::SetFixedOffsets(
        ToOffsetDeg(fixed_offset_yaw_slider_),
        ToOffsetDeg(fixed_offset_pitch_slider_));

    if (allow_exposure_update && exposure_controller_ != nullptr) {
      exposure_controller_->RequestEditingExposureTime(
          ToExposureUs(exposure_slider_));
    }
  }

  static void ReadSliderValues() {
    if (!initialized_) {
      return;
    }

    near_height_slider_ =
        cv::getTrackbarPos(kNearHeightTrackbar, kWindowName);
    far_height_slider_ =
        cv::getTrackbarPos(kFarHeightTrackbar, kWindowName);
    fixed_offset_yaw_slider_ =
        cv::getTrackbarPos(kFixedYawTrackbar, kWindowName);
    fixed_offset_pitch_slider_ =
        cv::getTrackbarPos(kFixedPitchTrackbar, kWindowName);
    exposure_slider_ =
        cv::getTrackbarPos(kExposureTrackbar, kWindowName);
  }

  static void DrawPanel() {
    SyncExposureSliderFromController();
    const auto heights = DistanceCalculator::GetCalibrationTargetHeights();
    const auto offsets = LaserAngleCalculator::GetFixedOffsets();
    const double exposure_us =
        exposure_controller_ != nullptr
            ? exposure_controller_->GetEditingExposureTime()
            : ToExposureUs(exposure_slider_);
    const double stage12_exposure_us =
        exposure_controller_ != nullptr
            ? exposure_controller_->GetStage12ExposureTime()
            : ToExposureUs(exposure_slider_);
    const double stage3_exposure_us =
        exposure_controller_ != nullptr
            ? exposure_controller_->GetStage3ExposureTime()
            : ToExposureUs(exposure_slider_);
    const auto edit_mode =
        exposure_controller_ != nullptr
            ? exposure_controller_->GetEditMode()
            : CameraTask::ExposureHotkeyController::ExposureMode::Stage12;
    const auto active_mode =
        exposure_controller_ != nullptr
            ? exposure_controller_->GetActiveMode()
            : CameraTask::ExposureHotkeyController::ExposureMode::Stage12;
    cv::Mat panel(266, 560, CV_8UC3, cv::Scalar(28, 30, 32));
    cv::putText(panel,
                "near_calibration_target_height: " +
                    cv::format("%.5f m",
                               heights.near_calibration_target_height),
                {12, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "far_calibration_target_height:  " +
                    cv::format("%.5f m",
                               heights.far_calibration_target_height),
                {12, 62}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "fixed_offset_yaw:              " +
                    cv::format("%.2f deg", offsets.fixed_offset_yaw),
                {12, 94}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "fixed_offset_pitch:            " +
                    cv::format("%.2f deg", offsets.fixed_offset_pitch),
                {12, 126}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "editing_exposure_time_us:      " +
                    cv::format("%.0f us", exposure_us),
                {12, 158}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_edit_mode:            " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        edit_mode)),
                {12, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_active_mode:          " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        active_mode)),
                {12, 222}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "stage1/2 " + cv::format("%.0f", stage12_exposure_us) +
                    " us  stage3 " + cv::format("%.0f", stage3_exposure_us) +
                    " us",
                {12, 252}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(170, 190, 200), 1, cv::LINE_AA);
    cv::imshow(kWindowName, panel);
  }

  static void SyncExposureSliderFromController() {
    if (!initialized_ || exposure_controller_ == nullptr) {
      return;
    }

    const auto edit_mode = exposure_controller_->GetEditMode();
    const int desired_slider =
        ToExposureSliderValue(exposure_controller_->GetEditingExposureTime());
    if (edit_mode == exposure_slider_mode_ &&
        desired_slider == exposure_slider_) {
      return;
    }

    exposure_slider_mode_ = edit_mode;
    exposure_slider_ = desired_slider;
    cv::setTrackbarPos(kExposureTrackbar, kWindowName, exposure_slider_);
  }

  inline static bool initialized_ = false;
  inline static int near_height_slider_ = 0;
  inline static int far_height_slider_ = 0;
  inline static int fixed_offset_yaw_slider_ = 0;
  inline static int fixed_offset_pitch_slider_ = 0;
  inline static int exposure_slider_ = ToExposureSliderValue(1000.0);
  inline static CameraTask::ExposureHotkeyController::ExposureMode
      exposure_slider_mode_ =
          CameraTask::ExposureHotkeyController::ExposureMode::Stage12;
  inline static CameraTask::ExposureHotkeyController *exposure_controller_ =
      nullptr;
};

} // namespace Tools
