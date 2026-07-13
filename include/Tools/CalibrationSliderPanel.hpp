/**
 * @file    include/Tools/CalibrationSliderPanel.hpp
 * @brief   OpenCV 可视化调参面板，用于曝光、目标尺寸和阵营筛选。
 *
 * 该面板在调试显示开启时提供独立窗口，允许现场切换 stage1/2 与 stage3
 * 的编辑对象，并通过滑块调整当前 stage 的激光目标垂直修正、曝光时间
 * 和目标阵营模式。距离标定参数仍集中保存在 LaserAngleCalculate.hpp 的
 * 调参区，避免面板误改距离模型。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraTask/ExposureHotkeyController.hpp"
#include "ImageRecognize/TargetClassFilter.hpp"
#include "Tools/LaserAngleCalculate.hpp"

namespace Tools {

class CalibrationSliderPanel {
public:
  static void Show(CameraTask::ExposureHotkeyController *exposure_controller,
                   ImageRecognize::TargetCampModeController
                       *target_camp_mode_controller = nullptr) {
    exposure_controller_ = exposure_controller;
    target_camp_mode_controller_ = target_camp_mode_controller;
    EnsureInitialized();
    DrawPanel();
  }

private:
  static constexpr const char *kWindowName = "Calibration Sliders";
  static constexpr const char *kLaserTrimTrackbar = "laser trim mm";
  static constexpr const char *kExposureTrackbar = "exposure x100us";
  static constexpr const char *kTargetCampTrackbar =
      "target camp 0R 1B 2All";
  static constexpr int kLaserTrimMinMm = -50;
  static constexpr int kLaserTrimMaxMm = 50;
  static constexpr int kLaserTrimSliderMax =
      kLaserTrimMaxMm - kLaserTrimMinMm;
  static constexpr int kExposureMinUs = 100;
  static constexpr int kExposureMaxUs = 10000;
  static constexpr int kExposureStepUs = 100;
  static constexpr int kExposureSliderMax =
      (kExposureMaxUs - kExposureMinUs) / kExposureStepUs;

  static int ToLaserTrimSliderValue(float trim_m) {
    const int trim_mm = static_cast<int>(trim_m * 1000.0f +
                                         (trim_m >= 0.0f ? 0.5f : -0.5f));
    return std::clamp(trim_mm - kLaserTrimMinMm, 0,
                      kLaserTrimSliderMax);
  }

  static float ToLaserTrimMeters(int slider_value) {
    const int trim_mm = kLaserTrimMinMm +
                        std::clamp(slider_value, 0, kLaserTrimSliderMax);
    return static_cast<float>(trim_mm) / 1000.0f;
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

  static CalibrationStage ToCalibrationStage(
      CameraTask::ExposureHotkeyController::ExposureMode mode) {
    return mode == CameraTask::ExposureHotkeyController::ExposureMode::Stage3
               ? CalibrationStage::Stage3
               : CalibrationStage::Stage12;
  }

  static CalibrationStage CurrentEditCalibrationStage() {
    if (exposure_controller_ == nullptr) {
      return CalibrationStage::Stage12;
    }
    return ToCalibrationStage(exposure_controller_->GetEditMode());
  }

  static const char *CalibrationStageName(CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? "stage3" : "stage1/2";
  }

  static void EnsureInitialized() {
    if (initialized_) {
      return;
    }

    if (exposure_controller_ != nullptr) {
      exposure_slider_ =
          ToExposureSliderValue(exposure_controller_->GetEditingExposureTime());
      exposure_slider_mode_ = exposure_controller_->GetEditMode();
    }

    SyncStageSlidersFromStage(
        ToCalibrationStage(exposure_slider_mode_), false);

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar(kLaserTrimTrackbar, kWindowName, nullptr,
                       kLaserTrimSliderMax, OnTrackbar);
    cv::createTrackbar(kExposureTrackbar, kWindowName, nullptr,
                       kExposureSliderMax, OnTrackbar);
    cv::createTrackbar(kTargetCampTrackbar, kWindowName, nullptr, 2,
                       OnTrackbar);
    cv::setTrackbarPos(kLaserTrimTrackbar, kWindowName, laser_trim_slider_);
    cv::setTrackbarPos(kExposureTrackbar, kWindowName, exposure_slider_);
    SyncTargetCampSliderFromController(true);

    initialized_ = true;
    ApplySliderValues(false);
  }

  static void OnTrackbar(int, void *) {
    if (!initialized_ || syncing_sliders_) {
      return;
    }

    ApplySliderValues(true);
  }

  static void ApplySliderValues(bool allow_exposure_update) {
    ReadSliderValues();
    const auto stage = CurrentEditCalibrationStage();
    DistanceCalculator::SetLaserTargetVerticalTrimMeters(
        stage, ToLaserTrimMeters(laser_trim_slider_));

    if (allow_exposure_update && exposure_controller_ != nullptr) {
      exposure_controller_->RequestEditingExposureTime(
          ToExposureUs(exposure_slider_));
    }

    if (target_camp_mode_controller_ != nullptr) {
      target_camp_mode_controller_->SetModeIndex(target_camp_slider_);
    }
  }

  static void ReadSliderValues() {
    if (!initialized_) {
      return;
    }

    laser_trim_slider_ = cv::getTrackbarPos(kLaserTrimTrackbar, kWindowName);
    exposure_slider_ =
        cv::getTrackbarPos(kExposureTrackbar, kWindowName);
    target_camp_slider_ =
        cv::getTrackbarPos(kTargetCampTrackbar, kWindowName);
  }

  static void DrawPanel() {
    SyncExposureSliderFromController();
    SyncTargetCampSliderFromController(false);
    const auto edit_stage = CurrentEditCalibrationStage();
    const float laser_trim_m =
        DistanceCalculator::GetLaserTargetVerticalTrimMeters(edit_stage);
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
    const auto target_camp_mode =
        target_camp_mode_controller_ != nullptr
            ? target_camp_mode_controller_->Get()
            : ImageRecognize::TargetCampModeFromIndex(
                  target_camp_slider_);
    cv::Mat panel(300, 620, CV_8UC3, cv::Scalar(28, 30, 32));
    cv::putText(panel,
                "laser_target_vertical_trim: " +
                    cv::format("%.1f mm", laser_trim_m * 1000.0f),
                {12, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "editing_exposure_time_us:      " +
                    cv::format("%.0f us", exposure_us),
                {12, 62}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_edit_mode:            " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        edit_mode)),
                {12, 94}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_active_mode:          " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        active_mode)),
                {12, 126}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "calibration_edit_stage:        " +
                    std::string(CalibrationStageName(edit_stage)),
                {12, 158}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "stage1/2 " + cv::format("%.0f", stage12_exposure_us) +
                    " us  stage3 " + cv::format("%.0f", stage3_exposure_us) +
                    " us",
                {12, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(170, 190, 200), 1, cv::LINE_AA);
    cv::putText(panel,
                "target_camp_mode:              " +
                    std::string(ImageRecognize::ToString(target_camp_mode)),
                {12, 222}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "laser pitch comp: parallel axes, 10-24m",
                {12, 254}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
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
    const bool edit_mode_changed = edit_mode != exposure_slider_mode_;
    if (!edit_mode_changed && desired_slider == exposure_slider_) {
      return;
    }

    exposure_slider_mode_ = edit_mode;
    exposure_slider_ = desired_slider;
    syncing_sliders_ = true;
    cv::setTrackbarPos(kExposureTrackbar, kWindowName, exposure_slider_);
    if (edit_mode_changed) {
      SyncStageSlidersFromStage(ToCalibrationStage(edit_mode), true);
    }
    syncing_sliders_ = false;
    if (edit_mode_changed) {
      ApplySliderValues(false);
    }
  }

  static void SyncTargetCampSliderFromController(bool force) {
    if (!initialized_ && !force) {
      return;
    }
    if (target_camp_mode_controller_ == nullptr) {
      return;
    }

    const int desired_slider =
        target_camp_mode_controller_->GetModeIndex();
    if (!force && desired_slider == target_camp_slider_) {
      return;
    }

    target_camp_slider_ = desired_slider;
    syncing_sliders_ = true;
    cv::setTrackbarPos(kTargetCampTrackbar, kWindowName,
                       target_camp_slider_);
    syncing_sliders_ = false;
  }

  static void SyncStageSlidersFromStage(CalibrationStage stage,
                                        bool update_trackbars) {
    laser_trim_slider_ = ToLaserTrimSliderValue(
        DistanceCalculator::GetLaserTargetVerticalTrimMeters(stage));

    if (!update_trackbars) {
      return;
    }

    cv::setTrackbarPos(kLaserTrimTrackbar, kWindowName, laser_trim_slider_);
  }

  inline static bool initialized_ = false;
  inline static bool syncing_sliders_ = false;
  inline static int laser_trim_slider_ = ToLaserTrimSliderValue(0.0f);
  inline static int exposure_slider_ = ToExposureSliderValue(1000.0);
  inline static int target_camp_slider_ = 0;
  inline static CameraTask::ExposureHotkeyController::ExposureMode
      exposure_slider_mode_ =
          CameraTask::ExposureHotkeyController::ExposureMode::Stage12;
  inline static CameraTask::ExposureHotkeyController *exposure_controller_ =
      nullptr;
  inline static ImageRecognize::TargetCampModeController
      *target_camp_mode_controller_ = nullptr;
};

} // namespace Tools
