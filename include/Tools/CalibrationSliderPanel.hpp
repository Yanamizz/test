/**
 * @file    include/Tools/CalibrationSliderPanel.hpp
 * @brief   OpenCV 可视化调参面板，用于曝光、目标尺寸和阵营筛选。
 *
 * 该面板在调试显示开启时提供独立窗口，允许现场切换 stage1/2 与 stage3
 * 的编辑对象，并通过滑块调整当前 stage 的等效目标高度/宽度、曝光时间
 * 和目标阵营模式。目标尺寸滑块会写入 DistanceCalculator 的运行时标定
 * 参数，用于实机距离估计；近/中/远三点的检测框像素样本仍集中保存在
 * LaserAngleCalculate.hpp 的调参区，避免滑块误改基准采样数据。
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
  static constexpr const char *kNearHeightTrackbar = "near target height";
  static constexpr const char *kFarHeightTrackbar = "far target height";
  static constexpr const char *kNearWidthTrackbar = "near target width";
  static constexpr const char *kFarWidthTrackbar = "far target width";
  static constexpr const char *kExposureTrackbar = "exposure x100us";
  static constexpr const char *kTargetCampTrackbar =
      "target camp 0R 1B 2All";
  static constexpr int kHeightScale = 100000;
  static constexpr int kStage12MinHeightTicks = 5500;
  static constexpr int kStage12MaxHeightTicks = 7000;
  static constexpr int kStage3MinHeightTicks = 4000;
  static constexpr int kStage3MaxHeightTicks = 7000;
  static constexpr int kStage12HeightSliderMax =
      kStage12MaxHeightTicks - kStage12MinHeightTicks;
  static constexpr int kStage3NearHeightSliderMax =
      kStage3MaxHeightTicks - kStage3MinHeightTicks;
  static constexpr int kHeightSliderMax =
      std::max(kStage12HeightSliderMax, kStage3NearHeightSliderMax);
  static constexpr int kWidthScale = 100000;
  static constexpr int kMinWidthTicks = 3000;
  static constexpr int kMaxWidthTicks = 7000;
  static constexpr int kWidthSliderMax = kMaxWidthTicks - kMinWidthTicks;
  static constexpr int kExposureMinUs = 100;
  static constexpr int kExposureMaxUs = 10000;
  static constexpr int kExposureStepUs = 100;
  static constexpr int kExposureSliderMax =
      (kExposureMaxUs - kExposureMinUs) / kExposureStepUs;

  struct HeightSliderRange {
    int min_ticks;
    int max_ticks;
  };

  enum class HeightCalibrationPoint {
    Near,
    Far,
  };

  static HeightSliderRange HeightRange(CalibrationStage stage,
                                       HeightCalibrationPoint point) {
    if (stage == CalibrationStage::Stage3 &&
        point == HeightCalibrationPoint::Near) {
      return {kStage3MinHeightTicks, kStage3MaxHeightTicks};
    }
    return {kStage12MinHeightTicks, kStage12MaxHeightTicks};
  }

  static int ToSliderValue(float height_m, CalibrationStage stage,
                           HeightCalibrationPoint point) {
    const auto range = HeightRange(stage, point);
    const int ticks = static_cast<int>(height_m * kHeightScale + 0.5f);
    return std::clamp(ticks - range.min_ticks, 0,
                      range.max_ticks - range.min_ticks);
  }

  static float ToHeightMeters(int slider_value, CalibrationStage stage,
                              HeightCalibrationPoint point) {
    const auto range = HeightRange(stage, point);
    const int slider_max = range.max_ticks - range.min_ticks;
    const int ticks =
        range.min_ticks + std::clamp(slider_value, 0, slider_max);
    return static_cast<float>(ticks) / static_cast<float>(kHeightScale);
  }

  static int ToWidthSliderValue(float width_m) {
    const int ticks = static_cast<int>(width_m * kWidthScale + 0.5f);
    return std::clamp(ticks - kMinWidthTicks, 0, kWidthSliderMax);
  }

  static float ToWidthMeters(int slider_value) {
    const int ticks =
        kMinWidthTicks + std::clamp(slider_value, 0, kWidthSliderMax);
    return static_cast<float>(ticks) / static_cast<float>(kWidthScale);
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

    SyncCalibrationSlidersFromStage(
        ToCalibrationStage(exposure_slider_mode_), false);

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar(kNearHeightTrackbar, kWindowName, nullptr,
                       kHeightSliderMax, OnTrackbar);
    cv::createTrackbar(kFarHeightTrackbar, kWindowName, nullptr,
                       kHeightSliderMax, OnTrackbar);
    cv::createTrackbar(kNearWidthTrackbar, kWindowName, nullptr,
                       kWidthSliderMax, OnTrackbar);
    cv::createTrackbar(kFarWidthTrackbar, kWindowName, nullptr,
                       kWidthSliderMax, OnTrackbar);
    cv::createTrackbar(kExposureTrackbar, kWindowName, nullptr,
                       kExposureSliderMax, OnTrackbar);
    cv::createTrackbar(kTargetCampTrackbar, kWindowName, nullptr, 2,
                       OnTrackbar);
    cv::setTrackbarPos(kNearHeightTrackbar, kWindowName, near_height_slider_);
    cv::setTrackbarPos(kFarHeightTrackbar, kWindowName, far_height_slider_);
    cv::setTrackbarPos(kNearWidthTrackbar, kWindowName, near_width_slider_);
    cv::setTrackbarPos(kFarWidthTrackbar, kWindowName, far_width_slider_);
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
    DistanceCalculator::SetCalibrationTargetHeights(
        stage,
        ToHeightMeters(near_height_slider_, stage,
                       HeightCalibrationPoint::Near),
        ToHeightMeters(far_height_slider_, stage, HeightCalibrationPoint::Far));
    DistanceCalculator::SetCalibrationTargetWidths(
        stage, ToWidthMeters(near_width_slider_),
        ToWidthMeters(far_width_slider_));

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

    near_height_slider_ =
        cv::getTrackbarPos(kNearHeightTrackbar, kWindowName);
    far_height_slider_ =
        cv::getTrackbarPos(kFarHeightTrackbar, kWindowName);
    near_width_slider_ = cv::getTrackbarPos(kNearWidthTrackbar, kWindowName);
    far_width_slider_ = cv::getTrackbarPos(kFarWidthTrackbar, kWindowName);
    exposure_slider_ =
        cv::getTrackbarPos(kExposureTrackbar, kWindowName);
    target_camp_slider_ =
        cv::getTrackbarPos(kTargetCampTrackbar, kWindowName);
  }

  static void DrawPanel() {
    SyncExposureSliderFromController();
    SyncTargetCampSliderFromController(false);
    const auto edit_stage = CurrentEditCalibrationStage();
    const auto heights = DistanceCalculator::GetCalibrationTargetHeights(
        edit_stage);
    const auto widths = DistanceCalculator::GetCalibrationTargetWidths(
        edit_stage);
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
    cv::Mat panel(414, 620, CV_8UC3, cv::Scalar(28, 30, 32));
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
                "editing_exposure_time_us:      " +
                    cv::format("%.0f us", exposure_us),
                {12, 158}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "near_calibration_target_width: " +
                    cv::format("%.5f m",
                               widths.near_calibration_target_width),
                {12, 94}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "far_calibration_target_width:  " +
                    cv::format("%.5f m",
                               widths.far_calibration_target_width),
                {12, 126}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "width_pixels n/m/f:            " +
                    cv::format("%.1f / %.1f / %.1f px", widths.near_width_pixel,
                               widths.mid_width_pixel,
                               widths.far_width_pixel),
                {12, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.50,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_edit_mode:            " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        edit_mode)),
                {12, 222}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "exposure_active_mode:          " +
                    std::string(CameraTask::ExposureHotkeyController::ModeName(
                        active_mode)),
                {12, 254}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "calibration_edit_stage:        " +
                    std::string(CalibrationStageName(edit_stage)),
                {12, 286}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "stage1/2 " + cv::format("%.0f", stage12_exposure_us) +
                    " us  stage3 " + cv::format("%.0f", stage3_exposure_us) +
                    " us",
                {12, 316}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(170, 190, 200), 1, cv::LINE_AA);
    cv::putText(panel,
                "target_camp_mode:              " +
                    std::string(ImageRecognize::ToString(target_camp_mode)),
                {12, 346}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 236, 240), 1, cv::LINE_AA);
    cv::putText(panel,
                "laser pitch comp: Dw/Dh fused, 10-24m",
                {12, 378}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
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
      SyncCalibrationSlidersFromStage(ToCalibrationStage(edit_mode), true);
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

  static void SyncCalibrationSlidersFromStage(CalibrationStage stage,
                                              bool update_trackbars) {
    const auto heights = DistanceCalculator::GetCalibrationTargetHeights(stage);
    const auto widths = DistanceCalculator::GetCalibrationTargetWidths(stage);
    near_height_slider_ =
        ToSliderValue(heights.near_calibration_target_height, stage,
                      HeightCalibrationPoint::Near);
    far_height_slider_ =
        ToSliderValue(heights.far_calibration_target_height, stage,
                      HeightCalibrationPoint::Far);
    near_width_slider_ =
        ToWidthSliderValue(widths.near_calibration_target_width);
    far_width_slider_ =
        ToWidthSliderValue(widths.far_calibration_target_width);

    if (!update_trackbars) {
      return;
    }

    cv::setTrackbarPos(kNearHeightTrackbar, kWindowName, near_height_slider_);
    cv::setTrackbarPos(kFarHeightTrackbar, kWindowName, far_height_slider_);
    cv::setTrackbarPos(kNearWidthTrackbar, kWindowName, near_width_slider_);
    cv::setTrackbarPos(kFarWidthTrackbar, kWindowName, far_width_slider_);
  }

  inline static bool initialized_ = false;
  inline static bool syncing_sliders_ = false;
  inline static int near_height_slider_ = 0;
  inline static int far_height_slider_ = 0;
  inline static int near_width_slider_ = 0;
  inline static int far_width_slider_ = 0;
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
