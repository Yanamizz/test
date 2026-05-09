#pragma once

#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Tools/LaserAngleCalculate.hpp"

namespace Tools {

class CalibrationSliderPanel {
public:
  static void Show() {
    EnsureInitialized();
    ApplySliderValues();
    DrawPanel();
  }

private:
  static constexpr const char *kWindowName = "Calibration Sliders";
  static constexpr int kHeightScale = 100000;
  static constexpr int kMinHeightTicks = 5500;
  static constexpr int kMaxHeightTicks = 7000;
  static constexpr int kSliderMax = kMaxHeightTicks - kMinHeightTicks;

  static int ToSliderValue(float height_m) {
    const int ticks = static_cast<int>(height_m * kHeightScale + 0.5f);
    return std::clamp(ticks - kMinHeightTicks, 0, kSliderMax);
  }

  static float ToHeightMeters(int slider_value) {
    const int ticks =
        kMinHeightTicks + std::clamp(slider_value, 0, kSliderMax);
    return static_cast<float>(ticks) / static_cast<float>(kHeightScale);
  }

  static void EnsureInitialized() {
    if (initialized_) {
      return;
    }

    const auto heights = DistanceCalculator::GetCalibrationTargetHeights();
    near_height_slider_ =
        ToSliderValue(heights.near_calibration_target_height);
    far_height_slider_ = ToSliderValue(heights.far_calibration_target_height);

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("near target height", kWindowName,
                       &near_height_slider_, kSliderMax, OnTrackbar);
    cv::createTrackbar("far target height", kWindowName, &far_height_slider_,
                       kSliderMax, OnTrackbar);

    initialized_ = true;
    ApplySliderValues();
  }

  static void OnTrackbar(int, void *) { ApplySliderValues(); }

  static void ApplySliderValues() {
    DistanceCalculator::SetCalibrationTargetHeights(
        ToHeightMeters(near_height_slider_), ToHeightMeters(far_height_slider_));
  }

  static void DrawPanel() {
    const auto heights = DistanceCalculator::GetCalibrationTargetHeights();
    cv::Mat panel(82, 460, CV_8UC3, cv::Scalar(28, 30, 32));
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
    cv::imshow(kWindowName, panel);
  }

  inline static bool initialized_ = false;
  inline static int near_height_slider_ = 0;
  inline static int far_height_slider_ = 0;
};

} // namespace Tools
