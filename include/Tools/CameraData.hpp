/**
 * @file    include/Tools/CameraData.hpp
 * @brief   保存相机内参与畸变参数，供角度和距离计算模块复用。
 */

#pragma once

#include <opencv2/opencv.hpp>

namespace Tools {
class CameraData {
public:
  static constexpr double kFocalX = 18213.568586;
  static constexpr double kFocalY = 18211.286085;
  static constexpr double kPrincipalX = 960.548600;
  static constexpr double kPrincipalY = 600.710214;
  static constexpr double kDistCoeff0 = 0.348494;
  static constexpr double kDistCoeff1 = -0.665176;
  static constexpr double kDistCoeff2 = 0.008714;
  static constexpr double kDistCoeff3 = -0.007641;
  static constexpr double kDistCoeff4 = 0.000000;

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  CameraData() {
    cameraMatrix =
        (cv::Mat_<double>(3, 3) << kFocalX, 0.000000, kPrincipalX, 0.000000,
         kFocalY, kPrincipalY, 0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<double>(1, 5) << kDistCoeff0, kDistCoeff1,
                  kDistCoeff2, kDistCoeff3, kDistCoeff4);
  }
};
} // namespace Tools
