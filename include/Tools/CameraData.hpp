#pragma once

#include <opencv2/opencv.hpp>

namespace Tools {
class CameraData {
 public:
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  CameraData() {
    cameraMatrix = (cv::Mat_<double>(3, 3) << 1576.303044, 0.000000, 952.451125, 0.000000, 1578.069737, 599.901423,
                    0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<double>(1, 5) << -0.275212, 0.210437, -0.000083, 0.000589, 0.000000);
  }
};
}  // namespace Tools