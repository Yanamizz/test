#pragma once

#include <opencv2/opencv.hpp>

namespace Tools {
class CameraData {
public:
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  CameraData() {
    cameraMatrix =
        (cv::Mat_<double>(3, 3) << 18213.568586, 0.000000, 960.548600, 0.000000,
         18211.286085, 600.710214, 0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<double>(1, 5) << 0.348494, -0.665176, 0.008714,
                  -0.007641, 0.000000);
    // cameraMatrix =
    //     (cv::Mat_<double>(3, 3) << 17038.027983, 0.000000, 1145.765379,
    //      0.000000, 17055.879824, 1446.552834, 0.000000, 0.000000, 1.000000);
    // distCoeffs = (cv::Mat_<double>(1, 5) << -0.532606, 47.764189, 0.017590,
    //               0.002811, 0.000000);
  }
};
} // namespace Tools