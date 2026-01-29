#pragma once

#include "Tools/AngleCalculate.hpp"
#include "Tools/CameraData.hpp"

namespace Tools {
class DistanceCalculator {
 public:
  float CalculateDistance( float P_) { return (W_ * F_) / P_; }

 private:
  float P_ = 0.0f;  // 占位参数（保留原构造签名中的参数意义）
  float W_ = 0.2f;  // 目标实际边长，单位：米
  float F_ = CameraData().cameraMatrix.at<double>(0, 0);  // 相机焦距，单位：像素
};
class LaserAngleCalculator {
 public:
  float CalculateLaserAngle(float P) {
    float distance = DistanceCalculator().CalculateDistance(P);
    float angle_rad = std::atan2(laser_distance_, distance);
    float angle_deg = angle_rad * 180.0f / PI;
    return angle_deg;
  }

 private:
  float laser_distance_ = 0.03f;  // 激光传感器到相机中心的距离，单位：米
  cv::Mat cameraMatrix_ = CameraData().cameraMatrix;
  cv::Mat distCoeffs_ = CameraData().distCoeffs;
};

}  // namespace Tools