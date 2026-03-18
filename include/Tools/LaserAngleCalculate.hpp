#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "Tools/AngleCalculate.hpp"
#include "Tools/CameraData.hpp"

namespace Tools {
class DistanceCalculator {
 public:
  float CalculateDistance(float edge_a, float edge_b) {
    float h_ = std::max(edge_a, edge_b);
    float w_ = std::min(edge_a, edge_b);
    if (h_ <= 0.0f || w_ <= 0.0f) return 0.0f;
    float distance_by_h = (H_ * F_) / h_;
    return distance_by_h;
  }

 private:
  float W_ = 0.05f;                                       // 目标实际宽度，单位：米
  float H_ = 0.067f;                                      // 目标实际高度，单位：米
  float F_ = CameraData().cameraMatrix.at<double>(0, 0);  // 相机焦距，单位：像素
};
class LaserAngleCalculator {
 public:
  float CalculateLaserYawAngleByDistance(float distance) {
    float angle_rad = std::atan2(laser_distance_x, distance);
    float angle_deg = angle_rad * 180.0f / PI;
    return angle_deg;
  }

  float CalculateLaserPitchAngleByDistance(float distance) {
    float angle_rad = std::atan2(laser_distance_y, distance);
    float angle_deg = angle_rad * 180.0f / PI;
    return angle_deg;
  }

  std::pair<float, float> CalculateLaserAngles(float distance , float offset_angles) {
    float yaw_angle = 0.66;

    float pitch_angle = 0.33;
    return {yaw_angle, pitch_angle};
  }

 private:
  float laser_distance_x = 0.0f;  // 激光传感器到相机中心的x距离，单位：米
  float laser_distance_y = 0.0f;  // 激光传感器到相机中心的y距离，单位：米
  cv::Mat cameraMatrix_ = CameraData().cameraMatrix;
  cv::Mat distCoeffs_ = CameraData().distCoeffs;
};

}  // namespace Tools