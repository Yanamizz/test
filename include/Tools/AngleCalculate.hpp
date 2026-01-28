#pragma once
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <algorithm>  // for std::clamp

#include "KalmanFilter/KalmanFilter.hpp"

#define PI 3.1415926

namespace Tools {
class AngleCalculator {
 public:
  std::pair<float, float> CalculateAbsoluteAngles(float targetX, float targetY, float currentYaw, float currentPitch) {
    auto now = std::chrono::steady_clock::now();
    double dt = 0.033;  // 默认帧间隔 (30fps)
    if (is_initialized) {
      dt = std::chrono::duration<double>(now - last_time).count();
    }
    last_time = now;
    is_initialized = true;

    // 去畸变并计算偏移角
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    cv::Point2f pnt;
    std::vector<cv::Point2f> in;
    std::vector<cv::Point2f> out;
    in.push_back(cv::Point2f(targetX, targetY));
    // 对像素点去畸变
    cv::undistortPoints(in, out, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    pnt = out.front();
    // 去畸变后的比值
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;
    // 计算偏移角
    double offset_yaw = -std::atan(rxNew) / PI * 180.0;
    double offset_pitch = std::atan(ryNew) / PI * 180.0;
    double absolute_yaw = normalizeAngle(currentYaw + offset_yaw);
    double absolute_pitch = normalizeAngle(currentPitch + offset_pitch);

    float filtered_yaw = kf_yaw.update(absolute_yaw, dt);
    float filtered_pitch = kf_pitch.update(absolute_pitch, dt);

    return {filtered_yaw, filtered_pitch};
  }

 private:
  float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
  }
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1576.303044, 0.000000, 952.451125, 0.000000, 1578.069737,
                          599.901423, 0.000000, 0.000000, 1.000000);
  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.275212, 0.210437, -0.000083, 0.000589, 0.000000);

  KalmanFilter kf_yaw{0.01, 1.0};
  KalmanFilter kf_pitch{0.01, 1.0};
  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
};

}  // namespace Tools
