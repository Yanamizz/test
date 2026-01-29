#pragma once
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <algorithm>  // for std::clamp

#include "KalmanFilter/KalmanFilter.hpp"
#include "Tools/CameraData.hpp"

#define PI 3.1415926

namespace Tools {
class AngleCalculator {
 public:
  CameraData cameraData;
  std::pair<float, float> CalculateAbsoluteAngles(float targetX, float targetY, float currentYaw, float currentPitch) {
    auto now = std::chrono::steady_clock::now();
    double dt = 0.033;  // 默认帧间隔 (30fps)
    if (is_initialized) {
      dt = std::chrono::duration<double>(now - last_time).count();
    }
    last_time = now;
    is_initialized = true;

    // 去畸变并计算偏移角
    double fx = cameraData.cameraMatrix.at<double>(0, 0);
    double fy = cameraData.cameraMatrix.at<double>(1, 1);
    double cx = cameraData.cameraMatrix.at<double>(0, 2);
    double cy = cameraData.cameraMatrix.at<double>(1, 2);
    cv::Point2f pnt;
    std::vector<cv::Point2f> in;
    std::vector<cv::Point2f> out;
    in.push_back(cv::Point2f(targetX, targetY));
    // 对像素点去畸变
    cv::undistortPoints(in, out, cameraData.cameraMatrix, cameraData.distCoeffs, cv::noArray(),
                        cameraData.cameraMatrix);
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

  KalmanFilter kf_yaw{0.01, 1.0};
  KalmanFilter kf_pitch{0.01, 1.0};
  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
};

}  // namespace Tools
