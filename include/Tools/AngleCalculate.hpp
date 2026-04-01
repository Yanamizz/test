#pragma once
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>

#include "KalmanFilter/KalmanFilter.hpp"
#include "KalmanFilter/CubatureKalmanFilter.hpp"
#include "KalmanFilter/ExtendedKalmanFilter.hpp"
#include "KalmanFilter/UnscentedKalmanFilter.hpp"
#include "Tools/CameraData.hpp"

#define PI 3.1415926

namespace Tools {

enum class FilterType {
  KF,
  EKF,
  UKF,
  CKF,
};

inline const char* ToString(FilterType type) {
  switch (type) {
    case FilterType::KF:
      return "KF";
    case FilterType::EKF:
      return "EKF";
    case FilterType::UKF:
      return "UKF";
    case FilterType::CKF:
      return "CKF";
    default:
      return "UNKNOWN";
  }
}

class AngleCalculator {
 public:
  explicit AngleCalculator(FilterType type = FilterType::UKF) : filter_type(type) {}

  void SetFilterType(FilterType type) {
    if (filter_type == type) return;
    filter_type = type;
    filter_initialized = false;
  }

  FilterType GetFilterType() const { return filter_type; }

  bool SetFilterTypeFromString(const std::string& type) {
    std::string s;
    s.reserve(type.size());
    for (char c : type) {
      if (c >= 'a' && c <= 'z') {
        s.push_back(static_cast<char>(c - 'a' + 'A'));
      } else {
        s.push_back(c);
      }
    }

    if (s == "KF") {
      SetFilterType(FilterType::KF);
      return true;
    }
    if (s == "EKF") {
      SetFilterType(FilterType::EKF);
      return true;
    }
    if (s == "UKF") {
      SetFilterType(FilterType::UKF);
      return true;
    }
    if (s == "CKF") {
      SetFilterType(FilterType::CKF);
      return true;
    }
    return false;
  }

  CameraData cameraData;
  std::pair<float, float> CalculateAbsoluteAngles(float targetX, float targetY, float currentYaw, float currentPitch,
                                                  double dt_from_main = -1.0) {
    double dt = 0.05;  // 默认帧间隔 (20fps)
    if (dt_from_main > 0.0) {
      dt = dt_from_main;
    } else {
      auto now = std::chrono::steady_clock::now();
      if (is_initialized) {
        dt = std::chrono::duration<double>(now - last_time).count();
      }
      last_time = now;
      is_initialized = true;
    }

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
    double offset_yaw = -std::atan2(rxNew, 1.0) / PI * 180.0;
    double offset_pitch = std::atan2(ryNew, 1.0) / PI * 180.0;

    if (std::abs(offset_yaw) > 5.0f) offset_yaw = 5.0f * (offset_yaw > 0 ? 1.0f : -1.0f);  // 限制超出范围的角度
    if (std::abs(offset_pitch) > 5.0f) offset_pitch = 5.0f * (offset_pitch > 0 ? 1.0f : -1.0f);  // 限制超出范围的角度

    double absolute_yaw = currentYaw + offset_yaw;
    double absolute_pitch = currentPitch + offset_pitch;

    // 冷启动时先用首帧测量初始化滤波器，避免从 0° 拉到 ±180° 导致发送直接饱和到 ±5°。
    if (!filter_initialized) {
      ResetFilters(absolute_yaw, absolute_pitch);
      filter_initialized = true;
      return {static_cast<float>(absolute_yaw), static_cast<float>(absolute_pitch)};
    }

    float filtered_yaw = static_cast<float>(UpdateYaw(absolute_yaw, dt));
    float filtered_pitch = static_cast<float>(UpdatePitch(absolute_pitch, dt));

    return {filtered_yaw, filtered_pitch};
  }

 private:
  void ResetFilters(double yaw, double pitch) {
    kf_yaw.reset(yaw, 0.0);
    kf_pitch.reset(pitch, 0.0);

    ekf_yaw.reset(yaw, 0.0);
    ekf_pitch.reset(pitch, 0.0);

    ukf_yaw.reset(yaw, 0.0);
    ukf_pitch.reset(pitch, 0.0);

    ckf_yaw.reset(yaw, 0.0);
    ckf_pitch.reset(pitch, 0.0);
  }

  double UpdateYaw(double measurement, double dt) {
    switch (filter_type) {
      case FilterType::KF:
        return kf_yaw.update(measurement, dt);
      case FilterType::EKF:
        return ekf_yaw.update(measurement, dt);
      case FilterType::UKF:
        return ukf_yaw.update(measurement, dt);
      case FilterType::CKF:
        return ckf_yaw.update(measurement, dt);
      default:
        return ukf_yaw.update(measurement, dt);
    }
  }

  double UpdatePitch(double measurement, double dt) {
    switch (filter_type) {
      case FilterType::KF:
        return kf_pitch.update(measurement, dt);
      case FilterType::EKF:
        return ekf_pitch.update(measurement, dt);
      case FilterType::UKF:
        return ukf_pitch.update(measurement, dt);
      case FilterType::CKF:
        return ckf_pitch.update(measurement, dt);
      default:
        return ukf_pitch.update(measurement, dt);
    }
  }

  FilterType filter_type = FilterType::UKF;

  KalmanFilter kf_yaw{1.0, 0.05};
  KalmanFilter kf_pitch{0.01, 1.5};

  ExtendedKalmanFilter ekf_yaw{1.0, 0.05};
  ExtendedKalmanFilter ekf_pitch{0.01, 1.5};

  UnscentedKalmanFilter ukf_yaw{1.0, 0.05};
  UnscentedKalmanFilter ukf_pitch{0.01, 1.5};

  CubatureKalmanFilter ckf_yaw{1.0, 0.05};
  CubatureKalmanFilter ckf_pitch{0.01, 1.5};

  bool filter_initialized = false;

  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
};

}  // namespace Tools
