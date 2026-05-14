#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>
#include <vector>

#include "Tools/CameraData.hpp"

namespace Tools {
constexpr float kPi = 3.1415926f;

class DistanceCalculator {
public:
  struct CalibrationTargetHeights {
    float near_calibration_target_height;
    float far_calibration_target_height;
  };

  static CalibrationTargetHeights GetCalibrationTargetHeights() {
    const auto params = Params();
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static void SetCalibrationTargetHeights(float near_height, float far_height) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    params.near_calibration_target_height = near_height;
    params.far_calibration_target_height = far_height;
  }

  float CalculateDistance(float pixel_h) {
    return CalculateDistanceByPixelHeight(pixel_h);
  }

  float CalculateDistance(float x1, float y1, float x2, float y2) {
    const float raw_h = std::abs(y2 - y1);
    if (!std::isfinite(raw_h) || raw_h <= 0.0f)
      return 0.0f;

    const float corrected_h = UndistortedBoxHeight(x1, y1, x2, y2);
    return CalculateDistanceByPixelHeight(corrected_h > 0.0f ? corrected_h
                                                             : raw_h);
  }

private:
  struct TunableParams {
    float near_calibration_target_height;
    float near_pixel;
    float far_calibration_target_height;
    float far_pixel;
    float distance_filter_alpha;
    float filter_reset_ratio;
  };

  float CalculateDistanceByPixelHeight(float pixel_h) {
    if (!std::isfinite(pixel_h) || pixel_h <= 0.0f)
      return 0.0f;

    const float raw_distance = EstimateCalibratedDistanceByHeight(pixel_h);
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0f)
      return 0.0f;
    return FilterDistance(raw_distance);
  }

  float UndistortedBoxHeight(float x1, float y1, float x2, float y2) const {
    if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) ||
        !std::isfinite(y2)) {
      return 0.0f;
    }

    const float center_x = 0.5f * (x1 + x2);
    std::vector<cv::Point2f> in{{center_x, y1}, {center_x, y2}};
    std::vector<cv::Point2f> out;
    cv::undistortPoints(in, out, camera_data_.cameraMatrix,
                        camera_data_.distCoeffs, cv::noArray(),
                        camera_data_.cameraMatrix);
    if (out.size() != 2) {
      return 0.0f;
    }

    return std::abs(out[1].y - out[0].y);
  }

  float EstimateCalibratedDistanceByHeight(float pixel_h) const {
    const auto params = Params();
    const float near_height = params.near_calibration_target_height;
    const float far_height = params.far_calibration_target_height;
    const float near_pixel = params.near_pixel;
    const float far_pixel = params.far_pixel;

    if (near_pixel <= 0.0f || far_pixel <= 0.0f || near_height <= 0.0f ||
        far_height <= 0.0f || std::abs(far_pixel - near_pixel) <= 1e-6f) {
      return EstimateDistanceByHeight(far_height, pixel_h);
    }

    const float t = std::clamp(
        (pixel_h - near_pixel) / (far_pixel - near_pixel), 0.0f, 1.0f);
    const float target_height = near_height + t * (far_height - near_height);
    return EstimateDistanceByHeight(target_height, pixel_h);
  }

  float EstimateDistanceByHeight(float target_height, float pixel_h) const {
    if (!std::isfinite(target_height) || target_height <= 0.0f ||
        !std::isfinite(pixel_h) || pixel_h <= 0.0f ||
        !std::isfinite(focal_y_px_) || focal_y_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_height * focal_y_px_) / pixel_h;
  }

  float FilterDistance(float raw_distance) {
    const auto params = Params();
    if (!has_filtered_distance_ || filtered_distance_ <= 0.0f) {
      filtered_distance_ = raw_distance;
      has_filtered_distance_ = true;
      return raw_distance;
    }

    const float reset_threshold =
        filtered_distance_ * params.filter_reset_ratio;
    if (std::abs(raw_distance - filtered_distance_) > reset_threshold) {
      filtered_distance_ = raw_distance;
      return raw_distance;
    }

    filtered_distance_ +=
        params.distance_filter_alpha * (raw_distance - filtered_distance_);
    return filtered_distance_;
  }

  static TunableParams Params() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams();
  }

  static TunableParams &MutableParams() {
    static TunableParams params = DefaultParams();
    return params;
  }

  static std::mutex &ParamsMutex() {
    static std::mutex mutex;
    return mutex;
  }

  static const TunableParams &DefaultParams() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.0591f, // near_calibration_target_height: 近点补偿目标高度（米）
        83.9f, // near_pixel: 近点标定时目标在图像中的像素高度（px）
        0.0655f, // far_calibration_target_height: 远点补偿目标高度（米）
        59.56f, // far_pixel: 远点标定时目标在图像中的像素高度（px）
        0.25f, // distance_filter_alpha: 一阶滤波系数，越大响应越快
        0.5f   // filter_reset_ratio: 距离突变超过该比例时重置滤波
    };
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  CameraData camera_data_;
  float focal_y_px_ = camera_data_.cameraMatrix.at<double>(1, 1);
};

class LaserAngleCalculator {
public:
  struct FixedOffsets {
    float fixed_offset_yaw;
    float fixed_offset_pitch;
  };

  static FixedOffsets GetFixedOffsets() {
    const auto params = Params();
    return {params.fixed_offset_yaw, params.fixed_offset_pitch};
  }

  static void SetFixedOffsets(float fixed_offset_yaw,
                              float fixed_offset_pitch) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    params.fixed_offset_yaw = fixed_offset_yaw;
    params.fixed_offset_pitch = fixed_offset_pitch;
  }

  float CalculateLaserYawAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).first;
  }

  float CalculateLaserPitchAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).second;
  }

  std::pair<float, float> CalculateLaserAngles(float distance, float, float) {
    // 激光和相机的连线始终垂直于相机视线；新 pitch 正方向与旧约定相反，
    // 激光在上方时向下补偿为负值。
    const auto params = Params();
    const float current_pitch = ComputePitchCorrectionDeg(distance);
    const float reference_pitch =
        ComputePitchCorrectionDeg(params.reference_distance_m);
    return {params.fixed_offset_yaw,
            reference_pitch - current_pitch + params.fixed_offset_pitch};
  }

private:
  struct TunableParams {
    float laser_height_above_camera_m;
    float reference_distance_m;
    float min_valid_distance_m;
    float fixed_offset_yaw;
    float fixed_offset_pitch;
  };

  static float ComputePitchCorrectionDeg(float distance) {
    const float safe_distance = SafeDistance(distance);
    const float laser_pitch_rad =
        std::atan2(Params().laser_height_above_camera_m, safe_distance);
    return laser_pitch_rad * 180.0f / kPi;
  }

  static float SafeDistance(float distance) {
    if (!std::isfinite(distance) || distance <= 0.0f)
      return Params().reference_distance_m;
    return std::max(distance, Params().min_valid_distance_m);
  }

  static TunableParams Params() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams();
  }

  static TunableParams &MutableParams() {
    static TunableParams params = DefaultParams();
    return params;
  }

  static std::mutex &ParamsMutex() {
    static std::mutex mutex;
    return mutex;
  }

  static const TunableParams &DefaultParams() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.090f, // laser_height_above_camera_m: 激光在相机上方 0.09m
        18.8f, // reference_distance_m: 用于归零的参考水平距离（米）
        0.1f, // min_valid_distance_m: 仅用于防止接近 0 的距离造成异常角度
        // 发送日志里的 offset 仍按旧方向显示；这里填入其反号作为固定补偿。
        0.0617f, // fixed_offset_yaw:
                 // 固定补偿的yaw角（度），正值会让激光整体向右转
        0.0087f, // fixed_offset_pitch:
                 // 固定补偿的pitch角（度），正值会让激光整体向上转
    };
    return p;
  }
};

} // namespace Tools
