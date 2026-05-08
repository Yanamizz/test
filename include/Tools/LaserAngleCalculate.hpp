#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "Tools/CameraData.hpp"

namespace Tools {
constexpr float kPi = 3.1415926f;

class DistanceCalculator {
public:
  float CalculateDistance(float edge_a, float edge_b) {
    static_cast<void>(edge_b);
    const float h_ = edge_a;
    if (h_ <= 0.0f)
      return 0.0f;

    const float raw_distance = EstimateCalibratedDistanceByHeight(h_);
    return FilterDistance(raw_distance);
  }

private:
  struct TunableParams {
    float near_calibration_distance_m;
    float near_calibration_target_height;
    float near_pixel;
    float far_calibration_distance_m;
    float far_calibration_target_height;
    float far_pixel;
    float distance_filter_alpha;
    float filter_reset_ratio;
  };

  float EstimateCalibratedDistanceByHeight(float pixel_h) const {
    const float near_height = Params().near_calibration_target_height;
    const float far_height = Params().far_calibration_target_height;
    const float near_pixel = Params().near_pixel;
    const float far_pixel = Params().far_pixel;

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
    return (target_height * focal_px_) / pixel_h;
  }

  float FilterDistance(float raw_distance) {
    if (!has_filtered_distance_ || filtered_distance_ <= 0.0f) {
      filtered_distance_ = raw_distance;
      has_filtered_distance_ = true;
      return raw_distance;
    }

    const float reset_threshold =
        filtered_distance_ * Params().filter_reset_ratio;
    if (std::abs(raw_distance - filtered_distance_) > reset_threshold) {
      filtered_distance_ = raw_distance;
      return raw_distance;
    }

    filtered_distance_ +=
        Params().distance_filter_alpha * (raw_distance - filtered_distance_);
    return filtered_distance_;
  }

  static const TunableParams &Params() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        13.0f, // near_calibration_distance_m: 近点标定距离（米）
        0.0591f, // near_calibration_target_height: 近点补偿目标高度（米）
        83.9f, // near_pixel: 近点标定时目标在图像中的像素高度（px）
        20.0f, // far_calibration_distance_m: 远点标定距离（米）
        0.0655f, // far_calibration_target_height: 远点补偿目标高度（米）
        59.56f, // far_pixel: 远点标定时目标在图像中的像素高度（px）
        0.25f, // distance_filter_alpha: 一阶滤波系数，越大响应越快
        0.5f   // filter_reset_ratio: 距离突变超过该比例时重置滤波
    };
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  float focal_px_ =
      CameraData().cameraMatrix.at<double>(0, 0); // 相机焦距（像素）
};

class LaserAngleCalculator {
public:
  float CalculateLaserYawAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).first;
  }

  float CalculateLaserPitchAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).second;
  }

  std::pair<float, float> CalculateLaserAngles(float distance,
                                               float offset_yaw_deg,
                                               float offset_pitch_deg) {
    static_cast<void>(offset_yaw_deg);
    static_cast<void>(offset_pitch_deg);

    // 激光和相机的连线始终垂直于相机视线；激光在上方时，pitch 补偿为 atan(高度
    // / 距离)。
    const float current_pitch = ComputePitchCorrectionDeg(distance);
    const float reference_pitch =
        ComputePitchCorrectionDeg(Params().reference_distance_m);
    return {Params().fixed_offset_yaw,
            current_pitch - reference_pitch + Params().fixed_offset_pitch};
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
    if (distance <= 0.0f)
      return Params().reference_distance_m;
    return std::max(distance, Params().min_valid_distance_m);
  }

  static const TunableParams &Params() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.090f, // laser_height_above_camera_m: 激光在相机上方 0.09m
        18.8f, // reference_distance_m: 用于归零的参考水平距离（米）
        0.1f, // min_valid_distance_m: 仅用于防止接近 0 的距离造成异常角度
        0.0f, // fixed_offset_yaw: 固定补偿的yaw角（度），正值会让激光整体向右转
        0.0f, // fixed_offset_pitch:
              // 固定补偿的pitch角（度），正值会让激光整体向下转
    };
    return p;
  }
};

} // namespace Tools
