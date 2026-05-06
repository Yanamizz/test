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
    float h_ = std::max(edge_a, edge_b);
    float w_ = std::min(edge_a, edge_b);
    if (h_ <= 0.0f || w_ <= 0.0f)
      return 0.0f;
    float distance_by_h = (Params().target_height * focal_px_) / h_;
    float distance_by_w = (Params().target_width * focal_px_) / w_;
    const float raw_distance =
        FuseDistance(distance_by_h, distance_by_w, h_, w_);
    return FilterDistance(raw_distance);
  }

private:
  struct TunableParams {
    float target_height;
    float target_width;
    float width_weight_scale;
    float distance_filter_alpha;
    float filter_reset_ratio;
  };

  static float FuseDistance(float distance_by_h, float distance_by_w,
                            float pixel_h, float pixel_w) {
    const float height_weight = pixel_h * pixel_h;
    const float width_weight = pixel_w * pixel_w * Params().width_weight_scale;
    return (distance_by_h * height_weight + distance_by_w * width_weight) /
           (height_weight + width_weight);
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
        0.060f, // target_height: 目标实际高度（米）
        0.055f, // target_width: 目标实际宽度（米）
        0.15f, // width_weight_scale: 宽度框更容易受姿态影响，略降权
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
