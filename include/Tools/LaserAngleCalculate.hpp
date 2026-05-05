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
    float w = std::min(edge_a, edge_b);
    if (h_ <= 0.0f || w <= 0.0f)
      return 0.0f;
    float distance_by_h = (Params().target_height_m * focal_px_) / h_;
    return distance_by_h;
  }

private:
  struct TunableParams {
    float target_height_m;
  };

  static const TunableParams &Params() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.060f // target_height_m: 目标实际高度（米）
    };
    return p;
  }

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
    // 相机坐标系中的横向基线模型：基线会随云台刚体旋转，
    // 这里返回的是“激光相对相机视线”的补偿角。
    const float current_yaw =
        ComputeYawCorrectionDeg(distance, offset_yaw_deg, offset_pitch_deg);
    const float current_pitch =
        ComputePitchCorrectionDeg(distance, offset_yaw_deg, offset_pitch_deg);
    const float reference_yaw =
        ComputeYawCorrectionDeg(Params().reference_distance_m, 0.0f, 0.0f);
    const float reference_pitch =
        ComputePitchCorrectionDeg(Params().reference_distance_m, 0.0f, 0.0f);
    const float fixed_offset_yaw = Params().fixed_offset_yaw;
    const float fixed_offset_pitch = Params().fixed_offset_pitch;
    return {current_yaw - reference_yaw + fixed_offset_yaw,
            current_pitch - reference_pitch + fixed_offset_pitch};
  }

private:
  struct TunableParams {
    float laser_offset_x_m;
    float laser_offset_y_m;
    float reference_distance_m;
    float min_valid_distance_m;
    float fixed_offset_yaw;
    float fixed_offset_pitch;
  };

  static float ComputeYawCorrectionDeg(float distance, float offset_yaw_deg,
                                       float offset_pitch_deg) {
    const float safe_distance =
        std::max(distance, Params().min_valid_distance_m);
    const float yaw_rad = offset_yaw_deg * kPi / 180.0f;
    const float pitch_rad = offset_pitch_deg * kPi / 180.0f;
    const float dir_x = -std::tan(yaw_rad);
    const float dir_y = std::tan(pitch_rad);
    const float inv_norm =
        1.0f / std::sqrt(dir_x * dir_x + dir_y * dir_y + 1.0f);

    const float target_x = safe_distance * dir_x * inv_norm;
    const float target_z = safe_distance * inv_norm;

    const float rel_x = target_x - Params().laser_offset_x_m;
    const float rel_z = target_z;
    const float laser_yaw_rad = -std::atan2(rel_x, rel_z);
    return laser_yaw_rad * 180.0f / kPi - offset_yaw_deg;
  }

  static float ComputePitchCorrectionDeg(float distance, float offset_yaw_deg,
                                         float offset_pitch_deg) {
    const float safe_distance =
        std::max(distance, Params().min_valid_distance_m);
    const float yaw_rad = offset_yaw_deg * kPi / 180.0f;
    const float pitch_rad = offset_pitch_deg * kPi / 180.0f;
    const float dir_x = -std::tan(yaw_rad);
    const float dir_y = std::tan(pitch_rad);
    const float inv_norm =
        1.0f / std::sqrt(dir_x * dir_x + dir_y * dir_y + 1.0f);

    const float target_x = safe_distance * dir_x * inv_norm;
    const float target_y = safe_distance * dir_y * inv_norm;
    const float target_z = safe_distance * inv_norm;

    const float rel_x = target_x - Params().laser_offset_x_m;
    const float rel_y = target_y - Params().laser_offset_y_m;
    const float rel_z = target_z;
    const float laser_pitch_rad =
        std::atan2(rel_y, std::sqrt(rel_x * rel_x + rel_z * rel_z));
    return laser_pitch_rad * 180.0f / kPi - offset_pitch_deg;
  }

  static const TunableParams &Params() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.0f, // laser_offset_x_m: 激光相对相机的横向偏移（米，向右为正）
        -0.0904f, // laser_offset_y_m: 激光相对相机的纵向偏移（米，向下为正）
        18.8f, // reference_distance_m: 用于归零的参考水平距离（米）
        10.0f, // min_valid_distance_m: 距离下限（米），防止近距数值异常
        0.0f, // fixed_offset_yaw: 固定补偿的yaw角（度），正值会让激光整体向右转
        -0.04f, // fixed_offset_pitch:
                // 固定补偿的pitch角（度），正值会让激光整体向下转
    };
    return p;
  }
};

} // namespace Tools
