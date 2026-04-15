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
    if (h_ <= 0.0f || w <= 0.0f) return 0.0f;
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
        0.072f  // target_height_m: 目标实际高度（米）
    };
    return p;
  }

  float focal_px_ = CameraData().cameraMatrix.at<double>(0, 0);  // 相机焦距（像素）
};
class LaserAngleCalculator {
 public:
  float CalculateLaserYawAngleByDistance(float distance) {
    const float safe_distance = std::max(distance, Params().min_valid_distance_m);
    float angle_rad = std::atan2(Params().laser_distance_x_m, safe_distance);
    float angle_deg = angle_rad * 180.0f / kPi;
    return angle_deg;
  }

  float CalculateLaserPitchAngleByDistance(float distance) {
    const float safe_distance = std::max(distance, Params().min_valid_distance_m);
    float angle_rad = std::atan2(Params().laser_distance_y_m, safe_distance);
    float angle_deg = angle_rad * 180.0f / kPi;
    return angle_deg;
  }

  std::pair<float, float> CalculateLaserAngles(float distance, float offset_angles) {
    (void)offset_angles;
    // 组合模型：
    // 1) fixed_*: 相机与激光不平行导致的固定安装角补偿（手动标定）
    // 2) distance_*: 相机与激光发射点平移导致的视差补偿（随距离变化）
    const float yaw_angle = Params().fixed_yaw_compensation_deg + CalculateLaserYawAngleByDistance(distance);
    const float pitch_angle = Params().fixed_pitch_compensation_deg + CalculateLaserPitchAngleByDistance(distance);
    return {yaw_angle, pitch_angle};
  }

 private:
  struct TunableParams {
    float laser_distance_x_m;
    float laser_distance_y_m;
    float min_valid_distance_m;
    float fixed_yaw_compensation_deg;
    float fixed_pitch_compensation_deg;
  };

  static const TunableParams &Params() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        0.0f,     // laser_distance_x_m: 激光器到相机中心x偏移（米）
        0.066f,   // laser_distance_y_m: 激光器到相机中心y偏移（米）
        0.2f,     // min_valid_distance_m: 距离下限（米），防止近距数值异常
        -0.085f,  // fixed_yaw_compensation_deg: 固定yaw补偿角（度）
        -1.07f    // fixed_pitch_compensation_deg: 固定pitch补偿角（度）
    };
    return p;
  }
};

}  // namespace Tools
