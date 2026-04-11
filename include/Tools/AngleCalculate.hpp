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

inline constexpr double kPi = 3.1415926;

namespace Tools {

enum class FilterType {
  KF,
  EKF,
  UKF,
  CKF,
};

inline const char *ToString(FilterType type) {
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
  explicit AngleCalculator() { ApplyTunableFilterGains(); }

  static FilterType ParseFilterType(const std::string &type) {
    std::string s;
    s.reserve(type.size());
    for (char c : type) {
      if (c >= 'a' && c <= 'z') {
        s.push_back(static_cast<char>(c - 'a' + 'A'));
      } else {
        s.push_back(c);
      }
    }

    if (s == "KF") return FilterType::KF;
    if (s == "EKF") return FilterType::EKF;
    if (s == "UKF") return FilterType::UKF;
    if (s == "CKF") return FilterType::CKF;
    return Params().default_filter_type;
  }

  CameraData cameraData;
  std::pair<float, float> CalculateAbsoluteAngles(float targetX, float targetY, float currentYaw, float currentPitch,
                                                  FilterType filter_type, double dt_from_main = -1.0) {
    double dt = Params().default_dt_sec;  // 默认帧间隔
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
    double offset_yaw = -std::atan2(rxNew, 1.0) / kPi * 180.0;
    double offset_pitch = std::atan2(ryNew, 1.0) / kPi * 180.0;

    if (std::abs(offset_yaw) > Params().max_offset_deg) {
      offset_yaw = Params().max_offset_deg * (offset_yaw > 0 ? 1.0 : -1.0);
    }
    if (std::abs(offset_pitch) > Params().max_offset_deg) {
      offset_pitch = Params().max_offset_deg * (offset_pitch > 0 ? 1.0 : -1.0);
    }

    double absolute_yaw = currentYaw + offset_yaw;
    double absolute_pitch = currentPitch + offset_pitch;

    UpdatePitchMotionHint(filter_type, absolute_pitch, dt);

    // 冷启动时先用首帧测量初始化滤波器，避免从 0° 拉到 ±180° 导致发送直接饱和到 ±5°。
    if (!filter_initialized) {
      ResetFilters(absolute_yaw, absolute_pitch);
      filter_initialized = true;
      last_absolute_pitch_ = absolute_pitch;
      has_last_absolute_pitch_ = true;
      return {static_cast<float>(absolute_yaw), static_cast<float>(absolute_pitch)};
    }

    float filtered_yaw =
        static_cast<float>(UpdateAngleByType(filter_type, absolute_yaw, dt, kf_yaw, ekf_yaw, ukf_yaw, ckf_yaw));
    float filtered_pitch = static_cast<float>(
        UpdateAngleByType(filter_type, absolute_pitch, dt, kf_pitch, ekf_pitch, ukf_pitch, ckf_pitch));

    if (pitch_motion_boost_frames_ > 0) {
      // 起步阶段给 pitch 更快一点的响应，后续由外部稳定层继续压抖。
      filtered_pitch = static_cast<float>(Params().pitch_startup_filtered_weight * filtered_pitch +
                                          Params().pitch_startup_measurement_weight * absolute_pitch);
      --pitch_motion_boost_frames_;
    }

    last_absolute_pitch_ = absolute_pitch;
    has_last_absolute_pitch_ = true;

    return {filtered_yaw, filtered_pitch};
  }

  cv::Point2f AbsoluteAnglesToPixel(float absoluteYaw, float absolutePitch, float currentYaw,
                                    float currentPitch) const {
    const double fx = cameraData.cameraMatrix.at<double>(0, 0);
    const double fy = cameraData.cameraMatrix.at<double>(1, 1);
    const double cx = cameraData.cameraMatrix.at<double>(0, 2);
    const double cy = cameraData.cameraMatrix.at<double>(1, 2);

    const double offset_yaw = static_cast<double>(absoluteYaw - currentYaw);
    const double offset_pitch = static_cast<double>(absolutePitch - currentPitch);

    const double rx = -std::tan(offset_yaw * kPi / 180.0);
    const double ry = std::tan(offset_pitch * kPi / 180.0);

    const float pred_x = static_cast<float>(cx + fx * rx);
    const float pred_y = static_cast<float>(cy + fy * ry);
    return {pred_x, pred_y};
  }

 private:
  void ResetFilters(double yaw, double pitch) {
    ApplyTunableFilterGains();

    kf_yaw.reset(yaw, 0.0);
    kf_pitch.reset(pitch, 0.0);

    ekf_yaw.reset(yaw, 0.0);
    ekf_pitch.reset(pitch, 0.0);

    ukf_yaw.reset(yaw, 0.0);
    ukf_pitch.reset(pitch, 0.0);

    ckf_yaw.reset(yaw, 0.0);
    ckf_pitch.reset(pitch, 0.0);

    ResetMotionHintState();
  }

  void ApplyTunableFilterGains() {
    kf_yaw = KalmanFilter{Params().yaw_filter_q, Params().yaw_filter_r};
    kf_pitch = KalmanFilter{Params().pitch_filter_q, Params().pitch_filter_r};

    ekf_yaw = ExtendedKalmanFilter{Params().yaw_filter_q, Params().yaw_filter_r};
    ekf_pitch = ExtendedKalmanFilter{Params().pitch_filter_q, Params().pitch_filter_r};

    ukf_yaw = UnscentedKalmanFilter{Params().yaw_filter_q, Params().yaw_filter_r};
    ukf_pitch = UnscentedKalmanFilter{Params().pitch_filter_q, Params().pitch_filter_r};

    ckf_yaw = CubatureKalmanFilter{Params().yaw_filter_q, Params().yaw_filter_r};
    ckf_pitch = CubatureKalmanFilter{Params().pitch_filter_q, Params().pitch_filter_r};
  }

  void ResetMotionHintState() {
    has_last_absolute_pitch_ = false;
    last_absolute_pitch_ = 0.0;
    pitch_velocity_hint_ = 0.0;
    pitch_motion_boost_frames_ = 0;
  }

  void PrimePitchVelocity(FilterType filter_type, double velocity) {
    switch (filter_type) {
      case FilterType::KF:
        kf_pitch.setVelocity(velocity);
        break;
      case FilterType::EKF:
        ekf_pitch.setVelocity(velocity);
        break;
      case FilterType::UKF:
        ukf_pitch.setVelocity(velocity);
        break;
      case FilterType::CKF:
        ckf_pitch.setVelocity(velocity);
        break;
      default:
        ukf_pitch.setVelocity(velocity);
        break;
    }
  }

  void UpdatePitchMotionHint(FilterType filter_type, double absolute_pitch, double dt) {
    if (dt <= 0.0) return;
    if (!has_last_absolute_pitch_) {
      last_absolute_pitch_ = absolute_pitch;
      has_last_absolute_pitch_ = true;
      return;
    }

    const double observed_velocity = (absolute_pitch - last_absolute_pitch_) / dt;
    const double velocity_change = observed_velocity - pitch_velocity_hint_;

    // 从静止开始明显运动时，提前给滤波器一个速度方向，缩短起步滞后。
    if (std::abs(observed_velocity) > Params().pitch_motion_trigger_velocity_deg_per_sec &&
        std::abs(velocity_change) > Params().pitch_motion_trigger_delta_velocity_deg_per_sec) {
      pitch_velocity_hint_ =
          Params().pitch_hint_old_weight * pitch_velocity_hint_ + Params().pitch_hint_new_weight * observed_velocity;
      pitch_motion_boost_frames_ = Params().pitch_motion_boost_frames;
      PrimePitchVelocity(filter_type, pitch_velocity_hint_);
    } else if (pitch_motion_boost_frames_ > 0) {
      pitch_velocity_hint_ *= Params().pitch_hint_decay;
      PrimePitchVelocity(filter_type, pitch_velocity_hint_);
    }

    last_absolute_pitch_ = absolute_pitch;
  }

  template <typename KF, typename EKF, typename UKF, typename CKF>
  static double UpdateAngleByType(FilterType filter_type, double measurement, double dt, KF &kf, EKF &ekf, UKF &ukf,
                                  CKF &ckf) {
    switch (filter_type) {
      case FilterType::KF:
        return kf.update(measurement, dt);
      case FilterType::EKF:
        return ekf.update(measurement, dt);
      case FilterType::UKF:
        return ukf.update(measurement, dt);
      case FilterType::CKF:
        return ckf.update(measurement, dt);
      default:
        return ukf.update(measurement, dt);
    }
  }

  struct TuningParams {
    FilterType default_filter_type;

    double default_dt_sec;
    double max_offset_deg;

    double yaw_filter_q;
    double yaw_filter_r;
    double pitch_filter_q;
    double pitch_filter_r;

    double pitch_startup_filtered_weight;
    double pitch_startup_measurement_weight;

    double pitch_motion_trigger_velocity_deg_per_sec;
    double pitch_motion_trigger_delta_velocity_deg_per_sec;
    double pitch_hint_old_weight;
    double pitch_hint_new_weight;
    double pitch_hint_decay;
    int pitch_motion_boost_frames;
  };

  static const TuningParams &Params();

  KalmanFilter kf_yaw{1.0, 0.05};
  KalmanFilter kf_pitch{0.05, 0.75};

  ExtendedKalmanFilter ekf_yaw{1.0, 0.05};
  ExtendedKalmanFilter ekf_pitch{0.05, 0.75};

  UnscentedKalmanFilter ukf_yaw{1.0, 0.05};
  UnscentedKalmanFilter ukf_pitch{0.05, 0.75};

  CubatureKalmanFilter ckf_yaw{1.0, 0.05};
  CubatureKalmanFilter ckf_pitch{0.05, 0.75};

  bool filter_initialized = false;

  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
  bool has_last_absolute_pitch_ = false;
  double last_absolute_pitch_ = 0.0;
  double pitch_velocity_hint_ = 0.0;
  int pitch_motion_boost_frames_ = 0;
};

inline const AngleCalculator::TuningParams &AngleCalculator::Params() {
  // ===== 调参集中区（统一放在文件末尾）=====
  // 你只需要改这里的数值即可。
  static const TuningParams p{
      FilterType::CKF,  // default_filter_type: 字符串解析失败时使用的默认滤波器类型

      0.05,  // default_dt_sec: 未提供帧间隔时使用的默认 dt（秒）
      5.0,   // max_offset_deg: 单帧像素解算得到的最大角度偏移限幅（度）

      1.6,    // yaw_filter_q: yaw 过程噪声 Q，调大以提升跟随性
      0.03,   // yaw_filter_r: yaw 测量噪声 R，调小以减少滞后
      0.01,   // pitch_filter_q: pitch 过程噪声 Q，调小以增强稳定性
      10.10,  // pitch_filter_r: pitch 测量噪声 R，调大以抑制抖动

      0.88,  // pitch_startup_filtered_weight: pitch 起步阶段滤波输出权重
      0.12,  // pitch_startup_measurement_weight: pitch 起步阶段测量值权重

      0.4,   // pitch_motion_trigger_velocity_deg_per_sec: 提高触发阈值，降低误触发
      1.0,   // pitch_motion_trigger_delta_velocity_deg_per_sec: 提高突变阈值，减少噪声触发
      0.80,  // pitch_hint_old_weight: 增强历史权重，降低瞬时抖动影响
      0.20,  // pitch_hint_new_weight: 降低新观测权重，减少毛刺
      0.75,  // pitch_hint_decay: 让速度提示更快衰减回稳态
      2      // pitch_motion_boost_frames: 缩短增强持续帧数，避免过冲
  };
  return p;
}

}  // namespace Tools
