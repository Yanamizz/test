/**
 * @file    include/Tools/AngleCalculate.hpp
 * @brief   根据图像目标位置计算云台绝对角度，并集成多种角度滤波策略。
 */

#pragma once
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include "KalmanFilter/KalmanFilters.hpp"
#include "Tools/AngleUtils.hpp"
#include "Tools/CameraData.hpp"
#include "Tools/RuntimeParams.hpp"

namespace Tools {

inline constexpr double kPi = 3.1415926;

enum class FilterType {
  NONE,
  KF,
  EKF,
  UKF,
  CKF,
  ONE_EURO,
};

inline const char *ToString(FilterType type) {
  switch (type) {
  case FilterType::NONE:
    return "NONE";
  case FilterType::KF:
    return "KF";
  case FilterType::EKF:
    return "EKF";
  case FilterType::UKF:
    return "UKF";
  case FilterType::CKF:
    return "CKF";
  case FilterType::ONE_EURO:
    return "ONE_EURO";
  default:
    return "UNKNOWN";
  }
}

struct AngleCommand {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yaw_velocity = 0.0f;
  float pitch_velocity = 0.0f;
};

class AngleCalculator {
public:
  explicit AngleCalculator() { ApplyTunableFilterGains(); }

  static FilterType ParseFilterType(const std::string &type) {
    std::string s;
    s.reserve(type.size());
    for (char c : type) {
      if (c == '_' || c == '-' || c == ' ') {
        continue;
      }
      if (c >= 'a' && c <= 'z') {
        s.push_back(static_cast<char>(c - 'a' + 'A'));
      } else {
        s.push_back(c);
      }
    }

    if (s == "NONE" || s == "RAW" || s == "NOFILTER" || s == "OFF" ||
        s == "DISABLE") {
      return FilterType::NONE;
    }
    if (s == "KF")
      return FilterType::KF;
    if (s == "EKF")
      return FilterType::EKF;
    if (s == "UKF")
      return FilterType::UKF;
    if (s == "CKF")
      return FilterType::CKF;
    if (s == "ONEEURO" || s == "ONEEUROFILTER" || s == "1EURO" ||
        s == "1EUROFILTER") {
      return FilterType::ONE_EURO;
    }
    return Params().default_filter_type;
  }

  CameraData cameraData;
  AngleCommand CalculateAbsoluteAnglesWithVelocity(float targetX, float targetY,
                                                   float currentYaw,
                                                   float currentPitch,
                                                   FilterType filter_type,
                                                   double dt_from_main = -1.0) {
    double dt = Params().default_dt_sec; // 默认帧间隔
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

    double fx = cameraData.cameraMatrix.at<double>(0, 0);
    double fy = cameraData.cameraMatrix.at<double>(1, 1);
    double cx = cameraData.cameraMatrix.at<double>(0, 2);
    double cy = cameraData.cameraMatrix.at<double>(1, 2);
    const double rxNew = (static_cast<double>(targetX) - cx) / fx;
    const double ryNew = (static_cast<double>(targetY) - cy) / fy;
    // yaw/pitch 正方向已相对旧云台约定反向：画面右侧为 +yaw，画面下方为 -pitch。
    double offset_yaw = std::atan2(rxNew, 1.0) / kPi * 180.0;
    double offset_pitch = -std::atan2(ryNew, 1.0) / kPi * 180.0;

    if (std::abs(offset_yaw) > Params().max_offset_deg) {
      offset_yaw = Params().max_offset_deg * (offset_yaw > 0 ? 1.0 : -1.0);
    }
    if (std::abs(offset_pitch) > Params().max_offset_deg) {
      offset_pitch = Params().max_offset_deg * (offset_pitch > 0 ? 1.0 : -1.0);
    }

    double absolute_yaw = currentYaw + offset_yaw;
    double absolute_pitch = currentPitch + offset_pitch;

    AngleCommand result{};

    // 冷启动时先用首帧测量初始化滤波器，避免从 0° 拉到 ±180° 导致发送直接饱和到
    // ±5°。
    if (!filter_initialized) {
      ResetFilters(absolute_yaw, absolute_pitch);
      filter_initialized = true;
      result.yaw = static_cast<float>(absolute_yaw);
      result.pitch = static_cast<float>(absolute_pitch);
      last_filtered_yaw_ = result.yaw;
      last_filtered_pitch_ = result.pitch;
      has_last_filtered_angles_ = true;
      has_last_smoothed_velocity_ = false;
      return result;
    }

    result.yaw = static_cast<float>(
        UpdateAngleByType(filter_type, absolute_yaw, dt, kf_yaw, ekf_yaw,
                          ukf_yaw, ckf_yaw, oneeuro_yaw));
    result.pitch = static_cast<float>(
        UpdateAngleByType(filter_type, absolute_pitch, dt, kf_pitch, ekf_pitch,
                          ukf_pitch, ckf_pitch, oneeuro_pitch));

    const double velocity_dt =
        std::clamp(dt, Params().velocity_dt_min_sec, Params().velocity_dt_max_sec);
    const float safe_dt = static_cast<float>(std::max(velocity_dt, 1e-6));
    if (has_last_filtered_angles_) {
      const float raw_yaw_velocity =
          NormalizeDeltaDeg(result.yaw - last_filtered_yaw_) / safe_dt;
      const float raw_pitch_velocity =
          (result.pitch - last_filtered_pitch_) / safe_dt;
      result.yaw_velocity =
          SmoothVelocityAxis(raw_yaw_velocity, safe_dt, Params().yaw_velocity_abs_limit_deg_per_sec,
                             Params().yaw_velocity_max_accel_deg_per_sec2,
                             Params().yaw_velocity_cutoff_hz,
                             Params().yaw_velocity_deadband_deg_per_sec,
                             &last_smoothed_yaw_velocity_);
      result.pitch_velocity =
          SmoothVelocityAxis(raw_pitch_velocity, safe_dt, Params().pitch_velocity_abs_limit_deg_per_sec,
                             Params().pitch_velocity_max_accel_deg_per_sec2,
                             Params().pitch_velocity_cutoff_hz,
                             Params().pitch_velocity_deadband_deg_per_sec,
                             &last_smoothed_pitch_velocity_);
    }
    last_filtered_yaw_ = result.yaw;
    last_filtered_pitch_ = result.pitch;
    has_last_filtered_angles_ = true;
    return result;
  }

  std::pair<float, float> CalculateAbsoluteAngles(float targetX, float targetY,
                                                  float currentYaw,
                                                  float currentPitch,
                                                  FilterType filter_type,
                                                  double dt_from_main = -1.0) {
    const AngleCommand result = CalculateAbsoluteAnglesWithVelocity(
        targetX, targetY, currentYaw, currentPitch, filter_type, dt_from_main);
    return {result.yaw, result.pitch};
  }

  cv::Point2f AbsoluteAnglesToPixel(float absoluteYaw, float absolutePitch,
                                    float currentYaw,
                                    float currentPitch) const {
    const double fx = cameraData.cameraMatrix.at<double>(0, 0);
    const double fy = cameraData.cameraMatrix.at<double>(1, 1);
    const double cx = cameraData.cameraMatrix.at<double>(0, 2);
    const double cy = cameraData.cameraMatrix.at<double>(1, 2);

    const double offset_yaw = static_cast<double>(absoluteYaw - currentYaw);
    const double offset_pitch =
        static_cast<double>(absolutePitch - currentPitch);

    const double rx = std::tan(offset_yaw * kPi / 180.0);
    const double ry = -std::tan(offset_pitch * kPi / 180.0);

    const float pred_x = static_cast<float>(cx + fx * rx);
    const float pred_y = static_cast<float>(cy + fy * ry);
    return {pred_x, pred_y};
  }

private:
  void ResetFilters(double yaw, double pitch) {
    ApplyTunableFilterGains();
    has_last_filtered_angles_ = false;
    has_last_smoothed_velocity_ = false;
    last_smoothed_yaw_velocity_ = 0.0f;
    last_smoothed_pitch_velocity_ = 0.0f;

    kf_yaw.reset(yaw, 0.0);
    kf_pitch.reset(pitch, 0.0);

    ekf_yaw.reset(yaw, 0.0);
    ekf_pitch.reset(pitch, 0.0);

    ukf_yaw.reset(yaw, 0.0);
    ukf_pitch.reset(pitch, 0.0);

    ckf_yaw.reset(yaw, 0.0);
    ckf_pitch.reset(pitch, 0.0);
  }

  void ApplyTunableFilterGains() {
    const auto &params = Params();

    kf_yaw = KalmanFilter{params.yaw_filter_q, params.yaw_filter_r};
    kf_pitch = KalmanFilter{params.pitch_filter_q, params.pitch_filter_r};

    ekf_yaw = ExtendedKalmanFilter{params.yaw_filter_q, params.yaw_filter_r};
    ekf_pitch =
        ExtendedKalmanFilter{params.pitch_filter_q, params.pitch_filter_r};

    ukf_yaw = UnscentedKalmanFilter{params.yaw_filter_q, params.yaw_filter_r};
    ukf_pitch =
        UnscentedKalmanFilter{params.pitch_filter_q, params.pitch_filter_r};

    ckf_yaw = CubatureKalmanFilter{params.yaw_filter_q, params.yaw_filter_r};
    ckf_pitch =
        CubatureKalmanFilter{params.pitch_filter_q, params.pitch_filter_r};

    oneeuro_yaw.reset();
    oneeuro_pitch.reset();
    oneeuro_yaw.setFrequency(params.oneeuro_freq_hz);
    oneeuro_pitch.setFrequency(params.oneeuro_freq_hz);
    oneeuro_yaw.setMinCutoff(params.oneeuro_min_cutoff_hz);
    oneeuro_pitch.setMinCutoff(params.oneeuro_min_cutoff_hz);
    oneeuro_yaw.setBeta(params.oneeuro_beta);
    oneeuro_pitch.setBeta(params.oneeuro_beta);
    oneeuro_yaw.setDerivativeCutoff(params.oneeuro_d_cutoff_hz);
    oneeuro_pitch.setDerivativeCutoff(params.oneeuro_d_cutoff_hz);
  }

  template <typename KF, typename EKF, typename UKF, typename CKF,
            typename OneEuro>
  static double UpdateAngleByType(FilterType filter_type, double measurement,
                                  double dt, KF &kf, EKF &ekf, UKF &ukf,
                                  CKF &ckf, OneEuro &oneeuro) {
    switch (filter_type) {
    case FilterType::NONE:
      return measurement;
    case FilterType::KF:
      return kf.update(measurement, dt);
    case FilterType::EKF:
      return ekf.update(measurement, dt);
    case FilterType::UKF:
      return ukf.update(measurement, dt);
    case FilterType::CKF:
      return ckf.update(measurement, dt);
    case FilterType::ONE_EURO:
      return oneeuro.filter(measurement, dt);
    default:
      return ukf.update(measurement, dt);
    }
  }

  float SmoothVelocityAxis(float raw_velocity, float dt_sec,
                           double velocity_abs_limit_deg_per_sec,
                           double velocity_max_accel_deg_per_sec2,
                           double velocity_cutoff_hz,
                           double velocity_deadband_deg_per_sec,
                           float *last_smoothed_velocity) {
    const float velocity_limit =
        static_cast<float>(std::max(0.0, velocity_abs_limit_deg_per_sec));
    const float clamped_raw_velocity = std::clamp(raw_velocity, -velocity_limit,
                                                  velocity_limit);

    if (!has_last_smoothed_velocity_) {
      *last_smoothed_velocity = clamped_raw_velocity;
      has_last_smoothed_velocity_ = true;
    } else {
      const float max_accel =
          static_cast<float>(std::max(0.0, velocity_max_accel_deg_per_sec2));
      const float max_delta = max_accel * dt_sec;
      const float delta = clamped_raw_velocity - *last_smoothed_velocity;
      const float accel_limited_velocity =
          *last_smoothed_velocity + std::clamp(delta, -max_delta, max_delta);

      const double cutoff_hz = std::max(0.0, velocity_cutoff_hz);
      float alpha = 1.0f;
      if (cutoff_hz > 0.0) {
        const float tau =
            1.0f / (2.0f * static_cast<float>(kPi) * static_cast<float>(cutoff_hz));
        alpha = dt_sec / (tau + dt_sec);
      }
      alpha = std::clamp(alpha, 0.0f, 1.0f);
      *last_smoothed_velocity =
          *last_smoothed_velocity + alpha * (accel_limited_velocity - *last_smoothed_velocity);
    }

    if (std::abs(*last_smoothed_velocity) <
        static_cast<float>(std::max(0.0, velocity_deadband_deg_per_sec))) {
      *last_smoothed_velocity = 0.0f;
    }

    return *last_smoothed_velocity;
  }

  struct TuningParams {
    FilterType default_filter_type;

    double default_dt_sec;
    double max_offset_deg;

    double yaw_filter_q;
    double yaw_filter_r;
    double pitch_filter_q;
    double pitch_filter_r;

    double oneeuro_freq_hz;
    double oneeuro_min_cutoff_hz;
    double oneeuro_beta;
    double oneeuro_d_cutoff_hz;

    double velocity_dt_min_sec;
    double velocity_dt_max_sec;
    double yaw_velocity_abs_limit_deg_per_sec;
    double pitch_velocity_abs_limit_deg_per_sec;
    double yaw_velocity_max_accel_deg_per_sec2;
    double pitch_velocity_max_accel_deg_per_sec2;
    double yaw_velocity_cutoff_hz;
    double pitch_velocity_cutoff_hz;
    double yaw_velocity_deadband_deg_per_sec;
    double pitch_velocity_deadband_deg_per_sec;
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

  OneEuroFilter oneeuro_yaw;
  OneEuroFilter oneeuro_pitch;

  bool filter_initialized = false;
  float last_filtered_yaw_ = 0.0f;
  float last_filtered_pitch_ = 0.0f;
  bool has_last_filtered_angles_ = false;
  float last_smoothed_yaw_velocity_ = 0.0f;
  float last_smoothed_pitch_velocity_ = 0.0f;
  bool has_last_smoothed_velocity_ = false;

  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
};

inline const AngleCalculator::TuningParams &AngleCalculator::Params() {
  static const TuningParams p{
      FilterType::ONE_EURO,

      0.03, // default_dt_sec
      5.0,  // max_offset_deg

      10.0, // yaw_filter_q
      0.01, // yaw_filter_r
      0.1,  // pitch_filter_q
      1.0,  // pitch_filter_r

      120.0, // oneeuro_freq_hz
      4.6,   // oneeuro_min_cutoff_hz
      6.5,   // oneeuro_beta
      2.5,   // oneeuro_d_cutoff_hz

      Tools::Params().angle_velocity_dt_min_sec,
      Tools::Params().angle_velocity_dt_max_sec,
      Tools::Params().angle_velocity_yaw_abs_limit_deg_per_sec,
      Tools::Params().angle_velocity_pitch_abs_limit_deg_per_sec,
      Tools::Params().angle_velocity_yaw_max_accel_deg_per_sec2,
      Tools::Params().angle_velocity_pitch_max_accel_deg_per_sec2,
      Tools::Params().angle_velocity_yaw_cutoff_hz,
      Tools::Params().angle_velocity_pitch_cutoff_hz,
      Tools::Params().angle_velocity_yaw_deadband_deg_per_sec,
      Tools::Params().angle_velocity_pitch_deadband_deg_per_sec
  };
  return p;
}

} // namespace Tools
