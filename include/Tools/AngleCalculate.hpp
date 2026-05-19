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
#include "Tools/CameraData.hpp"

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

inline float NormalizeDeltaDeg(float delta) {
  while (delta > 180.0f)
    delta -= 360.0f;
  while (delta < -180.0f)
    delta += 360.0f;
  return delta;
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
      return result;
    }

    result.yaw = static_cast<float>(
        UpdateAngleByType(filter_type, absolute_yaw, dt, kf_yaw, ekf_yaw,
                          ukf_yaw, ckf_yaw, oneeuro_yaw));
    result.pitch = static_cast<float>(
        UpdateAngleByType(filter_type, absolute_pitch, dt, kf_pitch, ekf_pitch,
                          ukf_pitch, ckf_pitch, oneeuro_pitch));

    const float safe_dt = static_cast<float>(std::max(dt, 1e-6));
    if (has_last_filtered_angles_) {
      result.yaw_velocity =
          NormalizeDeltaDeg(result.yaw - last_filtered_yaw_) / safe_dt;
      result.pitch_velocity = (result.pitch - last_filtered_pitch_) / safe_dt;
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

  std::chrono::steady_clock::time_point last_time;
  bool is_initialized = false;
};

inline const AngleCalculator::TuningParams &AngleCalculator::Params() {
  // ===== 调参集中区（统一放在文件末尾）=====
  // OneEuro 调参起点：静止目标继续往响应侧推进，后续按“振荡/滞后”逐步收窄。
  static const TuningParams p{
      FilterType::ONE_EURO, // default_filter_type: 调参模式默认使用 OneEuro

      0.03, // default_dt_sec: 未提供帧间隔时使用的默认 dt（秒）
      5.0, // max_offset_deg: 单帧像素解算得到的最大角度偏移限幅（度）

      10.0, // yaw_filter_q: yaw 过程噪声 Q，调大以提升跟随性
      0.01, // yaw_filter_r: yaw 测量噪声 R，调小以减少滞后
      0.1,  // pitch_filter_q: pitch 过程噪声 Q，调小以增强稳定性
      1.0,  // pitch_filter_r: pitch 测量噪声 R，调大以抑制抖动

      120.0, // oneeuro_freq_hz: OneEuro 采样频率（Hz），通常作为 dt
             // 异常时的回退值
      4.6, // oneeuro_min_cutoff_hz: 小幅降稳，压住静止和慢速振荡
      6.5, // oneeuro_beta: 降低运动响应增益，减少过度追随造成的振荡
      2.5 // oneeuro_d_cutoff_hz: 降低导数响应，避免把小抖动当作运动
  };
  return p;
}

} // namespace Tools
