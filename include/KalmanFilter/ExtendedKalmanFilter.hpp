/**
 * @file    include/KalmanFilter/ExtendedKalmanFilter.hpp
 * @brief   实现用于角度与角速度估计的 Extended Kalman Filter。
 *
 * 该封装使用线性化模型估计角度和角速度，适合需要比简单一阶滤波更强状态估计的
 * 控制链路。它隐藏第三方滤波器细节，只向 AngleCalculator 暴露统一的 reset/update
 * 风格接口。
 */

#pragma once

#include "KalmanFilter/AngleKalmanModels.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"

namespace Tools {

class ExtendedKalmanFilter {
 public:
  ExtendedKalmanFilter(double q = 0.01, double r = 1.0)
      : q_(SanitizePositive_(q, kDefaultProcessNoise)),
        r_(SanitizePositive_(r, kDefaultMeasurementNoise)),
        system_model_(q_, kalman_detail::kDefaultAngleDtSec),
        measurement_model_(r_) {
    Reset_(0.0, 0.0);
  }

  double update(double measurement, double dt) {
    system_model_.setDt(dt);

    kalman_detail::AngleMeasurement z;
    z(0) = measurement;

    filter_.predict(system_model_);
    filter_.update(measurement_model_, z);

    return filter_.getState()(0);
  }

  double getVelocity() const { return filter_.getState()(1); }

  void setVelocity(double velocity) {
    kalman_detail::AngleState state = filter_.getState();
    state(1) = velocity;
    filter_.init(state);
  }

  double getAngle() const { return filter_.getState()(0); }

  void reset(double initial_angle = 0.0, double initial_velocity = 0.0) { Reset_(initial_angle, initial_velocity); }

 private:
  static double SanitizePositive_(double value, double fallback) { return value > 0.0 ? value : fallback; }

  void Reset_(double initial_angle, double initial_velocity) {
    filter_ = Filter{};
    ApplyNoise_();

    kalman_detail::AngleState state;
    state(0) = initial_angle;
    state(1) = initial_velocity;
    filter_.init(state);
  }

  void ApplyNoise_() {
    system_model_.setProcessNoise(q_);
    measurement_model_.setMeasurementNoise(r_);
  }

  static constexpr double kDefaultProcessNoise = 0.01;
  static constexpr double kDefaultMeasurementNoise = 1.0;

  using Filter = Kalman::ExtendedKalmanFilter<kalman_detail::AngleState>;

  double q_;
  double r_;
  Filter filter_;
  kalman_detail::AngleSystemModel system_model_;
  kalman_detail::AngleMeasurementModel measurement_model_;
};

}  // namespace Tools
