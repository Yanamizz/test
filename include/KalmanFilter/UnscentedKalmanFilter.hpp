/**
 * @file    include/KalmanFilter/UnscentedKalmanFilter.hpp
 * @brief   实现用于角度与角速度估计的 Unscented Kalman Filter。
 *
 * 该封装使用 UKF 处理非线性角度状态估计，避免显式线性化模型带来的部分误差。
 * 对外保持与其它滤波器一致的更新接口，供 AngleCalculator 根据运行参数选择。
 */

#pragma once

#include "KalmanFilter/AngleKalmanModels.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

namespace Tools {

class UnscentedKalmanFilter {
 public:
  UnscentedKalmanFilter(double q = 0.01, double r = 1.0, double alpha = 1e-3, double beta = 2.0, double kappa = 0.0)
      : q_(SanitizePositive_(q, kDefaultProcessNoise)),
        r_(SanitizePositive_(r, kDefaultMeasurementNoise)),
        alpha_(alpha),
        beta_(beta),
        kappa_(kappa),
        filter_(alpha_, beta_, kappa_),
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
    filter_ = Filter{alpha_, beta_, kappa_};
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

  using Filter = Kalman::UnscentedKalmanFilter<kalman_detail::AngleState>;

  double q_;
  double r_;
  double alpha_;
  double beta_;
  double kappa_;
  Filter filter_;
  kalman_detail::AngleSystemModel system_model_;
  kalman_detail::AngleMeasurementModel measurement_model_;
};

}  // namespace Tools
