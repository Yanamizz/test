/**
 * @file    include/KalmanFilter/AngleKalmanModels.hpp
 * @brief   定义角度滤波所需的状态、观测以及系统和量测模型。
 *
 * 该文件为 KF/EKF/UKF/CKF 等滤波器提供统一的角度状态定义，通常包含角度与角速度，
 * 并描述状态转移和观测模型。Tools::AngleCalculator 通过这些模型在不同滤波器实现
 * 之间保持一致的输入输出语义。
 */

#pragma once

#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/Matrix.hpp"
#include "kalman/Types.hpp"

namespace Tools {
namespace kalman_detail {

inline constexpr double kDefaultAngleDtSec = 0.05;
inline constexpr double kDefaultAngleProcessNoise = 0.01;
inline constexpr double kDefaultAngleMeasurementNoise = 0.1;

struct AngleState : Kalman::Vector<double, 2> {
  KALMAN_VECTOR(AngleState, double, 2)
};

struct AngleMeasurement : Kalman::Vector<double, 1> {
  KALMAN_VECTOR(AngleMeasurement, double, 1)
};

inline Kalman::Covariance<AngleState> MakeStateCovariance(double scale) {
  const double safe_scale = scale > 0.0 ? scale : kDefaultAngleProcessNoise;
  return safe_scale * Kalman::Covariance<AngleState>::Identity();
}

inline Kalman::Covariance<AngleMeasurement> MakeMeasurementCovariance(double scale) {
  const double safe_scale = scale > 0.0 ? scale : kDefaultAngleMeasurementNoise;
  return safe_scale * Kalman::Covariance<AngleMeasurement>::Identity();
}

class AngleSystemModel : public Kalman::LinearizedSystemModel<AngleState> {
 public:
  explicit AngleSystemModel(double process_noise = kDefaultAngleProcessNoise, double dt = kDefaultAngleDtSec)
      : dt_(SanitizeDt_(dt)) {
    setProcessNoise(process_noise);
  }

  void setDt(double dt) { dt_ = SanitizeDt_(dt); }

  double dt() const { return dt_; }

  bool setProcessNoise(double process_noise) { return this->setCovariance(MakeStateCovariance(process_noise)); }

  State f(const State &x, const Control &) const override {
    State next;
    next(0) = x(0) + x(1) * dt_;
    next(1) = x(1);
    return next;
  }

 protected:
  void updateJacobians(const State &, const Control &) override {
    this->F.setZero();
    this->F(0, 0) = 1.0;
    this->F(0, 1) = dt_;
    this->F(1, 0) = 0.0;
    this->F(1, 1) = 1.0;
    this->W.setIdentity();
  }

 private:
  static double SanitizeDt_(double dt) { return dt > 0.0 ? dt : kDefaultAngleDtSec; }

  double dt_ = kDefaultAngleDtSec;
};

class AngleMeasurementModel : public Kalman::LinearizedMeasurementModel<AngleState, AngleMeasurement> {
 public:
  explicit AngleMeasurementModel(double measurement_noise = kDefaultAngleMeasurementNoise) {
    setMeasurementNoise(measurement_noise);
  }

  bool setMeasurementNoise(double measurement_noise) {
    return this->setCovariance(MakeMeasurementCovariance(measurement_noise));
  }

  Measurement h(const State &x) const override {
    Measurement measurement;
    measurement(0) = x(0);
    return measurement;
  }

 protected:
  void updateJacobians(const State &) override {
    this->H.setZero();
    this->H(0, 0) = 1.0;
    this->V.setIdentity();
  }
};

}  // namespace kalman_detail
}  // namespace Tools
