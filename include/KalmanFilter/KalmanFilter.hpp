#pragma once

#include <Eigen/Dense>
#include <cmath>

#include <kalman/SystemModel.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/Matrix.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <opencv2/core.hpp>

/** 状态向量 [x, y, vx, vy] */
struct State : public Kalman::Vector<double, 4> {
  KALMAN_VECTOR(State, double, 4)
  enum { X, Y, VX, VY };
};

/** 观测向量 [x, y] */
struct Measurement : public Kalman::Vector<double, 2> {
  KALMAN_VECTOR(Measurement, double, 2)
  enum { MX, MY };
};

/** 二维匀速系统模型：x_k = x_{k-1} + vx*dt, y_k = y_{k-1} + vy*dt */
class SystemModel : public Kalman::LinearizedSystemModel<State> {
 public:
  explicit SystemModel(double dt = 0.016) : dt_(dt) {}

  void setDt(double dt_seconds) { dt_ = dt_seconds; }

  State f(const State &prev_state) const override {
    State x_pred;
    x_pred(State::X) = prev_state(State::X) + prev_state(State::VX) * dt_;
    x_pred(State::Y) = prev_state(State::Y) + prev_state(State::VY) * dt_;
    x_pred(State::VX) = prev_state(State::VX);
    x_pred(State::VY) = prev_state(State::VY);
    return x_pred;
  }

  Kalman::Jacobian<State> getJacobianF(const State &) const override {
    Kalman::Jacobian<State> F = Kalman::Jacobian<State>::Identity();
    F(State::X, State::VX) = dt_;
    F(State::Y, State::VY) = dt_;
    return F;
  }

 private:
  double dt_;  ///< 时间步长（秒）
};

/** 观测模型：直接观测位置 [x, y] */
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State, Measurement> {
 public:
  Measurement h(const State &state) const override {
    Measurement z;
    z(Measurement::MX) = state(State::X);
    z(Measurement::MY) = state(State::Y);
    return z;
  }

  Kalman::Jacobian<Measurement, State> getJacobianH(const State &) const override {
    Kalman::Jacobian<Measurement, State> H = Kalman::Jacobian<Measurement, State>::Zero();
    H(Measurement::MX, State::X) = 1.0;
    H(Measurement::MY, State::Y) = 1.0;
    return H;
  }
};

/** 2D 目标跟踪器：输入检测框，输出平滑中心点和速度 */
class Tracker2D {
 public:
  Tracker2D() { init(); }

  void init() {
    State x0;
    x0.setZero();
    kf_.init(x0);
    // 可按需要调整 Q/R/P
    kf_.setProcessNoise(Kalman::Covariance<State>::Identity() * 1e-3);            // Q
    kf_.setMeasurementNoise(Kalman::Covariance<Measurement>::Identity() * 1e-2);  // R
    kf_.setStateCovariance(Kalman::Covariance<State>::Identity());                // P0
  }

  /**
   * @param bbox_rect 检测框 (x,y,w,h)，像素
   * @param dt_seconds 帧间隔（秒）
   * @return 滤波后中心点 (x,y)
   */
  Eigen::Vector2d update(const cv::Rect &bbox_rect, double dt_seconds) {
    sys_.setDt(dt_seconds);
    kf_.predict(sys_);
    Measurement z;
    z(Measurement::MX) = bbox_rect.x + bbox_rect.width * 0.5;
    z(Measurement::MY) = bbox_rect.y + bbox_rect.height * 0.5;
    auto x_post = kf_.update(meas_, z);
    return {x_post(State::X), x_post(State::Y)};
  }

  /** 当前估计速度 (vx, vy) */
  Eigen::Vector2d velocity() const {
    const auto &x = kf_.getState();
    return {x(State::VX), x(State::VY)};
  }

 private:
  SystemModel sys_;
  MeasurementModel meas_;
  Kalman::ExtendedKalmanFilter<State> kf_;
};