/**
 * @file    include\KalmanFilter\KalmanFilter.hpp
 * @brief   卡尔曼滤波：基于二维匀速模型的目标位置与速度估计
 *
 * @note    状态空间 [x, y, vx, vy]，观测空间 [x, y]
 * @note    输入为检测框 (xywh)，使用中心点作为观测；通过帧间 dt 估计速度
 */

#pragma once

#include <Eigen/Dense>
#include <kalman/LinearizedSystemModel.hpp>
#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <opencv2/core.hpp>

/**
 * @brief 状态向量 [x, y, vx, vy]
 */
struct State : public kalman::Vector<double, 4> {
  KALMAN_VECTOR(State, double, 4)
  enum { X, Y, VX, VY };
};

/**
 * @brief 观测向量 [x, y]
 */
struct Measurement : public kalman::Vector<double, 2> {
  KALMAN_VECTOR(Measurement, double, 2)
  enum { MX, MY };
};

/**
 * @brief 二维匀速运动系统模型（无控制量）
 * @note  x_k = x_{k-1} + vx*dt, y_k = y_{k-1} + vy*dt
 */
class SystemModel : public kalman::LinearizedSystemModel<State> {
 public:
  explicit SystemModel(double dt = 0.016) : dt_(dt) {}

  /**
   * @brief 设置时间步长
   * @param[in] dt_seconds 帧间隔（秒）
   */
  void setDt(double dt_seconds) { dt_ = dt_seconds; }

  /**
   * @brief 状态转移函数 f(x)
   * @param[in] prev_state 上一时刻状态
   * @return 预测状态
   */
  State f(const State &prev_state) const override {
    State x_pred;
    x_pred(State::X) = prev_state(State::X) + prev_state(State::VX) * dt_;
    x_pred(State::Y) = prev_state(State::Y) + prev_state(State::VY) * dt_;
    x_pred(State::VX) = prev_state(State::VX);
    x_pred(State::VY) = prev_state(State::VY);
    return x_pred;
  }

  /**
   * @brief 雅可比矩阵 F = df/dx
   * @return 4x4 雅可比
   */
  kalman::Jacobian<State> getJacobianF(const State &) const override {
    kalman::Jacobian<State> F = kalman::Jacobian<State>::Identity();
    F(State::X, State::VX) = dt_;
    F(State::Y, State::VY) = dt_;
    return F;
  }

 private:
  double dt_;  ///< 时间步长（秒）
};

/**
 * @brief 观测模型：直接观测位置 [x, y]
 */
class MeasurementModel : public kalman::LinearizedMeasurementModel<State, Measurement> {
 public:
  /**
   * @brief 观测函数 h(x)
   * @param[in] x 当前状态
   * @return 观测向量 [x, y]
   */
  Measurement h(const State &state) const override {
    Measurement measurement;
    measurement(Measurement::MX) = state(State::X);
    measurement(Measurement::MY) = state(State::Y);
    return measurement;
  }

  /**
   * @brief 雅可比矩阵 H = dh/dx
   * @return 2x4 雅可比
   */
  kalman::Jacobian<Measurement, State> getJacobianH(const State &) const override {
    kalman::Jacobian<Measurement, State> H = kalman::Jacobian<Measurement, State>::Zero();
    H(Measurement::MX, State::X) = 1.0;
    H(Measurement::MY, State::Y) = 1.0;
    return H;
  }
};

/**
 * @brief 2D 目标跟踪器：输入检测框，输出平滑后的中心点与速度
 */
class Tracker2D {
 public:
  Tracker2D() { init(); }

  /**
   * @brief 初始化滤波器状态与噪声
   */
  void init() {
    State x0;
    x0.setZero();
    kf_.init(x0);
    kf_.setProcessNoise(kalman::Covariance<State>::Identity() * 1e-3);            ///< 过程噪声 Q
    kf_.setMeasurementNoise(kalman::Covariance<Measurement>::Identity() * 1e-2);  ///< 测量噪声 R
    kf_.setStateCovariance(kalman::Covariance<State>::Identity());                ///< 初始协方差 P
  }

  /**
   * @brief 单帧更新：预测 + 校正
   * @param[in] bbox 检测框 (x,y,w,h)，像素坐标
   * @param[in] dt   帧间隔（秒）
   * @return 滤波后的中心点 (x,y)
   */
  Eigen::Vector2d update(const cv::Rect &bbox_rect, double dt_seconds) {
    sys_.setDt(dt_seconds);
    State x_pred = kf_.predict(sys_);
    Measurement measurement;
    measurement(Measurement::MX) = bbox_rect.x + bbox_rect.width * 0.5;
    measurement(Measurement::MY) = bbox_rect.y + bbox_rect.height * 0.5;
    State x_post = kf_.update(meas_, measurement);
    return {x_post(State::X), x_post(State::Y)};
  }

  /**
   * @brief 获取当前估计速度
   * @return (vx, vy)
   */
  Eigen::Vector2d velocity() const {
    const auto &x = kf_.getState();
    return {x(State::VX), x(State::VY)};
  }

 private:
  SystemModel sys_;                         ///< 系统模型（状态转移）
  MeasurementModel meas_;                   ///< 观测模型
  kalman::ExtendedKalmanFilter<State> kf_;  ///< EKF 核心
};