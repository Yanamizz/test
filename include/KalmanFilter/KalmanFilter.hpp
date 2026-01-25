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

#include <chrono>

namespace kalman = Kalman;

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
  /**
   * @brief 构造函数
   * @param[in] dt 时间步长（秒）
   */
  explicit SystemModel(double dt = 0.016) : dt_(dt) {}

  /**
   * @brief 设置时间步长
   * @param[in] dt_seconds 帧间隔（秒）
   */
  void setDt(double dt_seconds) { dt_ = dt_seconds; }

  /**
   * @brief 状态转移函数 f(x)
   * @param[in] prev_state 上一时刻状态
   * @param[in] u 控制输入（未使用）
   * @return 预测状态
   */
  State f(const State &prev_state, const Control &u) const override {
    (void)u;  // 忽略控制输入
    State x_pred;
    x_pred(State::X) = prev_state(State::X) + prev_state(State::VX) * dt_;
    x_pred(State::Y) = prev_state(State::Y) + prev_state(State::VY) * dt_;
    x_pred(State::VX) = prev_state(State::VX);
    x_pred(State::VY) = prev_state(State::VY);
    return x_pred;
  }

  /**
   * @brief 雅可比矩阵 F = df/dx
   * @param[in] x 当前状态
   * @param[in] u 控制输入（未使用）
   * @return 4x4 雅可比
   */
  kalman::Jacobian<State, State> getJacobianF(const State &x, const Control &u) const {
    (void)x;
    (void)u;  // 忽略
    kalman::Jacobian<State, State> F = kalman::Jacobian<State, State>::Identity();
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
   * @param[in] state 当前状态
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
  kalman::Jacobian<Measurement, State> getJacobianH(const State &) const {
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
  /**
   * @brief 构造函数
   * @note  初始化滤波器状态与噪声，内部启用自动 dt 计算
   */
  Tracker2D() { init(); }

  /**
   * @brief 初始化滤波器状态与噪声
   */
  void init() {
    State x0;
    x0.setZero();
    kf_.init(x0);

    // 增大测量噪声权重，减少震荡
    sys_.setCovariance(kalman::Covariance<State>::Identity() * 1e-3);  ///< 过程噪声 Q：越大越信任模型
    meas_.setCovariance(kalman::Covariance<Measurement>::Identity() * 1e-1);  ///< 测量噪声 R：越大越信任测量
    kf_.setCovariance(kalman::Covariance<State>::Identity());  ///< 初始协方差 P：越大越信任初始状态

    initialized_ = false;
    last_time_ = std::chrono::steady_clock::time_point{};
  }

  /**
   * @brief 单帧更新：预测 + 校正（内部自动计算 dt）
   * @param[in] bbox_rect 检测框 (x,y,w,h)，像素坐标
   * @return 滤波后的中心点 (x,y)
   */
  Eigen::Vector2d update(const cv::Rect &bbox_rect) {
    const double dt_seconds = computeDt();

    if (bbox_rect.area() == 0) {
      // 如果目标消失，返回最后一次更新的位置
      State x_last = kf_.getState();
      return {x_last(State::X), x_last(State::Y)};
    }

    return updateImpl(bbox_rect, dt_seconds);
  }

  /**
   * @brief 单帧更新：预测 + 校正（外部指定 dt）
   * @param[in] bbox_rect 检测框 (x,y,w,h)，像素坐标
   * @param[in] dt_seconds 帧间隔（秒）
   * @return 滤波后的中心点 (x,y)
   */
  Eigen::Vector2d update(const cv::Rect &bbox_rect, double dt_seconds) {
    if (bbox_rect.area() == 0) {
      // 如果目标消失，返回最后一次更新的位置
      State x_last = kf_.getState();
      return {x_last(State::X), x_last(State::Y)};
    }

    return updateImpl(bbox_rect, dt_seconds);
  }

 private:
  /**
   * @brief 计算相邻帧的时间间隔 dt
   * @return dt_seconds 帧间隔（秒）
   */
  double computeDt() {
    const auto now = std::chrono::steady_clock::now();

    // 第一次调用时使用默认 dt
    if (!initialized_) {
      initialized_ = true;
      last_time_ = now;
      return default_dt_;
    }

    const std::chrono::duration<double> delta = now - last_time_;
    last_time_ = now;

    // 参数验证：避免 dt 过小或为 0
    const double dt = delta.count();
    return (dt > 1e-6) ? dt : default_dt_;
  }

  /**
   * @brief 内部更新实现
   * @param[in] bbox_rect 检测框 (x,y,w,h)，像素坐标
   * @param[in] dt_seconds 帧间隔（秒）
   * @return 滤波后的中心点 (x,y)
   */
  Eigen::Vector2d updateImpl(const cv::Rect &bbox_rect, double dt_seconds) {
    sys_.setDt(dt_seconds);

    // 预测
    State x_pred = kf_.predict(sys_);

    // 构造测量
    Measurement measurement;
    measurement(Measurement::MX) = bbox_rect.x + bbox_rect.width * 0.5;
    measurement(Measurement::MY) = bbox_rect.y + bbox_rect.height * 0.5;

    // 更新
    State x_post = kf_.update(meas_, measurement);

    (void)x_pred;  // 若不需要调试输出，可忽略
    return {x_post(State::X), x_post(State::Y)};
  }

 private:
  SystemModel sys_;                         ///< 系统模型（状态转移）
  MeasurementModel meas_;                   ///< 观测模型
  kalman::ExtendedKalmanFilter<State> kf_;  ///< EKF 核心

  bool initialized_{false};                          ///< 是否完成首次时间戳初始化
  double default_dt_{0.033};                         ///< 默认时间步长（秒）
  std::chrono::steady_clock::time_point last_time_;  ///< 上一帧时间戳
};