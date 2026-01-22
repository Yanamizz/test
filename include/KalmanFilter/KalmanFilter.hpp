/**
 * @file examples/DroneTrackingUKF/main.cpp
 * @brief 使用UKF进行二维目标（无人机）运动状态估计的完整示例（位置+速度）
 *
 * @brief 本文件演示如何基于 mherb/kalman 使用 Unscented Kalman Filter (UKF)
 *        对视觉测量的目标位置进行预测与滤波。
 * @brief 状态向量为 [px, py, vx, vy]，观测向量为 [px, py]
 * @brief 你可以根据实际需求将状态扩展为 3D 或加入加速度/角速度等变量
 */

#pragma once

#include <kalman/UnscentedKalmanFilter.hpp>

#include <Eigen/Dense>
#include <iostream>

/**
 * @brief 定义UKF用到的状态向量
 *
 * @note 状态 x = [px, py, vx, vy]
 * @note px, py 为目标在像素坐标中的位置；vx, vy 为像素速度
 */
template <typename T>
class DroneState : public Kalman::Vector<T, 4> {
 public:
  KALMAN_VECTOR(DroneState, T, 4)

  /**
   * @brief x 方向位置
   */
  T px() const { return (*this)[0]; }
  T& px() { return (*this)[0]; }

  /**
   * @brief y 方向位置
   */
  T py() const { return (*this)[1]; }
  T& py() { return (*this)[1]; }

  /**
   * @brief x 方向速度
   */
  T vx() const { return (*this)[2]; }
  T& vx() { return (*this)[2]; }

  /**
   * @brief y 方向速度
   */
  T vy() const { return (*this)[3]; }
  T& vy() { return (*this)[3]; }
};

/**
 * @brief 控制输入（若无控制可忽略）
 *
 * @note 这里保留空控制输入结构，接口一致
 */
template <typename T>
class DroneControl : public Kalman::Vector<T, 0> {
 public:
  KALMAN_VECTOR(DroneControl, T, 0)
};

/**
 * @brief 系统模型（状态转移模型）
 *
 * @note 实现恒速度模型：
 *       px' = px + vx * dt
 *       py' = py + vy * dt
 *       vx' = vx
 *       vy' = vy
 */
template <typename T>
class DroneSystemModel : public Kalman::SystemModel<DroneState<T>, DroneControl<T>> {
 public:
  /**
   * @brief 构造函数，设置时间步长 dt
   * @param dt 时间间隔（秒）
   */
  explicit DroneSystemModel(T dt) : dt_(dt) {}

  /**
   * @brief 状态转移函数 f(x, u)
   * @param x 当前状态
   * @param u 控制输入（本例未使用）
   * @return 下一时刻状态
   */
  DroneState<T> f(const DroneState<T>& x, const DroneControl<T>& u) const override {
    (void)u;  // 控制输入未使用
    DroneState<T> x_next;
    x_next.px() = x.px() + x.vx() * dt_;
    x_next.py() = x.py() + x.vy() * dt_;
    x_next.vx() = x.vx();
    x_next.vy() = x.vy();
    return x_next;
  }

 private:
  T dt_;  ///< 时间步长
};

/**
 * @brief 位置观测向量 z = [px, py]
 */
template <typename T>
class DroneMeasurement : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(DroneMeasurement, T, 2)

  /**
   * @brief 测量的 x 方向位置
   */
  T px() const { return (*this)[0]; }
  T& px() { return (*this)[0]; }

  /**
   * @brief 测量的 y 方向位置
   */
  T py() const { return (*this)[1]; }
  T& py() { return (*this)[1]; }
};

/**
 * @brief 观测模型 h(x)
 *
 * @note 本模型仅测量位置，不测量速度
 */
template <typename T>
class DroneMeasurementModel : public Kalman::MeasurementModel<DroneState<T>, DroneMeasurement<T>> {
 public:
  /**
   * @brief 观测模型函数 h(x)
   * @param x 当前状态
   * @return 观测值 z
   */
  DroneMeasurement<T> h(const DroneState<T>& x) const override {
    DroneMeasurement<T> z;
    z.px() = x.px();
    z.py() = x.py();
    return z;
  }

  /**
   *
   * @brief 主函数调用
   *
   * @param[in] x,y  当前状态
   * @return 观测值 z
   */

  class DronePredict {
   public:
    DroneMeasurement<T> operator()(int x, int y, T dt) const {
      DroneState<T> x0;
      x0.setZero();
      DroneSystemModel<T> sys(dt);
      DroneControl<T> u;
      DroneMeasurementModel<T> measModel;
      Kalman::UnscentedKalmanFilter<DroneState<T>> ukf(1.0f);
      ukf.init(x0);
      DroneMeasurement<T> z;
      z.px() = x;
      z.py() = y;
      DroneState<T> x_pred = ukf.predict(sys, u);
      DroneState<T> x_est = ukf.update(measModel, z);
      return [ z.px(), z.py() ];
    }
  }
};
