#pragma once

#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace Tools {

/**
 * @brief 扩展卡尔曼滤波 (EKF) - 用于非线性系统
 *
 * 状态向量: [角度, 角速度] (2x1)
 *
 * 非线性状态方程: x(k) = f(x(k-1), dt)
 *   x[0] = x[0] + x[1] * dt           // 角度 += 角速度 * dt
 *   x[1] = x[1]                        // 角速度保持不变（零阶模型）
 *
 * 非线性观测方程: z(k) = h(x(k))
 *   z = x[0]  // 只观测角度，不观测角速度
 */
class ExtendedKalmanFilter {
 public:
  /**
   * @brief 构造函数
   * @param q 过程噪声系数 - 越大越相信预测，越小越相信测量
   * @param r 测量噪声系数 - 越大越相信预测，越小越相信测量
   */
  ExtendedKalmanFilter(double q = 0.01, double r = 1.0) {
    Q_coeff = q;
    R = r;
    // 初始状态: [角度, 角速度]
    X = (cv::Mat_<double>(2, 1) << 0.0, 0.0);
    // 初始协方差矩阵
    P = cv::Mat::eye(2, 2, CV_64F);
  }

  /**
   * @brief 更新滤波器（包含预测和更新两个阶段）
   * @param z_measurement 测量值（当前观测到的角度，单位：度）
   * @param dt 时间间隔（秒）
   * @return 滤波后的角度估计值（单位：度）
   */
  double update(double z_measurement, double dt) {
    if (dt <= 0.0) dt = 0.033;  // 默认30fps

    // ========== 预测阶段 (Predict) ==========
    // 计算Jacobian矩阵 F (状态转移矩阵的一阶导数)
    // f(x) = [x0 + x1*dt, x1]^T
    // F = df/dx = [[1, dt], [0, 1]]
    cv::Mat F = (cv::Mat_<double>(2, 2) << 1.0, dt, 0.0, 1.0);

    // 状态预测
    X = predictState(X, dt);

    // 协方差预测
    // P = F * P * F^T + Q
    cv::Mat Q = Q_coeff * cv::Mat::eye(2, 2, CV_64F);
    P = F * P * F.t() + Q;

    // ========== 更新阶段 (Update) ==========
    // 计算Jacobian矩阵 H (观测函数的一阶导数)
    // h(x) = x[0]  (只观测角度)
    // H = dh/dx = [1, 0]
    cv::Mat H = (cv::Mat_<double>(1, 2) << 1.0, 0.0);

    // 预测观测值
    double z_predicted = X.at<double>(0, 0);

    // 观测残差 (innovation)
    double y = z_measurement - z_predicted;

    // 残差协方差
    // S = H * P * H^T + R
    cv::Mat S = H * P * H.t() + cv::Mat(1, 1, CV_64F, R);

    // 卡尔曼增益
    // K = P * H^T * S^-1
    cv::Mat K = P * H.t() * S.inv();

    // 状态更新
    // X = X + K * y
    X = X + K * cv::Mat(1, 1, CV_64F, y);

    // 协方差更新
    // P = (I - K*H) * P
    P = (cv::Mat::eye(2, 2, CV_64F) - K * H) * P;

    return X.at<double>(0, 0);  // 返回估计的角度
  }

  /**
   * @brief 获取当前估计的角速度
   * @return 角速度（单位：度/秒）
   */
  double getVelocity() const { return X.at<double>(1, 0); }

  /**
   * @brief 获取当前估计的角度
   * @return 角度（单位：度）
   */
  double getAngle() const { return X.at<double>(0, 0); }

  /**
   * @brief 重置滤波器状态
   * @param initial_angle 初始角度（单位：度）
   * @param initial_velocity 初始角速度（单位：度/秒），默认为0
   */
  void reset(double initial_angle = 0.0, double initial_velocity = 0.0) {
    X = (cv::Mat_<double>(2, 1) << initial_angle, initial_velocity);
    P = cv::Mat::eye(2, 2, CV_64F);
  }

 private:
  /**
   * @brief 非线性状态预测函数
   * @param state [角度, 角速度]^T
   * @param dt 时间间隔
   * @return 预测后的状态
   */
  cv::Mat predictState(const cv::Mat& state, double dt) {
    double angle = state.at<double>(0, 0);
    double velocity = state.at<double>(1, 0);
    // x[0] = x[0] + x[1] * dt
    // x[1] = x[1]
    return (cv::Mat_<double>(2, 1) << angle + velocity * dt, velocity);
  }

  cv::Mat X;       // 状态向量 [角度, 角速度]^T
  cv::Mat P;       // 状态协方差矩阵 (2x2)
  double Q_coeff;  // 过程噪声系数
  double R;        // 测量噪声
};

}  // namespace Tools
