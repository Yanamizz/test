#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

namespace Tools {

/**
 * @brief 无迹卡尔曼滤波 (UKF - Unscented Kalman Filter)
 *
 * UKF 相比标准KF和EKF的优势：
 * 1. 无需计算Jacobian矩阵（自动处理非线性）
 * 2. 使用Sigma点，对非线性更敏感
 * 3. 二阶精度，精度通常优于EKF（特别是高度非线性时）
 * 4. 计算量略大于KF/EKF，但可实时运行
 *
 * 状态向量: [角度, 角速度] (2x1)
 *
 * 非线性状态方程: x(k) = f(x(k-1), dt)
 *   x[0] = x[0] + x[1] * dt           // 角度 += 角速度 * dt
 *   x[1] = x[1]                        // 角速度保持不变（零阶模型）
 *
 * 观测方程: z(k) = h(x(k))
 *   z = x[0]  // 只观测角度
 */
class UnscentedKalmanFilter {
 public:
  /**
   * @brief 构造函数
   * @param q 过程噪声系数 - 越大越相信预测（推荐 0.01 ~ 0.1）
   * @param r 测量噪声系数 - 越大越相信预测（推荐 0.5 ~ 2.0）
   * @param alpha UKF参数 - 控制Sigma点的展开程度（推荐 1e-3）
   * @param beta UKF参数 - 先验知识参数（推荐 2.0，高斯分布时最优）
   * @param kappa UKF参数 - 调节参数（推荐 0）
   */
  UnscentedKalmanFilter(double q = 0.01, double r = 1.0, double alpha = 1e-3, double beta = 2.0, double kappa = 0.0)
      : Q_coeff(q), R(r), alpha(alpha), beta(beta), kappa(kappa) {
    n = 2;  // 状态维数 [角度, 角速度]
    lambda = alpha * alpha * (n + kappa) - n;

    // 初始化权重
    initializeWeights();

    // 初始状态
    X = (cv::Mat_<double>(2, 1) << 0.0, 0.0);
    P = cv::Mat::eye(2, 2, CV_64F);
  }

  /**
   * @brief 更新滤波器（包含预测和更新两个阶段）
   * @param z_measurement 测量值（观测到的角度，单位：度）
   * @param dt 时间间隔（秒）
   * @return 滤波后的角度估计值（单位：度）
   */
  double update(double z_measurement, double dt) {
    if (dt <= 0.0) dt = 0.05;  // 默认20fps

    // ========== 预测阶段 (Predict) ==========

    // 1. 生成Sigma点
    std::vector<cv::Mat> sigma_points = generateSigmaPoints(X, P);

    // 2. 通过非线性状态方程传播Sigma点
    std::vector<cv::Mat> sigma_points_pred(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); ++i) {
      sigma_points_pred[i] = predictStateNonlinear(sigma_points[i], dt);
    }

    // 3. 计算预测的状态均值和协方差
    cv::Mat X_pred = computeWeightedMean(sigma_points_pred, wm);
    cv::Mat P_pred = computeWeightedCovariance(sigma_points_pred, X_pred, wc);

    // 4. 加上过程噪声
    cv::Mat Q = Q_coeff * cv::Mat::eye(n, n, CV_64F);
    P_pred = P_pred + Q;

    // ========== 更新阶段 (Update) ==========

    // 5. 为预测状态重新生成Sigma点
    std::vector<cv::Mat> sigma_points_upd = generateSigmaPoints(X_pred, P_pred);

    // 6. 通过非线性观测方程传播Sigma点
    std::vector<double> z_sigma(sigma_points_upd.size());
    for (size_t i = 0; i < sigma_points_upd.size(); ++i) {
      z_sigma[i] = sigma_points_upd[i].at<double>(0, 0);  // 观测就是角度
    }

    // 7. 计算预测的观测均值和协方差
    double z_pred = 0.0;
    for (size_t i = 0; i < z_sigma.size(); ++i) {
      z_pred += wm[i] * z_sigma[i];
    }

    double Pzz = 0.0;  // 观测协方差
    for (size_t i = 0; i < z_sigma.size(); ++i) {
      double dz = z_sigma[i] - z_pred;
      Pzz += wc[i] * dz * dz;
    }
    Pzz += R;  // 加上测量噪声

    // 8. 计算交叉协方差 Pxz (n x 1)
    cv::Mat Pxz = cv::Mat::zeros(n, 1, CV_64F);
    for (size_t i = 0; i < sigma_points_upd.size(); ++i) {
      cv::Mat dx = sigma_points_upd[i] - X_pred;  // (n x 1)
      double dz = z_sigma[i] - z_pred;            // scalar
      Pxz += wc[i] * dx * dz;                     // (n x 1)
    }

    // 9. 计算卡尔曼增益 K (n x 1)
    const double eps = 1e-9;
    if (std::abs(Pzz) < eps) Pzz = (Pzz >= 0.0 ? eps : -eps);
    cv::Mat K = Pxz / Pzz;  // (n x 1)

    // 10. 更新状态
    double innovation = z_measurement - z_pred;
    X = X_pred + K * innovation;  // (n x 1)

    // 11. 更新协方差
    // P = P_pred - K * Pzz * K^T  等价于  P_pred - K * Pxz^T
    P = P_pred - K * Pxz.t();  // (n x n)

    return X.at<double>(0, 0);  // 返回估计的角度
  }

  /**
   * @brief 获取当前估计的角速度
   * @return 角速度（单位：度/秒）
   */
  double getVelocity() const { return X.at<double>(1, 0); }

  /**
   * @brief 强制设置角速度（单位：度/秒）
   * @param velocity 角速度
   */
  void setVelocity(double velocity) { X.at<double>(1, 0) = velocity; }

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
    P = cv::Mat::eye(n, n, CV_64F);
  }

 private:
  /**
   * @brief 初始化UKF权重
   */
  void initializeWeights() {
    int sigma_points_count = 2 * n + 1;
    wm.resize(sigma_points_count);
    wc.resize(sigma_points_count);

    // 中心点权重
    wm[0] = lambda / (n + lambda);
    wc[0] = lambda / (n + lambda) + (1.0 - alpha * alpha + beta);

    // 其他Sigma点权重
    double w = 1.0 / (2.0 * (n + lambda));
    for (int i = 1; i < sigma_points_count; ++i) {
      wm[i] = w;
      wc[i] = w;
    }
  }

  /**
   * @brief 生成Sigma点集合（使用SVD分解）
   * @param mean 均值向量
   * @param cov 协方差矩阵
   * @return Sigma点向量（共 2n+1 个）
   */
  std::vector<cv::Mat> generateSigmaPoints(const cv::Mat& mean, const cv::Mat& cov) {
    std::vector<cv::Mat> sigma_points;
    sigma_points.push_back(mean);  // 中心点

    // 用特征值分解构造协方差矩阵平方根，避免Mat::diag/SVD形状歧义
    cv::Mat cov_regularized = cov + 1e-6 * cv::Mat::eye(n, n, CV_64F);

    // 强制对称，减小数值误差
    cov_regularized = 0.5 * (cov_regularized + cov_regularized.t());

    cv::Mat eigenvalues;   // (n x 1)
    cv::Mat eigenvectors;  // (n x n), 列向量为特征向量
    cv::eigen(cov_regularized, eigenvalues, eigenvectors);

    cv::Mat sqrtD = cv::Mat::zeros(n, n, CV_64F);
    for (int i = 0; i < n; ++i) {
      double ev = (eigenvalues.rows == 1) ? eigenvalues.at<double>(0, i) : eigenvalues.at<double>(i, 0);
      if (ev < 0.0) ev = 0.0;  // 数值保护
      sqrtD.at<double>(i, i) = std::sqrt(ev);
    }

    // 注意：cv::eigen 的特征向量按“行”返回，因此这里使用 V^T * sqrtD * V
    cv::Mat V = eigenvectors;
    cv::Mat sqrtP = V.t() * sqrtD * V;

    double scale = n + lambda;
    if (scale <= 0.0) scale = 1e-6;
    cv::Mat L = sqrtP * std::sqrt(scale);

    // 生成2n个偏离点
    for (int i = 0; i < n; ++i) {
      sigma_points.push_back(mean + L.col(i).clone());
    }
    for (int i = 0; i < n; ++i) {
      sigma_points.push_back(mean - L.col(i).clone());
    }

    return sigma_points;
  }

  /**
   * @brief 计算加权均值
   */
  cv::Mat computeWeightedMean(const std::vector<cv::Mat>& points, const std::vector<double>& weights) {
    cv::Mat mean = cv::Mat::zeros(n, 1, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
      mean = mean + weights[i] * points[i];
    }
    return mean;
  }

  /**
   * @brief 计算加权协方差
   */
  cv::Mat computeWeightedCovariance(const std::vector<cv::Mat>& points, const cv::Mat& mean,
                                    const std::vector<double>& weights) {
    cv::Mat cov = cv::Mat::zeros(n, n, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
      cv::Mat diff = points[i] - mean;
      cov = cov + weights[i] * (diff * diff.t());
    }
    return cov;
  }

  /**
   * @brief 非线性状态预测函数
   */
  cv::Mat predictStateNonlinear(const cv::Mat& state, double dt) {
    double angle = state.at<double>(0, 0);
    double velocity = state.at<double>(1, 0);
    return (cv::Mat_<double>(2, 1) << angle + velocity * dt, velocity);
  }

  // 状态变量
  cv::Mat X;  // 状态向量 [角度, 角速度]^T
  cv::Mat P;  // 状态协方差矩阵

  // UKF参数
  double Q_coeff;  // 过程噪声系数
  double R;        // 测量噪声
  double alpha;    // Sigma点展开参数 (1e-3 ~ 1)
  double beta;     // 先验知识参数 (2 for Gaussian)
  double kappa;    // 调节参数 (0 or 3-n)
  double lambda;   // lambda = alpha^2 * (n + kappa) - n

  int n;                   // 状态维数
  std::vector<double> wm;  // 均值权重
  std::vector<double> wc;  // 协方差权重
};

}  // namespace Tools
