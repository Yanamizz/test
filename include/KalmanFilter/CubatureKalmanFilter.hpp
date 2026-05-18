/**
 * @file    include/KalmanFilter/CubatureKalmanFilter.hpp
 * @brief   实现用于角度与角速度估计的 Cubature Kalman Filter。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Tools {

/**
 * @brief Cubature Kalman Filter (CKF) for a 2D state [angle, velocity].
 *
 * State model:
 *   x0 = x0 + x1 * dt
 *   x1 = x1
 *
 * Measurement model:
 *   z = x0
 */
class CubatureKalmanFilter {
 public:
  /**
   * @param q Process noise scale.
   * @param r Measurement noise.
   */
  CubatureKalmanFilter(double q = 0.01, double r = 1.0) : Q_coeff(q), R(r), n(2) {
    X = (cv::Mat_<double>(2, 1) << 0.0, 0.0);
    P = cv::Mat::eye(n, n, CV_64F);
  }

  /**
   * @brief Run one predict-update cycle.
   * @param z_measurement Measured angle.
   * @param dt Time step in seconds.
   * @return Estimated angle.
   */
  double update(double z_measurement, double dt) {
    if (dt <= 0.0) dt = 0.05;

    // Predict
    std::vector<cv::Mat> xi_pred = generateCubaturePoints(X, P);
    std::vector<cv::Mat> x_prop(xi_pred.size());
    for (size_t i = 0; i < xi_pred.size(); ++i) {
      x_prop[i] = predictStateNonlinear(xi_pred[i], dt);
    }

    cv::Mat X_pred = computeMean(x_prop);
    cv::Mat P_pred = computeCovariance(x_prop, X_pred);
    P_pred += Q_coeff * cv::Mat::eye(n, n, CV_64F);

    // Update
    std::vector<cv::Mat> xi_upd = generateCubaturePoints(X_pred, P_pred);

    std::vector<double> z_sigma(xi_upd.size());
    for (size_t i = 0; i < xi_upd.size(); ++i) {
      z_sigma[i] = xi_upd[i].at<double>(0, 0);
    }

    const double w = 1.0 / static_cast<double>(2 * n);
    double z_pred = 0.0;
    for (double z : z_sigma) z_pred += w * z;

    double Pzz = 0.0;
    cv::Mat Pxz = cv::Mat::zeros(n, 1, CV_64F);
    for (size_t i = 0; i < xi_upd.size(); ++i) {
      double dz = z_sigma[i] - z_pred;
      cv::Mat dx = xi_upd[i] - X_pred;
      Pzz += w * dz * dz;
      Pxz += w * dx * dz;
    }
    Pzz += R;

    const double eps = 1e-9;
    if (std::abs(Pzz) < eps) Pzz = (Pzz >= 0.0 ? eps : -eps);

    cv::Mat K = Pxz / Pzz;
    double innovation = z_measurement - z_pred;

    X = X_pred + K * innovation;
    P = P_pred - K * Pzz * K.t();

    // Keep covariance symmetric to limit numeric drift.
    P = 0.5 * (P + P.t());

    return X.at<double>(0, 0);
  }

  double getVelocity() const { return X.at<double>(1, 0); }

  void setVelocity(double velocity) { X.at<double>(1, 0) = velocity; }

  double getAngle() const { return X.at<double>(0, 0); }

  void reset(double initial_angle = 0.0, double initial_velocity = 0.0) {
    X = (cv::Mat_<double>(2, 1) << initial_angle, initial_velocity);
    P = cv::Mat::eye(n, n, CV_64F);
  }

 private:
  // Generate 2n cubature points x +/- sqrt(n) * S * e_i, where S*S^T = P.
  std::vector<cv::Mat> generateCubaturePoints(const cv::Mat& mean, const cv::Mat& cov) const {
    std::vector<cv::Mat> points;
    points.reserve(2 * n);

    cv::Mat cov_regularized = cov + 1e-6 * cv::Mat::eye(n, n, CV_64F);
    cov_regularized = 0.5 * (cov_regularized + cov_regularized.t());

    cv::Mat evals;  // 1 x n
    cv::Mat evecs;  // n x n (rows are eigenvectors in OpenCV)
    cv::eigen(cov_regularized, evals, evecs);

    cv::Mat sqrtD = cv::Mat::zeros(n, n, CV_64F);
    for (int i = 0; i < n; ++i) {
      double ev = evals.at<double>(0, i);
      if (ev < 0.0) ev = 0.0;
      sqrtD.at<double>(i, i) = std::sqrt(ev);
    }

    cv::Mat V = evecs;
    cv::Mat S = V.t() * sqrtD * V;
    cv::Mat L = S * std::sqrt(static_cast<double>(n));

    for (int i = 0; i < n; ++i) {
      cv::Mat delta = L.col(i).clone();
      points.push_back(mean + delta);
      points.push_back(mean - delta);
    }

    return points;
  }

  cv::Mat computeMean(const std::vector<cv::Mat>& points) const {
    const double w = 1.0 / static_cast<double>(2 * n);
    cv::Mat mean = cv::Mat::zeros(n, 1, CV_64F);
    for (const auto& point : points) {
      mean += w * point;
    }
    return mean;
  }

  cv::Mat computeCovariance(const std::vector<cv::Mat>& points, const cv::Mat& mean) const {
    const double w = 1.0 / static_cast<double>(2 * n);
    cv::Mat cov = cv::Mat::zeros(n, n, CV_64F);
    for (const auto& point : points) {
      cv::Mat diff = point - mean;
      cov += w * diff * diff.t();
    }
    return cov;
  }

  cv::Mat predictStateNonlinear(const cv::Mat& state, double dt) const {
    double angle = state.at<double>(0, 0);
    double velocity = state.at<double>(1, 0);
    return (cv::Mat_<double>(2, 1) << angle + velocity * dt, velocity);
  }

  cv::Mat X;
  cv::Mat P;
  double Q_coeff;
  double R;
  int n;
};

}  // namespace Tools
