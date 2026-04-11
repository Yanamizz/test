#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>

namespace Tools {

class KalmanFilter {
 public:
  KalmanFilter(double q = 0.01, double r = 0.1) {
    Q = q;                                 // 过程噪声：相信预测模型的程度
    R = r;                                 // 测量噪声：相信识别框的程度（增大此值可减少震荡）
    X = (cv::Mat_<double>(2, 1) << 0, 0);  // 初始状态 [角度, 角速度]
    P = cv::Mat::eye(2, 2, CV_64F);        // 初始协方差
  }

  // 输入观测值（绝对角度）和时间间隔 dt
  double update(double measurement, double dt) {
    // 1. 预测 (Predict)
    cv::Mat F = (cv::Mat_<double>(2, 2) << 1, dt, 0, 1);  // 状态转移矩阵
    X = F * X;
    P = F * P * F.t() + Q * cv::Mat::eye(2, 2, CV_64F);

    // 2. 更新 (Update)
    cv::Mat H = (cv::Mat_<double>(1, 2) << 1, 0);  // 观测矩阵：我们只能观测到角度
    cv::Mat S = H * P * H.t() + R;
    cv::Mat K = P * H.t() * S.inv();  // 卡尔曼增益

    cv::Mat Z = (cv::Mat_<double>(1, 1) << measurement);
    X = X + K * (Z - H * X);
    P = (cv::Mat::eye(2, 2, CV_64F) - K * H) * P;

    return X.at<double>(0, 0);  // 返回滤波后的角度
  }

  double getVelocity() const { return X.at<double>(1, 0); }  // 获取估算的角速度

  void setVelocity(double velocity) { X.at<double>(1, 0) = velocity; }

  double getAngle() const { return X.at<double>(0, 0); }

  void reset(double initial_angle = 0.0, double initial_velocity = 0.0) {
    X = (cv::Mat_<double>(2, 1) << initial_angle, initial_velocity);
    P = cv::Mat::eye(2, 2, CV_64F);
  }

 private:
  cv::Mat X, P;
  double Q, R;
};

}  // namespace Tools
