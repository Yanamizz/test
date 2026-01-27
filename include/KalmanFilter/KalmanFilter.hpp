/**
 * @file include/KalmanFilter/KalmanFilter.hpp
 * @brief 使用 third_lib/kalman 提供的 Square-Root Unscented Kalman Filter (SR-UKF)
 *
 * 说明：
 * - 原来的头文件中实现了一个自定义 UKF（状态 [x,y,v,yaw,yaw_rate]），
 *   为了使用第三方实现的更稳健的 SR-UKF，头文件现在提供了一个轻量的封装接口，
 *   使用 PIMPL（实现隐藏）以便在源文件中直接依赖 third_lib/kalman 的具体类型与模板参数。
 * - 好处：不把第三方模板细节（可能很复杂）暴露到库的公共头里，编译依赖更清晰。
 * - 接口变化：对外保持原有 Tracker2D 接口不变：
 *     - `Eigen::Vector2d update(const cv::Rect &bbox_rect)`
 *     - `Eigen::Vector2d update(const cv::Rect &bbox_rect, double dt_seconds)`
 *   因此现有调用代码无需修改。
 * - 实现注意事项（在 .cpp 中完成）：
 *     - 在实现文件中创建 `Impl`，并使用 `kalman::SquareRootUnscentedKalmanFilter` 或相应的 SR-UKF 类型，
 *       根据 third_lib/kalman 的示例（examples）设置过程模型、测量模型、过程/测量噪声、初始协方差等。
 *     - 如果希望把 SR-UKF 的 Q/R 等参数暴露为接口，请告诉我我会在头文件中加入相应 setter/getter。
 *
 * 注意：本头文件仅包含必要的轻量依赖（Eigen、OpenCV、chrono、memory），
 *       第三方 kalman 头文件应只出现在实现文件中以避免过度模板膨胀。
 */

#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <chrono>
#include <memory>

// 将实现放入头文件：直接包含 third_lib/kalman 的头以实现 SR-UKF。
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/MeasurementModel.hpp>
#include <kalman/SquareRootBase.hpp>
#include <kalman/Matrix.hpp>
#include <kalman/Types.hpp>
#include <cmath>

// 注意：为了把所有实现放在头文件中，下面把状态/模型/滤波器实例化写入头内，
// 这会增加包含该头的编译单元编译时间，但满足“只需要接口给主程序并在头内实现”的要求。

class UKFTracker {
 public:
  UKFTracker() {
    // 构造默认 SR-UKF（alpha, beta, kappa）并设置默认 Q/R
    filter_ = FilterType(1e-3, 2.0, 0.0);

    // 默认过程噪声（对角）
    Kalman::Covariance<StateType> Q = Kalman::Covariance<StateType>::Zero();
    Q(0, 0) = 0.1;
    Q(1, 1) = 0.1;
    Q(2, 2) = 1.0;
    Q(3, 3) = 0.5;
    Q(4, 4) = 1.0;
    systemModel_.setCovariance(Q);

    // 默认测量噪声
    Kalman::Covariance<MeasType> R = Kalman::Covariance<MeasType>::Zero();
    R(0, 0) = 5.0;
    R(1, 1) = 5.0;
    measurementModel_.setCovariance(R);

    initialized_ = false;
  }

  ~UKFTracker() = default;

  // 保持与原接口：预测并更新，输入为测量位置和时间间隔
  Eigen::Vector2d predictAndUpdate(const Eigen::Vector2d &meas, double dt) {
    if (!initialized_) {
      StateType s;
      s.setZero();
      s.x() = meas.x();
      s.y() = meas.y();
      s.v() = 0.0;
      s.yaw() = 0.0;
      s.yawd() = 0.0;
      filter_.init(s);
      initialized_ = true;
      return meas;
    }

    ControlType u;
    u.setZero();
    u.dt() = dt;
    filter_.predict(systemModel_, u);
    MeasType z;
    z.setZero();
    z.mx() = meas.x();
    z.my() = meas.y();
    filter_.update(measurementModel_, z);
    const StateType &s = filter_.getState();
    return Eigen::Vector2d(s.x(), s.y());
  }

  // 新增：直接让主程序传入识别器得到的参数（bbox 与置信度/score）进行滤波
  // 说明：新增接口不会改变原有 update 的行为，只是提供更直接的调用方式。
  Eigen::Vector2d updateWithDetection(const cv::Rect &bbox, double score = 1.0, double dt = 0.033) {
    // 若需要，可用 score 调整测量噪声（简单示例：score 越高噪声越小）
    if (score > 0 && score <= 1.0) {
      double scale = 1.0 - 0.9 * score;  // score=1 => scale=0.1
      Kalman::Covariance<MeasType> R = Kalman::Covariance<MeasType>::Zero();
      R(0, 0) = 5.0 * scale;
      R(1, 1) = 5.0 * scale;
      measurementModel_.setCovariance(R);
    }

    if (bbox.area() == 0) {
      // 无检测：返回上次估计
      return last_pos_;
    }
    Eigen::Vector2d meas(bbox.x + bbox.width * 0.5, bbox.y + bbox.height * 0.5);
    last_pos_ = predictAndUpdate(meas, dt);
    return last_pos_;
  }

  // 保持 setter，用于从外部设置 Q/R（如果主程序需要）
  void setProcessNoise(const Eigen::MatrixXd &Q_in) {
    Kalman::Covariance<StateType> Q = Kalman::Covariance<StateType>::Zero();
    for (int i = 0; i < StateType::RowsAtCompileTime && i < (int)Q_in.rows(); ++i)
      for (int j = 0; j < StateType::RowsAtCompileTime && j < (int)Q_in.cols(); ++j) Q(i, j) = Q_in(i, j);
    systemModel_.setCovariance(Q);
  }

  void setMeasNoise(const Eigen::Matrix2d &R_in) {
    Kalman::Covariance<MeasType> R = Kalman::Covariance<MeasType>::Zero();
    R(0, 0) = R_in(0, 0);
    R(1, 1) = R_in(1, 1);
    measurementModel_.setCovariance(R);
  }

 private:
  // ------- 在头内定义所有类型 -------
  // State: [x, y, v, yaw, yaw_rate]
  template <typename T>
  struct St : public Kalman::Vector<T, 5> {
    KALMAN_VECTOR(St, T, 5)
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t V = 2;
    static constexpr size_t YAW = 3;
    static constexpr size_t YAWD = 4;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T v() const { return (*this)[V]; }
    T yaw() const { return (*this)[YAW]; }
    T yawd() const { return (*this)[YAWD]; }

    T &x() { return (*this)[X]; }
    T &y() { return (*this)[Y]; }
    T &v() { return (*this)[V]; }
    T &yaw() { return (*this)[YAW]; }
    T &yawd() { return (*this)[YAWD]; }
  };

  // 为了兼容 kalman 库的类型机制，我们使用在匿名命名空间中定义的具体 types
  using StateType = St<double>;
  // Control: single scalar dt
  template <typename T>
  struct Ct : public Kalman::Vector<T, 1> {
    KALMAN_VECTOR(Ct, T, 1)
    static constexpr size_t DT = 0;
    T dt() const { return (*this)[DT]; }
    T &dt() { return (*this)[DT]; }
  };
  using ControlType = Ct<double>;
  // Measurement: [x,y]
  template <typename T>
  struct Mt : public Kalman::Vector<T, 2> {
    KALMAN_VECTOR(Mt, T, 2)
    static constexpr size_t MX = 0;
    static constexpr size_t MY = 1;
    T mx() const { return (*this)[MX]; }
    T my() const { return (*this)[MY]; }
    T &mx() { return (*this)[MX]; }
    T &my() { return (*this)[MY]; }
  };
  using MeasType = Mt<double>;

  using FilterType = Kalman::SquareRootUnscentedKalmanFilter<StateType>;

  // 简单系统/测量模型定义（与实现文件相同）
  struct CTSystemModel : public Kalman::SystemModel<StateType, ControlType, Kalman::SquareRootBase> {
    StateType f(const StateType &x, const ControlType &u) const override {
      StateType out = x;
      double px = x(0), py = x(1), v = x(2), yaw = x(3), yawd = x(4), dt = u(0);
      if (std::abs(yawd) > 1e-6) {
        px += (v / yawd) * (std::sin(yaw + yawd * dt) - std::sin(yaw));
        py += (v / yawd) * (-std::cos(yaw + yawd * dt) + std::cos(yaw));
      } else {
        px += v * std::cos(yaw) * dt;
        py += v * std::sin(yaw) * dt;
      }
      out(0) = px;
      out(1) = py;
      out(2) = v;
      out(3) = yaw + yawd * dt;
      out(4) = yawd;
      return out;
    }
  };

  struct PositionMeasurementModel : public Kalman::MeasurementModel<StateType, MeasType, Kalman::SquareRootBase> {
    MeasType h(const StateType &x) const override {
      MeasType m;
      m(0) = x(0);
      m(1) = x(1);
      return m;
    }
  };

  // members
  FilterType filter_;
  CTSystemModel systemModel_;
  PositionMeasurementModel measurementModel_;
  bool initialized_{false};
  Eigen::Vector2d last_pos_{0, 0};
};

class Tracker2D {
 public:
  Tracker2D();
  ~Tracker2D();

  // 保持原接口：传入检测框，返回平滑后的中心点
  Eigen::Vector2d update(const cv::Rect &bbox_rect);
  Eigen::Vector2d update(const cv::Rect &bbox_rect, double dt_seconds);

  // 可选：如果需要直接设置 SR-UKF 的参数，可以在实现中添加对应方法并在此声明

  void init();

 private:
  double computeDt();

  UKFTracker ukf_;
  bool initialized_{false};
  double default_dt_{0.033};
  std::chrono::steady_clock::time_point last_time_;
  Eigen::Vector2d last_pos_{0, 0};
};

/*
 * 说明（开发者阅读）：
 * - 在 include/KalmanFilter/KalmanFilter.cpp 中实现 `UKFTracker::Impl`：
 *     struct UKFTracker::Impl {
 *       // 使用 third_lib/kalman 的 SR-UKF 类型、过程/测量模型的具体实现
 *     };
 * - Impl 应实现：初始化滤波器、生成 sigma 点或调用库 API、执行 predict/update、以及 Q/R 设置。
 * - 之所以把实现放到 .cpp，是为了避免把 kalman 的模板实现暴露在头文件，减少编译时间并避免模板冲突。
 *
 * 如果你希望我直接在头文件中把 SR-UKF 类型与模型展开（例如：直接依赖
 * `kalman/SquareRootUnscentedKalmanFilter.hpp` 并在头内实现），请回复确认，我会把全部具体实现放回头文件中。
 */
