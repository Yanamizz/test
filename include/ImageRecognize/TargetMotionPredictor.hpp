#pragma once

#include <algorithm>
#include <array>

#include <opencv2/opencv.hpp>

#include "ImageRecognize/OutputDataProcess.hpp"

namespace ImageRecognize {

struct MotionPredictionResult {
  bool valid = false;
  std::array<float, 6> box{};
  cv::Point2f center{};
};

class TargetMotionPredictor {
 public:
  void Reset() {
    initialized_ = false;
    observed_center_ = {};
    estimated_center_ = {};
    smoothed_velocity_ = {};
    observed_size_ = {1.0f, 1.0f};
    estimated_box_ = {};
    elapsed_since_observation_sec_ = 0.0;
  }

  bool HasState() const { return initialized_; }

  MotionPredictionResult ObserveAndPredict(const std::array<float, 6> &box, double dt, const cv::Size &frame_size) {
    const cv::Point2f observed_center = BoxCenter_(box);
    const cv::Size2f observed_size = BoxSize_(box);
    const float safe_dt = SafeDt_(dt);

    if (!initialized_) {
      initialized_ = true;
      observed_center_ = observed_center;
      estimated_center_ = observed_center;
      observed_size_ = observed_size;
      estimated_box_ = box;
      elapsed_since_observation_sec_ = 0.0;
      return MakePrediction_(box, observed_center, observed_size, dt, frame_size, false);
    }

    const double elapsed =
        std::max(elapsed_since_observation_sec_ + static_cast<double>(safe_dt), static_cast<double>(kMinDtSec));
    const cv::Point2f measured_velocity{(observed_center.x - observed_center_.x) / static_cast<float>(elapsed),
                                        (observed_center.y - observed_center_.y) / static_cast<float>(elapsed)};
    smoothed_velocity_.x =
        kVelocitySmoothing * smoothed_velocity_.x + (1.0f - kVelocitySmoothing) * measured_velocity.x;
    smoothed_velocity_.y =
        kVelocitySmoothing * smoothed_velocity_.y + (1.0f - kVelocitySmoothing) * measured_velocity.y;

    observed_center_ = observed_center;
    observed_size_ = observed_size;
    elapsed_since_observation_sec_ = 0.0;

    const auto prediction = MakePrediction_(box, observed_center, observed_size, dt, frame_size, true);
    estimated_center_ = prediction.center;
    estimated_box_ = prediction.box;
    return prediction;
  }

  MotionPredictionResult Predict(double dt, const cv::Size &frame_size) {
    if (!initialized_) {
      return {};
    }

    elapsed_since_observation_sec_ += static_cast<double>(SafeDt_(dt));
    const auto prediction = MakePrediction_(estimated_box_, estimated_center_, observed_size_, dt, frame_size, true);
    estimated_center_ = prediction.center;
    estimated_box_ = prediction.box;
    return prediction;
  }

 private:
  static constexpr float kVelocitySmoothing = 0.60f;
  static constexpr float kPredictionHorizonScale = 1.0f;
  static constexpr float kMaxPredictionHorizonSec = 0.05f;
  static constexpr float kMinDtSec = 1e-3f;

  static cv::Point2f BoxCenter_(const std::array<float, 6> &box) {
    return {0.5f * (box[0] + box[2]), 0.5f * (box[1] + box[3])};
  }

  static cv::Size2f BoxSize_(const std::array<float, 6> &box) {
    return {std::max(1.0f, box[2] - box[0]), std::max(1.0f, box[3] - box[1])};
  }

  static float SafeDt_(double dt) { return static_cast<float>(std::max(dt, static_cast<double>(kMinDtSec))); }

  MotionPredictionResult MakePrediction_(const std::array<float, 6> &base_box, const cv::Point2f &base_center,
                                         const cv::Size2f &size, double dt, const cv::Size &frame_size,
                                         bool use_velocity) const {
    MotionPredictionResult prediction{};
    if (!initialized_) {
      return prediction;
    }

    cv::Point2f center = base_center;
    if (use_velocity) {
      const float safe_dt = SafeDt_(dt);
      const float horizon = std::min(safe_dt * kPredictionHorizonScale, kMaxPredictionHorizonSec);
      center.x += smoothed_velocity_.x * horizon;
      center.y += smoothed_velocity_.y * horizon;
    }

    if (frame_size.width > 0) {
      center.x = std::clamp(center.x, 0.0f, static_cast<float>(frame_size.width - 1));
    }
    if (frame_size.height > 0) {
      center.y = std::clamp(center.y, 0.0f, static_cast<float>(frame_size.height - 1));
    }

    const float half_w = 0.5f * size.width;
    const float half_h = 0.5f * size.height;

    std::array<float, 6> predicted_box = base_box;
    predicted_box[0] = center.x - half_w;
    predicted_box[1] = center.y - half_h;
    predicted_box[2] = center.x + half_w;
    predicted_box[3] = center.y + half_h;

    if (frame_size.width > 0) {
      const float max_x = static_cast<float>(frame_size.width - 1);
      predicted_box[0] = std::clamp(predicted_box[0], 0.0f, max_x);
      predicted_box[2] = std::clamp(predicted_box[2], 0.0f, max_x);
      if (predicted_box[2] < predicted_box[0]) {
        std::swap(predicted_box[0], predicted_box[2]);
      }
    }
    if (frame_size.height > 0) {
      const float max_y = static_cast<float>(frame_size.height - 1);
      predicted_box[1] = std::clamp(predicted_box[1], 0.0f, max_y);
      predicted_box[3] = std::clamp(predicted_box[3], 0.0f, max_y);
      if (predicted_box[3] < predicted_box[1]) {
        std::swap(predicted_box[1], predicted_box[3]);
      }
    }

    prediction.valid = true;
    prediction.center = center;
    prediction.box = predicted_box;
    return prediction;
  }

  bool initialized_ = false;
  cv::Point2f observed_center_{};
  cv::Point2f estimated_center_{};
  cv::Point2f smoothed_velocity_{};
  cv::Size2f observed_size_{1.0f, 1.0f};
  std::array<float, 6> estimated_box_{};
  double elapsed_since_observation_sec_ = 0.0;
};

}  // namespace ImageRecognize