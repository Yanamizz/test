/**
 * @file    include/Tools/LaserAngleCalculate.hpp
 * @brief   提供目标距离估计、目标尺寸标定与激光 pitch 几何补偿能力。
 */

#pragma once

#include "Tools/CameraData.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>

namespace Tools {
inline constexpr float kLinearInterpEpsilon = 1e-6f;
inline constexpr float kRadiansToDegrees = 57.29577951308232f;
inline constexpr float kNearCalibrationHorizontalDistanceM = 10.0f;
inline constexpr float kFarCalibrationHorizontalDistanceM = 24.0f;

enum class CalibrationStage {
  Stage12,
  Stage3,
};

class DistanceCalculator {
public:
  struct CalibrationTargetHeights {
    float near_calibration_target_height;
    float far_calibration_target_height;
  };

  struct CalibrationTargetWidths {
    float near_calibration_target_width;
    float far_calibration_target_width;
    float near_width_pixel;
    float far_width_pixel;
  };

  enum class DistanceSource {
    None,
    Width,
    Height,
  };

  struct DistanceDebugInfo {
    float used_distance = 0.0f;
    float width_distance = 0.0f;
    float height_distance = 0.0f;
    DistanceSource source = DistanceSource::None;
  };

  static CalibrationTargetHeights GetCalibrationTargetHeights() {
    const auto params = ParamsForStage(ActiveStage());
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetWidths GetCalibrationTargetWidths() {
    return GetCalibrationTargetWidths(ActiveStage());
  }

  static float
  CalculateLaserPitchCompensationDeg(float distance_m,
                                     float visual_pitch_offset_deg = 0.0f) {
    if (!std::isfinite(distance_m) || distance_m <= 0.0f) {
      return 0.0f;
    }

    const auto params = ParamsForStage(ActiveStage());
    if (!params.enable_laser_pitch_compensation ||
        distance_m < params.laser_comp_min_distance_m ||
        distance_m > params.laser_comp_max_distance_m ||
        params.laser_z_offset_m <= 0.0f || params.laser_converge_x_m <= 0.0f) {
      return 0.0f;
    }

    const float visual_pitch_rad = visual_pitch_offset_deg / kRadiansToDegrees;
    const float converge_angle =
        std::atan2(params.laser_z_offset_m, params.laser_converge_x_m);
    const float required_laser_angle = std::atan2(
        distance_m * std::tan(visual_pitch_rad) - params.laser_z_offset_m,
        distance_m);
    return (required_laser_angle + converge_angle - visual_pitch_rad) *
           kRadiansToDegrees;
  }

  static CalibrationTargetHeights
  GetCalibrationTargetHeights(CalibrationStage stage) {
    const auto params = ParamsForStage(stage);
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetWidths
  GetCalibrationTargetWidths(CalibrationStage stage) {
    const auto params = ParamsForStage(stage);
    return {params.near_calibration_target_width,
            params.far_calibration_target_width, params.near_width_pixel,
            params.far_width_pixel};
  }

  static void SetCalibrationTargetHeights(float near_height, float far_height) {
    SetCalibrationTargetHeights(ActiveStage(), near_height, far_height);
  }

  static void SetCalibrationTargetHeights(CalibrationStage stage,
                                          float near_height, float far_height) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageParams(params, stage);
    stage_params.near_calibration_target_height = near_height;
    stage_params.far_calibration_target_height = far_height;
  }

  static void SetCalibrationTargetWidths(CalibrationStage stage,
                                         float near_width, float far_width) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageParams(params, stage);
    stage_params.near_calibration_target_width = near_width;
    stage_params.far_calibration_target_width = far_width;
  }

  static void SetActiveStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    MutableParams().active_stage = stage;
  }

  static CalibrationStage ActiveStage() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams().active_stage;
  }

  void ResetFilter() {
    has_filtered_distance_ = false;
    filtered_distance_ = 0.0f;
  }

  float CalculateDistance(float pixel_h) {
    return CalculateDistanceByPixelHeight(pixel_h);
  }

  float CalculateDistance(float x1, float y1, float x2, float y2) {
    return CalculateDistanceWithDebug(x1, y1, x2, y2).used_distance;
  }

  DistanceDebugInfo CalculateDistanceWithDebug(float x1, float y1, float x2,
                                               float y2) {
    const float raw_w = std::abs(x2 - x1);
    const float raw_h = std::abs(y2 - y1);
    if ((!std::isfinite(raw_w) || raw_w <= 0.0f) &&
        (!std::isfinite(raw_h) || raw_h <= 0.0f)) {
      return {};
    }

    return CalculateDistanceByPixelSize(raw_w, raw_h);
  }

  static const char *DistanceSourceName(DistanceSource source) {
    switch (source) {
    case DistanceSource::Width:
      return "WIDTH";
    case DistanceSource::Height:
      return "HEIGHT";
    case DistanceSource::None:
    default:
      return "NONE";
    }
  }

private:
  struct StageTunableParams {
    float near_calibration_target_height;
    float near_pixel;
    float far_calibration_target_height;
    float far_pixel;
    float near_calibration_target_width;
    float near_width_pixel;
    float far_calibration_target_width;
    float far_width_pixel;
    float laser_z_offset_m;
    float laser_converge_x_m;
    float laser_comp_min_distance_m;
    float laser_comp_max_distance_m;
    bool enable_laser_pitch_compensation;
  };

  struct TunableParams {
    StageTunableParams stage12;
    StageTunableParams stage3;
    float distance_filter_alpha;
    float filter_reset_ratio;
    CalibrationStage active_stage;
  };

  struct RuntimeSnapshot {
    StageTunableParams stage;
    float distance_filter_alpha;
    float filter_reset_ratio;
  };

  float CalculateDistanceByPixelHeight(float pixel_h) {
    if (!std::isfinite(pixel_h) || pixel_h <= 0.0f)
      return 0.0f;

    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    const float raw_distance =
        EstimateCalibratedDistanceByHeight(pixel_h, params.stage);
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0f)
      return 0.0f;
    return FilterDistance(raw_distance, params);
  }

  DistanceDebugInfo CalculateDistanceByPixelSize(float pixel_w, float pixel_h) {
    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    DistanceDebugInfo result{};
    result.width_distance =
        EstimateCalibratedDistanceByWidth(pixel_w, params.stage);
    result.height_distance =
        EstimateCalibratedDistanceByHeight(pixel_h, params.stage);

    if (IsValidDistance(result.width_distance)) {
      result.used_distance = FilterDistance(result.width_distance, params);
      result.source = DistanceSource::Width;
      return result;
    }

    if (IsValidDistance(result.height_distance)) {
      result.used_distance = FilterDistance(result.height_distance, params);
      result.source = DistanceSource::Height;
    }
    return result;
  }

  float
  EstimateCalibratedDistanceByHeight(float pixel_h,
                                     const StageTunableParams &params) const {
    const float near_height = params.near_calibration_target_height;
    const float far_height = params.far_calibration_target_height;
    const float near_pixel = params.near_pixel;
    const float far_pixel = params.far_pixel;

    if (near_pixel <= 0.0f || far_pixel <= 0.0f || near_height <= 0.0f ||
        far_height <= 0.0f ||
        std::abs(far_pixel - near_pixel) <= kLinearInterpEpsilon) {
      return EstimateDistanceByHeight(far_height, pixel_h);
    }

    const float t = std::clamp(
        (pixel_h - near_pixel) / (far_pixel - near_pixel), 0.0f, 1.0f);
    const float target_height = near_height + t * (far_height - near_height);
    return EstimateDistanceByHeight(target_height, pixel_h);
  }

  float
  EstimateCalibratedDistanceByWidth(float pixel_w,
                                    const StageTunableParams &params) const {
    const float near_width = params.near_calibration_target_width;
    const float far_width = params.far_calibration_target_width;
    const float near_pixel = params.near_width_pixel;
    const float far_pixel = params.far_width_pixel;

    if (!IsValidPixel(pixel_w)) {
      return 0.0f;
    }
    if (!IsValidPixel(near_pixel) || !IsValidPixel(far_pixel) ||
        near_width <= 0.0f || far_width <= 0.0f ||
        std::abs(far_pixel - near_pixel) <= kLinearInterpEpsilon) {
      return EstimateDistanceByWidth(far_width, pixel_w);
    }

    const float t = std::clamp(
        (pixel_w - near_pixel) / (far_pixel - near_pixel), 0.0f, 1.0f);
    const float target_width = near_width + t * (far_width - near_width);
    return EstimateDistanceByWidth(target_width, pixel_w);
  }

  float EstimateDistanceByHeight(float target_height, float pixel_h) const {
    if (!std::isfinite(target_height) || target_height <= 0.0f ||
        !std::isfinite(pixel_h) || pixel_h <= 0.0f ||
        !std::isfinite(focal_y_px_) || focal_y_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_height * focal_y_px_) / pixel_h;
  }

  float EstimateDistanceByWidth(float target_width, float pixel_w) const {
    if (!std::isfinite(target_width) || target_width <= 0.0f ||
        !IsValidPixel(pixel_w) || !std::isfinite(focal_x_px_) ||
        focal_x_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_width * focal_x_px_) / pixel_w;
  }

  float FilterDistance(float raw_distance, const RuntimeSnapshot &params) {
    if (!has_filtered_distance_ || filtered_distance_ <= 0.0f) {
      filtered_distance_ = raw_distance;
      has_filtered_distance_ = true;
      return raw_distance;
    }

    const float reset_threshold =
        filtered_distance_ * params.filter_reset_ratio;
    if (std::abs(raw_distance - filtered_distance_) > reset_threshold) {
      filtered_distance_ = raw_distance;
      return raw_distance;
    }

    filtered_distance_ +=
        params.distance_filter_alpha * (raw_distance - filtered_distance_);
    return filtered_distance_;
  }

  static RuntimeSnapshot SnapshotCurrentRuntime_() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    const auto &params = MutableParams();
    return {StageParams(params, params.active_stage),
            params.distance_filter_alpha, params.filter_reset_ratio};
  }

  static StageTunableParams ParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageParams(MutableParams(), stage);
  }

  static StageTunableParams &StageParams(TunableParams &params,
                                         CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? params.stage3 : params.stage12;
  }

  static const StageTunableParams &StageParams(const TunableParams &params,
                                               CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? params.stage3 : params.stage12;
  }

  static bool IsValidPixel(float pixel) {
    return std::isfinite(pixel) && pixel > 0.0f;
  }

  static bool IsValidDistance(float distance_m) {
    return std::isfinite(distance_m) && distance_m > 0.0f;
  }

  static constexpr float
  CalibrationTargetMetersFromPixel(float horizontal_distance_m, float pixel,
                                   double focal_px) {
    return (horizontal_distance_m > 0.0f && pixel > 0.0f && focal_px > 0.0)
               ? static_cast<float>(horizontal_distance_m * pixel / focal_px)
               : 0.0f;
  }

  static TunableParams &MutableParams() {
    static TunableParams params = DefaultParams();
    return params;
  }

  static std::mutex &ParamsMutex() {
    static std::mutex mutex;
    return mutex;
  }

  static const TunableParams &DefaultParams() {
    // ===== 调参集中区（统一放在文件末尾）=====
    static const TunableParams p{
        {CalibrationTargetMetersFromPixel(kNearCalibrationHorizontalDistanceM,
                                          111.982f, CameraData::kFocalY),
         // stage12 near_calibration_target_height（米，按 near/far pixel 与
         // 水平距离自动反推）
         111.982f, // stage12 near_pixel（px）
         CalibrationTargetMetersFromPixel(kFarCalibrationHorizontalDistanceM,
                                          47.952f, CameraData::kFocalY),
         47.952f, // stage12 far_pixel（px）
         CalibrationTargetMetersFromPixel(kNearCalibrationHorizontalDistanceM,
                                          105.029f, CameraData::kFocalX),
         105.029f, // stage12 near_width_pixel（px，未标定时置 0）
         CalibrationTargetMetersFromPixel(kFarCalibrationHorizontalDistanceM,
                                          43.274f, CameraData::kFocalX),
         43.274f, // stage12 far_width_pixel（px，未标定时置 0）
         0.090f,  // stage12 laser_z_offset_m: 激光在相机上方 0.09m
         14.313f, // stage12 laser_converge_x_m: 光轴交汇前向距离
         10.0f,   // stage12 laser_comp_min_distance_m
         24.0f,   // stage12 laser_comp_max_distance_m
         true},   // stage12 enable_laser_pitch_compensation

        {CalibrationTargetMetersFromPixel(kNearCalibrationHorizontalDistanceM,
                                          102.006f, CameraData::kFocalY),
         // stage3 near_calibration_target_height（米，按 near/far pixel 与
         // 水平距离自动反推）
         102.006f, // stage3 near_pixel（px）
         CalibrationTargetMetersFromPixel(kFarCalibrationHorizontalDistanceM,
                                          56.941f, CameraData::kFocalY),
         56.941f, // stage3 far_pixel（px）
         CalibrationTargetMetersFromPixel(kNearCalibrationHorizontalDistanceM,
                                          85.880f, CameraData::kFocalX),
         85.880f, // stage3 near_width_pixel（px，未标定时置 0）
         CalibrationTargetMetersFromPixel(kFarCalibrationHorizontalDistanceM,
                                          49.605f, CameraData::kFocalX),
         49.605f, // stage3 far_width_pixel（px，未标定时置 0）
         0.090f,  // stage3 laser_z_offset_m: 激光在相机上方 0.09m
         14.313f, // stage3 laser_converge_x_m: 光轴交汇前向距离
         10.0f,   // stage3 laser_comp_min_distance_m
         24.0f,   // stage3 laser_comp_max_distance_m
         true},   // stage3 enable_laser_pitch_compensation
        0.25f, // distance_filter_alpha: 一阶滤波系数，越大响应越快
        0.5f,  // filter_reset_ratio: 距离突变超过该比例时重置滤波
        CalibrationStage::Stage12 // active_stage: 当前使用的标定阶段
    };
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  CameraData camera_data_;
  float focal_x_px_ = camera_data_.cameraMatrix.at<double>(0, 0);
  float focal_y_px_ = camera_data_.cameraMatrix.at<double>(1, 1);
};

} // namespace Tools
