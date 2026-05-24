/**
 * @file    include/Tools/LaserAngleCalculate.hpp
 * @brief   提供目标距离估计与激光打击角度补偿计算能力。
 */

#pragma once

#include "Tools/CameraData.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>

namespace Tools {
inline constexpr float kLaserAnglePi = 3.1415926f;
inline constexpr float kLinearInterpEpsilon = 1e-6f;
inline constexpr float kRadToDeg = 180.0f / kLaserAnglePi;

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

  static CalibrationTargetHeights GetCalibrationTargetHeights() {
    const auto params = ParamsForStage(ActiveStage());
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetHeights
  GetCalibrationTargetHeights(CalibrationStage stage) {
    const auto params = ParamsForStage(stage);
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
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
    const float raw_h = std::abs(y2 - y1);
    if (!std::isfinite(raw_h) || raw_h <= 0.0f)
      return 0.0f;

    return CalculateDistanceByPixelHeight(raw_h);
  }

private:
  struct StageTunableParams {
    float near_calibration_target_height;
    float near_pixel;
    float far_calibration_target_height;
    float far_pixel;
  };

  struct TunableParams {
    StageTunableParams stage12;
    StageTunableParams stage3;
    float distance_filter_alpha;
    float filter_reset_ratio;
    CalibrationStage active_stage;
  };

  float CalculateDistanceByPixelHeight(float pixel_h) {
    if (!std::isfinite(pixel_h) || pixel_h <= 0.0f)
      return 0.0f;

    const float raw_distance = EstimateCalibratedDistanceByHeight(pixel_h);
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0f)
      return 0.0f;
    return FilterDistance(raw_distance);
  }

  float EstimateCalibratedDistanceByHeight(float pixel_h) const {
    const auto params = ParamsForStage(ActiveStage());
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

  float EstimateDistanceByHeight(float target_height, float pixel_h) const {
    if (!std::isfinite(target_height) || target_height <= 0.0f ||
        !std::isfinite(pixel_h) || pixel_h <= 0.0f ||
        !std::isfinite(focal_y_px_) || focal_y_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_height * focal_y_px_) / pixel_h;
  }

  float FilterDistance(float raw_distance) {
    const auto params = Params();
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

  static TunableParams Params() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams();
  }

  static StageTunableParams ParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageParams(MutableParams(), stage);
  }

  static StageTunableParams &StageParams(TunableParams &params,
                                         CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? params.stage3 : params.stage12;
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
        {0.06275f, // stage12 near_calibration_target_height（米）
         94.920f,  // stage12 near_pixel（px）
         0.06759f, // stage12 far_calibration_target_height（米）
         61.626f}, // stage12 far_pixel（px）
        {0.05904f, // stage3 near_calibration_target_height（米）
         77.834f,  // stage3 near_pixel（px）
         0.06650f, // stage3 far_calibration_target_height（米）
         61.014f}, // stage3 far_pixel（px）
        0.25f, // distance_filter_alpha: 一阶滤波系数，越大响应越快
        0.5f,  // filter_reset_ratio: 距离突变超过该比例时重置滤波
        CalibrationStage::Stage12 // active_stage: 当前使用的标定阶段
    };
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  CameraData camera_data_;
  float focal_y_px_ = camera_data_.cameraMatrix.at<double>(1, 1);
};

class LaserAngleCalculator {
public:
  struct PitchDistanceCompensation {
    float near_distance_m;
    float near_pitch_offset_deg;
    float far_distance_m;
    float far_pitch_offset_deg;
  };

  static PitchDistanceCompensation GetPitchDistanceCompensation() {
    const auto params = ParamsForStage(ActiveStage());
    return {params.pitch_comp_near_distance_m,
            params.pitch_comp_near_offset_deg, params.pitch_comp_far_distance_m,
            params.pitch_comp_far_offset_deg};
  }

  static PitchDistanceCompensation
  GetPitchDistanceCompensation(CalibrationStage stage) {
    const auto params = ParamsForStage(stage);
    return {params.pitch_comp_near_distance_m,
            params.pitch_comp_near_offset_deg, params.pitch_comp_far_distance_m,
            params.pitch_comp_far_offset_deg};
  }

  static void SetPitchDistanceCompensation(float near_distance_m,
                                           float near_pitch_offset_deg,
                                           float far_distance_m,
                                           float far_pitch_offset_deg) {
    SetPitchDistanceCompensation(ActiveStage(), near_distance_m,
                                 near_pitch_offset_deg, far_distance_m,
                                 far_pitch_offset_deg);
  }

  static void SetPitchDistanceCompensation(CalibrationStage stage,
                                           float near_distance_m,
                                           float near_pitch_offset_deg,
                                           float far_distance_m,
                                           float far_pitch_offset_deg) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageParams(params, stage);
    stage_params.pitch_comp_near_distance_m = near_distance_m;
    stage_params.pitch_comp_near_offset_deg = near_pitch_offset_deg;
    stage_params.pitch_comp_far_distance_m = far_distance_m;
    stage_params.pitch_comp_far_offset_deg = far_pitch_offset_deg;
  }

  static void SetActiveStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    MutableParams().active_stage = stage;
  }

  static CalibrationStage ActiveStage() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams().active_stage;
  }

  float CalculateLaserYawAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).first;
  }

  float CalculateLaserPitchAngleByDistance(float distance) {
    return CalculateLaserAngles(distance, 0.0f, 0.0f).second;
  }

  std::pair<float, float> CalculateLaserAngles(float distance, float, float) {
    // 激光和相机的连线始终垂直于相机视线；新 pitch 正方向与旧约定相反，
    // 激光在上方时向下补偿为负值。
    const auto params = Params();
    const float current_pitch =
        ComputePitchCorrectionDeg(distance, params.laser_height_above_camera_m);
    const float reference_pitch = ComputePitchCorrectionDeg(
        params.reference_distance_m, params.laser_height_above_camera_m);
    const float distance_pitch_offset =
        ComputeDistancePitchOffsetDeg(distance, params);
    return {0.0f, reference_pitch - current_pitch + distance_pitch_offset};
  }

private:
  struct StageTunableParams {
    float pitch_comp_near_distance_m;
    float pitch_comp_near_offset_deg;
    float pitch_comp_far_distance_m;
    float pitch_comp_far_offset_deg;
  };

  struct TunableParams {
    float laser_height_above_camera_m;
    float reference_distance_m;
    float min_valid_distance_m;
    StageTunableParams stage12;
    StageTunableParams stage3;
    CalibrationStage active_stage;
  };

  static float ComputePitchCorrectionDeg(float distance,
                                         float laser_height_above_camera_m) {
    const float safe_distance = SafeDistance(distance);
    const float laser_pitch_rad =
        std::atan2(laser_height_above_camera_m, safe_distance);
    return laser_pitch_rad * kRadToDeg;
  }

  static float ComputeDistancePitchOffsetDeg(float distance,
                                             const TunableParams &params) {
    const auto stage_params = StageParams(params, params.active_stage);
    const float near_distance = stage_params.pitch_comp_near_distance_m;
    const float far_distance = stage_params.pitch_comp_far_distance_m;
    const float near_offset = stage_params.pitch_comp_near_offset_deg;
    const float far_offset = stage_params.pitch_comp_far_offset_deg;
    if (!std::isfinite(distance) || !std::isfinite(near_distance) ||
        !std::isfinite(far_distance) || !std::isfinite(near_offset) ||
        !std::isfinite(far_offset) ||
        std::abs(far_distance - near_distance) <= kLinearInterpEpsilon) {
      return 0.0f;
    }

    const float t =
        std::clamp((distance - near_distance) / (far_distance - near_distance),
                   0.0f, 1.0f);
    return near_offset + t * (far_offset - near_offset);
  }

  static float SafeDistance(float distance) {
    if (!std::isfinite(distance) || distance <= 0.0f)
      return Params().reference_distance_m;
    return std::max(distance, Params().min_valid_distance_m);
  }

  static TunableParams Params() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return MutableParams();
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
        0.090f, // laser_height_above_camera_m: 激光在相机上方 0.09m
        18.8f, // reference_distance_m: 用于归零的参考水平距离（米）
        0.1f, // min_valid_distance_m: 仅用于防止接近 0 的距离造成异常角度
        {12.0f,   // stage12 pitch_comp_near_distance_m（米）
         0.13f,   // stage12 pitch_comp_near_offset_deg，正值向上
         20.0f,   // stage12 pitch_comp_far_distance_m（米）
         0.125f}, // stage12 pitch_comp_far_offset_deg，正值向上
        {12.0f,   // stage3 pitch_comp_near_distance_m（米）
         0.14f,   // stage3 pitch_comp_near_offset_deg，正值向上
         20.0f,   // stage3 pitch_comp_far_distance_m（米）
         0.13f},  // stage3 pitch_comp_far_offset_deg，正值向上
        CalibrationStage::Stage12 // active_stage: 当前使用的激光补偿阶段
    };
    return p;
  }
};

} // namespace Tools
