/**
 * @file    include/Tools/LaserAngleCalculate.hpp
 * @brief   目标距离估计、实机尺寸标定与激光 pitch 几何补偿工具。
 *
 * 本文件负责把检测框像素尺寸转换为目标水平距离，并根据相机/激光的
 * 物理安装偏移计算需要追加到视觉 pitch 偏角上的补偿角。距离估计按
 * stage1/2 与 stage3 分别维护标定参数：近/远距离处的检测框高度像素、
 * 宽度像素，以及由实机数据反推得到的等效目标高度/宽度。运行时会同时
 * 计算宽度距离 Dw 与高度距离 Dh，优先在二者一致时做加权融合，单一路径
 * 不可信时回退到有效路径，并对反距离做一阶滤波以降低远距离像素抖动
 * 放大的 pitch 震荡风险。
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
    Fused,
  };

  struct DistanceDebugInfo {
    float used_distance = 0.0f;
    float width_distance = 0.0f;
    float height_distance = 0.0f;
    DistanceSource source = DistanceSource::None;
  };

  static CalibrationTargetHeights GetCalibrationTargetHeights() {
    const auto params = DistanceCalibrationParamsForStage(ActiveStage());
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

    const auto params = LaserCompensationParamsForStage(ActiveStage());
    if (!params.enable_laser_pitch_compensation ||
        distance_m < params.laser_comp_min_distance_m ||
        distance_m > params.laser_comp_max_distance_m ||
        params.laser_z_offset_m <= 0.0f || params.laser_converge_x_m <= 0.0f) {
      return 0.0f;
    }

    const float visual_pitch_rad = visual_pitch_offset_deg / kRadiansToDegrees;
    const float converge_angle =
        std::atan2(params.laser_z_offset_m, params.laser_converge_x_m);
    const float target_vertical_trim_m = params.laser_target_vertical_trim_m;
    const float required_laser_angle =
        std::atan2(distance_m * std::tan(visual_pitch_rad) -
                       params.laser_z_offset_m - target_vertical_trim_m,
                   distance_m);
    return (required_laser_angle + converge_angle - visual_pitch_rad) *
           kRadiansToDegrees;
  }

  static float
  CalculateLaserPitchCompensationDeg(const DistanceDebugInfo &distance_debug,
                                     float visual_pitch_offset_deg = 0.0f) {
    const auto params = LaserCompensationParamsForStage(ActiveStage());
    const float distance_m = distance_debug.used_distance;
    if (!std::isfinite(distance_m) || distance_m <= 0.0f) {
      return 0.0f;
    }

    if (!params.enable_laser_pitch_compensation ||
        params.laser_z_offset_m <= 0.0f || params.laser_converge_x_m <= 0.0f) {
      return 0.0f;
    }
    if (distance_m < params.laser_comp_min_distance_m ||
        distance_m > params.laser_comp_max_distance_m) {
      if (distance_m < params.laser_comp_min_distance_m) {
        return CalculateLaserPitchCompensationDeg(
            params.laser_comp_min_distance_m, visual_pitch_offset_deg);
      } else {
        return CalculateLaserPitchCompensationDeg(
            params.laser_comp_max_distance_m, visual_pitch_offset_deg);
      }
    }
    const float base_comp_deg =
        CalculateLaserPitchCompensationDeg(distance_m, visual_pitch_offset_deg);
    return base_comp_deg;
  }

  static CalibrationTargetHeights
  GetCalibrationTargetHeights(CalibrationStage stage) {
    const auto params = DistanceCalibrationParamsForStage(stage);
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetWidths
  GetCalibrationTargetWidths(CalibrationStage stage) {
    const auto params = DistanceCalibrationParamsForStage(stage);
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
    auto &stage_params = StageDistanceCalibrationParams(params, stage);
    stage_params.near_calibration_target_height = near_height;
    stage_params.far_calibration_target_height = far_height;
  }

  static void SetCalibrationTargetWidths(CalibrationStage stage,
                                         float near_width, float far_width) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageDistanceCalibrationParams(params, stage);
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
    filtered_inverse_distance_ = 0.0f;
  }

  float CalculateDistance(float box_height_pixel) {
    return CalculateDistanceByPixelHeight(box_height_pixel);
  }

  float CalculateDistance(float x1, float y1, float x2, float y2) {
    return CalculateDistanceWithDebug(x1, y1, x2, y2).used_distance;
  }

  DistanceDebugInfo CalculateDistanceWithDebug(float x1, float y1, float x2,
                                               float y2) {
    const float box_width_pixel = std::abs(x2 - x1);
    const float box_height_pixel = std::abs(y2 - y1);
    if ((!std::isfinite(box_width_pixel) || box_width_pixel <= 0.0f) &&
        (!std::isfinite(box_height_pixel) || box_height_pixel <= 0.0f)) {
      return {};
    }

    return CalculateDistanceByPixelSize(box_width_pixel, box_height_pixel);
  }

  static const char *DistanceSourceName(DistanceSource source) {
    switch (source) {
    case DistanceSource::Fused:
      return "FUSED";
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
  static constexpr float kDistanceFusionMaxRelativeGap = 0.16f;
  static constexpr float kDistanceFusionMaxHeightWeight = 0.25f;

  struct DistanceCalibrationParams {
    float near_calibration_target_height;
    float near_height_pixel;
    float far_calibration_target_height;
    float far_height_pixel;
    float near_calibration_target_width;
    float near_width_pixel;
    float far_calibration_target_width;
    float far_width_pixel;
  };

  struct LaserPitchCompensationParams {
    float laser_z_offset_m;
    float laser_converge_x_m;
    float laser_comp_min_distance_m;
    float laser_comp_max_distance_m;
    bool enable_laser_pitch_compensation;
    float laser_target_vertical_trim_m;
  };

  struct TunableParams {
    struct {
      DistanceCalibrationParams stage12;
      DistanceCalibrationParams stage3;
    } distance_calibration;
    struct {
      LaserPitchCompensationParams stage12;
      LaserPitchCompensationParams stage3;
    } laser_pitch_compensation;
    struct {
      float alpha;
      float reset_ratio;
    } distance_filter;
    CalibrationStage active_stage;
  };

  struct RuntimeSnapshot {
    DistanceCalibrationParams distance_calibration;
    float distance_filter_alpha;
    float distance_filter_reset_ratio;
  };

  float CalculateDistanceByPixelHeight(float box_height_pixel) {
    if (!std::isfinite(box_height_pixel) || box_height_pixel <= 0.0f)
      return 0.0f;

    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    const float raw_distance =
        EstimateCalibratedDistanceByHeight(box_height_pixel,
                                           params.distance_calibration);
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0f)
      return 0.0f;
    return FilterDistance(raw_distance, params);
  }

  DistanceDebugInfo CalculateDistanceByPixelSize(float box_width_pixel,
                                                 float box_height_pixel) {
    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    DistanceDebugInfo result{};
    result.width_distance =
        EstimateCalibratedDistanceByWidth(box_width_pixel,
                                          params.distance_calibration);
    result.height_distance =
        EstimateCalibratedDistanceByHeight(box_height_pixel,
                                           params.distance_calibration);

    const float selected_distance =
        SelectDistanceEstimate(result, &result.source);
    if (IsValidDistance(selected_distance)) {
      result.used_distance = FilterDistance(selected_distance, params);
      return result;
    }

    return result;
  }

  float
  EstimateCalibratedDistanceByHeight(
      float box_height_pixel, const DistanceCalibrationParams &params) const {
    const float near_height = params.near_calibration_target_height;
    const float far_height = params.far_calibration_target_height;
    const float near_height_pixel = params.near_height_pixel;
    const float far_height_pixel = params.far_height_pixel;

    if (!IsValidPixel(near_height_pixel) || !IsValidPixel(far_height_pixel) ||
        near_height <= 0.0f || far_height <= 0.0f ||
        std::abs(far_height_pixel - near_height_pixel) <=
            kLinearInterpEpsilon) {
      return EstimateDistanceByHeight(far_height, box_height_pixel);
    }

    const float t = std::clamp(
        (box_height_pixel - near_height_pixel) /
            (far_height_pixel - near_height_pixel),
        0.0f, 1.0f);
    const float target_height = near_height + t * (far_height - near_height);
    return EstimateDistanceByHeight(target_height, box_height_pixel);
  }

  float
  EstimateCalibratedDistanceByWidth(
      float box_width_pixel, const DistanceCalibrationParams &params) const {
    const float target_width =
        EstimateCalibratedTargetWidthByPixelWidth(box_width_pixel, params);
    return EstimateDistanceByWidth(target_width, box_width_pixel);
  }

  float EstimateCalibratedTargetWidthByPixelWidth(
      float box_width_pixel, const DistanceCalibrationParams &params) const {
    const float near_width = params.near_calibration_target_width;
    const float far_width = params.far_calibration_target_width;
    const float near_width_pixel = params.near_width_pixel;
    const float far_width_pixel = params.far_width_pixel;

    if (!IsValidPixel(box_width_pixel)) {
      return 0.0f;
    }
    if (!IsValidPixel(near_width_pixel) || !IsValidPixel(far_width_pixel) ||
        near_width <= 0.0f || far_width <= 0.0f ||
        std::abs(far_width_pixel - near_width_pixel) <= kLinearInterpEpsilon) {
      return far_width;
    }

    const float t = std::clamp(
        (box_width_pixel - near_width_pixel) /
            (far_width_pixel - near_width_pixel),
        0.0f, 1.0f);
    return near_width + t * (far_width - near_width);
  }

  float EstimateDistanceByHeight(float target_height,
                                 float box_height_pixel) const {
    if (!std::isfinite(target_height) || target_height <= 0.0f ||
        !std::isfinite(box_height_pixel) || box_height_pixel <= 0.0f ||
        !std::isfinite(focal_y_px_) || focal_y_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_height * focal_y_px_) / box_height_pixel;
  }

  float EstimateDistanceByWidth(float target_width,
                                float box_width_pixel) const {
    if (!std::isfinite(target_width) || target_width <= 0.0f ||
        !IsValidPixel(box_width_pixel) || !std::isfinite(focal_x_px_) ||
        focal_x_px_ <= 0.0f) {
      return 0.0f;
    }
    return (target_width * focal_x_px_) / box_width_pixel;
  }

  float SelectDistanceEstimate(const DistanceDebugInfo &distance_debug,
                               DistanceSource *source) const {
    const bool has_width_distance = IsValidDistance(distance_debug.width_distance);
    const bool has_height_distance =
        IsValidDistance(distance_debug.height_distance);

    if (has_width_distance && has_height_distance) {
      const float larger_distance =
          std::max(distance_debug.width_distance, distance_debug.height_distance);
      const float smaller_distance =
          std::min(distance_debug.width_distance, distance_debug.height_distance);
      const float relative_gap =
          larger_distance > 0.0f
              ? (larger_distance - smaller_distance) / larger_distance
              : 1.0f;
      if (relative_gap <= kDistanceFusionMaxRelativeGap) {
        if (source != nullptr) {
          *source = DistanceSource::Fused;
        }
        const float height_weight =
            std::clamp(1.0f - relative_gap / kDistanceFusionMaxRelativeGap,
                       0.0f, 1.0f) *
            kDistanceFusionMaxHeightWeight;
        const float width_weight = 1.0f - height_weight;
        return width_weight * distance_debug.width_distance +
               height_weight * distance_debug.height_distance;
      }
    }

    if (has_width_distance) {
      if (source != nullptr) {
        *source = DistanceSource::Width;
      }
      return distance_debug.width_distance;
    }

    if (has_height_distance) {
      if (source != nullptr) {
        *source = DistanceSource::Height;
      }
      return distance_debug.height_distance;
    }

    if (source != nullptr) {
      *source = DistanceSource::None;
    }
    return 0.0f;
  }

  float FilterDistance(float raw_distance, const RuntimeSnapshot &params) {
    if (!IsValidDistance(raw_distance)) {
      return 0.0f;
    }

    if (!has_filtered_distance_ || filtered_distance_ <= 0.0f ||
        filtered_inverse_distance_ <= 0.0f) {
      filtered_inverse_distance_ = 1.0f / raw_distance;
      filtered_distance_ = raw_distance;
      has_filtered_distance_ = true;
      return raw_distance;
    }

    const float reset_threshold =
        filtered_distance_ * params.distance_filter_reset_ratio;
    if (std::abs(raw_distance - filtered_distance_) > reset_threshold) {
      filtered_inverse_distance_ = 1.0f / raw_distance;
      filtered_distance_ = raw_distance;
      return raw_distance;
    }

    const float raw_inverse_distance = 1.0f / raw_distance;
    filtered_inverse_distance_ +=
        params.distance_filter_alpha *
        (raw_inverse_distance - filtered_inverse_distance_);
    filtered_distance_ = 1.0f / filtered_inverse_distance_;
    return filtered_distance_;
  }

  static RuntimeSnapshot SnapshotCurrentRuntime_() {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    const auto &params = MutableParams();
    return {StageDistanceCalibrationParams(params, params.active_stage),
            params.distance_filter.alpha, params.distance_filter.reset_ratio};
  }

  static DistanceCalibrationParams
  DistanceCalibrationParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageDistanceCalibrationParams(MutableParams(), stage);
  }

  static LaserPitchCompensationParams
  LaserCompensationParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageLaserCompensationParams(MutableParams(), stage);
  }

  static DistanceCalibrationParams &
  StageDistanceCalibrationParams(TunableParams &params,
                                 CalibrationStage stage) {
    return stage == CalibrationStage::Stage3
               ? params.distance_calibration.stage3
               : params.distance_calibration.stage12;
  }

  static const DistanceCalibrationParams &
  StageDistanceCalibrationParams(const TunableParams &params,
                                 CalibrationStage stage) {
    return stage == CalibrationStage::Stage3
               ? params.distance_calibration.stage3
               : params.distance_calibration.stage12;
  }

  static const LaserPitchCompensationParams &
  StageLaserCompensationParams(const TunableParams &params,
                               CalibrationStage stage) {
    return stage == CalibrationStage::Stage3
               ? params.laser_pitch_compensation.stage3
               : params.laser_pitch_compensation.stage12;
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
    // ===== 调参集中区：先按功能分组，再在功能内区分 stage =====
    static const TunableParams p{
        {
            {
                CalibrationTargetMetersFromPixel(
                    kNearCalibrationHorizontalDistanceM, 111.982f,
                    CameraData::kFocalY),
                // stage12 near_calibration_target_height（米，按 height pixel
                // 与水平距离自动反推）
                111.982f, // stage12 near_height_pixel（px）
                CalibrationTargetMetersFromPixel(
                    kFarCalibrationHorizontalDistanceM, 47.952f,
                    CameraData::kFocalY),
                47.952f, // stage12 far_height_pixel（px）
                CalibrationTargetMetersFromPixel(
                    kNearCalibrationHorizontalDistanceM, 105.029f,
                    CameraData::kFocalX),
                105.029f, // stage12 near_width_pixel（px，未标定时置 0）
                CalibrationTargetMetersFromPixel(
                    kFarCalibrationHorizontalDistanceM, 43.274f,
                    CameraData::kFocalX),
                43.274f, // stage12 far_width_pixel（px，未标定时置 0）
            },
            {
                CalibrationTargetMetersFromPixel(
                    kNearCalibrationHorizontalDistanceM, 102.006f,
                    CameraData::kFocalY),
                // stage3 near_calibration_target_height（米，按 height pixel
                // 与水平距离自动反推）
                102.006f, // stage3 near_height_pixel（px）
                CalibrationTargetMetersFromPixel(
                    kFarCalibrationHorizontalDistanceM, 56.941f,
                    CameraData::kFocalY),
                56.941f, // stage3 far_height_pixel（px）
                CalibrationTargetMetersFromPixel(
                    kNearCalibrationHorizontalDistanceM, 85.880f,
                    CameraData::kFocalX),
                85.880f, // stage3 near_width_pixel（px，未标定时置 0）
                CalibrationTargetMetersFromPixel(
                    kFarCalibrationHorizontalDistanceM, 49.605f,
                    CameraData::kFocalX),
                49.605f, // stage3 far_width_pixel（px，未标定时置 0）
            },
        },
        {
            {
                0.090f,  // stage12 laser_z_offset_m: 激光在相机上方 0.09m
                14.313f, // stage12 laser_converge_x_m: 光轴交汇前向距离
                10.0f,   // stage12 laser_comp_min_distance_m
                24.0f,   // stage12 laser_comp_max_distance_m
                true,    // stage12 enable_laser_pitch_compensation
                0.004f,  // stage12 laser_target_vertical_trim_m: 目标面下移 4mm
            },
            {
                0.090f,  // stage3 laser_z_offset_m: 激光在相机上方 0.09m
                14.313f, // stage3 laser_converge_x_m: 光轴交汇前向距离
                10.0f,   // stage3 laser_comp_min_distance_m
                24.0f,   // stage3 laser_comp_max_distance_m
                true,    // stage3 enable_laser_pitch_compensation
                0.004f,  // stage3 laser_target_vertical_trim_m
            },
        },
        {
            0.25f, // distance_filter.alpha: 一阶滤波系数，越大响应越快
            0.5f,  // distance_filter.reset_ratio: 距离突变超过该比例时重置
        },
        CalibrationStage::Stage12 // active_stage: 当前使用的标定阶段
    };
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  float filtered_inverse_distance_ = 0.0f;
  float focal_x_px_ = static_cast<float>(CameraData::kFocalX);
  float focal_y_px_ = static_cast<float>(CameraData::kFocalY);
};

} // namespace Tools
