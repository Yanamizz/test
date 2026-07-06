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
#include <array>
#include <cmath>
#include <mutex>

namespace Tools {
inline constexpr float kLinearInterpEpsilon = 1e-6f;
inline constexpr float kRadiansToDegrees = 57.29577951308232f;
inline constexpr float kNearCalibrationHorizontalDistanceM = 10.0f;
inline constexpr float kMidCalibrationHorizontalDistanceM = 17.0f;
inline constexpr float kFarCalibrationHorizontalDistanceM = 24.0f;

enum class CalibrationStage {
  Stage12,
  Stage3,
};

class DistanceCalculator {
public:
  static constexpr std::size_t kCalibrationSampleCount = 3;

  struct PixelDistanceCalibrationSample {
    float distance_m = 0.0f;
    float width_pixel = 0.0f;
    float height_pixel = 0.0f;
  };

  using CalibrationSampleSet =
      std::array<PixelDistanceCalibrationSample, kCalibrationSampleCount>;

  struct CalibrationTargetHeights {
    float near_calibration_target_height;
    float far_calibration_target_height;
  };

  struct CalibrationTargetWidths {
    float near_calibration_target_width;
    float far_calibration_target_width;
    float near_width_pixel;
    float mid_width_pixel;
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
    float legacy_width_distance = 0.0f;
    float legacy_height_distance = 0.0f;
    float shadow_width_distance = 0.0f;
    float shadow_height_distance = 0.0f;
    bool shadow_width_valid = false;
    bool shadow_height_valid = false;
  };

  static CalibrationTargetHeights GetCalibrationTargetHeights() {
    const auto params = LegacyDistanceCalibrationParamsForStage(ActiveStage());
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetWidths GetCalibrationTargetWidths() {
    return GetCalibrationTargetWidths(ActiveStage());
  }

  static CalibrationSampleSet GetCalibrationSamples() {
    return GetCalibrationSamples(ActiveStage());
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
    const auto params = LegacyDistanceCalibrationParamsForStage(stage);
    return {params.near_calibration_target_height,
            params.far_calibration_target_height};
  }

  static CalibrationTargetWidths
  GetCalibrationTargetWidths(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    const auto &params = MutableParams();
    const auto &legacy = StageLegacyDistanceCalibrationParams(params, stage);
    const auto &samples = StageDistanceCalibrationSamples(params, stage);
    return {legacy.near_calibration_target_width,
            legacy.far_calibration_target_width, legacy.near_width_pixel,
            samples[kMidCalibrationSampleIndex].width_pixel,
            legacy.far_width_pixel};
  }

  static CalibrationSampleSet GetCalibrationSamples(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageDistanceCalibrationSamples(MutableParams(), stage);
  }

  static void SetCalibrationTargetHeights(float near_height, float far_height) {
    SetCalibrationTargetHeights(ActiveStage(), near_height, far_height);
  }

  static void SetCalibrationTargetHeights(CalibrationStage stage,
                                          float near_height, float far_height) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageLegacyDistanceCalibrationParams(params, stage);
    stage_params.near_calibration_target_height = near_height;
    stage_params.far_calibration_target_height = far_height;
  }

  static void SetCalibrationTargetWidths(CalibrationStage stage,
                                         float near_width, float far_width) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    auto &params = MutableParams();
    auto &stage_params = StageLegacyDistanceCalibrationParams(params, stage);
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
  static constexpr std::size_t kNearCalibrationSampleIndex = 0;
  static constexpr std::size_t kMidCalibrationSampleIndex = 1;
  static constexpr std::size_t kFarCalibrationSampleIndex = 2;
  static constexpr float kDistanceFusionMaxRelativeGap = 0.16f;
  static constexpr float kDistanceFusionMaxHeightWeight = 0.25f;

  struct LegacyDistanceCalibrationParams {
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
      LegacyDistanceCalibrationParams stage12;
      LegacyDistanceCalibrationParams stage3;
    } distance_calibration;
    struct {
      CalibrationSampleSet stage12;
      CalibrationSampleSet stage3;
    } distance_samples;
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
    LegacyDistanceCalibrationParams legacy_distance_calibration;
    CalibrationSampleSet distance_samples;
    float distance_filter_alpha;
    float distance_filter_reset_ratio;
  };

  float CalculateDistanceByPixelHeight(float box_height_pixel) {
    if (!std::isfinite(box_height_pixel) || box_height_pixel <= 0.0f)
      return 0.0f;

    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    bool shadow_valid = false;
    const float shadow_distance = EstimateDistanceFromHeightSamplesShadow(
        box_height_pixel, params.distance_samples, &shadow_valid);
    const float raw_distance =
        shadow_valid
            ? shadow_distance
            : EstimateCalibratedDistanceByHeight(
                  box_height_pixel, params.legacy_distance_calibration);
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0f)
      return 0.0f;
    return FilterDistance(raw_distance, params);
  }

  DistanceDebugInfo CalculateDistanceByPixelSize(float box_width_pixel,
                                                 float box_height_pixel) {
    const RuntimeSnapshot params = SnapshotCurrentRuntime_();
    DistanceDebugInfo result{};
    result.legacy_width_distance = EstimateCalibratedDistanceByWidth(
        box_width_pixel, params.legacy_distance_calibration);
    result.legacy_height_distance = EstimateCalibratedDistanceByHeight(
        box_height_pixel, params.legacy_distance_calibration);
    result.shadow_width_distance = EstimateDistanceFromWidthSamplesShadow(
        box_width_pixel, params.distance_samples, &result.shadow_width_valid);
    result.shadow_height_distance = EstimateDistanceFromHeightSamplesShadow(
        box_height_pixel, params.distance_samples, &result.shadow_height_valid);
    result.width_distance = result.shadow_width_valid
                                ? result.shadow_width_distance
                                : result.legacy_width_distance;
    result.height_distance = result.shadow_height_valid
                                 ? result.shadow_height_distance
                                 : result.legacy_height_distance;

    const float selected_distance =
        SelectDistanceEstimate(result, &result.source);
    if (IsValidDistance(selected_distance)) {
      result.used_distance = FilterDistance(selected_distance, params);
      return result;
    }

    return result;
  }

  float EstimateCalibratedDistanceByHeight(
      float box_height_pixel,
      const LegacyDistanceCalibrationParams &params) const {
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

    const float t = std::clamp((box_height_pixel - near_height_pixel) /
                                   (far_height_pixel - near_height_pixel),
                               0.0f, 1.0f);
    const float target_height = near_height + t * (far_height - near_height);
    return EstimateDistanceByHeight(target_height, box_height_pixel);
  }

  float EstimateCalibratedDistanceByWidth(
      float box_width_pixel,
      const LegacyDistanceCalibrationParams &params) const {
    const float target_width =
        EstimateCalibratedTargetWidthByPixelWidth(box_width_pixel, params);
    return EstimateDistanceByWidth(target_width, box_width_pixel);
  }

  float EstimateCalibratedTargetWidthByPixelWidth(
      float box_width_pixel,
      const LegacyDistanceCalibrationParams &params) const {
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

    const float t = std::clamp((box_width_pixel - near_width_pixel) /
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

  float
  EstimateDistanceFromWidthSamplesShadow(float box_width_pixel,
                                         const CalibrationSampleSet &samples,
                                         bool *valid) const {
    return EstimateDistanceFromSamplesShadow(
        box_width_pixel, samples,
        [](const PixelDistanceCalibrationSample &sample) {
          return sample.width_pixel;
        },
        valid);
  }

  float
  EstimateDistanceFromHeightSamplesShadow(float box_height_pixel,
                                          const CalibrationSampleSet &samples,
                                          bool *valid) const {
    return EstimateDistanceFromSamplesShadow(
        box_height_pixel, samples,
        [](const PixelDistanceCalibrationSample &sample) {
          return sample.height_pixel;
        },
        valid);
  }

  template <typename PixelAccessor>
  float EstimateDistanceFromSamplesShadow(float box_pixel,
                                          const CalibrationSampleSet &samples,
                                          PixelAccessor pixel_accessor,
                                          bool *valid) const {
    struct ShadowFitPoint {
      float inverse_pixel = 0.0f;
      float distance_m = 0.0f;
    };

    if (valid != nullptr) {
      *valid = false;
    }
    if (!IsValidPixel(box_pixel)) {
      return 0.0f;
    }

    std::array<ShadowFitPoint, kCalibrationSampleCount> points{};
    std::size_t count = 0;
    for (const auto &sample : samples) {
      const float pixel = pixel_accessor(sample);
      if (!IsValidPixel(pixel) || !IsValidDistance(sample.distance_m)) {
        continue;
      }
      points[count++] = ShadowFitPoint{1.0f / pixel, sample.distance_m};
    }

    if (count < 2) {
      return 0.0f;
    }

    std::sort(points.begin(),
              points.begin() + static_cast<std::ptrdiff_t>(count),
              [](const ShadowFitPoint &lhs, const ShadowFitPoint &rhs) {
                return lhs.inverse_pixel < rhs.inverse_pixel;
              });

    const float box_inverse_pixel = 1.0f / box_pixel;
    if (valid != nullptr) {
      *valid = true;
    }

    if (box_inverse_pixel <= points[0].inverse_pixel) {
      return points[0].distance_m;
    }
    if (box_inverse_pixel >= points[count - 1].inverse_pixel) {
      return points[count - 1].distance_m;
    }

    for (std::size_t i = 1; i < count; ++i) {
      if (box_inverse_pixel > points[i].inverse_pixel) {
        continue;
      }
      const float x0 = points[i - 1].inverse_pixel;
      const float x1 = points[i].inverse_pixel;
      if (std::abs(x1 - x0) <= kLinearInterpEpsilon) {
        return points[i].distance_m;
      }
      const float t =
          std::clamp((box_inverse_pixel - x0) / (x1 - x0), 0.0f, 1.0f);
      return points[i - 1].distance_m +
             t * (points[i].distance_m - points[i - 1].distance_m);
    }

    return points[count - 1].distance_m;
  }

  float SelectDistanceEstimate(const DistanceDebugInfo &distance_debug,
                               DistanceSource *source) const {
    const bool has_width_distance =
        IsValidDistance(distance_debug.width_distance);
    const bool has_height_distance =
        IsValidDistance(distance_debug.height_distance);

    if (has_width_distance && has_height_distance) {
      const float larger_distance = std::max(distance_debug.width_distance,
                                             distance_debug.height_distance);
      const float smaller_distance = std::min(distance_debug.width_distance,
                                              distance_debug.height_distance);
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
    return {StageLegacyDistanceCalibrationParams(params, params.active_stage),
            StageDistanceCalibrationSamples(params, params.active_stage),
            params.distance_filter.alpha, params.distance_filter.reset_ratio};
  }

  static LegacyDistanceCalibrationParams
  LegacyDistanceCalibrationParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageLegacyDistanceCalibrationParams(MutableParams(), stage);
  }

  static LaserPitchCompensationParams
  LaserCompensationParamsForStage(CalibrationStage stage) {
    std::lock_guard<std::mutex> lk(ParamsMutex());
    return StageLaserCompensationParams(MutableParams(), stage);
  }

  static LegacyDistanceCalibrationParams &
  StageLegacyDistanceCalibrationParams(TunableParams &params,
                                       CalibrationStage stage) {
    return stage == CalibrationStage::Stage3
               ? params.distance_calibration.stage3
               : params.distance_calibration.stage12;
  }

  static const LegacyDistanceCalibrationParams &
  StageLegacyDistanceCalibrationParams(const TunableParams &params,
                                       CalibrationStage stage) {
    return stage == CalibrationStage::Stage3
               ? params.distance_calibration.stage3
               : params.distance_calibration.stage12;
  }

  static CalibrationSampleSet &
  StageDistanceCalibrationSamples(TunableParams &params,
                                  CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? params.distance_samples.stage3
                                             : params.distance_samples.stage12;
  }

  static const CalibrationSampleSet &
  StageDistanceCalibrationSamples(const TunableParams &params,
                                  CalibrationStage stage) {
    return stage == CalibrationStage::Stage3 ? params.distance_samples.stage3
                                             : params.distance_samples.stage12;
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

  static LegacyDistanceCalibrationParams
  MakeLegacyDistanceCalibrationParams(const CalibrationSampleSet &samples) {
    const auto &near_sample = samples[kNearCalibrationSampleIndex];
    const auto &far_sample = samples[kFarCalibrationSampleIndex];
    return {
        CalibrationTargetMetersFromPixel(near_sample.distance_m,
                                         near_sample.height_pixel,
                                         CameraData::kFocalY),
        near_sample.height_pixel,
        CalibrationTargetMetersFromPixel(far_sample.distance_m,
                                         far_sample.height_pixel,
                                         CameraData::kFocalY),
        far_sample.height_pixel,
        CalibrationTargetMetersFromPixel(near_sample.distance_m,
                                         near_sample.width_pixel,
                                         CameraData::kFocalX),
        near_sample.width_pixel,
        CalibrationTargetMetersFromPixel(
            far_sample.distance_m, far_sample.width_pixel, CameraData::kFocalX),
        far_sample.width_pixel,
    };
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
    static const TunableParams p = [] {
      const CalibrationSampleSet stage12_samples{{
          {kNearCalibrationHorizontalDistanceM, 105.029f, 111.982f},
          {kMidCalibrationHorizontalDistanceM, 61.713f, 65.551f},
          {kFarCalibrationHorizontalDistanceM, 43.274f, 47.952f},
      }};
      const CalibrationSampleSet stage3_samples{{
          {kNearCalibrationHorizontalDistanceM, 73.543f, 98.528f},
          {kMidCalibrationHorizontalDistanceM, 47.984f, 64.574f},
          {kFarCalibrationHorizontalDistanceM, 39.462f, 51.549f},
      }};
      return TunableParams{
          {
              MakeLegacyDistanceCalibrationParams(stage12_samples),
              MakeLegacyDistanceCalibrationParams(stage3_samples),
          },
          {
              stage12_samples,
              stage3_samples,
          },
          {
              {
                  -0.065f, // stage12 laser_z_offset_m: 激光在相机下方 0.065m
                  14.313f, // stage12 laser_converge_x_m: 光轴交汇前向距离
                  10.0f,   // stage12 laser_comp_min_distance_m
                  24.0f,   // stage12 laser_comp_max_distance_m
                  false,   // stage12 enable_laser_pitch_compensation
                  0.0f, // stage12 laser_target_vertical_trim_m: 目标面下移 4mm
              },
              {
                  -0.065f, // stage3 laser_z_offset_m: 激光在相机下方 0.065m
                  14.313f, // stage3 laser_converge_x_m: 光轴交汇前向距离
                  10.0f,   // stage3 laser_comp_min_distance_m
                  24.0f,   // stage3 laser_comp_max_distance_m
                  false,   // stage3 enable_laser_pitch_compensation
                  0.0f,    // stage3 laser_target_vertical_trim_m
              },
          },
          {
              0.25f, // distance_filter.alpha: 一阶滤波系数，越大响应越快
              0.5f, // distance_filter.reset_ratio: 距离突变超过该比例时重置
          },
          CalibrationStage::Stage12 // active_stage: 当前使用的标定阶段
      };
    }();
    return p;
  }

  bool has_filtered_distance_ = false;
  float filtered_distance_ = 0.0f;
  float filtered_inverse_distance_ = 0.0f;
  float focal_x_px_ = static_cast<float>(CameraData::kFocalX);
  float focal_y_px_ = static_cast<float>(CameraData::kFocalY);
};

} // namespace Tools
