/**
 * @file    include/ImageRecognize/TargetTrackPipeline.hpp
 * @brief   收口目标框筛选、跨帧关联与可选稳定化逻辑。
 *
 * TargetTrackPipeline 是主流程检测后处理的组合入口，依次执行阵营过滤、
 * 目标关联、stage3 检测框稳定化和跟踪结果生成。它把多模块中间状态收口
 * 为统一结果，方便 ImagePredict.cc 只消费 tracked_box / tracked_center 等输出。
 */

#pragma once

#include <vector>

#include "ImageRecognize/OutputDataProcess.hpp"
#include "ImageRecognize/TargetAssociation.hpp"
#include "ImageRecognize/TargetClassFilter.hpp"
#include "ImageRecognize/TemporalBoxStabilizer.hpp"

namespace ImageRecognize {

struct TargetTrackPipelineResult {
  std::vector<DetectionBox> candidate_boxes;
  CrossFrameTargetTrackerResult track_result;
  bool has_tracked_box = false;
  DetectionBox tracked_box{};
  bool track_alive = false;
};

class TargetTrackPipeline {
 public:
  TargetTrackPipeline() = default;

  void Reset() {
    tracker_.Reset();
    stage12_stabilizer_.Reset();
    stage3_stabilizer_.Reset();
  }

  TargetTrackPipelineResult Update(const PredictResult &result,
                                   TargetCampMode target_camp_mode,
                                   bool enable_box_stabilization,
                                   double dt = -1.0) {
    TargetTrackPipelineResult pipeline_result{};
    pipeline_result.candidate_boxes =
        FilterTrackBoxes(result.boxes, target_camp_mode);
    pipeline_result.track_result = tracker_.Update(pipeline_result.candidate_boxes);
    pipeline_result.track_alive =
        pipeline_result.track_result.has_box || tracker_.HasRecentLock();

    if (!pipeline_result.track_result.has_box) {
      return pipeline_result;
    }

    pipeline_result.tracked_box = pipeline_result.track_result.box;
    if (enable_box_stabilization) {
      pipeline_result.tracked_box = stage3_stabilizer_.Update(
          pipeline_result.tracked_box,
          pipeline_result.track_result.matched_history, dt);
    } else {
      pipeline_result.tracked_box = stage12_stabilizer_.Update(
          pipeline_result.tracked_box,
          pipeline_result.track_result.matched_history, dt);
    }
    pipeline_result.has_tracked_box = true;
    return pipeline_result;
  }

 private:
  static TemporalBoxStabilizer MakeStage12Stabilizer_() {
    TemporalBoxStabilizerParams params;
    params.min_iou_to_stabilize = 0.38f;
    params.max_center_distance_ratio = 0.55f;
    params.max_area_ratio = 1.30f;
    params.max_height_expand_ratio = 1.03f;
    params.center_min_cutoff_hz = 1.1;
    params.center_beta = 0.18;
    params.size_min_cutoff_hz = 0.55;
    params.size_beta = 0.04;
    params.center_x_min_cutoff_hz = 2.25;
    params.center_y_min_cutoff_hz = 1.05;
    params.center_x_beta = 0.42;
    params.center_y_beta = 0.16;
    params.width_min_cutoff_hz = 0.90;
    params.height_min_cutoff_hz = 0.38;
    params.width_beta = 0.06;
    params.height_beta = 0.015;
    return TemporalBoxStabilizer(params);
  }

  static TemporalBoxStabilizer MakeStage3Stabilizer_() {
    TemporalBoxStabilizerParams params;
    params.min_iou_to_stabilize = 0.42f;
    params.max_center_distance_ratio = 0.48f;
    params.max_area_ratio = 1.22f;
    params.max_height_expand_ratio = 1.02f;
    params.center_min_cutoff_hz = 0.85;
    params.center_beta = 0.06;
    params.size_min_cutoff_hz = 0.40;
    params.size_beta = 0.012;
    params.center_x_min_cutoff_hz = 0.95;
    params.center_y_min_cutoff_hz = 0.62;
    params.center_x_beta = 0.08;
    params.center_y_beta = 0.04;
    params.width_min_cutoff_hz = 0.42;
    params.height_min_cutoff_hz = 0.24;
    params.width_beta = 0.012;
    params.height_beta = 0.004;
    return TemporalBoxStabilizer(params);
  }

  CrossFrameTargetTracker tracker_{};
  TemporalBoxStabilizer stage12_stabilizer_{MakeStage12Stabilizer_()};
  TemporalBoxStabilizer stage3_stabilizer_{MakeStage3Stabilizer_()};
};

} // namespace ImageRecognize
