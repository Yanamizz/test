/**
 * @file    include/ImageRecognize/TargetTrackPipeline.hpp
 * @brief   收口目标框筛选、跨帧关联与可选稳定化逻辑。
 */

#pragma once

#include <array>
#include <vector>

#include "ImageRecognize/OutputDataProcess.hpp"
#include "ImageRecognize/TargetAssociation.hpp"
#include "ImageRecognize/TargetClassFilter.hpp"
#include "ImageRecognize/TemporalBoxStabilizer.hpp"

namespace ImageRecognize {

struct TargetTrackPipelineResult {
  std::vector<std::array<float, 6>> candidate_boxes;
  CrossFrameTargetTrackerResult track_result;
  bool has_tracked_box = false;
  std::array<float, 6> tracked_box{};
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
    params.max_area_ratio = 1.45f;
    params.max_height_expand_ratio = 1.06f;
    params.center_min_cutoff_hz = 1.1;
    params.center_beta = 0.18;
    params.size_min_cutoff_hz = 0.85;
    params.size_beta = 0.10;
    return TemporalBoxStabilizer(params);
  }

  CrossFrameTargetTracker tracker_{};
  TemporalBoxStabilizer stage12_stabilizer_{MakeStage12Stabilizer_()};
  TemporalBoxStabilizer stage3_stabilizer_{};
};

} // namespace ImageRecognize
