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
    stabilizer_.Reset();
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
      pipeline_result.tracked_box = stabilizer_.Update(
          pipeline_result.tracked_box,
          pipeline_result.track_result.matched_history, dt);
    }
    pipeline_result.has_tracked_box = true;
    return pipeline_result;
  }

 private:
  CrossFrameTargetTracker tracker_{};
  TemporalBoxStabilizer stabilizer_{};
};

} // namespace ImageRecognize
