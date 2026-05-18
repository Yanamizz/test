/**
 * @file    include/ImageRecognize/TargetAssociation.hpp
 * @brief   提供跨帧目标框关联、粘连跟踪与丢失容忍逻辑。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "OutputDataProcess.hpp"

namespace ImageRecognize {

inline float Clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

inline float BoxWidth(const std::array<float, 6> &box) { return std::max(0.0f, box[2] - box[0]); }

inline float BoxHeight(const std::array<float, 6> &box) { return std::max(0.0f, box[3] - box[1]); }

inline float BoxArea(const std::array<float, 6> &box) { return BoxWidth(box) * BoxHeight(box); }

inline float BoxIoU(const std::array<float, 6> &a, const std::array<float, 6> &b) {
  const float xx1 = std::max(a[0], b[0]);
  const float yy1 = std::max(a[1], b[1]);
  const float xx2 = std::min(a[2], b[2]);
  const float yy2 = std::min(a[3], b[3]);
  const float inter_w = std::max(0.0f, xx2 - xx1);
  const float inter_h = std::max(0.0f, yy2 - yy1);
  const float inter = inter_w * inter_h;
  const float uni = BoxArea(a) + BoxArea(b) - inter;
  return (uni <= 0.0f) ? 0.0f : (inter / uni);
}

inline float BoxCenterScore(const std::array<float, 6> &reference, const std::array<float, 6> &candidate) {
  const float ref_cx = 0.5f * (reference[0] + reference[2]);
  const float ref_cy = 0.5f * (reference[1] + reference[3]);
  const float cand_cx = 0.5f * (candidate[0] + candidate[2]);
  const float cand_cy = 0.5f * (candidate[1] + candidate[3]);

  const float dx = cand_cx - ref_cx;
  const float dy = cand_cy - ref_cy;
  const float center_distance = std::sqrt(dx * dx + dy * dy);

  const float ref_w = std::max(1.0f, BoxWidth(reference));
  const float ref_h = std::max(1.0f, BoxHeight(reference));
  const float ref_diag = std::sqrt(ref_w * ref_w + ref_h * ref_h);
  const float normalized_distance = center_distance / std::max(1.0f, 0.5f * ref_diag);
  return 1.0f / (1.0f + normalized_distance);
}

struct CrossFrameTargetTrackerParams {
  float iou_weight;                   // 普通关联时的 IoU 权重：越大越偏向重叠度高的框
  float center_weight;                // 普通关联时的中心点权重：越大越偏向位置接近的框
  float confidence_weight;            // 普通关联时的置信度权重：避免低置信框误抢目标
  float sticky_iou_weight;            // 粘连模式下的 IoU 权重：优先沿用上一帧目标
  float sticky_center_weight;         // 粘连模式下的中心点权重：辅助维持几何连续性
  float sticky_confidence_threshold;  // 前一帧框达到该置信度后，优先走“粘连”分支
  std::size_t max_lost_frames;        // 允许短暂漏检的帧数：短暂丢框时不立刻放弃跟踪

  CrossFrameTargetTrackerParams();
};

struct CrossFrameTargetTrackerResult {
  bool has_box = false;
  bool matched_history = false;
  float association_score = 0.0f;
  std::size_t selected_index = 0;
  std::array<float, 6> box{};
};

class CrossFrameTargetTracker {
 public:
  explicit CrossFrameTargetTracker(const CrossFrameTargetTrackerParams &params = CrossFrameTargetTrackerParams{})
      : params_(params) {}

  void Reset() {
    initialized_ = false;
    missed_frames_ = 0;
    anchor_box_ = {};
  }

  CrossFrameTargetTrackerResult Update(const std::vector<std::array<float, 6>> &boxes) {
    CrossFrameTargetTrackerResult result;

    // 没有检测结果时，只累计丢失帧数；在容忍窗口内保持锁定，避免一帧漏检就立刻抖动或放开。
    if (boxes.empty()) {
      if (initialized_) {
        ++missed_frames_;
        if (missed_frames_ > params_.max_lost_frames) {
          Reset();
        }
      }
      return result;
    }

    std::size_t selected_index = 0;
    float association_score = 1.0f;
    bool matched_history = false;

    if (!initialized_) {
      // 第一帧没有历史锚点，直接选置信度最高的框作为初始目标。
      selected_index = SelectHighestScoreIndex_(boxes);
      matched_history = false;
      initialized_ = true;
    } else if (anchor_box_[4] >= params_.sticky_confidence_threshold) {
      // 上一帧框本身置信度还足够高时，不再单纯追求“谁更高置信”，
      // 而是优先挑与上一帧最连续的框，减少高置信新框把目标抢走造成的跳变。
      selected_index = SelectStickyIndex_(boxes, &association_score);
      matched_history = true;
    } else {
      // 常规模式：综合 IoU、中心点和置信度做关联，选一个最像历史目标的候选。
      selected_index = SelectAssociatedIndex_(boxes, &association_score);
      matched_history = true;
    }

    const auto &selected_box = boxes[selected_index];
    anchor_box_ = selected_box;
    missed_frames_ = 0;
    initialized_ = true;

    result.has_box = true;
    result.matched_history = matched_history;
    result.association_score = association_score;
    result.selected_index = selected_index;
    result.box = selected_box;
    return result;
  }

  bool HasRecentLock() const { return initialized_ && missed_frames_ <= params_.max_lost_frames; }

 private:
  std::size_t SelectHighestScoreIndex_(const std::vector<std::array<float, 6>> &boxes) const {
    std::size_t best_index = 0;
    for (std::size_t i = 1; i < boxes.size(); ++i) {
      if (boxes[i][4] > boxes[best_index][4]) best_index = i;
    }
    return best_index;
  }

  float AssociationScore_(const std::array<float, 6> &reference, const std::array<float, 6> &candidate) const {
    const float iou = BoxIoU(reference, candidate);
    const float center_score = BoxCenterScore(reference, candidate);
    const float confidence = Clamp01(candidate[4]);
    return params_.iou_weight * iou + params_.center_weight * center_score + params_.confidence_weight * confidence;
  }

  float StickyScore_(const std::array<float, 6> &reference, const std::array<float, 6> &candidate) const {
    const float iou = BoxIoU(reference, candidate);
    const float center_score = BoxCenterScore(reference, candidate);
    // 粘连分支故意不看候选框置信度，避免“新的高置信框”打断上一帧已经稳定的目标。
    return params_.sticky_iou_weight * iou + params_.sticky_center_weight * center_score;
  }

  std::size_t SelectStickyIndex_(const std::vector<std::array<float, 6>> &boxes, float *sticky_score) const {
    std::size_t best_index = 0;
    float best_score = StickyScore_(anchor_box_, boxes[0]);

    for (std::size_t i = 1; i < boxes.size(); ++i) {
      const float score = StickyScore_(anchor_box_, boxes[i]);
      if (score > best_score) {
        best_score = score;
        best_index = i;
      }
    }

    if (sticky_score != nullptr) {
      *sticky_score = best_score;
    }
    return best_index;
  }

  std::size_t SelectAssociatedIndex_(const std::vector<std::array<float, 6>> &boxes, float *association_score) const {
    std::size_t best_index = 0;
    float best_score = AssociationScore_(anchor_box_, boxes[0]);

    for (std::size_t i = 1; i < boxes.size(); ++i) {
      const float score = AssociationScore_(anchor_box_, boxes[i]);
      if (score > best_score) {
        best_score = score;
        best_index = i;
      }
    }

    if (association_score != nullptr) {
      *association_score = best_score;
    }
    return best_index;
  }

  CrossFrameTargetTrackerParams params_;
  bool initialized_ = false;
  std::size_t missed_frames_ = 0;
  std::array<float, 6> anchor_box_{};
};

inline CrossFrameTargetTrackerParams::CrossFrameTargetTrackerParams()
    : iou_weight(0.75f),                   // 历史框关联的 IoU 权重，略提高以减少换框
      center_weight(0.2f),                 // 历史框关联的中心点权重，略降低以保留几何连续性
      confidence_weight(0.04f),            // 历史框关联的置信度权重，进一步降低高置信新框抢占概率
      sticky_iou_weight(0.84f),            // 粘连模式下的 IoU 权重，优先维持上一帧目标
      sticky_center_weight(0.16f),         // 粘连模式下的中心点权重
      sticky_confidence_threshold(0.75f),  // 前一帧框达到该置信度时继续粘连
      max_lost_frames(6) {}                // 允许的最长丢失帧数，略增大以避免短暂漏检就松锁

}  // namespace ImageRecognize
