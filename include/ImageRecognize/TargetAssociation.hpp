/**
 * @file    include/ImageRecognize/TargetAssociation.hpp
 * @brief   跨帧目标关联与轻量平滑工具。
 *
 * @details
 * 这个头文件的目标不是“把框算得更复杂”，而是“让框更稳”。核心思路分成三层：
 * 1. 先在多目标候选里尽量选中同一个目标，减少框在不同候选之间来回跳。
 * 2. 再对选中的框做死区处理，小幅抖动直接冻结，不让像素级噪声传到外部。
 * 3. 当目标确实在移动时，再按运动幅度自适应调整平滑强度，既稳又不至于完全跟不上。
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
  float reset_smoothing_threshold;    // 关联分数太低时重置平滑，避免错误继承到新目标
  float center_deadband_px;           // 中心点死区：小于这个位移就认为没怎么动
  float size_deadband_px;             // 尺寸死区：小于这个变化就认为框尺寸没怎么变
  float min_alpha;                    // 平滑系数下限：越小越稳，但跟随更慢
  float max_alpha;                    // 平滑系数上限：越小越稳，但对真实运动的响应也更保守
  std::size_t max_lost_frames;        // 允许短暂漏检的帧数：短暂丢框时不立刻放弃跟踪

  CrossFrameTargetTrackerParams();
};

struct CrossFrameTargetTrackerResult {
  bool has_box = false;
  bool matched_history = false;
  bool reset_smoothing = false;
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
    smoother_.Reset();
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
    bool reset_smoothing = true;

    if (!initialized_) {
      // 第一帧没有历史锚点，直接选置信度最高的框作为初始目标。
      selected_index = SelectHighestScoreIndex_(boxes);
      matched_history = false;
      reset_smoothing = true;
      initialized_ = true;
    } else if (anchor_box_[4] >= params_.sticky_confidence_threshold) {
      // 上一帧框本身置信度还足够高时，不再单纯追求“谁更高置信”，
      // 而是优先挑与上一帧最连续的框，减少高置信新框把目标抢走造成的跳变。
      selected_index = SelectStickyIndex_(boxes, &association_score);
      matched_history = true;
      reset_smoothing = (missed_frames_ > 0) || (association_score < params_.reset_smoothing_threshold);
    } else {
      // 常规模式：综合 IoU、中心点和置信度做关联，选一个最像历史目标的候选。
      selected_index = SelectAssociatedIndex_(boxes, &association_score);
      matched_history = true;
      reset_smoothing = (missed_frames_ > 0) || (association_score < params_.reset_smoothing_threshold);
    }

    const auto &selected_box = boxes[selected_index];
    if (reset_smoothing) {
      // 如果关联质量明显变差，先清掉平滑状态，避免把旧轨迹硬拖到新目标上。
      smoother_.Reset();
    }

    // 先选目标，再做平滑。这样平滑只处理“同一个目标”的连续抖动，不负责跨目标纠缠。
    const auto smoothed_box = smoother_.Update(selected_box, params_);
    anchor_box_ = selected_box;
    missed_frames_ = 0;
    initialized_ = true;

    result.has_box = true;
    result.matched_history = matched_history;
    result.reset_smoothing = reset_smoothing;
    result.association_score = association_score;
    result.selected_index = selected_index;
    result.box = smoothed_box;
    return result;
  }

  bool HasRecentLock() const { return initialized_ && missed_frames_ <= params_.max_lost_frames; }

 private:
  struct SmoothedBoxState {
    bool initialized = false;
    std::array<float, 6> box{};

    void Reset() { initialized = false; }

    std::array<float, 6> Update(const std::array<float, 6> &current_box, const CrossFrameTargetTrackerParams &params) {
      if (!initialized) {
        // 第一次进来没有历史框，直接输出当前框，不做平滑。
        box = current_box;
        initialized = true;
        return box;
      }

      const float prev_cx = 0.5f * (box[0] + box[2]);
      const float prev_cy = 0.5f * (box[1] + box[3]);
      const float prev_w = std::max(1.0f, box[2] - box[0]);
      const float prev_h = std::max(1.0f, box[3] - box[1]);

      const float curr_cx = 0.5f * (current_box[0] + current_box[2]);
      const float curr_cy = 0.5f * (current_box[1] + current_box[3]);
      const float curr_w = std::max(1.0f, current_box[2] - current_box[0]);
      const float curr_h = std::max(1.0f, current_box[3] - current_box[1]);

      const float center_delta = std::hypot(curr_cx - prev_cx, curr_cy - prev_cy);
      const float size_delta = std::hypot(curr_w - prev_w, curr_h - prev_h);

      if (center_delta < params.center_deadband_px && size_delta < params.size_deadband_px) {
        // 小位移、小尺寸变化直接冻结几何，只更新置信度和类别，避免框在原地“抖尾巴”。
        box[4] = current_box[4];
        box[5] = current_box[5];
        return box;
      }

      // 运动越大，alpha 越大；运动越小，alpha 越小。
      // 这相当于一个自适应 EMA：静止时更稳，移动时更跟手。
      const float motion_ratio = std::max(center_delta / std::max(1.0f, params.center_deadband_px),
                                          size_delta / std::max(1.0f, params.size_deadband_px));
      const float alpha = std::clamp(params.min_alpha + 0.08f * motion_ratio, params.min_alpha, params.max_alpha);

      const float smooth_cx = prev_cx + alpha * (curr_cx - prev_cx);
      const float smooth_cy = prev_cy + alpha * (curr_cy - prev_cy);
      const float smooth_w = prev_w + alpha * (curr_w - prev_w);
      const float smooth_h = prev_h + alpha * (curr_h - prev_h);

      const float half_w = 0.5f * std::max(1.0f, smooth_w);
      const float half_h = 0.5f * std::max(1.0f, smooth_h);

      box[0] = smooth_cx - half_w;
      box[1] = smooth_cy - half_h;
      box[2] = smooth_cx + half_w;
      box[3] = smooth_cy + half_h;
      if (box[2] < box[0]) std::swap(box[0], box[2]);
      if (box[3] < box[1]) std::swap(box[1], box[3]);
      box[4] = current_box[4];
      box[5] = current_box[5];
      return box;
    }
  };

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
  SmoothedBoxState smoother_;
};

inline CrossFrameTargetTrackerParams::CrossFrameTargetTrackerParams()
    : iou_weight(0.80f),                  // 历史框关联的 IoU 权重，略提高以减少换框
      center_weight(0.32f),               // 历史框关联的中心点权重，略降低以保留几何连续性
      confidence_weight(0.08f),           // 历史框关联的置信度权重，降低高置信新框抢占概率
      sticky_iou_weight(0.80f),           // 粘连模式下的 IoU 权重，优先维持上一帧目标
      sticky_center_weight(0.20f),        // 粘连模式下的中心点权重
      sticky_confidence_threshold(0.8f),  // 前一帧框达到该置信度时继续粘连
      reset_smoothing_threshold(0.24f),   // 关联分数低于该值时重置平滑，值越低越不容易跳
      center_deadband_px(2.0f),           // 中心点死区（像素），增大后小抖动直接冻结
      size_deadband_px(10.0f),            // 尺寸死区（像素），增大后小抖动直接冻结
      min_alpha(0.10f),                   // 自适应平滑最小系数，降低后更稳
      max_alpha(0.24f),                   // 自适应平滑最大系数，降低后更稳
      max_lost_frames(5) {}               // 允许的最长丢失帧数，略增大以避免短暂漏检就松锁

}  // namespace ImageRecognize
