/**
 * @file    include/ImageRecognize/TargetClassFilter.hpp
 * @brief   根据阵营模式筛选可跟踪目标类别并输出过滤后的检测框。
 *
 * TargetClassFilter 定义红/蓝/全选等阵营模式，把原始检测结果过滤为当前
 * 允许跟踪和控制的候选框集合。该模块同时提供阵营模式控制器，用于 GUI
 * 调参面板和主流程共享同一份筛选状态。
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cctype>
#include <string>
#include <vector>

#include "ImageRecognize/OutputDataProcess.hpp"

namespace ImageRecognize {

enum class TargetCampMode {
  RedAndPurple,
  BlueAndPurple,
  All,
};

inline TargetCampMode ParseTargetCampMode(const std::string &mode) {
  std::string normalized = mode;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  if (normalized == "RED" || normalized == "RED_AND_PURPLE") {
    return TargetCampMode::RedAndPurple;
  }
  if (normalized == "BLUE" || normalized == "BLUE_AND_PURPLE") {
    return TargetCampMode::BlueAndPurple;
  }
  if (normalized == "ALL") {
    return TargetCampMode::All;
  }
  return TargetCampMode::RedAndPurple;
}

inline const char *ToString(TargetCampMode mode) {
  switch (mode) {
  case TargetCampMode::RedAndPurple:
    return "RED_AND_PURPLE";
  case TargetCampMode::BlueAndPurple:
    return "BLUE_AND_PURPLE";
  case TargetCampMode::All:
    return "ALL";
  default:
    return "UNKNOWN";
  }
}

inline int TargetCampModeToIndex(TargetCampMode mode) {
  switch (mode) {
  case TargetCampMode::RedAndPurple:
    return 0;
  case TargetCampMode::BlueAndPurple:
    return 1;
  case TargetCampMode::All:
    return 2;
  default:
    return 0;
  }
}

inline TargetCampMode TargetCampModeFromIndex(int value) {
  switch (value) {
  case 0:
    return TargetCampMode::RedAndPurple;
  case 1:
    return TargetCampMode::BlueAndPurple;
  case 2:
    return TargetCampMode::All;
  default:
    return TargetCampMode::RedAndPurple;
  }
}

class TargetCampModeController {
public:
  explicit TargetCampModeController(TargetCampMode initial_mode)
      : mode_index_(TargetCampModeToIndex(initial_mode)) {}

  TargetCampMode Get() const {
    return TargetCampModeFromIndex(
        mode_index_.load(std::memory_order_acquire));
  }

  int GetModeIndex() const {
    return mode_index_.load(std::memory_order_acquire);
  }

  bool SetModeIndex(int value) {
    const int normalized = TargetCampModeToIndex(TargetCampModeFromIndex(value));
    return mode_index_.exchange(normalized, std::memory_order_acq_rel) !=
           normalized;
  }

private:
  std::atomic<int> mode_index_;
};

inline bool ShouldTrackClassId(int class_id, TargetCampMode mode) {
  if (class_id == 2 || class_id == 3) {
    return true;
  }

  switch (mode) {
  case TargetCampMode::RedAndPurple:
    return class_id == 0;
  case TargetCampMode::BlueAndPurple:
    return class_id == 1;
  case TargetCampMode::All:
    return class_id == 0 || class_id == 1 || class_id == 2 || class_id == 3;
  default:
    return false;
  }
}

inline std::vector<DetectionBox>
FilterTrackBoxes(const std::vector<DetectionBox> &boxes, TargetCampMode mode) {
  std::vector<DetectionBox> filtered;
  filtered.reserve(boxes.size());
  for (const auto &box : boxes) {
    if (ShouldTrackClassId(BoxClassId(box), mode)) {
      filtered.push_back(box);
    }
  }
  return filtered;
}

} // namespace ImageRecognize
