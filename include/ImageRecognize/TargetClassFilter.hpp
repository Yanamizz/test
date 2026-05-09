#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <string>
#include <vector>

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

inline std::vector<std::array<float, 6>>
FilterTrackBoxes(const std::vector<std::array<float, 6>> &boxes,
                 TargetCampMode mode) {
  std::vector<std::array<float, 6>> filtered;
  filtered.reserve(boxes.size());
  for (const auto &box : boxes) {
    const int class_id = static_cast<int>(box[5]);
    if (ShouldTrackClassId(class_id, mode)) {
      filtered.push_back(box);
    }
  }
  return filtered;
}

} // namespace ImageRecognize
