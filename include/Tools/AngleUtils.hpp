/**
 * @file    include/Tools/AngleUtils.hpp
 * @brief   提供角度处理的通用轻量工具函数。
 */

#pragma once

namespace Tools {

inline float NormalizeDeltaDeg(float delta) {
  while (delta > 180.0f) {
    delta -= 360.0f;
  }
  while (delta < -180.0f) {
    delta += 360.0f;
  }
  return delta;
}

} // namespace Tools

