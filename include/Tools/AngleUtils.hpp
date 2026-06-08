/**
 * @file    include/Tools/AngleUtils.hpp
 * @brief   提供角度处理的通用轻量工具函数。
 *
 * 该文件包含角度归一化、最短角差、限幅、死区等无状态小函数，供角度计算、
 * 扫描控制和发送前处理复用。函数保持 header-only，避免在热路径引入额外依赖。
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
