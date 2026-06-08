/**
 * @file    include/Tools/CameraRoiRuntime.hpp
 * @brief   保存相机 ROI 运行时实际生效状态，供控制链路读取。
 *
 * CameraRoiRuntime 记录采集线程最终应用到相机的 ROI 开关、尺寸和偏移，用于
 * 角度计算时修正主点位置。该结构反映“实际生效”的相机状态，而不是单纯的
 * RuntimeParams 请求值。
 */

#pragma once

#include <atomic>

namespace Tools {

inline std::atomic<bool> &CameraRoiRuntimeEnabledFlag() {
  static std::atomic<bool> enabled{false};
  return enabled;
}

inline std::atomic<int> &CameraRoiRuntimeOffsetXValue() {
  static std::atomic<int> offset_x{0};
  return offset_x;
}

inline std::atomic<int> &CameraRoiRuntimeOffsetYValue() {
  static std::atomic<int> offset_y{0};
  return offset_y;
}

inline void SetCameraRoiRuntime(bool enabled, int offset_x, int offset_y) {
  CameraRoiRuntimeOffsetXValue().store(offset_x, std::memory_order_release);
  CameraRoiRuntimeOffsetYValue().store(offset_y, std::memory_order_release);
  CameraRoiRuntimeEnabledFlag().store(enabled, std::memory_order_release);
}

inline bool CameraRoiRuntimeEnabled() {
  return CameraRoiRuntimeEnabledFlag().load(std::memory_order_acquire);
}

inline int CameraRoiRuntimeOffsetX() {
  return CameraRoiRuntimeOffsetXValue().load(std::memory_order_acquire);
}

inline int CameraRoiRuntimeOffsetY() {
  return CameraRoiRuntimeOffsetYValue().load(std::memory_order_acquire);
}

} // namespace Tools
