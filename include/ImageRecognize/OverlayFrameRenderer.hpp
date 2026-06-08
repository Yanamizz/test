/**
 * @file    include/ImageRecognize/OverlayFrameRenderer.hpp
 * @brief   根据同一份可视化快照生成显示、全程录像和目标录像画面。
 *
 * OverlayFrameRenderer 将主流程收集的 OverlayData、检测结果和阶段状态
 * 绘制到输出帧上，保证显示窗口与录像使用一致的叠加语义。该模块只处理
 * 图像渲染，不改变识别结果、跟踪状态或串口发送命令。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "ImageRecognize/ImageShow.hpp"

namespace ImageRecognize {

struct OverlayFrameRenderInput {
  const cv::Mat *source_frame = nullptr;
  const ImageRecognize::DataProcessResult *result = nullptr;
  double fps = 0.0;
  int stage = 0;
  double progress = 0.0;
  int threshold = 0;
  OverlayData overlay_data{};
  bool has_tracked_box = false;
  DetectionBox tracked_box{};
  bool need_full_overlay_frame = false;
  bool need_target_status_frame = false;
};

struct OverlayFrameRenderResult {
  cv::Mat full_overlay_frame;
  cv::Mat target_status_frame;
};

inline bool HasSourceFrame(const OverlayFrameRenderInput &input);
inline bool CanRenderFullOverlay(const OverlayFrameRenderInput &input);
inline cv::Mat CloneSourceFrame(const OverlayFrameRenderInput &input);

inline OverlayFrameRenderResult
RenderOverlayFrames(const OverlayFrameRenderInput &input) {
  OverlayFrameRenderResult output{};
  if (!HasSourceFrame(input)) {
    return output;
  }

  if (CanRenderFullOverlay(input)) {
    output.full_overlay_frame = CloneSourceFrame(input);
    DrawFullOverlay(output.full_overlay_frame, *input.result, input.fps,
                    input.stage, input.progress, input.threshold,
                    input.overlay_data, input.has_tracked_box,
                    input.tracked_box);
  }

  if (input.need_target_status_frame) {
    output.target_status_frame = CloneSourceFrame(input);
    ImageShow::DrawStatusText(output.target_status_frame, input.fps,
                              input.stage, input.progress, input.threshold,
                              input.overlay_data);
  }

  return output;
}

inline bool HasSourceFrame(const OverlayFrameRenderInput &input) {
  return input.source_frame != nullptr && !input.source_frame->empty();
}

inline bool CanRenderFullOverlay(const OverlayFrameRenderInput &input) {
  return input.need_full_overlay_frame && input.result != nullptr;
}

inline cv::Mat CloneSourceFrame(const OverlayFrameRenderInput &input) {
  return input.source_frame != nullptr ? input.source_frame->clone() : cv::Mat{};
}

} // namespace ImageRecognize
