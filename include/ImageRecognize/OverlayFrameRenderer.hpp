/**
 * @file    include/ImageRecognize/OverlayFrameRenderer.hpp
 * @brief   根据同一份可视化快照生成显示、全程录像和目标录像画面。
 */

#pragma once

#include <array>

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
  std::array<float, 6> tracked_box{};
  bool need_full_overlay_frame = false;
  bool need_target_status_frame = false;
};

struct OverlayFrameRenderResult {
  cv::Mat full_overlay_frame;
  cv::Mat target_status_frame;
};

inline OverlayFrameRenderResult
RenderOverlayFrames(const OverlayFrameRenderInput &input) {
  OverlayFrameRenderResult output{};
  if (input.source_frame == nullptr || input.source_frame->empty()) {
    return output;
  }

  if (input.need_full_overlay_frame && input.result != nullptr) {
    output.full_overlay_frame = input.source_frame->clone();
    DrawFullOverlay(output.full_overlay_frame, *input.result, input.fps,
                    input.stage, input.progress, input.threshold,
                    input.overlay_data, input.has_tracked_box,
                    input.tracked_box);
  }

  if (input.need_target_status_frame) {
    output.target_status_frame = input.source_frame->clone();
    ImageShow::DrawStatusText(output.target_status_frame, input.fps,
                              input.stage, input.progress, input.threshold,
                              input.overlay_data);
  }

  return output;
}

} // namespace ImageRecognize
