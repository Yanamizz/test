/**
 * @file    include/ImageRecognize/ImageShow.hpp
 * @brief   提供检测框、预测点、距离和锁定进度等图像可视化绘制能力。
 */

#pragma once

#include "OutputDataProcess.hpp"
#include <array>
#include <opencv2/opencv.hpp>
#include <string>
namespace ImageRecognize {

struct OverlayData {
  bool show_detection_center = false;
  cv::Point2f detection_center{};
  bool show_tracked_center = false;
  cv::Point2f tracked_center{};
  bool show_distance = false;
  float distance = 0.0f;
  bool show_box_size_debug = false;
  float box_width_px = 0.0f;
  float box_height_px = 0.0f;
  bool show_distance_debug = false;
  float width_distance = 0.0f;
  float height_distance = 0.0f;
  float tilt_ratio_dh_over_dw = 1.0f;
  float tilt_angle_deg = 0.0f;
  float expected_height_pixel_from_width = 0.0f;
  float tilt_ratio_deadband_dh_over_dw = 1.0f;
  float tilt_pitch_correction_deg = 0.0f;
  const char *distance_source = "NONE";
  bool show_angle_offset_debug = false;
  float yaw_offset_deg = 0.0f;
  float pitch_offset_deg = 0.0f;
};

class ImageShow {
public:
  static const char *ClassName(int class_id) {
    switch (class_id) {
    case 0:
      return "red";
    case 1:
      return "blue";
    case 2:
      return "purple";
    case 3:
      return "target";
    default:
      return "unknown";
    }
  }

  /**
   * @brief 仅绘制检测框、中心点和 FPS，不执行窗口展示。
   * @param frame 输入输出图像帧
   * @param result 检测结果
   * @param fps 帧率
   */
  static void DrawNow(cv::Mat &frame,
                      const ImageRecognize::DataProcessResult &result,
                      double fps) {
    // 绘制结果
    for (const auto &box : result.boxes) {
      const cv::Point pt1{static_cast<int>(box[0]), static_cast<int>(box[1])};
      const cv::Point pt2{static_cast<int>(box[2]), static_cast<int>(box[3])};

      cv::rectangle(frame, pt1, pt2, {0, 255, 0}, 2);
      const int class_id = static_cast<int>(box[5]);
      const std::string label =
          std::string(ClassName(class_id)) + " " + cv::format("%.2f", box[4]);
      cv::putText(frame, label, {pt1.x, pt1.y - 6}, cv::FONT_HERSHEY_SIMPLEX,
                  0.5, {0, 255, 0}, 1);

      // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
      int cx = static_cast<int>((pt1.x + pt2.x) * 0.5f);
      int cy = static_cast<int>((pt1.y + pt2.y) * 0.5f);
      cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
    }
    cv::putText(frame, "FPS: " + std::to_string(fps), {10, 30},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
  }

  static void DrawStatusText(cv::Mat &frame, double fps, int stage,
                             double progress, int threshold,
                             const OverlayData &overlay_data = {}) {
    cv::putText(frame, "FPS: " + std::to_string(fps), {10, 30},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
    ShowLockProgress(frame, stage, progress, threshold);
    if (overlay_data.show_distance) {
      ShowDistance(frame, overlay_data.distance);
    }
    if (overlay_data.show_box_size_debug) {
      ShowBoxSizeDebug(frame, overlay_data.box_width_px,
                       overlay_data.box_height_px);
    }
    if (overlay_data.show_distance_debug) {
      ShowDistanceDebug(frame, overlay_data.width_distance,
                        overlay_data.height_distance,
                        overlay_data.tilt_ratio_dh_over_dw,
                        overlay_data.tilt_angle_deg,
                        overlay_data.expected_height_pixel_from_width,
                        overlay_data.tilt_ratio_deadband_dh_over_dw,
                        overlay_data.tilt_pitch_correction_deg,
                        overlay_data.distance_source);
    }
    if (overlay_data.show_angle_offset_debug) {
      ShowAngleOffsetDebug(frame, overlay_data.yaw_offset_deg,
                           overlay_data.pitch_offset_deg);
    }
  }

  static void ShowFrame(const cv::Mat &frame) {
    static bool window_initialized = false;
    if (!window_initialized) {
      cv::namedWindow("Detection Result", cv::WINDOW_NORMAL);
      cv::resizeWindow("Detection Result", kWindowWidth, kWindowHeight);
      window_initialized = true;
    }
    cv::imshow("Detection Result", frame);
  }

  /**
   * @brief 兼容旧接口：绘制后立刻展示。
   */
  static void ShowNow(cv::Mat &frame,
                      const ImageRecognize::DataProcessResult &result,
                      double fps) {
    DrawNow(frame, result, fps);
    ShowFrame(frame);
  }

  static void ShowDetectionCenter(cv::Mat &frame, double cx, double cy) {
    DrawCenterMarker(frame, cx, cy, {0, 165, 255}, "DET");
  }

  static void ShowPred(cv::Mat &frame, double pred_cx, double pred_cy) {
    DrawCenterMarker(frame, pred_cx, pred_cy, {0, 255, 0}, "PRED");
  }

  static void ShowDistance(cv::Mat &frame, float distance) {
    cv::putText(frame, "Distance: " + std::to_string(distance), {10, 60},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
  }

  static void ShowBoxSizeDebug(cv::Mat &frame, float box_width_px,
                               float box_height_px) {
    cv::putText(frame,
                "Wpx: " + cv::format("%.1f", box_width_px) +
                    " Hpx: " + cv::format("%.1f", box_height_px),
                {10, 122}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 255, 0}, 2);
  }

  static void ShowDistanceDebug(cv::Mat &frame, float width_distance,
                                float height_distance,
                                float tilt_ratio_dh_over_dw,
                                float tilt_angle_deg,
                                float expected_height_pixel_from_width,
                                float tilt_ratio_deadband_dh_over_dw,
                                float tilt_pitch_correction_deg,
                                const char *distance_source) {
    const std::string source =
        distance_source != nullptr ? distance_source : "NONE";
    cv::putText(frame,
                "Dw: " + cv::format("%.2f", width_distance) +
                    " Dh: " + cv::format("%.2f", height_distance) +
                    " TiltAng: " + cv::format("%.1f", tilt_angle_deg) +
                    " Eh: " +
                    cv::format("%.1f", expected_height_pixel_from_width) +
                    " R: " + cv::format("%.3f", tilt_ratio_dh_over_dw) + "/" +
                    cv::format("%.3f", tilt_ratio_deadband_dh_over_dw) +
                    " Tc: " + cv::format("%.3f", tilt_pitch_correction_deg) +
                    " Src: " + source,
                {10, 150}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 255, 0}, 2);
  }

  static void ShowAngleOffsetDebug(cv::Mat &frame, float yaw_offset_deg,
                                   float pitch_offset_deg) {
    cv::putText(frame,
                "YawOff: " + cv::format("%.3f", yaw_offset_deg) +
                    " PitchOff: " + cv::format("%.3f", pitch_offset_deg),
                {10, 178}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 255, 0}, 2);
  }

  static void ShowLockProgress(cv::Mat &frame, int stage, double progress,
                               int threshold) {
    const std::string text =
        "Stage: " + std::to_string(stage) + "  P: " +
        cv::format("%.1f/%d", progress, threshold);
    cv::putText(frame, text, {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.85,
                {0, 255, 255}, 2);
  }

  /**
   * @brief 等待按键退出
   *@ return true 如果按q或ESC退出 */
  static int PollKey() { return cv::waitKey(1); }

  static bool IsExitKey(int key) {
    key &= 0xff;
    return (key == 'q' || key == 'Q' || key == 27);
  }

  static bool WaitForExit() { return IsExitKey(PollKey()); }

private:
  static void DrawCenterMarker(cv::Mat &frame, double cx, double cy,
                               const cv::Scalar &color, const char *label) {
    const cv::Point center{static_cast<int>(cx), static_cast<int>(cy)};
    cv::circle(frame, center, 5, color, -1);
    cv::putText(frame, label, {center.x + 6, center.y - 6},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
  }

  static constexpr int kWindowWidth = 960;
  static constexpr int kWindowHeight = 600;
};

inline void DrawTrackedBox(cv::Mat &frame, const std::array<float, 6> &box) {
  const cv::Point pt1{static_cast<int>(box[0]), static_cast<int>(box[1])};
  const cv::Point pt2{static_cast<int>(box[2]), static_cast<int>(box[3])};
  cv::rectangle(frame, pt1, pt2, {0, 255, 255}, 3);

  const cv::Point center{static_cast<int>((box[0] + box[2]) * 0.5f),
                         static_cast<int>((box[1] + box[3]) * 0.5f)};
  cv::circle(frame, center, 4, {0, 255, 255}, -1);

  const std::string label = std::string("Track ") + cv::format("%.2f", box[4]);
  cv::putText(frame, label, {pt1.x, pt1.y - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
              {0, 255, 255}, 1);
}

inline cv::Point2f BoxCenter(const std::array<float, 6> &box) {
  return {0.5f * (box[0] + box[2]), 0.5f * (box[1] + box[3])};
}

inline void DrawFullOverlay(cv::Mat &frame,
                            const ImageRecognize::DataProcessResult &result,
                            double fps, int stage, double progress,
                            int threshold,
                            const OverlayData &overlay_data,
                            bool has_tracked_box,
                            const std::array<float, 6> &tracked_box) {
  ImageShow::ShowLockProgress(frame, stage, progress, threshold);
  ImageShow::DrawNow(frame, result, fps);
  if (overlay_data.show_detection_center) {
    ImageShow::ShowDetectionCenter(frame, overlay_data.detection_center.x,
                                   overlay_data.detection_center.y);
  }
  if (overlay_data.show_tracked_center) {
    ImageShow::ShowPred(frame, overlay_data.tracked_center.x,
                        overlay_data.tracked_center.y);
  }
  if (overlay_data.show_distance) {
    ImageShow::ShowDistance(frame, overlay_data.distance);
  }
  if (overlay_data.show_box_size_debug) {
    ImageShow::ShowBoxSizeDebug(frame, overlay_data.box_width_px,
                                overlay_data.box_height_px);
  }
  if (overlay_data.show_distance_debug) {
    ImageShow::ShowDistanceDebug(frame, overlay_data.width_distance,
                                 overlay_data.height_distance,
                                 overlay_data.tilt_ratio_dh_over_dw,
                                 overlay_data.tilt_angle_deg,
                                 overlay_data.expected_height_pixel_from_width,
                                 overlay_data.tilt_ratio_deadband_dh_over_dw,
                                 overlay_data.tilt_pitch_correction_deg,
                                 overlay_data.distance_source);
  }
  if (overlay_data.show_angle_offset_debug) {
    ImageShow::ShowAngleOffsetDebug(frame, overlay_data.yaw_offset_deg,
                                    overlay_data.pitch_offset_deg);
  }
  if (has_tracked_box) {
    DrawTrackedBox(frame, tracked_box);
  }
}
} // namespace ImageRecognize
