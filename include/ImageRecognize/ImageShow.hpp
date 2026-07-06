/**
 * @file    include/ImageRecognize/ImageShow.hpp
 * @brief   检测、跟踪、距离和控制调试信息的图像叠加绘制工具。
 *
 * ImageShow 只负责把主流程已经计算好的 OverlayData 绘制到 OpenCV 图像
 * 上，不参与识别、跟踪、距离估计或串口发送决策。当前叠加信息包括
 * 检测框与类别、检测/跟踪中心点、FPS、阶段锁定进度、框宽高、Dw/Dh
 * 距离调试、距离来源、yaw/pitch 发送偏角、LaserPc 激光 pitch 补偿、
 * 最终绝对角 AbsY/AbsP 以及扫描状态。OverlayData 中的 show_* 字段用于
 * 控制每组调试文本是否绘制，便于主流程按帧状态选择性展示。
 */

#pragma once

#include "OutputDataProcess.hpp"
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
  const char *distance_source = "NONE";
  bool show_shadow_distance_debug = false;
  float shadow_width_distance = 0.0f;
  float shadow_height_distance = 0.0f;
  bool shadow_width_valid = false;
  bool shadow_height_valid = false;
  bool show_angle_offset_debug = false;
  float yaw_offset_deg = 0.0f;
  float pitch_offset_deg = 0.0f;
  bool show_laser_pitch_comp_debug = false;
  float laser_pitch_comp_deg = 0.0f;
  bool show_absolute_angle_debug = false;
  float absolute_yaw_deg = 0.0f;
  float absolute_pitch_deg = 0.0f;
  bool show_scan_state_debug = false;
  bool scan_active = false;
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

  static void DrawDetections(cv::Mat &frame,
                             const ImageRecognize::DataProcessResult &result) {
    for (const auto &box : result.boxes) {
      const cv::Point pt1{static_cast<int>(BoxX1(box)),
                          static_cast<int>(BoxY1(box))};
      const cv::Point pt2{static_cast<int>(BoxX2(box)),
                          static_cast<int>(BoxY2(box))};

      cv::rectangle(frame, pt1, pt2, {0, 255, 0}, 2);
      const std::string label =
          std::string(ClassName(BoxClassId(box))) + " " +
          cv::format("%.2f", BoxScore(box));
      cv::putText(frame, label, {pt1.x, pt1.y - 6}, cv::FONT_HERSHEY_SIMPLEX,
                  0.5, {0, 255, 0}, 1);

      // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
      int cx = static_cast<int>((pt1.x + pt2.x) * 0.5f);
      int cy = static_cast<int>((pt1.y + pt2.y) * 0.5f);
      cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
    }
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
                        overlay_data.height_distance, overlay_data.distance_source);
    }
    if (overlay_data.show_shadow_distance_debug) {
      ShowShadowDistanceDebug(frame, overlay_data.shadow_width_distance,
                              overlay_data.shadow_height_distance,
                              overlay_data.shadow_width_valid,
                              overlay_data.shadow_height_valid);
    }
    if (overlay_data.show_angle_offset_debug) {
      ShowAngleOffsetDebug(frame, overlay_data.yaw_offset_deg,
                           overlay_data.pitch_offset_deg);
    }
    if (overlay_data.show_laser_pitch_comp_debug ||
        overlay_data.show_absolute_angle_debug ||
        overlay_data.show_scan_state_debug) {
      ShowControlDebug(frame, overlay_data);
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
                                float height_distance, const char *distance_source) {
    const std::string source =
        distance_source != nullptr ? distance_source : "NONE";
    cv::putText(frame,
                "Dw: " + cv::format("%.2f", width_distance) +
                    " Dh: " + cv::format("%.2f", height_distance) +
                    " Src: " + source,
                {10, 150}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 255, 0}, 2);
  }

  static void ShowShadowDistanceDebug(cv::Mat &frame, float shadow_width_distance,
                                      float shadow_height_distance,
                                      bool shadow_width_valid,
                                      bool shadow_height_valid) {
    const std::string dw_text = shadow_width_valid
                                    ? cv::format("%.2f", shadow_width_distance)
                                    : std::string("--");
    const std::string dh_text = shadow_height_valid
                                    ? cv::format("%.2f", shadow_height_distance)
                                    : std::string("--");
    cv::putText(frame,
                "DwL: " + dw_text + " DhL: " + dh_text,
                {10, 178}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 220, 120}, 2);
  }

  static void ShowAngleOffsetDebug(cv::Mat &frame, float yaw_offset_deg,
                                   float pitch_offset_deg) {
    cv::putText(frame,
                "YawOff: " + cv::format("%.3f", yaw_offset_deg) +
                    " PitchOff: " + cv::format("%.3f", pitch_offset_deg),
                {10, 206}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {255, 255, 0}, 2);
  }

  static void ShowControlDebug(cv::Mat &frame,
                               const OverlayData &overlay_data) {
    std::string text;
    if (overlay_data.show_laser_pitch_comp_debug) {
      text += "LaserPc: " + cv::format("%.3f", overlay_data.laser_pitch_comp_deg);
    }
    if (overlay_data.show_absolute_angle_debug) {
      if (!text.empty()) {
        text += " ";
      }
      text += "AbsY: " + cv::format("%.2f", overlay_data.absolute_yaw_deg) +
              " AbsP: " + cv::format("%.2f", overlay_data.absolute_pitch_deg);
    }
    if (overlay_data.show_scan_state_debug) {
      if (!text.empty()) {
        text += " ";
      }
      text += std::string("Scan: ") + (overlay_data.scan_active ? "ON" : "OFF");
    }
    if (!text.empty()) {
      cv::putText(frame, text, {10, 234}, cv::FONT_HERSHEY_SIMPLEX, 0.62,
                  {255, 255, 0}, 2);
    }
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

inline void DrawTrackedBox(cv::Mat &frame, const DetectionBox &box) {
  const cv::Point pt1{static_cast<int>(BoxX1(box)),
                      static_cast<int>(BoxY1(box))};
  const cv::Point pt2{static_cast<int>(BoxX2(box)),
                      static_cast<int>(BoxY2(box))};
  cv::rectangle(frame, pt1, pt2, {0, 255, 255}, 3);

  const BoxCenterPoint box_center = BoxCenter(box);
  const cv::Point center{static_cast<int>(box_center.x),
                         static_cast<int>(box_center.y)};
  cv::circle(frame, center, 4, {0, 255, 255}, -1);

  const std::string label =
      std::string("Track ") + cv::format("%.2f", BoxScore(box));
  cv::putText(frame, label, {pt1.x, pt1.y - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
              {0, 255, 255}, 1);
}

inline void DrawFullOverlay(cv::Mat &frame,
                            const ImageRecognize::DataProcessResult &result,
                            double fps, int stage, double progress,
                            int threshold,
                            const OverlayData &overlay_data,
                            bool has_tracked_box,
                            const DetectionBox &tracked_box) {
  ImageShow::DrawStatusText(frame, fps, stage, progress, threshold,
                            overlay_data);
  ImageShow::DrawDetections(frame, result);
  if (overlay_data.show_detection_center) {
    ImageShow::ShowDetectionCenter(frame, overlay_data.detection_center.x,
                                   overlay_data.detection_center.y);
  }
  if (overlay_data.show_tracked_center) {
    ImageShow::ShowPred(frame, overlay_data.tracked_center.x,
                        overlay_data.tracked_center.y);
  }
  if (has_tracked_box) {
    DrawTrackedBox(frame, tracked_box);
  }
}
} // namespace ImageRecognize
