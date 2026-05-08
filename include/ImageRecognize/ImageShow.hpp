#pragma once

#include "OutputDataProcess.hpp"
#include "opencv2/highgui.hpp"
#include <array>
#include <opencv2/opencv.hpp>
#include <string>
namespace ImageRecognize {
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
   * @brief 绘制检测框、中心点和 FPS，并在窗口中显示图像
   * @param frame 输入输出图像帧
   * @param boxes 检测框列表，每个框为 [x1, y1, x2, y2, confidence]
   */

  static void ShowNow(cv::Mat &frame,
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

  static void ShowAngles(cv::Mat &frame, float yaw, float pitch, float imu_yaw,
                         float imu_pitch, float offset_yaw, float offset_pitch,
                         float distance) {
    (void)frame;
    (void)yaw;
    (void)pitch;
    (void)imu_yaw;
    (void)imu_pitch;
    std::string text = " Offset_Yaw: " + std::to_string(offset_yaw) +
                       " Offset_Pitch: " + std::to_string(offset_pitch) +
                       " Distance: " + std::to_string(distance);
    // std::cout << text << std::endl;
    cv::putText(frame, "Distance: " + std::to_string(distance), {10, 60},
                cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
  }

  /**
   * @brief 等待按键退出
   *@ return true 如果按q或ESC退出 */
  static bool WaitForExit() {
    char key = static_cast<char>(cv::waitKey(1));
    return (key == 'q' || key == 27);
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
} // namespace ImageRecognize
