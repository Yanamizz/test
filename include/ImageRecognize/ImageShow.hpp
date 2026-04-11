#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include "OutputDataProcess.hpp"
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
      default:
        return "unknown";
    }
  }

  /**
   * @brief 绘制检测框、中心点和 FPS，并在窗口中显示图像
   * @param frame 输入输出图像帧
   * @param boxes 检测框列表，每个框为 [x1, y1, x2, y2, confidence]
   */

  static void ShowNow(cv::Mat &frame, const ImageRecognize::DataProcessResult &result, double fps) {
    // 绘制结果
    for (const auto &box : result.boxes) {
      cv::rectangle(frame, {static_cast<int>(box[0]), static_cast<int>(box[1])},
                    {static_cast<int>(box[2]), static_cast<int>(box[3])}, {0, 255, 0}, 2);
      const int class_id = static_cast<int>(box[5]);
      const std::string label = std::string(ClassName(class_id)) + " " + cv::format("%.2f", box[4]);
      cv::putText(frame, label, {static_cast<int>(box[0]), static_cast<int>(box[1]) - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  {0, 255, 0}, 1);

      // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
      int cx = static_cast<int>((box[0] + box[2]) * 0.5f);
      int cy = static_cast<int>((box[1] + box[3]) * 0.5f);
      cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
    }
    cv::putText(frame, "FPS: " + std::to_string(fps), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    static bool window_initialized = false;
    if (!window_initialized) {
      cv::namedWindow("Detection Result", cv::WINDOW_NORMAL);
      cv::resizeWindow("Detection Result", 640, 640);
      window_initialized = true;
    }
    cv::imshow("Detection Result", frame);
  }
  static void ShowPred(cv::Mat &frame, double pred_cx, double pred_cy) {
    cv::circle(frame, {static_cast<int>(pred_cx), static_cast<int>(pred_cy)}, 4, {0, 255, 0}, -1);
  }

  static void ShowAngles(cv::Mat &frame, float yaw, float pitch, float imu_yaw, float imu_pitch, float offset_yaw,
                         float offset_pitch, float distance) {
    (void)frame;
    (void)yaw;
    (void)pitch;
    (void)imu_yaw;
    (void)imu_pitch;
    std::string text = " Offset_Yaw: " + std::to_string(offset_yaw) + " Offset_Pitch: " + std::to_string(offset_pitch) +
                       " Distance: " + std::to_string(distance);
    std::cout << text << std::endl;
  }

  /**
   * @brief 等待按键退出
   *@ return true 如果按q或ESC退出 */
  static bool WaitForExit() {
    char key = static_cast<char>(cv::waitKey(1));
    return (key == 'q' || key == 27);
  }
};
}  // namespace ImageRecognize