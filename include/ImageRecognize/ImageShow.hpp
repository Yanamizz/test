#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "OutputDataProcess.hpp"
namespace ImageRecognize {
class ImageShow {
 public:
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
      cv::putText(frame, "Now", {static_cast<int>(box[0]), static_cast<int>(box[1]) - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  {0, 255, 0}, 1);

      // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
      int cx = static_cast<int>((box[0] + box[2]) * 0.5f);
      int cy = static_cast<int>((box[1] + box[3]) * 0.5f);
      cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
    }
    cv::putText(frame, "FPS: " + std::to_string(fps), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 0}, 2);
    cv::imshow("Detection Result", frame);
  }

  static void ShowAngles(cv::Mat &frame, float yaw, float pitch, float imu_yaw, float imu_pitch, float offset_yaw,
                         float offset_pitch, float distance) {
    cv::putText(frame, "Pitch: " + std::to_string(pitch), {10, 100}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255}, 2);
    cv::putText(frame, "Yaw: " + std::to_string(yaw), {10, 135}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255}, 2);
    cv::putText(frame, "IMU Pitch: " + std::to_string(imu_pitch), {10, 170}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                {255, 0, 255}, 2);
    cv::putText(frame, "IMU Yaw: " + std::to_string(imu_yaw), {10, 205}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255},
                2);
    cv::putText(frame, "Offset Pitch: " + std::to_string(offset_pitch), {10, 240}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                {255, 0, 255}, 2);
    cv::putText(frame, "Offset Yaw: " + std::to_string(offset_yaw), {10, 275}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                {255, 0, 255}, 2);
    cv::putText(frame, "Distance: " + std::to_string(distance), {10, 310}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255},
                2);
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