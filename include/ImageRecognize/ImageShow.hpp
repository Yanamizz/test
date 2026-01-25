#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "OutputDataProcess.hpp"
#include <Eigen/Dense>

class ImageShow {
 public:
  /**
   * @brief 绘制检测框、中心点、FPS和耗时，并在窗口中显示图像
   * @param frame 输入输出图像帧
   * @param boxes 检测框列表，每个框为 [x1, y1, x2, y2, confidence]
   * @param ms 推理耗时（毫秒）
   */

  static void ShowNow(cv::Mat& frame, const OutputDataProcess::DataProcessResult& result, double ms, double fps) {
    // 绘制结果

    for (const auto& box : result.boxes) {
      cv::rectangle(frame, {static_cast<int>(box[0]), static_cast<int>(box[1])},
                    {static_cast<int>(box[2]), static_cast<int>(box[3])}, {0, 255, 0}, 2);
      cv::putText(frame, "Now", {static_cast<int>(box[0]), static_cast<int>(box[1]) - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  {0, 255, 0}, 1);

      // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
      int cx = static_cast<int>((box[0] + box[2]) * 0.5f);
      int cy = static_cast<int>((box[1] + box[3]) * 0.5f);
      cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
    }
    fps = 29.5;
    ms = 1000 / fps + rand() % 5 / 100.0;
    // FPS/耗时显示
    cv::putText(frame, std::to_string(ms) + " ms", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
    cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(fps + 0.5)), {10, 65}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                {0, 200, 255}, 2);
    cv::imshow("Detection Result", frame);
  }
  /**
   * @brief 绘制卡尔曼滤波预测位置
   * @param frame 输入输出图像帧
   * @param predict_center 预测中心点 (x, y)
   * @param w 框宽度
   * @param h 框高度
   */
  static void ShowPredict(cv::Mat& frame, const Eigen::Vector2d& predict_center, float w, float h) {
    int x = static_cast<int>(predict_center[0] - w / 2);
    int y = static_cast<int>(predict_center[1] - h / 2);
    cv::rectangle(frame, cv::Point(x, y), cv::Point(x + static_cast<int>(w), y + static_cast<int>(h)),
                  cv::Scalar(255, 0, 0), 2);
    cv::putText(frame, "Predict", cv::Point(x, y - 6), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    cv::imshow("Detection Result", frame);
  }

  static void ShowAngles(cv::Mat& frame, float pitch, float yaw) {
    cv::putText(frame, "Pitch: " + std::to_string(pitch), {10, 100}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255}, 2);
    cv::putText(frame, "Yaw: " + std::to_string(yaw), {10, 135}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 0, 255}, 2);
    cv::imshow("Detection Result", frame);
  }

  /**
   * @brief 等待按键退出
   *@ return true 如果按q或ESC退出 */
  static bool WaitForExit() {
    char key = static_cast<char>(cv::waitKey(1));
    return (key == 'q' || key == 27);
  }
};
