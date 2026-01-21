#include <opencv2/opencv.hpp>
#include <iostream>
#include "ImageRecognize/ImagePredict.hpp"
#include <string>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <chrono>
#include <vector>

int main() {
  try {
    // 模型路径
    std::string model_path = "/home/hanni/code/rm/test/ImageRecognize/model/best.onnx";
    ImagePredict::ImagePredict predictor(model_path);  // 复用会话

    cv::VideoCapture cap(0);
    // 打开摄像头，优先用 V4L2 后端，失败则回退
    cap.release();  // Release the previous capture before re-opening
    if (!cap.open("/dev/video0", cv::CAP_V4L2)) {
      std::cerr << "V4L2 打开 /dev/video0 失败，尝试 CAP_ANY/index 0" << std::endl;
      if (!cap.open(0, cv::CAP_ANY)) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
      }
    }
    // 设置常见支持格式与分辨率/FPS（根据设备能力，可能被驱动调整）
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  // 优先 MJPG，降低解码成本
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cv::Mat frame;
    int frame_count = 0;
    double fps = 0.0;

    auto last_fps_time = std::chrono::steady_clock::now();
    while (true) {
      if (!cap.read(frame) || frame.empty()) {
        std::cerr << "读取摄像头帧失败" << std::endl;
        break;
      }

      auto t0 = std::chrono::high_resolution_clock::now();
      auto result = predictor.run(frame);
      auto t1 = std::chrono::high_resolution_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

      // 更新 FPS（滑动窗口约 0.5s）
      frame_count++;
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
      if (elapsed_ms >= 500) {
        fps = static_cast<double>(frame_count) * 1000.0 / static_cast<double>(elapsed_ms);
        frame_count = 0;
        last_fps_time = now;
      }

      // 绘制结果
      for (const auto &box : result.boxes) {
        cv::rectangle(frame, {static_cast<int>(box[0]), static_cast<int>(box[1])},
                      {static_cast<int>(box[2]), static_cast<int>(box[3])}, {0, 255, 0}, 2);
        cv::putText(frame, std::to_string(box[4]),
                    {static_cast<int>(box[0]), std::max(0, static_cast<int>(box[1]) - 6)}, cv::FONT_HERSHEY_SIMPLEX,
                    0.5, {0, 255, 0}, 1);

        // 计算并绘制中心点：((x1+x2)/2, (y1+y2)/2)
        int cx = static_cast<int>((box[0] + box[2]) * 0.5f);
        int cy = static_cast<int>((box[1] + box[3]) * 0.5f);
        cv::circle(frame, {cx, cy}, 4, {0, 0, 255}, -1);
      }

      // FPS/耗时显示
      cv::putText(frame, std::to_string(ms) + " ms", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
      cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(fps + 0.5)), {10, 65}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  {0, 200, 255}, 2);
      cv::imshow("ImagePredict - Camera", frame);
      // 按 q 退出
      char key = static_cast<char>(cv::waitKey(1));
      if (key == 'q' || key == 27) break;
    }
  }

  catch (const Ort::Exception &e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}