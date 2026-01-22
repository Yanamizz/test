#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <vector>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict.hpp"
#include "KalmanFilter/KalmanFilter.hpp"

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
    Tracker2D ekf;
    Eigen::Vector2d last_predict_center(0, 0);
    Eigen::Vector2d last_measure_center(0, 0);
    Eigen::Vector2d measured_velocity(0, 0);
    float last_w = 0, last_h = 0;
    bool has_detection = false;
    bool has_measured = false;
    auto last_measure_time = std::chrono::steady_clock::now();
    auto last_predicted_time = std::chrono::steady_clock::now();
    while (true) {
      if (!cap.read(frame) || frame.empty()) {
        std::cerr << "读取摄像头帧失败" << std::endl;
        break;
      }

      auto t0 = std::chrono::high_resolution_clock::now();
      auto result = predictor.run(frame);
      auto t1 = std::chrono::high_resolution_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

      const auto frame_time = std::chrono::steady_clock::now();

      // 更新 FPS（滑动窗口约 0.5s）
      frame_count++;
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
      if (elapsed_ms >= 500) {
        fps = static_cast<double>(frame_count) * 1000.0 / static_cast<double>(elapsed_ms);
        frame_count = 0;
        last_fps_time = now;
      }

      cv::Rect bound_boxes = result.boxes.empty()
                                 ? cv::Rect()
                                 : cv::Rect(static_cast<int>(result.boxes[0][0]), static_cast<int>(result.boxes[0][1]),
                                            static_cast<int>(result.boxes[0][2] - result.boxes[0][0]),
                                            static_cast<int>(result.boxes[0][3] - result.boxes[0][1]));

      if (!result.boxes.empty()) {
        // 有检测，更新 EKF
        Eigen::Vector2d measure_center(bound_boxes.x + bound_boxes.width * 0.5,
                                       bound_boxes.y + bound_boxes.height * 0.5);
        if (has_measured) {
          const double measured_dt = std::chrono::duration<double>(frame_time - last_measure_time).count();
          if (measured_dt > 1e-6) {
            measured_velocity = (measure_center - last_measure_center) / measured_dt;
          }
        }
        last_measure_center = measure_center;
        last_measure_time = frame_time;
        last_predicted_time = frame_time;
        has_measured = true;

        last_predict_center = ekf.update(bound_boxes);
        last_w = bound_boxes.width;
        last_h = bound_boxes.height;
        has_detection = true;
      } else if (has_detection) {
        // 无检测，但之前有，进行纯预测
        const double predicted_dt = std::chrono::duration<double>(frame_time - last_predicted_time).count();
        if (predicted_dt > 1e-6) {
          ekf.predict(predicted_dt);
          last_predict_center = last_predict_center + measured_velocity * predicted_dt;
          last_predicted_time = frame_time;
        }
      }

      ImageShow::ShowNow(frame, result, ms, fps);
      if (has_detection) {
        ImageShow::ShowPredict(frame, last_predict_center, last_w, last_h);
      }
      if (ImageShow::WaitForExit()) break;
    }
  }

  catch (const Ort::Exception &e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}