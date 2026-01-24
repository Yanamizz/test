#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <vector>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict.hpp"
#include "ImageRecognize/AngleCalculate.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "CameraTask/Getimage.hpp"
#include "SerialTask/SerialSend.hpp"
#include "SerialTask/SerialConfig.hpp"

#define minimum_angle 1.0f  // 最小角度阈值，避免发送过小角度

int main() {
  try {
    // 模型路径
    std::string model_path = "/home/hanni/code/rm/test/ImageRecognize/model/best.onnx";
    ImagePredict::ImagePredict predictor(model_path);  // 复用会话

    CameraTask::GalaxyCamera camera;

    // cv::VideoCapture cap(0);
    // cap.release();
    // if (!cap.open("/dev/video0", cv::CAP_V4L2)) {
    //   std::cerr << "无法打开摄像头" << std::endl;
    //   return -1;
    // }
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    // cap.set(cv::CAP_PROP_FPS, 30);

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

    if (!camera.open() || !camera.start()) {
      std::cerr << "无法打开 Galaxy 相机" << std::endl;
      return -1;
    }

    while (true) {
      frame = camera.grab(1000);

      auto t0 = std::chrono::high_resolution_clock::now();
      auto result = predictor.run(frame);

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
        // 无检测，但之前有，保持最后位置
        last_predict_center = last_predict_center;
      }

      DetectionResult predict_detection;
      predict_detection.x = last_predict_center[0] - last_w / 2;
      predict_detection.y = last_predict_center[1] - last_h / 2;
      predict_detection.w = last_w;
      predict_detection.h = last_h;
      predict_detection.center_x = last_predict_center[0];
      predict_detection.center_y = last_predict_center[1];
      AngleCalculator angle_calculator;
      GimbalAngles angles = angle_calculator.CalculateGimbalAngles(predict_detection);

      // 通过串口发送角度数据
      serial::Serial serial_port;
      SerialTask::DefaultConfig(serial_port);

      if (abs(angles.yaw) > minimum_angle && abs(angles.pitch) > minimum_angle) {
        SerialTask::SerialSend(serial_port, static_cast<float>(-angles.pitch), static_cast<float>(-angles.yaw));
      }

      auto t1 = std::chrono::high_resolution_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

      ImageShow::ShowNow(frame, result, ms, fps);
      if (has_detection) {
        ImageShow::ShowPredict(frame, last_predict_center, last_w, last_h);
        ImageShow::ShowAngles(frame, static_cast<float>(angles.pitch), static_cast<float>(angles.yaw));
      }
      if (ImageShow::WaitForExit()) break;
    }

  }

  catch (const Ort::Exception &e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}