/**
 * @file src/ImagePredict.cc
 * @brief 主应用入口：负责摄像头采集、模型推理、视觉显示与串口通信流水线。
 *
 * 该文件将图像采集、推理与串口发送拆分到不同线程，通过帧缓存和命令队列保持最新帧
 * 同时在检测到目标时才发送角度，满足低延迟需求。
 */

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include <thread>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict.hpp"
#include "ImageRecognize/AngleCalculate.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "CameraTask/Getimage.hpp"
#include "SerialTask/SerialSend.hpp"
#include "SerialTask/SerialConfig.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/CalculateOffsetAngles.hpp"

// 最小角度阈值，避免发送过小角度
#define minimum_angle 1.0f

const std::string model_path = "/home/hanni/code/rm/src/model/best.onnx";

// 串口接收线程控制标志与最新角度（供主线程读取）
std::atomic<bool> g_running(true);
SerialTask::EulerAngles g_latest_angles = {0.0f, 0.0f, 0.0f};
std::mutex g_angles_mutex;

/**
 * @brief 共享帧缓存，用于在采集线程与推理线程之间传递最新帧。
 */
struct FrameBuffer {
  std::shared_ptr<cv::Mat> frame;
  bool ready = false;
  std::mutex mutex;
  std::condition_variable cv;
};

/**
 * @brief 串口发送命令队列，仅保留最新角度以避免旧帧发送。
 */
struct SerialCommandQueue {
  std::deque<GimbalAngles> commands;
  std::mutex mutex;
  std::condition_variable cv;
};

void ReceiveThread(serial::Serial *serial_port);
void CaptureThread(CameraTask::GalaxyCamera *camera, FrameBuffer *frame_buffer);
void InferenceThread(ImagePredict::ImagePredict *predictor, Tracker2D *ekf, FrameBuffer *frame_buffer,
                     SerialCommandQueue *serial_queue, FPSCounter *fps_counter);
void SerialThread(serial::Serial *serial_port, SerialCommandQueue *serial_queue, std::atomic<bool> *serial_ready);

/**
 * @brief 程序入口，初始化硬件与模块后启动采集/推理/串口线程。
 * @return 返回 0 表示正常退出，非零表示错误。
 */
int main() {
  try {
    serial::Serial serial_port;
    std::atomic<bool> serial_ready(false);
    bool serial_available = false;
    try {
      SerialTask::DefaultConfig(serial_port);
      serial_port.open();
      serial_available = true;
      serial_ready.store(true);
    } catch (const std::exception &e) {
      std::cerr << "串口打开失败，继续运行但将禁用串口功能: " << e.what() << std::endl;
    }

    ImagePredict::ImagePredict predictor(model_path);
    CameraTask::GalaxyCamera camera;
    Tracker2D ekf;
    FrameBuffer frame_buffer;
    SerialCommandQueue serial_queue;
    FPSCounter fps_counter(500);

    if (!camera.open() || !camera.start()) {
      std::cerr << "无法打开 Galaxy 相机" << std::endl;
      return -1;
    }

    std::thread receiver;
    if (serial_available) {
      receiver = std::thread(ReceiveThread, &serial_port);
    }

    std::thread capture_thread(CaptureThread, &camera, &frame_buffer);
    std::thread inference_thread(InferenceThread, &predictor, &ekf, &frame_buffer, &serial_queue, &fps_counter);
    std::thread serial_thread(SerialThread, &serial_port, &serial_queue, &serial_ready);

    inference_thread.join();
    g_running = false;
    frame_buffer.cv.notify_all();
    serial_queue.cv.notify_all();

    if (capture_thread.joinable()) {
      capture_thread.join();
    }
    if (serial_thread.joinable()) {
      serial_thread.join();
    }
    if (serial_available && receiver.joinable()) {
      receiver.join();
    }
    if (serial_available) {
      serial_port.close();
    }
    return 0;
  }

  catch (const Ort::Exception &e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}

/**
 * @brief 串口读取线程，周期性获取 IMU 报文并更新角度缓存。
 * @param serial_port 已配置好参数并打开的串口实例。
 * @brief 串口模块：接收线程实现（将接收逻辑集中在此，便于后续抽取为 SerialManager）
 * @brief 说明：该线程负责不断从串口读取 IMU 四元数并更新全局 `g_latest_angles`。
 * @brief 当 `g_running` 变为 false 时线程会退出。
 */
void ReceiveThread(serial::Serial *serial_port) {
  while (g_running) {
    SerialTask::EulerAngles temp_angles;
    if (SerialTask::ReadIMUData(*serial_port, temp_angles)) {
      std::lock_guard<std::mutex> lock(g_angles_mutex);
      g_latest_angles = temp_angles;
    }
    // 让出时间片，避免忙等待占用过多 CPU
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

/**
 * @brief 摄像头采集线程，不断获取最新帧并推送到共享缓存。
 * @param camera GalaxyCamera 实例。
 * @param buffer 共享帧缓存用于与推理线程通信。
 */
void CaptureThread(CameraTask::GalaxyCamera *camera, FrameBuffer *buffer) {
  while (g_running) {
    cv::Mat frame = camera->grab(1000);
    if (frame.empty()) {
      continue;
    }
    auto shared_frame = std::make_shared<cv::Mat>(frame);
    {
      std::lock_guard<std::mutex> lock(buffer->mutex);
      buffer->frame = shared_frame;
      buffer->ready = true;
    }
    buffer->cv.notify_one();
  }
}

/**
 * @brief 串口发送线程，从命令队列取最新角度并通过串口发送。
 * @param serial_port 已打开的串口。
 * @param queue 存放待发送角度的队列结构。
 * @param serial_ready 表示串口初始化是否完成的原子标志。
 */
void SerialThread(serial::Serial *serial_port, SerialCommandQueue *queue, std::atomic<bool> *serial_ready) {
  while (g_running) {
    GimbalAngles angles;
    {
      std::unique_lock<std::mutex> lock(queue->mutex);
      queue->cv.wait(lock, [&]() { return !queue->commands.empty() || !g_running; });
      if (!queue->commands.empty()) {
        angles = queue->commands.front();
        queue->commands.pop_front();
      } else if (!g_running) {
        break;
      } else {
        continue;
      }
    }
    if (!serial_ready->load()) {
      continue;
    }
    SerialTask::EulerAngles current_angles;
    {
      std::lock_guard<std::mutex> lock(g_angles_mutex);
      current_angles = g_latest_angles;
    }
    SerialTask::SerialSend(*serial_port, current_angles, static_cast<float>(angles.pitch),
                           static_cast<float>(angles.yaw));
  }
}

/**
 * @brief 推理线程：从帧缓存取帧、跑模型、更新预测、发送结果。
 * @param predictor 模型推理器。
 * @param ekf 卡尔曼滤波器实例。
 * @param buffer 图像帧缓存。
 * @param queue 串口命令队列。
 * @param fps_counter FPS 统计器。
 */
void InferenceThread(ImagePredict::ImagePredict *predictor, Tracker2D *ekf, FrameBuffer *buffer,
                     SerialCommandQueue *queue, FPSCounter *fps_counter) {
  OffsetAngles offset_calculator;
  Eigen::Vector2d last_predict_center(0, 0);
  float last_w = 0, last_h = 0;
  bool has_detection = false;
  while (g_running) {
    std::shared_ptr<cv::Mat> frame_ptr;
    {
      std::unique_lock<std::mutex> lock(buffer->mutex);
      buffer->cv.wait(lock, [&]() { return buffer->ready || !g_running; });
      if (!buffer->ready) {
        if (!g_running) {
          break;
        }
        continue;
      }
      frame_ptr = buffer->frame;
      buffer->ready = false;
    }
    if (!frame_ptr || frame_ptr->empty()) {
      continue;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = predictor->run(*frame_ptr);
    cv::Rect bound_boxes;
    bool detection_now = !result.boxes.empty();
    if (detection_now) {
      bound_boxes = cv::Rect(static_cast<int>(result.boxes[0][0]), static_cast<int>(result.boxes[0][1]),
                             static_cast<int>(result.boxes[0][2] - result.boxes[0][0]),
                             static_cast<int>(result.boxes[0][3] - result.boxes[0][1]));
      last_predict_center = ekf->update(bound_boxes);
      last_w = bound_boxes.width;
      last_h = bound_boxes.height;
      has_detection = true;
    } else {
      has_detection = false;
      last_predict_center = Eigen::Vector2d(0, 0);
      last_w = 0;
      last_h = 0;
    }

    GimbalAngles angles = offset_calculator.CalculateOffsetAngles(last_predict_center, last_w, last_h, has_detection);
    if (has_detection && (abs(angles.yaw) > minimum_angle || abs(angles.pitch) > minimum_angle)) {
      std::lock_guard<std::mutex> lock(queue->mutex);
      if (queue->commands.size() >= 1) {
        queue->commands.pop_front();
      }
      queue->commands.emplace_back(angles);
      queue->cv.notify_one();
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    fps_counter->tick();
    double fps = fps_counter->get();

    ImageShow::ShowNow(*frame_ptr, result, ms, fps);
    if (has_detection) {
      ImageShow::ShowPredict(*frame_ptr, last_predict_center, last_w, last_h);
      ImageShow::ShowAngles(*frame_ptr, static_cast<float>(angles.pitch), static_cast<float>(angles.yaw));
    }
    if (ImageShow::WaitForExit()) {
      g_running = false;
      buffer->cv.notify_all();
      queue->cv.notify_all();
      break;
    }
  }
}
