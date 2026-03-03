#include <iostream>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <deque>
#include <iterator>
#include <atomic>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "CameraTask/GetImage.hpp"

std::string model_path = "/home/hanni/code/rm/src/model/best.onnx";
std::atomic<bool> g_running(true);          // 全局运行标志
static std::mutex g_frame_mutex;            // 保护最新帧的互斥锁
static std::condition_variable g_frame_cv;  // 通知预测线程有新帧到达的条件变量
static std::mutex g_result_mutex;           // 保护最新预测结果的互斥锁
                                            // IMU 数据缓冲区
static std::deque<std::pair<std::chrono::steady_clock::time_point, SerialTask::EulerAngles>> g_imu_buffer;
static std::mutex g_imu_mutex;                       // 保护 IMU 缓冲区的互斥锁
static std::atomic<float> g_current_yaw_rate{0.0f};  // 即时角速度（由 IMUReadThread 计算并更新）——单位 deg/s
static std::atomic<float> g_current_pitch_rate{0.0f};  // 即时角速度（由 IMUReadThread 计算并更新）——单位 deg/s

// 全局：只保存最新一帧
struct FrameItem {
  cv::Mat frame;
  std::chrono::steady_clock::time_point ts{};
};

static FrameItem g_latest_frame_item;  // 最新帧条目

static bool g_has_frame = false;    // 是否有新帧到达
static bool has_detection = false;  // 是否有目标被检测到
static float minimum_angle = 1.0f;  // 最小角度阈值，低于该值不发送偏移
static float g_send_abs_yaw = 0.0f;
static float g_send_abs_pitch = 0.0f;

// 全局预测结果（获得 result 后写入）
struct ResultWithImu {
  ImageRecognize::PredictResult result;
  SerialTask::EulerAngles imu;
  std::chrono::steady_clock::time_point imu_ts{};  // 匹配到的 IMU 时间戳
  std::chrono::steady_clock::duration delay{};
};

static std::shared_ptr<ResultWithImu> g_last_matched_result;  // 保护最新预测结果的互斥锁

static const std::chrono::milliseconds g_imu_buffer_max_age(1000);

void CaptureThread(CameraTask::GalaxyCamera *camera);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port);

int main() {
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  ImageRecognize::ImagePredict predictor(model_path);

  if (!port.isOpen()) {
    SerialTask::DefaultConfig(port);
    try {
      port.open();
    } catch (const std::exception &e) {
      std::cerr << "Failed to open IMU serial port in main: " << e.what() << std::endl;
      return -1;
    }
  }

  std::thread image_capture(CaptureThread, &camera);
  std::thread image_predict(ImagePredictThread, std::ref(predictor));
  std::thread imu_read(IMUReadThread, std::ref(port));
  std::thread imu_send(IMUSendThread, std::ref(port));

  if (image_capture.joinable()) image_capture.join();
  if (image_predict.joinable()) image_predict.join();
  if (imu_read.joinable()) imu_read.join();
  if (imu_send.joinable()) imu_send.join();

  return 0;
}

void CaptureThread(CameraTask::GalaxyCamera *camera) {
  if (!camera->open()) {
    std::cerr << "Failed to open camera." << std::endl;
    g_running = false;
    return;
  }
  static Tools::AngleCalculator angle_calculator;  // 持久化 AngleCalculator，避免每次调用时重置 lastTime
  static Tools::LaserAngleCalculator laser_angle_calculator;  // 持久化 LaserAngleCalculator，避免每次调用时重置
  while (g_running) {
    cv::Mat frame = camera->grab(1000);
    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 只保留最新一帧（附带时间戳）：直接覆盖并通知预测线程
    {
      std::lock_guard<std::mutex> lk(g_frame_mutex);
      g_latest_frame_item.frame = frame.clone();
      g_latest_frame_item.ts = std::chrono::steady_clock::now();
      g_has_frame = true;
    }
    g_frame_cv.notify_one();
  }

  camera->close();
}

void ImagePredictThread(ImageRecognize::ImagePredict &predictor) {
  FPSCounter fps_counter;
  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;
  while (g_running) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point frame_ts{};
    // 等待最新帧
    {
      std::unique_lock<std::mutex> lk(g_frame_mutex);
      g_frame_cv.wait(lk, [] { return g_has_frame || !g_running; });
      if (!g_running) break;
      if (g_has_frame) {
        frame = g_latest_frame_item.frame.clone();
        frame_ts = g_latest_frame_item.ts;
        // 已取走最新帧，标记为无新帧（下一次仍可被覆盖）
        g_has_frame = false;
      }
    }

    if (frame.empty()) continue;
    ImageRecognize::PredictResult result;
    try {
      result = predictor.run(frame);
      // 关联最近一次 IMU 状态并记录延迟
      ResultWithImu wrapped{result};
      {
        std::lock_guard<std::mutex> lk(g_imu_mutex);
        if (!g_imu_buffer.empty()) {
          // 寻找与 frame_ts 时间差最小的 IMU 条目
          auto best_it = g_imu_buffer.begin();
          auto best_diff = std::chrono::steady_clock::duration::max();
          for (auto it = g_imu_buffer.begin(); it != g_imu_buffer.end(); ++it) {
            auto diff = (it->first > frame_ts) ? (it->first - frame_ts) : (frame_ts - it->first);
            if (diff < best_diff) {
              best_diff = diff;
              best_it = it;
            }
          }

          wrapped.imu = best_it->second;
          wrapped.imu_ts = best_it->first;
          wrapped.delay = frame_ts - best_it->first;

          // 删除缓冲区中时间点 t 及之前的 IMU 条目，避免重复使用已匹配的数据
          g_imu_buffer.erase(g_imu_buffer.begin(), std::next(best_it));
        } else {
          wrapped.delay = std::chrono::steady_clock::duration::zero();
        }
      }
      {
        std::lock_guard<std::mutex> lk(g_result_mutex);
        g_last_matched_result = std::make_shared<ResultWithImu>(std::move(wrapped));
      }
    } catch (const std::exception &e) {
      std::cerr << "ImagePredictThread exception: " << e.what() << std::endl;
    }

    cv::Point2d offset_angles;
    if (!result.boxes.empty()) {
      float center_x = (result.boxes[0][0] + result.boxes[0][2]) / 2.0f;
      float center_y = (result.boxes[0][1] + result.boxes[0][3]) / 2.0f;
      float width = result.boxes[0][2] - result.boxes[0][0];
      float height = result.boxes[0][3] - result.boxes[0][1];
      float distance = Tools::DistanceCalculator().CalculateDistance(width > height ? width : height);

      SerialTask::EulerAngles matched_imu{};
      bool has_matched_imu = false;

      std::lock_guard<std::mutex> lk(g_result_mutex);
      if (g_last_matched_result) {
        matched_imu = g_last_matched_result->imu;
        has_matched_imu = true;
      }

      static Tools::AngleCalculator angle_calculator;  // 持久化 AngleCalculator，避免每次调用时重置 lastTime

      if (has_matched_imu) {
        double dt = 0.033;
        if (has_prev_frame_ts) {
          dt = std::chrono::duration<double>(frame_ts - prev_frame_ts).count();
        }
        prev_frame_ts = frame_ts;
        has_prev_frame_ts = true;
        if (dt <= 0.0 || dt > 0.2) dt = 0.033;

        auto [absolute_yaw, absolute_pitch] =
            angle_calculator.CalculateAbsoluteAngles(center_x, center_y, matched_imu.yaw, matched_imu.pitch, dt);
        offset_angles.x = absolute_yaw - matched_imu.yaw;
        offset_angles.y = absolute_pitch - matched_imu.pitch;

        static Tools::LaserAngleCalculator laser_angle_calculator;

        float laser_angle = laser_angle_calculator.CalculateLaserAngle(width > height ? width : height);

        if (abs(offset_angles.x) > minimum_angle || abs(offset_angles.y) > minimum_angle) {
          has_detection = true;
          g_send_abs_yaw = absolute_yaw - laser_angle;
          g_send_abs_pitch = absolute_pitch;
          std::cout << "Offset angles (deg): Yaw = " << g_send_abs_yaw << ", Pitch = " << g_send_abs_pitch << std::endl;

        } else {
          has_detection = false;
        }
        ImageRecognize::ImageShow::ShowAngles(frame, absolute_yaw, absolute_pitch, matched_imu.yaw, matched_imu.pitch,
                                              offset_angles.x, offset_angles.y, distance);
      }
    }

    // 可视化显示
    fps_counter.tick();
    double fps = fps_counter.get();

    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - frame_ts).count();
    ImageRecognize::ImageShow::ShowNow(frame, result, elapsed_ms, fps);

    // 处理 GUI 事件并允许按键退出
    if (ImageRecognize::ImageShow::WaitForExit()) {
      g_running = false;
      break;
    }
  }
}

void IMUReadThread(serial::Serial &port) {
  // 用于计算即时速率的上一次 IMU
  SerialTask::EulerAngles prev_local_imu{};
  std::chrono::steady_clock::time_point prev_local_ts{};

  while (g_running) {
    SerialTask::EulerAngles angles;
    if (SerialTask::ReadIMUData(port, angles)) {
      auto ts = std::chrono::steady_clock::now();
      prev_local_imu = angles;
      prev_local_ts = ts;

      std::lock_guard<std::mutex> lk(g_imu_mutex);
      // 添加到缓冲区末尾
      g_imu_buffer.emplace_back(ts, angles);

      // 裁剪过旧的 IMU 条目，保持缓冲区只包含最近一段时间的数据
      while (!g_imu_buffer.empty() && (ts - g_imu_buffer.front().first) > g_imu_buffer_max_age) {
        g_imu_buffer.pop_front();
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  if (port.isOpen()) port.close();
}

void IMUSendThread(serial::Serial &port) {
  while (g_running) {
    if (has_detection) {
      SerialTask::SerialSend(port, g_send_abs_pitch, g_send_abs_yaw);
      has_detection = false;
    }
  }
}