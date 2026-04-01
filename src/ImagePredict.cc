#include <iostream>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <deque>
#include <iterator>
#include <atomic>

#include "ImageRecognize/ImageShow.hpp"
// #include "ImageRecognize/ImagePredict_ONNX.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"  // 切换到 OpenVINO 时，注释上一行并启用这一行
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
// #include "Tools/SaveImage.hpp"
#include "CameraTask/GetImage.hpp"

std::string model_path = "/home/nuc/Downloads/rm/src/model/best_v8s.xml";
std::atomic<bool> g_running(true);          // 全局运行标志
static std::mutex g_frame_mutex;            // 保护最新帧的互斥锁
static std::condition_variable g_frame_cv;  // 通知预测线程有新帧到达的条件变量
                                            // IMU 数据缓冲区
static std::deque<std::pair<std::chrono::steady_clock::time_point, SerialTask::EulerAngles>> g_imu_buffer;
static std::mutex g_imu_mutex;                       // 保护 IMU 缓冲区的互斥锁
static std::atomic<float> g_current_yaw_rate{0.0f};  // 即时角速度（由 IMUReadThread 计算并更新）——单位 deg/sgre
static std::atomic<float> g_current_pitch_rate{0.0f};  // 即时角速度（由 IMUReadThread 计算并更新）——单位 deg/s
static std::atomic<float> g_current_imu_yaw{0.0f};    // 最新 IMU 绝对 Yaw（由 IMUReadThread 更新）
static std::atomic<float> g_current_imu_pitch{0.0f};  // 最新 IMU 绝对 Pitch（由 IMUReadThread 更新）

// 全局：双buffer避免重复clone
struct FrameItem {
  cv::Mat frame;
  std::chrono::steady_clock::time_point ts{};
};

static FrameItem g_frame_buffers[2];     // 双buffer
static std::atomic<int> g_write_idx{0};  // 当前写入buffer索引
static std::atomic<int> g_read_idx{-1};  // 当前可读buffer索引，-1表示无新帧

static std::atomic<bool> has_detection{false};  // 是否有目标被检测到
static float minimum_angle = 0.008f;            // 最小角度阈值，低于该值不发送偏移
static std::atomic<float> g_send_abs_yaw{0.0f};
static std::atomic<float> g_send_abs_pitch{0.0f};
static std::atomic<float> g_send_offset_yaw{0.0f};
static std::atomic<float> g_send_offset_pitch{0.0f};
static constexpr float g_max_send_delta = 5.0f;        // 每次相对当前IMU允许的最大角差
static const std::string g_angle_filter_type = "UKF";  // 可选: KF / EKF / UKF / CKF

static inline float NormalizeDeltaDeg(float delta) {
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

static const std::chrono::milliseconds g_imu_buffer_max_age(1000);

void CaptureThread(CameraTask::GalaxyCamera *camera);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port);

int main() {
  Tools::BindCurrentThreadToBigCores();
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  std::unique_ptr<ImageRecognize::ImagePredict> predictor;
  try {
    predictor = std::make_unique<ImageRecognize::ImagePredict>(model_path);
  } catch (const std::exception &e) {
    std::cerr << "Failed to initialize OpenVINO model: " << e.what() << std::endl;
    std::cerr << "Configured model_path: " << model_path << std::endl;
    return -2;
  }

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
  std::thread image_predict(ImagePredictThread, std::ref(*predictor));
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
  while (g_running) {
    cv::Mat frame = camera->grab(1000);
    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 双buffer写入：无锁切换，避免clone
    int write_idx = g_write_idx.load(std::memory_order_relaxed);
    g_frame_buffers[write_idx].frame = std::move(frame);  // 移动语义，避免拷贝
    g_frame_buffers[write_idx].ts = std::chrono::steady_clock::now();

    // 原子切换读写索引
    int next_write = 1 - write_idx;
    g_write_idx.store(next_write, std::memory_order_release);
    g_read_idx.store(write_idx, std::memory_order_release);
    g_frame_cv.notify_one();
  }

  camera->close();
}

void ImagePredictThread(ImageRecognize::ImagePredict &predictor) {
  FPSCounter fps_counter;
  static Tools::AngleCalculator angle_calculator;  // 持久化 AngleCalculator，避免每次调用时重置 lastTime
  static Tools::LaserAngleCalculator laser_angle_calculator;
  static Tools::DistanceCalculator distance_calculator;

  const bool filter_ok = angle_calculator.SetFilterTypeFromString(g_angle_filter_type);
  if (filter_ok) {
    std::cout << "[AngleFilter] using type: " << Tools::ToString(angle_calculator.GetFilterType()) << std::endl;
  } else {
    std::cerr << "[AngleFilter] invalid type: " << g_angle_filter_type << ", fallback to "
              << Tools::ToString(angle_calculator.GetFilterType()) << std::endl;
  }

  // static Tools::SaveImageOnNoTarget no_target_saver(5, "captures");
  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;

  // 首次捕获目标时做软启动，抑制远距离目标首次追踪的大过冲。
  bool target_locked_last_frame = false;
  int lock_frame_count = 0;
  float last_cmd_delta_yaw = 0.0f;
  float last_cmd_delta_pitch = 0.0f;

  while (g_running) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point frame_ts{};
    // 等待最新帧（双buffer无锁读取）
    {
      std::unique_lock<std::mutex> lk(g_frame_mutex);
      g_frame_cv.wait(lk, [] { return g_read_idx.load(std::memory_order_acquire) >= 0 || !g_running; });
      if (!g_running) break;

      int read_idx = g_read_idx.load(std::memory_order_acquire);
      if (read_idx >= 0) {
        frame = g_frame_buffers[read_idx].frame;  // 直接引用，无clone
        frame_ts = g_frame_buffers[read_idx].ts;
      }
    }

    if (frame.empty()) continue;
    ImageRecognize::PredictResult result;
    SerialTask::EulerAngles matched_imu{};
    bool has_matched_imu = false;
    try {
      result = predictor.run(frame);

      // 关联最近一次 IMU 状态并记录延迟
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

          matched_imu = best_it->second;
          has_matched_imu = true;

          // 删除缓冲区中时间点 t 及之前的 IMU 条目，避免重复使用已匹配的数据
          g_imu_buffer.erase(g_imu_buffer.begin(), std::next(best_it));
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "ImagePredictThread exception: " << e.what() << std::endl;
    }

    cv::Point2d offset_angles;
    [[maybe_unused]] const bool detected_target = !result.boxes.empty();
    if (!result.boxes.empty()) {
      float center_x = (result.boxes[0][0] + result.boxes[0][2]) / 2.0f;
      float center_y = (result.boxes[0][1] + result.boxes[0][3]) / 2.0f;
      float width = result.boxes[0][2] - result.boxes[0][0];
      float height = result.boxes[0][3] - result.boxes[0][1];
      float distance = distance_calculator.CalculateDistance(height, width);

      if (has_matched_imu) {
        double dt = 0.05;
        if (has_prev_frame_ts) {
          dt = std::chrono::duration<double>(frame_ts - prev_frame_ts).count();
        }
        prev_frame_ts = frame_ts;
        has_prev_frame_ts = true;
        if (dt <= 0.0 || dt > 0.2) dt = 0.05;

        auto [absolute_yaw, absolute_pitch] =
            angle_calculator.CalculateAbsoluteAngles(center_x, center_y, matched_imu.yaw, matched_imu.pitch, dt);
        // 必须用“最短角差”，否则跨越 ±180° 时会出现 300° 级突变
        offset_angles.x = NormalizeDeltaDeg(absolute_yaw - matched_imu.yaw);
        offset_angles.y = NormalizeDeltaDeg(absolute_pitch - matched_imu.pitch);

        auto [laser_yaw_angle, laser_pitch_angle] =
            laser_angle_calculator.CalculateLaserAngles(distance, offset_angles.x);

        float delta_yaw_raw = NormalizeDeltaDeg(static_cast<float>(offset_angles.x + laser_yaw_angle));
        float delta_pitch_raw = NormalizeDeltaDeg(static_cast<float>(offset_angles.y + laser_pitch_angle));
        if (abs(delta_pitch_raw) > minimum_angle || abs(delta_yaw_raw) > minimum_angle) {
          // 直接基于“当前IMU到目标的角差”叠加激光补偿，再限幅后转回绝对角。

          // 新锁定目标时渐进放开限幅，避免第一拍打满导致过冲。
          if (!target_locked_last_frame) {
            lock_frame_count = 0;
            last_cmd_delta_yaw = 0.0f;
            last_cmd_delta_pitch = 0.0f;
          }
          target_locked_last_frame = true;
          lock_frame_count = std::min(lock_frame_count + 1, 30);

          const float dynamic_limit = std::min(g_max_send_delta, 1.2f + 0.45f * static_cast<float>(lock_frame_count));
          float delta_yaw = std::clamp(delta_yaw_raw, -dynamic_limit, dynamic_limit);
          float delta_pitch = std::clamp(delta_pitch_raw, -dynamic_limit, dynamic_limit);

          // 进一步限制单帧命令变化量，抑制远离中心时首次追踪抖动。
          const float max_step = (lock_frame_count < 10) ? 0.5f : 0.9f;
          delta_yaw = std::clamp(delta_yaw, last_cmd_delta_yaw - max_step, last_cmd_delta_yaw + max_step);
          delta_pitch = std::clamp(delta_pitch, last_cmd_delta_pitch - max_step, last_cmd_delta_pitch + max_step);

          // 全局安全限幅
          delta_yaw = std::clamp(delta_yaw, -g_max_send_delta, g_max_send_delta);
          delta_pitch = std::clamp(delta_pitch, -g_max_send_delta, g_max_send_delta);

          const float cmd_delta_yaw = delta_yaw;
          const float cmd_delta_pitch = delta_pitch;

          const float send_abs_yaw = matched_imu.yaw + cmd_delta_yaw;
          const float send_abs_pitch = matched_imu.pitch + cmd_delta_pitch;

          last_cmd_delta_yaw = delta_yaw;
          last_cmd_delta_pitch = delta_pitch;

          g_send_abs_yaw.store(send_abs_yaw, std::memory_order_release);
          g_send_abs_pitch.store(send_abs_pitch, std::memory_order_release);
          g_send_offset_yaw.store(cmd_delta_yaw, std::memory_order_release);
          g_send_offset_pitch.store(cmd_delta_pitch, std::memory_order_release);

          has_detection.store(true, std::memory_order_release);
          ImageRecognize::ImageShow::ShowAngles(frame, send_abs_yaw, send_abs_pitch, matched_imu.yaw, matched_imu.pitch,
                                                cmd_delta_yaw, cmd_delta_pitch, distance);
        } else {
          has_detection.store(false, std::memory_order_release);
          target_locked_last_frame = false;
          lock_frame_count = 0;
        }
      } else {
        has_detection.store(false, std::memory_order_release);
        target_locked_last_frame = false;
        lock_frame_count = 0;
      }
    } else {
      has_detection.store(false, std::memory_order_release);
      target_locked_last_frame = false;
      lock_frame_count = 0;
    }

    // 可视化显示
    fps_counter.tick();
    double fps = fps_counter.get();

    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - frame_ts).count();
    ImageRecognize::ImageShow::ShowNow(frame, result, elapsed_ms, fps);

    // 无目标时，每隔若干帧保存图像到本次运行目录
    // no_target_saver.Update(frame, detected_target);

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

      if (prev_local_ts.time_since_epoch().count() != 0) {
        const double dt = std::chrono::duration<double>(ts - prev_local_ts).count();
        if (dt > 1e-4 && dt < 0.2) {
          const float yaw_rate = NormalizeDeltaDeg(angles.yaw - prev_local_imu.yaw) / static_cast<float>(dt);
          const float pitch_rate = NormalizeDeltaDeg(angles.pitch - prev_local_imu.pitch) / static_cast<float>(dt);
          g_current_yaw_rate.store(std::clamp(yaw_rate, -720.0f, 720.0f), std::memory_order_release);
          g_current_pitch_rate.store(std::clamp(pitch_rate, -720.0f, 720.0f), std::memory_order_release);
        }
      }

      prev_local_imu = angles;
      prev_local_ts = ts;
      g_current_imu_yaw.store(angles.yaw, std::memory_order_release);
      g_current_imu_pitch.store(angles.pitch, std::memory_order_release);

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
    if (has_detection.load(std::memory_order_acquire)) {
      float pitch = g_send_abs_pitch.load(std::memory_order_acquire);
      float yaw = g_send_abs_yaw.load(std::memory_order_acquire);
      float offset_pitch = g_send_offset_pitch.load(std::memory_order_acquire);
      float offset_yaw = g_send_offset_yaw.load(std::memory_order_acquire);
      std::cout << std::fixed << std::setprecision(2) << "°, Offset Yaw: " << offset_yaw
                << "°, Offset Pitch: " << offset_pitch << "°" << std::endl;
      if (abs(offset_yaw) > 0.6f)
        if (offset_yaw > 0)
          yaw += 3.0f * (1.0f - minimum_angle / abs(offset_yaw));  // 根据偏移量大小动态调整补偿力度，越接近中心越温和
        else
          yaw -= 3.0f * (1.0f - minimum_angle / abs(offset_yaw));

      SerialTask::SerialSend(port, pitch, yaw, 0x01);
      has_detection.store(false, std::memory_order_release);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}