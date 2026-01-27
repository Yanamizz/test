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
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <queue>
#include <serial/serial.h>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict.hpp"
#include "ImageRecognize/AngleCalculate.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "CameraTask/Getimage.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "SerialTask/SerialConfig.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/CalculateOffsetAngles.hpp"

// 最小角度阈值，避免发送过小角度
#define minimum_angle 1.0f

const std::string model_path = "/home/hanni/code/rm/src/model/best.onnx";

// 限幅 + 平滑：限制每次最大变化并对发送值做一阶低通，避免突跳震荡
static float last_sent_pitch_deg = 0.0f, last_sent_yaw_deg = 0.0f;
const float max_delta_deg = 5.0f;       // 每次最大变化（度），调整以平衡抖动与响应
const float smoothing_alpha = 0.6f;     // 平滑系数：send = alpha*last + (1-alpha)*desired
const float MIN_PITCH_SEND_DEG = 0.3f;  // 单轴最小触发阈值（度）
const float MIN_YAW_SEND_DEG = 0.3f;    // 单轴最小触发阈值（度）
const size_t kAngleHistory = 7;         // 用于中值滤波的窗口大小（奇数）

// 串口接收线程控制标志与最新角度（供主线程读取）
std::atomic<bool> g_running(true);
SerialTask::EulerAngles g_latest_angles = {0.0f, 0.0f, 0.0f};
std::mutex g_angles_mutex;

// IMU 时间序列缓冲，用于按帧时间对齐 IMU 姿态
struct ImuSample {
  std::chrono::steady_clock::time_point ts;
  Eigen::Quaterniond q;  // 四元数 w,x,y,z
};

std::deque<ImuSample> g_imu_buffer;
std::mutex g_imu_mutex;
const size_t kMaxImuBuf = 500;  // 保留最近若干 IMU 帧，取决于串口频率
// 最近若干解析出的欧拉角，用于中值/去抖
std::deque<SerialTask::EulerAngles> g_imu_angle_history;
std::mutex g_imu_angle_history_mutex;

/**
 * @brief 双帧缓冲：线程安全，仅保留两帧（old, new）。
 * Capture 线程 push 新帧，若满则丢弃最老帧；推理线程每次取出最新帧并清空缓冲。
 */
struct FrameBuffer {
  struct FrameItem {
    std::shared_ptr<cv::Mat> frame;
    std::chrono::steady_clock::time_point ts;
  };

  std::deque<FrameItem> frames;  // 最多保持 2 帧
  std::mutex mutex;
  std::condition_variable cv;

  void push_frame(const std::shared_ptr<cv::Mat> &f, const std::chrono::steady_clock::time_point &ts) {
    std::lock_guard<std::mutex> lock(mutex);
    if (frames.size() >= 2) frames.pop_front();
    frames.push_back({f, ts});
    cv.notify_one();
  }

  // 等待并返回最新一帧，返回时清空缓冲，保证主循环总是处理最新帧
  FrameItem pop_latest() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&]() { return !frames.empty() || !g_running; });
    if (frames.empty()) {
      return FrameItem{nullptr, std::chrono::steady_clock::time_point{}};
    }
    auto latest = frames.back();
    frames.clear();
    return latest;
  }
};

void ReceiveThread(serial::Serial *serial_port);
void CaptureThread(CameraTask::GalaxyCamera *camera, FrameBuffer *frame_buffer);
void InferenceThread(ImagePredict::ImagePredict *predictor, Tracker2D *tracker, FrameBuffer *frame_buffer,
                     serial::Serial *serial_port, std::atomic<bool> *serial_ready, FPSCounter *fps_counter);

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
    Tracker2D tracker;
    FrameBuffer frame_buffer;
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
    std::thread inference_thread(InferenceThread, &predictor, &tracker, &frame_buffer, &serial_port, &serial_ready,
                                 &fps_counter);

    inference_thread.join();
    g_running = false;
    frame_buffer.cv.notify_all();

    if (capture_thread.joinable()) {
      capture_thread.join();
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
// 解析并保存 IMU 数据（包括四元数），供按时间对齐查询使用
void ReceiveThread(serial::Serial *serial_port) {
  while (g_running) {
    if (!serial_port->isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    GimbalImuFrame_SCM_t latest_frame{};
    bool ok = false;
    try {
      ok = SerialTask::ReadIMUFrame(*serial_port, latest_frame);
    } catch (const std::exception &e) {
      std::cerr << "Serial read exception: " << e.what() << std::endl;
      ok = false;
    }

    if (ok) {
      // 读取四元数并记录时间戳（四元数缓冲用于按帧时间插值）
      ImuSample imu_sample;
      imu_sample.ts = std::chrono::steady_clock::now();
      imu_sample.q = Eigen::Quaterniond(static_cast<double>(latest_frame.q0), static_cast<double>(latest_frame.q1),
                                        static_cast<double>(latest_frame.q2), static_cast<double>(latest_frame.q3));
      {
        std::lock_guard<std::mutex> lk(g_imu_mutex);
        g_imu_buffer.push_back(imu_sample);
        while (g_imu_buffer.size() > kMaxImuBuf) g_imu_buffer.pop_front();
      }

      // 同时维持向后兼容的欧拉角缓存（度），使用头文件提供的转换逻辑
      SerialTask::EulerAngles angles;
      Eigen::Quaterniond imu_quaternion_raw(static_cast<double>(latest_frame.q0), static_cast<double>(latest_frame.q1),
                                            static_cast<double>(latest_frame.q2), static_cast<double>(latest_frame.q3));
      SerialTask::QuaternionToEuler(imu_quaternion_raw, angles);

      {
        std::lock_guard<std::mutex> lk(g_imu_angle_history_mutex);
        g_imu_angle_history.push_back(angles);
        if (g_imu_angle_history.size() > kAngleHistory) g_imu_angle_history.pop_front();
      }

      SerialTask::EulerAngles filtered = angles;
      {
        std::lock_guard<std::mutex> lk(g_imu_angle_history_mutex);
        std::vector<double> pitch_history_values, yaw_history_values;
        pitch_history_values.reserve(g_imu_angle_history.size());
        yaw_history_values.reserve(g_imu_angle_history.size());
        for (auto &a : g_imu_angle_history) {
          pitch_history_values.push_back(a.pitch);
          yaw_history_values.push_back(a.yaw);
        }
        auto median_of = [](std::vector<double> &v) {
          if (v.empty()) return 0.0;
          std::sort(v.begin(), v.end());
          return v[v.size() / 2];
        };
        filtered.pitch = static_cast<float>(median_of(pitch_history_values));
        filtered.yaw = static_cast<float>(median_of(yaw_history_values));
      }
      {
        std::lock_guard<std::mutex> lock(g_angles_mutex);
        g_latest_angles = filtered;
      }
      std::cout << "\n收到角度 原始(Pitch,Yn)=" << angles.pitch << "," << angles.yaw
                << " 过滤后(Pitch,Yn)=" << filtered.pitch << "," << filtered.yaw << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
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
    auto ts = std::chrono::steady_clock::now();
    buffer->push_frame(shared_frame, ts);
  }
}
// the sequence: get frame -> inference -> get IMU -> calc offset -> send.

/**
 * @brief 推理线程：从帧缓存取帧、跑模型、更新预测、发送结果。
 * @param predictor 模型推理器。
 * @param ekf 卡尔曼滤波器实例。
 * @param buffer 图像帧缓存。
 * @param queue 串口命令队列。
 * @param fps_counter FPS 统计器。
 */
void InferenceThread(ImagePredict::ImagePredict *predictor, Tracker2D *tracker, FrameBuffer *buffer,
                     serial::Serial *serial_port, std::atomic<bool> *serial_ready, FPSCounter *fps_counter) {
  OffsetAngles offset_calculator;
  Eigen::Vector2d last_predicted_center(0, 0);
  float last_width = 0, last_height = 0;
  bool detection_present = false;
  while (g_running) {
    auto item = buffer->pop_latest();
    if (!item.frame) {
      if (!g_running) break;
      continue;
    }
    auto frame_ptr = item.frame;
    auto frame_ts = item.ts;

    auto time_start = std::chrono::high_resolution_clock::now();
    auto result = predictor->run(*frame_ptr);

    cv::Rect bound_boxes;
    bool detection_now = !result.boxes.empty();
    if (detection_now) {
      bound_boxes = cv::Rect(static_cast<int>(result.boxes[0][0]), static_cast<int>(result.boxes[0][1]),
                             static_cast<int>(result.boxes[0][2] - result.boxes[0][0]),
                             static_cast<int>(result.boxes[0][3] - result.boxes[0][1]));
      last_predicted_center = tracker->update(bound_boxes);
      last_width = bound_boxes.width;
      last_height = bound_boxes.height;
      detection_present = true;
    } else {
      detection_present = false;
      last_predicted_center = Eigen::Vector2d(0, 0);
      last_width = 0;
      last_height = 0;
    }

    GimbalAngles angles =
        offset_calculator.CalculateOffsetAngles(last_predicted_center, last_width, last_height, detection_present);
    if (detection_present) {
      // 获取与帧时间对齐的 IMU 姿态（插值或小幅外推）
      Eigen::Quaterniond imu_quaternion_at_frame = Eigen::Quaterniond::Identity();
      {
        std::lock_guard<std::mutex> lk(g_imu_mutex);
        // 查找区间
        if (g_imu_buffer.empty()) {
        } else if (frame_ts <= g_imu_buffer.front().ts) {
          imu_quaternion_at_frame = g_imu_buffer.front().q;
        } else if (frame_ts >= g_imu_buffer.back().ts) {
          // 小幅外推：用最近两个样本估计旋转速率
          if (g_imu_buffer.size() >= 2) {
            const ImuSample &sample_a = g_imu_buffer[g_imu_buffer.size() - 2];
            const ImuSample &sample_b = g_imu_buffer.back();
            double dt = std::chrono::duration<double>(sample_b.ts - sample_a.ts).count();
            if (dt <= 1e-6) {
              imu_quaternion_at_frame = sample_b.q;
            } else {
              Eigen::Quaterniond delta_quaternion = sample_b.q * sample_a.q.conjugate();
              // 轴角
              Eigen::AngleAxisd angle_axis(delta_quaternion);
              double angle = angle_axis.angle();
              Eigen::Vector3d axis = angle_axis.axis();
              double dt_ext = std::chrono::duration<double>(frame_ts - sample_b.ts).count();
              if (std::abs(dt_ext) > 0.05) {
                imu_quaternion_at_frame = sample_b.q;
              } else {
                double angle_ext = (angle / dt) * dt_ext;
                Eigen::Quaterniond dq_ext(Eigen::AngleAxisd(angle_ext, axis));
                imu_quaternion_at_frame = sample_b.q * dq_ext;
              }
            }
          } else {
            imu_quaternion_at_frame = g_imu_buffer.back().q;
          }
        } else {
          // 插值
          for (size_t i = 1; i < g_imu_buffer.size(); ++i) {
            if (g_imu_buffer[i].ts >= frame_ts) {
              const ImuSample &sample_a = g_imu_buffer[i - 1];
              const ImuSample &sample_b = g_imu_buffer[i];
              double interp_alpha =
                  double((frame_ts - sample_a.ts).count()) / double((sample_b.ts - sample_a.ts).count());
              imu_quaternion_at_frame = sample_a.q.slerp(interp_alpha, sample_b.q);
              break;
            }
          }
        }
      }

      // 转换为欧拉角并发送
      if (serial_ready->load()) {
        // 将 imu_quaternion_at_frame 转为 EulerAngles（度）
        SerialTask::EulerAngles imu_angles_at_frame;
        SerialTask::QuaternionToEuler(imu_quaternion_at_frame, imu_angles_at_frame);

        float desired_pitch = static_cast<float>(imu_angles_at_frame.pitch + (angles.pitch));
        float desired_yaw = static_cast<float>(imu_angles_at_frame.yaw + (angles.yaw));

        // 归一化差值到 [-180,180]
        auto normalize_angle_delta = [](float d) {
          while (d > 180.0f) d -= 360.0f;
          while (d < -180.0f) d += 360.0f;
          return d;
        };

        float delta_pitch = normalize_angle_delta(desired_pitch - last_sent_pitch_deg);
        if (delta_pitch > max_delta_deg) delta_pitch = max_delta_deg;
        if (delta_pitch < -max_delta_deg) delta_pitch = -max_delta_deg;
        float delta_yaw = normalize_angle_delta(desired_yaw - last_sent_yaw_deg);
        if (delta_yaw > max_delta_deg) delta_yaw = max_delta_deg;
        if (delta_yaw < -max_delta_deg) delta_yaw = -max_delta_deg;

        float send_pitch_candidate = last_sent_pitch_deg + delta_pitch;
        float send_yaw_candidate = last_sent_yaw_deg + delta_yaw;

        // 指数平滑（度）
        float send_pitch = smoothing_alpha * last_sent_pitch_deg + (1.0f - smoothing_alpha) * send_pitch_candidate;
        float send_yaw = smoothing_alpha * last_sent_yaw_deg + (1.0f - smoothing_alpha) * send_yaw_candidate;

        // 计算相对偏移（度）
        float pitch_offset_to_send = send_pitch - static_cast<float>(imu_angles_at_frame.pitch);
        float yaw_offset_to_send = send_yaw - static_cast<float>(imu_angles_at_frame.yaw);

        // 详细发送前日志：时间、IMU窗口、imu_at_frame、期望与偏移
        {
          std::lock_guard<std::mutex> lk(g_imu_mutex);
          auto now = std::chrono::steady_clock::now();
          auto front_ts = g_imu_buffer.empty() ? std::chrono::steady_clock::time_point{} : g_imu_buffer.front().ts;
          auto back_ts = g_imu_buffer.empty() ? std::chrono::steady_clock::time_point{} : g_imu_buffer.back().ts;
          auto to_ms = [](std::chrono::steady_clock::time_point t) -> long long {
            return std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
          };
          std::cout << "\n[DEBUG SEND] frame_ts=" << to_ms(frame_ts) << " now=" << to_ms(now)
                    << " imu_front=" << (front_ts == std::chrono::steady_clock::time_point{} ? 0 : to_ms(front_ts))
                    << " imu_back=" << (back_ts == std::chrono::steady_clock::time_point{} ? 0 : to_ms(back_ts))
                    << " imu_at_frame(p,y)=" << imu_angles_at_frame.pitch << "," << imu_angles_at_frame.yaw
                    << " desired(p,y)=" << desired_pitch << "," << desired_yaw << " send(p,y)=" << send_pitch << ","
                    << send_yaw << " delta(p,y)=" << pitch_offset_to_send << "," << yaw_offset_to_send << std::endl;
        }

        // 根据单轴阈值决定是否发送
        if (std::abs(pitch_offset_to_send) > MIN_PITCH_SEND_DEG || std::abs(yaw_offset_to_send) > MIN_YAW_SEND_DEG) {
          try {
            SerialTask::SerialSend(*serial_port, imu_angles_at_frame, pitch_offset_to_send, yaw_offset_to_send);
            last_sent_pitch_deg = send_pitch;
            last_sent_yaw_deg = send_yaw;
          } catch (const std::exception &e) {
            std::cerr << "串口发送失败: " << e.what() << std::endl;
          }
        } else {
          // 不发送，仅输出说明
          std::cout << "[DEBUG SEND] skip send: below per-axis thresholds" << std::endl;
        }
      }
    }

    fps_counter->tick();
    double fps = fps_counter->get();
    auto time_end = std::chrono::high_resolution_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    std::cout << "\rfps:" << fps << ", ms:" << elapsed_ms << ", pitch:" << angles.pitch << ", yaw:" << angles.yaw
              << std::flush;
    ImageShow::ShowNow(*frame_ptr, result, elapsed_ms, fps);
    if (detection_present) {
      ImageShow::ShowPredict(*frame_ptr, last_predicted_center, last_width, last_height);
      ImageShow::ShowAngles(*frame_ptr, static_cast<float>(angles.pitch), static_cast<float>(angles.yaw));
    }
    if (ImageShow::WaitForExit()) {
      g_running = false;
      buffer->cv.notify_all();
      break;
    }
  }
}
