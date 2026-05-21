#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <vector>

#include "CameraTask/CameraTask.hpp"
#include "ImageRecognize/ImagePredictCommandLine.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/TargetTracking.hpp"
#include "NetworkTask/NetworkTask.hpp"
#include "SerialTask/SerialTask.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/CalibrationSliderPanel.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/RuntimeParams.hpp"
#include "Tools/RuntimeStats.hpp"
#include "Tools/SaveImage.hpp"
#include "Tools/SaveVideo.hpp"
#include "Tools/ScanController.hpp"
#include "Tools/VideoInput.hpp"

namespace {
using ImageRecognize::ImagePredictCommandLineOptions;
using Tools::LatencyStats;
using Tools::PixelHeightStats;
using Tools::PrintLatencyStats;
using Tools::PrintPixelHeightStats;

struct AimbotSendCommand {
  float absolute_pitch = 0.0f;
  float absolute_yaw = 0.0f;
  float offset_pitch = 0.0f;
  float offset_yaw = 0.0f;
  float pitch_velocity = 0.0f;
  float yaw_velocity = 0.0f;
  uint8_t aimbot_state = 0x00;
  std::chrono::steady_clock::time_point source_frame_ts{};
  std::chrono::steady_clock::time_point enqueue_ts{};
};
} // namespace

std::atomic<bool> g_running(true); // 全局运行标志
static std::mutex g_frame_mutex;   // 保护最新帧的互斥锁
static std::condition_variable g_frame_cv; // 通知预测线程有新帧到达的条件变量
static SerialTask::ImuBuffer g_imu_buffer; // IMU 数据缓冲区

// 全局：双buffer避免重复clone
struct FrameItem {
  cv::Mat frame;
  std::chrono::steady_clock::time_point ts{};
};

static FrameItem g_frame_buffers[2];    // 双buffer
static std::atomic<int> g_write_idx{0}; // 当前写入buffer索引
static std::atomic<int> g_read_idx{-1}; // 当前可读buffer索引，-1表示无新帧

// 线程间共享的控制输出；图像线程写入，串口线程读取并发送。
static std::mutex g_pending_send_mutex;
static std::condition_variable g_pending_send_cv;
static AimbotSendCommand g_pending_send;
static bool g_has_pending_send = false;
static std::atomic<uint8_t> g_aimbot_target{SerialTask::kAimbotTargetMin};
static std::atomic<bool> g_send_is_scan{false};
static std::mutex g_scan_controller_mutex;
static std::atomic<bool> g_target_visible{false};
static std::atomic<int> g_aerial_robot_stage{
    ImageRecognize::AerialRobotLaserLockJudge::kInitialStage};
static CameraTask::ExposureHotkeyController g_exposure_controller;
static std::mutex g_serial_mutex;
static std::mutex g_send_latency_mutex;
static Tools::LatencyStats g_send_latency_total;
static Tools::LatencyStats g_send_latency_window;

namespace {
static void RequestStop() {
  g_running.store(false, std::memory_order_release);
  g_frame_cv.notify_all();
  g_pending_send_cv.notify_all();
}

static void ClearPendingSend() {
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    g_send_is_scan.store(false, std::memory_order_release);
    g_has_pending_send = false;
  }
  g_pending_send_cv.notify_one();
}

static void StorePendingSend(const AimbotSendCommand &command) {
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    g_pending_send = command;
    g_send_is_scan.store(false, std::memory_order_release);
    g_has_pending_send = true;
  }
  g_pending_send_cv.notify_one();
}

static bool TakePendingSend(AimbotSendCommand *command) {
  std::lock_guard<std::mutex> lk(g_pending_send_mutex);
  if (!g_has_pending_send) {
    return false;
  }

  *command = g_pending_send;
  g_has_pending_send = false;
  return true;
}

static void StartScanMode() {
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    g_has_pending_send = false;
    g_send_is_scan.store(true, std::memory_order_release);
  }
  g_pending_send_cv.notify_one();
}

static void DecrementAimbotTargetSaturated() {
  uint8_t old_value = g_aimbot_target.load(std::memory_order_acquire);
  while (old_value > SerialTask::kAimbotTargetMin) {
    if (g_aimbot_target.compare_exchange_weak(
            old_value, static_cast<uint8_t>(old_value - 1),
            std::memory_order_acq_rel, std::memory_order_acquire)) {
      return;
    }
  }
}

static double MaxSerialAimbotFrameHz() {
  constexpr double kBitsPerByteOnSerial = 10.0;
  constexpr double kSafetyRatio = 0.85;
  return SerialTask::DEFAULT_BAUD_RATE * kSafetyRatio /
         (kBitsPerByteOnSerial * sizeof(AimbotFrame_SCM_t));
}

static double EffectiveScanSendHz() {
  return std::clamp(Tools::Params().scan_send_hz, 1.0,
                    MaxSerialAimbotFrameHz());
}

static std::chrono::milliseconds SerialReconnectInterval() {
  return std::chrono::milliseconds(1000);
}

static void WaitForNormalSendWork() {
  std::unique_lock<std::mutex> lk(g_pending_send_mutex);
  g_pending_send_cv.wait_for(lk, SerialReconnectInterval(), []() {
    return !g_running.load(std::memory_order_acquire) || g_has_pending_send ||
           g_send_is_scan.load(std::memory_order_acquire);
  });
}

static void WaitForScanStateChangeFor(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lk(g_pending_send_mutex);
  g_pending_send_cv.wait_for(lk, timeout, []() {
    return !g_running.load(std::memory_order_acquire) ||
           !g_send_is_scan.load(std::memory_order_acquire) ||
           g_has_pending_send ||
           g_target_visible.load(std::memory_order_acquire);
  });
}

static void WaitUntilNextScanSend(std::chrono::steady_clock::time_point time) {
  std::unique_lock<std::mutex> lk(g_pending_send_mutex);
  g_pending_send_cv.wait_until(lk, time, []() {
    return !g_running.load(std::memory_order_acquire) ||
           !g_send_is_scan.load(std::memory_order_acquire) ||
           g_has_pending_send ||
           g_target_visible.load(std::memory_order_acquire);
  });
}

static void SendAimbotCommand(serial::Serial &port,
                              const AimbotSendCommand &command) {
  SerialTask::SerialSend(
      port, command.absolute_pitch, command.absolute_yaw, command.offset_pitch,
      command.offset_yaw, command.pitch_velocity, command.yaw_velocity,
      command.aimbot_state, g_aimbot_target.load(std::memory_order_acquire));
}

static void CloseSerialPort(serial::Serial &port) {
  std::lock_guard<std::mutex> lk(g_serial_mutex);
  if (port.isOpen()) {
    port.close();
  }
}

static void HandleSerialWriteFailure(serial::Serial &port,
                                     const std::exception &e) {
  std::cerr << "警告：IMU 串口发送失败，停止发送并等待重连：" << e.what()
            << std::endl;
  CloseSerialPort(port);
  ClearPendingSend();
}

static bool TryReopenSerialPort(serial::Serial &port) {
  std::lock_guard<std::mutex> lk(g_serial_mutex);
  if (port.isOpen()) {
    return true;
  }

  try {
    SerialTask::DefaultConfig(port);
    port.open();
    std::cerr << "信息：IMU 串口重连成功。" << std::endl;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "警告：IMU 串口重连失败：" << e.what() << std::endl;
    return false;
  }
}

static bool OpenSerialPort(serial::Serial &port) {
  std::lock_guard<std::mutex> lk(g_serial_mutex);
  if (port.isOpen()) {
    return true;
  }

  SerialTask::DefaultConfig(port);
  try {
    port.open();
    return true;
  } catch (const std::exception &e) {
    std::cerr << "警告：打开 IMU 串口失败，将在无串口模式下继续运行："
              << e.what() << std::endl;
    return false;
  }
}

static void JoinIfNeeded(std::thread &thread) {
  if (thread.joinable()) {
    thread.join();
  }
}

static void PrintPredictSettings(
    Tools::FilterType filter_type,
    ImageRecognize::TargetCampMode target_camp_mode, bool enable_display,
    bool enable_scan_mode, bool enable_save_no_target_images,
    bool /*enable_save_target_videos*/, bool enable_latency_profile,
    bool enable_calibration_sliders, bool enable_send_log) {
  std::cout << "[角度滤波] 类型: " << Tools::ToString(filter_type) << std::endl;
  std::cout << "[跟踪阵营] 模式: " << ImageRecognize::ToString(target_camp_mode)
            << std::endl;
  std::cout << "[显示窗口] 启用: " << (enable_display ? "true" : "false")
            << std::endl;
  std::cout << "[标定滑块] 启用: "
            << (enable_calibration_sliders ? "true" : "false") << std::endl;
  std::cout << "[扫描模式] 启用: " << (enable_scan_mode ? "true" : "false")
            << std::endl;
  const double effective_scan_send_hz = EffectiveScanSendHz();
  std::cout << "[扫描模式] 发送频率: " << effective_scan_send_hz;
  if (effective_scan_send_hz < Tools::Params().scan_send_hz) {
    std::cout << "（配置值 " << Tools::Params().scan_send_hz
              << "，已受串口带宽限制）";
  }
  std::cout << " Hz" << std::endl;
  std::cout << "[异常图片保存] 启用: "
            << (enable_save_no_target_images ? "true" : "false") << std::endl;
  std::cout << "[延迟统计] 启用: "
            << (enable_latency_profile ? "true" : "false") << std::endl;
  std::cout << "[发送日志] 启用: " << (enable_send_log ? "true" : "false")
            << std::endl;
}

static bool ResolveOption(const std::optional<bool> &option, bool fallback) {
  return option.value_or(fallback);
}

static std::chrono::steady_clock::time_point ProfileNow(bool enabled) {
  return enabled ? std::chrono::steady_clock::now()
                 : std::chrono::steady_clock::time_point{};
}

static void AddLatencySample(bool enabled, LatencyStats &total,
                             LatencyStats &window,
                             Tools::LatencyBucket LatencyStats::*bucket,
                             const std::chrono::steady_clock::time_point &t0,
                             const std::chrono::steady_clock::time_point &t1) {
  if (!enabled) {
    return;
  }

  total.Add(total.*bucket, t0, t1);
  window.Add(window.*bucket, t0, t1);
}

static void AddLatencyFrame(bool enabled, LatencyStats &total,
                            LatencyStats &window) {
  if (!enabled) {
    return;
  }

  total.AddFrame();
  window.AddFrame();
}

static void AddSendLatencySample(
    const AimbotSendCommand &command,
    const std::chrono::steady_clock::time_point &serial_send_time) {
  std::lock_guard<std::mutex> lk(g_send_latency_mutex);
  g_send_latency_total.Add(g_send_latency_total.queue_to_serial_ns,
                           command.enqueue_ts, serial_send_time);
  g_send_latency_window.Add(g_send_latency_window.queue_to_serial_ns,
                            command.enqueue_ts, serial_send_time);
  g_send_latency_total.Add(g_send_latency_total.capture_to_serial_ns,
                           command.source_frame_ts, serial_send_time);
  g_send_latency_window.Add(g_send_latency_window.capture_to_serial_ns,
                            command.source_frame_ts, serial_send_time);
  g_send_latency_total.AddFrame();
  g_send_latency_window.AddFrame();
}

static int
ResolveLaserJudgeClassId(const ImageRecognize::PredictResult &result) {
  for (const auto &box : result.boxes) {
    const int class_id = static_cast<int>(box[5]);
    if (ImageRecognize::AerialRobotLaserLockJudge::IsPurpleClassId(class_id)) {
      return class_id;
    }
  }

  if (result.boxes.empty()) {
    return -1;
  }
  return static_cast<int>(result.boxes.front()[5]);
}

static void
NormalizeStage3PredictResult(ImageRecognize::PredictResult *result) {
  for (auto &box : result->boxes) {
    box[5] = 3.0f;
  }
}

static void UpdateAerialRobotStageAndSwitchFlag(
    bool has_predict_result, const ImageRecognize::PredictResult &result,
    double stage_dt, ImageRecognize::AerialRobotLaserLockJudge *stage_judge,
    bool *pending_stage3_switch,
    const std::chrono::milliseconds &stage3_switch_target_lost_delay) {
  if (!has_predict_result ||
      g_aerial_robot_stage.load(std::memory_order_acquire) >=
          ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
    return;
  }

  const int previous_stage =
      g_aerial_robot_stage.load(std::memory_order_acquire);
  const int laser_judge_class_id = ResolveLaserJudgeClassId(result);
  const int stage = stage_judge->Update(laser_judge_class_id, stage_dt);
  g_aerial_robot_stage.store(stage, std::memory_order_release);
  if (stage != previous_stage) {
    DecrementAimbotTargetSaturated();
  }
  if (stage >= ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage &&
      previous_stage <
          ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
    *pending_stage3_switch = true;
    std::cout << "[空中机器人阶段] 达到 stage=3，等待目标丢失持续 "
              << stage3_switch_target_lost_delay.count() << "ms 后切换模型"
              << std::endl;
  }
}

static void UpdateTargetVisibleAndLostSince(
    bool track_alive, const std::chrono::steady_clock::time_point &now,
    std::chrono::steady_clock::time_point *target_lost_since,
    bool *target_lost_since_initialized) {
  g_target_visible.store(track_alive, std::memory_order_release);
  if (track_alive) {
    *target_lost_since_initialized = false;
    return;
  }

  if (!*target_lost_since_initialized) {
    *target_lost_since = now;
    *target_lost_since_initialized = true;
  }
}

static bool Stage3SwitchTargetLostLongEnough(
    bool target_lost_since_initialized,
    const std::chrono::steady_clock::time_point &now,
    const std::chrono::steady_clock::time_point &target_lost_since,
    const std::chrono::milliseconds &stage3_switch_target_lost_delay) {
  return target_lost_since_initialized &&
         (now - target_lost_since) >= stage3_switch_target_lost_delay;
}

// 等待相机线程交付下一帧；raw_frame 保留给未命中保存逻辑使用。
static bool SnapshotLatestFrame(
    bool has_last_submitted_frame_ts,
    const std::chrono::steady_clock::time_point &last_submitted_frame_ts,
    cv::Mat *frame, std::chrono::steady_clock::time_point *frame_ts) {
  std::unique_lock<std::mutex> lk(g_frame_mutex);
  const auto has_new_frame = [&]() {
    const int read_idx = g_read_idx.load(std::memory_order_acquire);
    if (read_idx < 0) {
      return false;
    }
    return !has_last_submitted_frame_ts ||
           g_frame_buffers[read_idx].ts != last_submitted_frame_ts;
  };

  g_frame_cv.wait(lk, [&]() { return !g_running || has_new_frame(); });
  if (!g_running) {
    return false;
  }

  const int read_idx = g_read_idx.load(std::memory_order_acquire);
  if (read_idx < 0) {
    return false;
  }

  *frame = g_frame_buffers[read_idx].frame;
  *frame_ts = g_frame_buffers[read_idx].ts;
  if (frame->empty()) {
    return false;
  }
  return true;
}

} // namespace

void CaptureThread(CameraTask::GalaxyCamera *camera,
                   std::optional<std::string> video_path);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor,
                        Tools::ScanController &scan_controller,
                        ImagePredictCommandLineOptions command_line_options);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port, Tools::ScanController &scan_controller,
                   bool enable_send_log);
void AimbotTargetReceiveThread();

int main(int argc, char **argv) {
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  std::unique_ptr<ImageRecognize::ImagePredict> predictor;
  const auto command_line_options =
      ImageRecognize::ParseImagePredictCommandLine(argc, argv);

  g_exposure_controller.SetExposureTimes(
      Tools::Params().stage12_exposure_time_us,
      Tools::Params().stage3_exposure_time_us);
  g_exposure_controller.LoadRuntimeParams();
  camera.setExposureTime(g_exposure_controller.GetStage12ExposureTime());

  cv::setUseOptimized(true);
  cv::setNumThreads(1);

  Tools::BindCurrentThreadToBigCores();

  try {
    predictor = std::make_unique<ImageRecognize::ImagePredict>(
        Tools::Params().stage12_model_path,
        Tools::Params().openvino_device_name);
  } catch (const std::exception &e) {
    std::cerr << "初始化 OpenVINO 模型失败：" << e.what() << std::endl;
    std::cerr << "当前 stage12_model_path: "
              << Tools::Params().stage12_model_path << std::endl;
    std::cerr << "当前 device_name: " << Tools::Params().openvino_device_name
              << std::endl;
    return -2;
  }

  // 初始化结束后主线程转移到辅助核，避免与推理关键路径争抢性能核。
  Tools::BindCurrentThreadToAuxCores();
  const bool serial_initially_open = OpenSerialPort(port);
  if (!serial_initially_open) {
    std::cout << "[IMU] 启动时串口不可用，后台将持续重连。" << std::endl;
  }

  std::thread image_capture(CaptureThread, &camera,
                            command_line_options.video_path);
  Tools::ScanController scan_controller;
  std::thread image_predict(ImagePredictThread, std::ref(*predictor),
                            std::ref(scan_controller), command_line_options);
  std::thread aimbot_target_receive(AimbotTargetReceiveThread);
  std::thread imu_read(IMUReadThread, std::ref(port));
  const bool enable_send_log = ResolveOption(
      command_line_options.enable_send_log, Tools::Params().enable_send_log);
  std::thread imu_send(IMUSendThread, std::ref(port), std::ref(scan_controller),
                       enable_send_log);
  JoinIfNeeded(image_capture);
  JoinIfNeeded(image_predict);
  JoinIfNeeded(imu_read);
  JoinIfNeeded(imu_send);
  JoinIfNeeded(aimbot_target_receive);

  // 只有曝光滑块属于可持久化运行参数；其他标定滑块只改内存，需手动记录。
  g_exposure_controller.SaveRuntimeParams();

  return 0;
}

void CaptureThread(CameraTask::GalaxyCamera *camera,
                   std::optional<std::string> video_path) {
  Tools::BindCurrentThreadToAuxCore(0);
  if (video_path.has_value()) {
    const std::string normalized_video_path =
        Tools::NormalizeVideoPathFromCli(*video_path);
    cv::VideoCapture video_capture;
    std::string open_error;
    if (!Tools::OpenVideoCaptureFromPath(normalized_video_path, &video_capture,
                                         &open_error)) {
      std::cerr << "打开视频输入失败：" << open_error << std::endl;
      RequestStop();
      return;
    }

    double source_fps = video_capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(source_fps) || source_fps <= 1.0) {
      source_fps = 30.0;
      std::cout << "[视频输入] 源 FPS 无效，回退到 " << source_fps << std::endl;
    }
    const auto frame_interval = std::chrono::duration<double>(1.0 / source_fps);
    auto next_frame_time = std::chrono::steady_clock::now();

    std::cout << "[视频输入] 已启用: " << normalized_video_path
              << "（loop=true, fps=" << source_fps << ")" << std::endl;
    while (g_running) {
      const auto now = std::chrono::steady_clock::now();
      if (now < next_frame_time) {
        std::this_thread::sleep_until(next_frame_time);
        if (!g_running) {
          break;
        }
      }

      cv::Mat frame;
      if (!video_capture.read(frame) || frame.empty()) {
        video_capture.set(cv::CAP_PROP_POS_FRAMES, 0.0);
        cv::Mat retry_frame;
        if (!video_capture.read(retry_frame) || retry_frame.empty()) {
          video_capture.release();
          if (!Tools::OpenVideoCaptureFromPath(normalized_video_path,
                                               &video_capture, &open_error)) {
            std::cerr << "循环视频重开失败：" << open_error << std::endl;
            RequestStop();
            break;
          }
          source_fps = video_capture.get(cv::CAP_PROP_FPS);
          if (!std::isfinite(source_fps) || source_fps <= 1.0) {
            source_fps = 30.0;
          }
          std::cout << "[视频输入] 循环重开成功: " << normalized_video_path
                    << "（fps=" << source_fps << ")" << std::endl;
          continue;
        }
        frame = std::move(retry_frame);
        std::cout << "[视频输入] 回到开头继续播放。" << std::endl;
      }

      int write_idx = g_write_idx.load(std::memory_order_relaxed);
      g_frame_buffers[write_idx].frame = std::move(frame);
      g_frame_buffers[write_idx].ts = std::chrono::steady_clock::now();

      int next_write = 1 - write_idx;
      g_write_idx.store(next_write, std::memory_order_release);
      g_read_idx.store(write_idx, std::memory_order_release);
      g_frame_cv.notify_one();

      next_frame_time +=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              frame_interval);
      if (next_frame_time < std::chrono::steady_clock::now() - frame_interval) {
        next_frame_time = std::chrono::steady_clock::now();
      }
    }
    video_capture.release();
    return;
  }

  if (!camera->open()) {
    std::cerr << "打开相机失败。" << std::endl;
    RequestStop();
    return;
  }
  while (g_running) {
    g_exposure_controller.ApplyPendingChange(camera);
    cv::Mat frame = camera->grab(Tools::Params().capture_timeout_ms);
    if (frame.empty()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Tools::Params().capture_empty_sleep_ms));
      continue;
    }

    // 双 buffer 避免相机线程和推理线程争用同一帧。
    int write_idx = g_write_idx.load(std::memory_order_relaxed);
    g_frame_buffers[write_idx].frame = std::move(frame);
    g_frame_buffers[write_idx].ts = std::chrono::steady_clock::now();

    int next_write = 1 - write_idx;
    g_write_idx.store(next_write, std::memory_order_release);
    g_read_idx.store(write_idx, std::memory_order_release);
    g_frame_cv.notify_one();
  }

  camera->close();
}

void ImagePredictThread(ImageRecognize::ImagePredict &predictor,
                        Tools::ScanController &scan_controller,
                        ImagePredictCommandLineOptions command_line_options) {
  Tools::BindCurrentThreadToBigCores();
  FPSCounter fps_counter;
  static Tools::AngleCalculator
      angle_calculator; // 持久化 AngleCalculator，避免每次调用时重置 lastTime
  static Tools::LaserAngleCalculator laser_angle_calculator;
  static Tools::DistanceCalculator distance_calculator;
  static ImageRecognize::CrossFrameTargetTracker target_tracker;
  static ImageRecognize::AerialRobotLaserLockJudge aerial_robot_stage_judge;
  static const Tools::FilterType filter_type =
      Tools::AngleCalculator::ParseFilterType(
          Tools::Params().angle_filter_type);
  static const ImageRecognize::TargetCampMode target_camp_mode =
      ImageRecognize::ParseTargetCampMode(Tools::Params().target_camp_mode);
  const bool enable_display = ResolveOption(command_line_options.enable_display,
                                            Tools::Params().enable_display);
  const bool enable_calibration_sliders =
      ResolveOption(command_line_options.enable_calibration_sliders,
                    Tools::Params().enable_calibration_sliders);
  const bool enable_send_log = ResolveOption(
      command_line_options.enable_send_log, Tools::Params().enable_send_log);
  const bool enable_scan_mode = ResolveOption(
      command_line_options.enable_scan_mode, Tools::Params().enable_scan_mode);
  const bool enable_save_no_target_images =
      ResolveOption(command_line_options.enable_save_no_target_images,
                    Tools::Params().enable_save_no_target_images);
  const bool enable_save_target_videos =
      Tools::Params().enable_save_target_videos;
  const bool enable_latency_profile =
      ResolveOption(command_line_options.enable_latency_profile,
                    Tools::Params().enable_latency_profile);

  PrintPredictSettings(filter_type, target_camp_mode, enable_display,
                       enable_scan_mode, enable_save_no_target_images,
                       enable_save_target_videos, enable_latency_profile,
                       enable_calibration_sliders, enable_send_log);
  static std::unique_ptr<Tools::SaveImageOnNoTarget> no_target_saver;
  static std::unique_ptr<Tools::SaveVideoOnTarget> target_video_saver;
  if (enable_save_no_target_images && !no_target_saver) {
    no_target_saver =
        std::make_unique<Tools::SaveImageOnNoTarget>(5, "captures");
  }
  if (enable_save_target_videos && !target_video_saver) {
    target_video_saver = std::make_unique<Tools::SaveVideoOnTarget>(
        Tools::Params().target_video_fps, "target_videos");
  }

  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;
  LatencyStats latency_total;
  LatencyStats latency_window;
  PixelHeightStats pixel_height_stats;
  std::uint64_t ui_frame_counter = 0;
  const auto scan_trigger_delay = std::chrono::milliseconds(500);
  const auto stage3_switch_target_lost_delay = std::chrono::milliseconds(
      std::max(0, Tools::Params().stage3_switch_target_lost_delay_ms));
  std::chrono::steady_clock::time_point target_lost_since{};
  bool target_lost_since_initialized = false;
  bool infer_inflight = false;
  bool has_last_submitted_frame_ts = false;
  cv::Mat inflight_frame;
  std::chrono::steady_clock::time_point inflight_frame_ts{};
  std::chrono::steady_clock::time_point inflight_infer_start{};
  const double max_infer_fps = Tools::Params().max_infer_fps;
  const auto infer_submit_interval =
      max_infer_fps > 0.0
          ? std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / max_infer_fps))
          : std::chrono::steady_clock::duration::zero();
  std::chrono::steady_clock::time_point next_infer_submit_time =
      std::chrono::steady_clock::now();
  std::unique_ptr<ImageRecognize::ImagePredict> stage3_predictor;
  ImageRecognize::ImagePredict *active_predictor = &predictor;
  bool using_stage3_predictor = false;
  bool pending_stage3_switch = false;

  const auto reset_tracking_state = [&]() {
    target_tracker.Reset();
    distance_calculator.ResetFilter();
    ClearPendingSend();
  };

  const auto switch_to_stage3_predictor = [&](const char *reason) -> bool {
    if (using_stage3_predictor) {
      return true;
    }

    try {
      g_exposure_controller.SetActiveMode(
          CameraTask::ExposureHotkeyController::ExposureMode::Stage3);
      Tools::DistanceCalculator::SetActiveStage(
          Tools::CalibrationStage::Stage3);
      Tools::LaserAngleCalculator::SetActiveStage(
          Tools::CalibrationStage::Stage3);
      if (!stage3_predictor) {
        stage3_predictor = std::make_unique<ImageRecognize::ImagePredict>(
            Tools::Params().stage3_model_path,
            Tools::Params().openvino_device_name);
      }
      active_predictor = stage3_predictor.get();
      using_stage3_predictor = true;
      pending_stage3_switch = false;
      reset_tracking_state();
      std::cout << "[空中机器人阶段] 切换到 stage3，模型="
                << Tools::Params().stage3_model_path
                << " 曝光(us)=" << Tools::Params().stage3_exposure_time_us
                << " 原因=" << reason << std::endl;
      return true;
    } catch (const std::exception &e) {
      std::cerr << "切换到 stage3 模型失败：" << e.what() << std::endl;
      RequestStop();
      return false;
    }
  };

  const auto switch_to_stage12_predictor = [&](const char *reason) {
    if (!using_stage3_predictor) {
      return;
    }

    g_exposure_controller.SetActiveMode(
        CameraTask::ExposureHotkeyController::ExposureMode::Stage12);
    Tools::DistanceCalculator::SetActiveStage(Tools::CalibrationStage::Stage12);
    Tools::LaserAngleCalculator::SetActiveStage(
        Tools::CalibrationStage::Stage12);
    active_predictor = &predictor;
    using_stage3_predictor = false;
    pending_stage3_switch = false;
    reset_tracking_state();
    std::cout << "[空中机器人阶段] 切换到 stage1/2，模型="
              << Tools::Params().stage12_model_path
              << " 曝光(us)=" << Tools::Params().stage12_exposure_time_us
              << " 原因=" << reason << std::endl;
  };

  while (g_running) {
    if (!infer_inflight) {
      if (infer_submit_interval !=
          std::chrono::steady_clock::duration::zero()) {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_infer_submit_time) {
          std::this_thread::sleep_until(next_infer_submit_time);
          if (!g_running) {
            break;
          }
        }
      }

      cv::Mat next_frame;
      std::chrono::steady_clock::time_point next_frame_ts{};
      if (!SnapshotLatestFrame(has_last_submitted_frame_ts, inflight_frame_ts,
                               &next_frame, &next_frame_ts)) {
        continue;
      }

      try {
        inflight_infer_start = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::capture_to_submit_ns, next_frame_ts,
                         inflight_infer_start);
        active_predictor->startAsync(next_frame);
        inflight_frame = std::move(next_frame);
        inflight_frame_ts = next_frame_ts;
        infer_inflight = true;
        has_last_submitted_frame_ts = true;
        if (infer_submit_interval !=
            std::chrono::steady_clock::duration::zero()) {
          next_infer_submit_time =
              std::chrono::steady_clock::now() + infer_submit_interval;
        }
      } catch (const std::exception &e) {
        std::cerr << "ImagePredictThread 异步提交异常：" << e.what()
                  << std::endl;
      }
      continue;
    }

    // loop 计时从等待异步结果前开始，包含 isAsyncReady 等待开销。
    const auto t_loop_start = ProfileNow(enable_latency_profile);
    if (!g_running) {
      break;
    }

    cv::Mat frame = inflight_frame;
    cv::Mat raw_frame;
    const auto frame_ts = inflight_frame_ts;

    ImageRecognize::PredictResult result;
    SerialTask::EulerAngles matched_imu{};
    bool has_matched_imu = false;
    double frame_dt = 0.0;
    double stage_dt =
        ImageRecognize::AerialRobotLaserLockJudge::kDefaultDeltaSeconds;
    bool has_realtime_frame_dt = false;
    bool has_predict_result = false;
    std::chrono::steady_clock::time_point infer_end_time{};
    try {
      result = active_predictor->getAsyncResult();
      if (using_stage3_predictor) {
        NormalizeStage3PredictResult(&result);
      }
      has_predict_result = true;
      // 仅统计“完整主流程每产出一帧推理结果”的吞吐。
      fps_counter.tick();
      const auto t_infer_end = ProfileNow(enable_latency_profile);
      infer_end_time = t_infer_end;
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::capture_to_result_ns, inflight_frame_ts,
                       t_infer_end);
      infer_inflight = false;
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::infer_ns, inflight_infer_start,
                       t_infer_end);

      if (has_prev_frame_ts) {
        frame_dt =
            std::chrono::duration<double>(frame_ts - prev_frame_ts).count();
        if (frame_dt > 0.0) {
          stage_dt = frame_dt;
        }
        has_realtime_frame_dt = true;
      }
      prev_frame_ts = frame_ts;
      has_prev_frame_ts = true;
      if (!has_realtime_frame_dt || frame_dt <= 0.0 ||
          frame_dt > Tools::Params().dt_max_sec)
        frame_dt = 0.0;

      // 关联最近一次 IMU 状态并记录延迟
      const auto t_imu_match_start = ProfileNow(enable_latency_profile);
      has_matched_imu = g_imu_buffer.MatchForFrame(
          frame_ts, &matched_imu,
          std::chrono::milliseconds(Tools::Params().imu_buffer_max_age_ms));
      const auto t_imu_match_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::imu_match_ns, t_imu_match_start,
                       t_imu_match_end);
    } catch (const std::exception &e) {
      infer_inflight = false;
      std::cerr << "ImagePredictThread 异常：" << e.what() << std::endl;
    }

    UpdateAerialRobotStageAndSwitchFlag(
        has_predict_result, result, stage_dt, &aerial_robot_stage_judge,
        &pending_stage3_switch, stage3_switch_target_lost_delay);

    std::array<float, 6> tracked_box{};
    bool has_tracked_box = false;
    const auto track_boxes =
        ImageRecognize::FilterTrackBoxes(result.boxes, target_camp_mode);
    const auto t_select_start = ProfileNow(enable_latency_profile);
    const auto track_result = target_tracker.Update(track_boxes);
    const auto t_select_end = ProfileNow(enable_latency_profile);
    AddLatencySample(enable_latency_profile, latency_total, latency_window,
                     &LatencyStats::select_box_ns, t_select_start,
                     t_select_end);
    const bool track_alive =
        track_result.has_box || target_tracker.HasRecentLock();
    const auto now = std::chrono::steady_clock::now();
    UpdateTargetVisibleAndLostSince(track_alive, now, &target_lost_since,
                                    &target_lost_since_initialized);

    const bool stage3_switch_target_lost_long_enough =
        Stage3SwitchTargetLostLongEnough(target_lost_since_initialized, now,
                                         target_lost_since,
                                         stage3_switch_target_lost_delay);
    if (pending_stage3_switch && !using_stage3_predictor && !track_alive &&
        stage3_switch_target_lost_long_enough) {
      switch_to_stage3_predictor("stage3_target_lost");
      const auto t_loop_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::loop_ns, t_loop_start, t_loop_end);
      AddLatencyFrame(enable_latency_profile, latency_total, latency_window);
      continue;
    }

    if (track_result.has_box) {
      tracked_box = track_result.box;
      has_tracked_box = true;
    }

    if (has_tracked_box) {
      if (enable_display && track_result.has_box) {
        const cv::Point2f detection_center =
            ImageRecognize::BoxCenter(track_result.box);
        ImageRecognize::ImageShow::ShowDetectionCenter(
            frame, detection_center.x, detection_center.y);
      }

      const cv::Point2f tracked_center = ImageRecognize::BoxCenter(tracked_box);
      if (enable_display) {
        ImageRecognize::ImageShow::ShowPred(frame, tracked_center.x,
                                            tracked_center.y);
      }

      const float center_x = tracked_center.x;
      const float center_y = tracked_center.y;
      const auto &distance_box =
          track_result.has_box ? track_result.box : tracked_box;
      const float height = ImageRecognize::BoxHeight(distance_box);
      pixel_height_stats.Add(height);
      const float distance = distance_calculator.CalculateDistance(
          distance_box[0], distance_box[1], distance_box[2], distance_box[3]);

      if (has_matched_imu) {
        const auto t_angle_start = ProfileNow(enable_latency_profile);
        Tools::AngleCommand angle_command;
        if (has_realtime_frame_dt) {
          angle_command = angle_calculator.CalculateAbsoluteAnglesWithVelocity(
              center_x, center_y, matched_imu.yaw, matched_imu.pitch,
              filter_type, frame_dt);
        } else {
          angle_command = angle_calculator.CalculateAbsoluteAnglesWithVelocity(
              center_x, center_y, matched_imu.yaw, matched_imu.pitch,
              filter_type);
        }
        const float filtered_yaw = angle_command.yaw;
        const float filtered_pitch = angle_command.pitch;
        const float yaw_velocity = angle_command.yaw_velocity;
        const float pitch_velocity = angle_command.pitch_velocity;
        const auto t_angle_end = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::angle_calc_ns, t_angle_start,
                         t_angle_end);

        // 必须用最短角差，否则跨越 ±180° 时会出现 300° 级突变。
        const float offset_yaw_angle =
            Tools::NormalizeDeltaDeg(filtered_yaw - matched_imu.yaw);
        const float offset_pitch_angle =
            Tools::NormalizeDeltaDeg(filtered_pitch - matched_imu.pitch);

        auto [laser_yaw_angle, laser_pitch_angle] =
            laser_angle_calculator.CalculateLaserAngles(
                distance, offset_yaw_angle, offset_pitch_angle);

        const auto t_control_start = ProfileNow(enable_latency_profile);
        float delta_yaw_raw = Tools::NormalizeDeltaDeg(
            static_cast<float>(offset_yaw_angle + laser_yaw_angle));
        float delta_pitch_raw = Tools::NormalizeDeltaDeg(
            static_cast<float>(offset_pitch_angle + laser_pitch_angle));
        if (std::abs(delta_yaw_raw) > Tools::Params().minimum_angle_deg ||
            std::abs(delta_pitch_raw) > Tools::Params().minimum_angle_deg) {
          const float cmd_delta_yaw =
              std::clamp(delta_yaw_raw, -Tools::Params().max_send_delta_deg,
                         Tools::Params().max_send_delta_deg);
          const float cmd_delta_pitch =
              std::clamp(delta_pitch_raw, -Tools::Params().pitch_abs_limit,
                         Tools::Params().pitch_abs_limit);

          const float send_abs_yaw = matched_imu.yaw + cmd_delta_yaw;
          const float send_abs_pitch = matched_imu.pitch + cmd_delta_pitch;
          const auto command_enqueue_time = std::chrono::steady_clock::now();

          StorePendingSend(AimbotSendCommand{send_abs_pitch, send_abs_yaw,
                                             cmd_delta_pitch, cmd_delta_yaw,
                                             pitch_velocity, yaw_velocity, 0x01,
                                             frame_ts, command_enqueue_time});
          AddLatencySample(enable_latency_profile, latency_total,
                           latency_window, &LatencyStats::result_to_control_ns,
                           infer_end_time, command_enqueue_time);
          if (enable_display) {
            ImageRecognize::ImageShow::ShowDistance(frame, distance);
          }
        } else {
          ClearPendingSend();
        }

        {
          std::lock_guard<std::mutex> lk(g_scan_controller_mutex);
          scan_controller.Reset();
        }
        const auto t_control_end = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::control_calc_ns, t_control_start,
                         t_control_end);
      } else {
        ClearPendingSend();
      }
    } else {
      if (enable_scan_mode && has_matched_imu && !track_alive &&
          target_lost_since_initialized &&
          (now - target_lost_since) >= scan_trigger_delay) {
        StartScanMode();
      } else {
        ClearPendingSend();
      }
    }

    // 可视化显示
    const double fps = fps_counter.get();

    // 渲染打点：统计显示和 GUI 轮询开销
    const auto t_render_start = ProfileNow(enable_latency_profile);
    const bool do_display =
        enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(std::max(
                                1, Tools::Params().display_every_n_frames)) ==
         0);
    const bool do_gui_poll =
        enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(std::max(
                                1, Tools::Params().gui_poll_every_n_frames)) ==
         0);
    ++ui_frame_counter;

    if (do_display) {
      frame = inflight_frame.clone();
      ImageRecognize::ImageShow::ShowLockProgress(
          frame, g_aerial_robot_stage.load(std::memory_order_acquire),
          aerial_robot_stage_judge.Progress(),
          aerial_robot_stage_judge.CurrentThreshold());
      ImageRecognize::ImageShow::ShowNow(frame, result, fps);
      if (enable_calibration_sliders) {
        Tools::CalibrationSliderPanel::Show(&g_exposure_controller);
      }
      if (has_tracked_box) {
        ImageRecognize::DrawTrackedBox(frame, tracked_box);
      }
    }

    // 当检测框数量不是 1 个时，按间隔保存画框前原图
    if (enable_save_no_target_images && no_target_saver) {
      raw_frame = inflight_frame.clone();
      no_target_saver->Update(raw_frame, result.boxes.size() != 1);
    }
    if (enable_save_target_videos && target_video_saver) {
      const cv::Mat &video_frame = do_display ? frame : inflight_frame;
      target_video_saver->Update(video_frame, has_tracked_box);
    }

    // 处理 GUI 事件并允许按键退出
    bool should_exit = false;
    if (do_gui_poll) {
      const auto previous_exposure_mode = g_exposure_controller.GetActiveMode();
      should_exit = g_exposure_controller.HandleGuiKey(
          ImageRecognize::ImageShow::PollKey());
      const auto current_exposure_mode = g_exposure_controller.GetActiveMode();
      if (!should_exit && current_exposure_mode != previous_exposure_mode) {
        if (current_exposure_mode ==
            CameraTask::ExposureHotkeyController::ExposureMode::Stage3) {
          switch_to_stage3_predictor("exposure_debug_stage3");
        } else {
          switch_to_stage12_predictor("exposure_debug_stage12");
        }
      }
    }
    if (should_exit) {
      RequestStop();
      const auto t_render_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::render_ns, t_render_start, t_render_end);
      const auto t_loop_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::loop_ns, t_loop_start, t_loop_end);
      AddLatencyFrame(enable_latency_profile, latency_total, latency_window);
      break;
    }

    const auto t_render_end = ProfileNow(enable_latency_profile);
    AddLatencySample(enable_latency_profile, latency_total, latency_window,
                     &LatencyStats::render_ns, t_render_start, t_render_end);

    const auto t_loop_end = ProfileNow(enable_latency_profile);
    AddLatencySample(enable_latency_profile, latency_total, latency_window,
                     &LatencyStats::loop_ns, t_loop_start, t_loop_end);
    AddLatencyFrame(enable_latency_profile, latency_total, latency_window);

    if (enable_latency_profile &&
        latency_window.frames >=
            static_cast<std::uint64_t>(
                Tools::Params().latency_print_interval_frames)) {
      PrintLatencyStats(latency_window, "窗口");
      {
        std::lock_guard<std::mutex> lk(g_send_latency_mutex);
        PrintLatencyStats(g_send_latency_window, "窗口-串口链路");
        g_send_latency_window = LatencyStats{};
      }
      latency_window = LatencyStats{};
    }
  }

  if (enable_latency_profile) {
    {
      std::lock_guard<std::mutex> lk(g_send_latency_mutex);
      if (g_send_latency_window.frames > 0)
        PrintLatencyStats(g_send_latency_window, "窗口尾-串口链路");
      PrintLatencyStats(g_send_latency_total, "总计-串口链路");
    }
    if (latency_window.frames > 0)
      PrintLatencyStats(latency_window, "窗口尾");
    PrintLatencyStats(latency_total, "总计");
  }
  PrintPixelHeightStats(pixel_height_stats);
}

void IMUReadThread(serial::Serial &port) {
  Tools::BindCurrentThreadToAuxCore(1);
  std::vector<uint8_t> imu_read_buffer;
  while (g_running) {
    try {
      if (!TryReopenSerialPort(port)) {
        std::this_thread::sleep_for(SerialReconnectInterval());
        continue;
      }

      GimbalImuFrame_SCM_t latest_frame{};
      size_t read_count = 0;
      {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        read_count = SerialTask::ReadAvailableIMUBytes(port, imu_read_buffer);
      }
      SerialTask::EulerAngles angles{};
      const bool read_ok =
          SerialTask::ParseLatestIMUFrame(imu_read_buffer.data(), read_count,
                                          latest_frame) &&
          SerialTask::TryToEulerAngles(latest_frame, angles);
      if (read_ok) {
        auto ts = std::chrono::steady_clock::now();
        g_imu_buffer.Add(
            ts, angles,
            std::chrono::milliseconds(Tools::Params().imu_buffer_max_age_ms));
      } else {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Tools::Params().imu_read_fail_sleep_ms));
      }
    } catch (const std::exception &e) {
      std::cerr << "警告：IMU 串口读取失败：" << e.what() << std::endl;
      CloseSerialPort(port);
      std::this_thread::sleep_for(SerialReconnectInterval());
    }
  }
  CloseSerialPort(port);
}

void IMUSendThread(serial::Serial &port, Tools::ScanController &scan_controller,
                   bool enable_send_log) {
  Tools::BindCurrentThreadToAuxCore(2);
  using Clock = std::chrono::steady_clock;
  const double scan_send_hz = EffectiveScanSendHz();
  const auto scan_send_interval = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / scan_send_hz));
  const auto scan_origin_hold_duration = std::chrono::milliseconds(
      std::max(0, Tools::Params().scan_origin_hold_ms));

  auto next_scan_send_time = Clock::now();
  bool last_scan_mode = false;
  bool scan_waiting_at_origin = false;
  Clock::time_point scan_origin_deadline = Clock::now();

  while (g_running) {
    if (!TryReopenSerialPort(port)) {
      std::this_thread::sleep_for(SerialReconnectInterval());
      continue;
    }

    const bool scan_mode = g_send_is_scan.load(std::memory_order_acquire);
    const bool target_visible =
        g_target_visible.load(std::memory_order_acquire);

    if (scan_mode) {
      if (target_visible) {
        ClearPendingSend();
        last_scan_mode = false;
        scan_waiting_at_origin = false;
        next_scan_send_time = Clock::now();
        continue;
      }

      const auto now = Clock::now();
      if (!last_scan_mode) {
        next_scan_send_time = now;
        last_scan_mode = true;
        scan_waiting_at_origin = true;
        scan_origin_deadline = now + scan_origin_hold_duration;
        {
          std::lock_guard<std::mutex> lk(g_scan_controller_mutex);
          scan_controller.Reset();
        }
      }

      if (now < next_scan_send_time) {
        WaitUntilNextScanSend(next_scan_send_time);
        continue;
      }

      SerialTask::EulerAngles latest_imu{};
      if (!g_imu_buffer.GetLatest(&latest_imu)) {
        WaitForScanStateChangeFor(
            std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
        continue;
      }

      Tools::ScanCommand scan_command{};
      {
        std::lock_guard<std::mutex> lk(g_scan_controller_mutex);
        if (scan_waiting_at_origin) {
          scan_command = scan_controller.BuildOriginCommand(latest_imu.yaw,
                                                            latest_imu.pitch);
        } else {
          scan_command =
              scan_controller.BuildCommand(latest_imu.yaw, latest_imu.pitch);
        }
      }

      try {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        const auto scan_send_time = std::chrono::steady_clock::now();
        SendAimbotCommand(
            port,
            AimbotSendCommand{
                scan_command.absolute_pitch_deg, scan_command.absolute_yaw_deg,
                scan_command.offset_pitch_deg, scan_command.offset_yaw_deg,
                scan_command.pitch_velocity_deg_per_sec,
                scan_command.yaw_velocity_deg_per_sec,
                scan_command.aimbot_state, scan_send_time, scan_send_time});
      } catch (const std::exception &e) {
        HandleSerialWriteFailure(port, e);
        WaitForScanStateChangeFor(
            std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
        continue;
      }
      if (scan_waiting_at_origin && now >= scan_origin_deadline) {
        scan_waiting_at_origin = false;
        {
          std::lock_guard<std::mutex> lk(g_scan_controller_mutex);
          scan_controller.Reset();
        }
      }
      next_scan_send_time = now + scan_send_interval;
      continue;
    }

    last_scan_mode = false;
    scan_waiting_at_origin = false;
    AimbotSendCommand command;
    if (TakePendingSend(&command)) {
      if (enable_send_log) {
        // std::cout << std::fixed << "发送偏角：offset_yaw=" <<
        // command.offset_yaw << "°，offset_pitch="
        //           << command.offset_pitch << "°" << std::endl;
      }
      try {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        SendAimbotCommand(port, command);
        AddSendLatencySample(command, std::chrono::steady_clock::now());
      } catch (const std::exception &e) {
        HandleSerialWriteFailure(port, e);
        WaitForNormalSendWork();
        continue;
      }
    } else {
      WaitForNormalSendWork();
    }
  }
}

void AimbotTargetReceiveThread() {
  Tools::BindCurrentThreadToAuxCore(3);
  NetworkTask::RunAimbotTargetReceiver(g_aimbot_target, []() {
    return g_running.load(std::memory_order_acquire);
  });
}
