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
#include "ImageRecognize/StagePredictorController.hpp"
#include "ImageRecognize/TargetTrackPipeline.hpp"
#include "ImageRecognize/TargetTracking.hpp"
#include "ImageRecognize/YoloLightPreprocess.hpp"
#include "SerialTask/SerialTask.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/AimbotLaserStateController.hpp"
#include "Tools/CalibrationSliderPanel.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/LostTargetRecoveryController.hpp"
#include "Tools/RuntimeParams.hpp"
#include "Tools/RuntimeStats.hpp"
#include "Tools/SaveImage.hpp"
#include "Tools/SaveVideo.hpp"
#include "Tools/ScanController.hpp"
#include "Tools/ScanSendController.hpp"
#include "Tools/Stage3AutoScanBoundsController.hpp"
#include "Tools/Stage12PitchPostprocess.hpp"
#include "Tools/StageRuntimeProfile.hpp"
#include "Tools/VideoInput.hpp"

namespace {
using ImageRecognize::ImagePredictCommandLineOptions;
using Tools::LatencyStats;
using Tools::PixelSizeStats;
using Tools::PrintLatencyStats;
using Tools::PrintPixelSizeStats;
using Tools::PrintSerialLatencyStats;

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

struct TrackedTargetAbsoluteAngles {
  bool valid = false;
  float yaw = 0.0f;
  float pitch = 0.0f;
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
static std::atomic<bool> g_send_is_scan{false};
static std::mutex g_scan_controller_mutex;
static std::atomic<bool> g_target_visible{false};
static CameraTask::ExposureHotkeyController g_exposure_controller;
static std::mutex g_serial_mutex;
static std::mutex g_send_latency_mutex;
static Tools::LatencyStats g_send_latency_total;
static Tools::LatencyStats g_send_latency_window;
static std::atomic<bool> g_stage3_roi_mode{false};
static std::atomic<bool> g_scan_stage3_mode{false};
static Tools::AimbotLaserStateController g_aimbot_laser_state_controller;
static Tools::Stage3AutoScanBoundsController
    g_stage3_auto_scan_bounds_controller;

namespace {
static void RequestStop() {
  g_running.store(false, std::memory_order_release);
  g_frame_cv.notify_all();
  g_pending_send_cv.notify_all();
}

static void ClearPendingSend() {
  bool should_notify = false;
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    should_notify =
        g_has_pending_send || g_send_is_scan.load(std::memory_order_acquire);
    g_send_is_scan.store(false, std::memory_order_release);
    g_has_pending_send = false;
  }
  if (should_notify) {
    g_pending_send_cv.notify_one();
  }
}

static void StopScanModeKeepPendingSend() {
  bool should_notify = false;
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    should_notify = g_send_is_scan.load(std::memory_order_acquire);
    g_send_is_scan.store(false, std::memory_order_release);
  }
  if (should_notify) {
    g_pending_send_cv.notify_one();
  }
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
  bool should_notify = false;
  {
    std::lock_guard<std::mutex> lk(g_pending_send_mutex);
    should_notify =
        g_has_pending_send || !g_send_is_scan.load(std::memory_order_acquire);
    g_has_pending_send = false;
    g_send_is_scan.store(true, std::memory_order_release);
  }
  if (should_notify) {
    g_pending_send_cv.notify_one();
  }
}

static double MaxSerialAimbotFrameHz() {
  constexpr double kBitsPerByteOnSerial = 10.0;
  constexpr double kSafetyRatio = 0.85;
  return SerialTask::DEFAULT_BAUD_RATE * kSafetyRatio /
         (kBitsPerByteOnSerial * sizeof(AimbotFrame_SCM_t));
}

static Tools::StageRuntimeProfile StageProfile(Tools::RuntimeStage stage) {
  return Tools::MakeStageRuntimeProfile(stage, MaxSerialAimbotFrameHz());
}

static Tools::StageRuntimeProfile StageProfile(bool stage3_mode) {
  return StageProfile(Tools::RuntimeStageFromBool(stage3_mode));
}

static Tools::ScanSendController::TickConfig
EffectiveScanTickConfig(bool stage3_mode) {
  const auto profile = StageProfile(stage3_mode);
  auto controller_config = profile.scan.controller_config;
  if (stage3_mode && g_stage3_auto_scan_bounds_controller.IsAutoMode()) {
    controller_config =
        g_stage3_auto_scan_bounds_controller.EffectiveControllerConfig();
  }
  return {profile.scan.effective_send_hz,
          std::chrono::milliseconds(profile.scan.origin_hold_ms),
          controller_config};
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
  SerialTask::SerialSend(port, command.absolute_pitch, command.absolute_yaw,
                         command.offset_pitch, command.offset_yaw,
                         command.pitch_velocity, command.yaw_velocity,
                         command.aimbot_state,
                         g_aimbot_laser_state_controller.CurrentWireTarget());
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
    bool enable_save_target_videos, bool enable_save_full_run_video,
    bool enable_latency_profile, bool enable_calibration_sliders,
    bool enable_send_log) {
  const auto print_scan_profile =
      [](const Tools::StageRuntimeProfile &profile) {
        std::cout << "[扫描模式] " << profile.DisplayName()
                  << " 发送频率: " << profile.scan.effective_send_hz;
        if (profile.scan.effective_send_hz < profile.scan.configured_send_hz) {
          std::cout << "（配置值 " << profile.scan.configured_send_hz
                    << "，已受串口带宽限制）";
        }
        std::cout << " Hz，yaw_speed=" << profile.scan.yaw_speed_deg_per_sec
                  << " deg/s，lambda_percent="
                  << profile.scan.pitch_wavelength_percent
                  << "，A_percent=" << profile.scan.pitch_amplitude_percent
                  << std::endl;
      };

  std::cout << "[角度滤波] 类型: " << Tools::ToString(filter_type) << std::endl;
  std::cout << "[跟踪阵营] 模式: " << ImageRecognize::ToString(target_camp_mode)
            << std::endl;
  std::cout << "[显示窗口] 启用: " << (enable_display ? "true" : "false")
            << std::endl;
  std::cout << "[标定滑块] 启用: "
            << (enable_calibration_sliders ? "true" : "false") << std::endl;
  std::cout << "[扫描模式] 启用: " << (enable_scan_mode ? "true" : "false")
            << std::endl;
  const auto stage12_profile = StageProfile(Tools::RuntimeStage::Stage12);
  print_scan_profile(stage12_profile);
  const auto stage3_profile = StageProfile(Tools::RuntimeStage::Stage3);
  print_scan_profile(stage3_profile);
  std::cout << "[扫描模式] " << stage3_profile.DisplayName()
            << " 丢失续行: " << stage3_profile.lost_target_coast.duration_ms
            << " ms，yaw_speed="
            << stage3_profile.lost_target_coast.yaw_speed_deg_per_sec
            << " deg/s，pitch_speed="
            << stage3_profile.lost_target_coast.pitch_speed_deg_per_sec
            << " deg/s，reacquire_delay="
            << stage3_profile.lost_target_coast.reacquire_confirm_ms
            << " ms，reacquire_gate="
            << stage3_profile.lost_target_coast.reacquire_gate_deg << " deg"
            << std::endl;
  std::cout << "[异常图片保存] 启用: "
            << (enable_save_no_target_images ? "true" : "false") << std::endl;
  std::cout << "[目标视频保存] 启用: "
            << (enable_save_target_videos ? "true" : "false") << std::endl;
  std::cout << "[全程录像] 启用: "
            << (enable_save_full_run_video ? "true" : "false") << std::endl;
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

static double
ElapsedMilliseconds(const std::chrono::steady_clock::time_point &start,
                    const std::chrono::steady_clock::time_point &end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
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
    ImageRecognize::StagePredictorController *stage_predictor_controller) {
  if (!g_aimbot_laser_state_controller.IsStageJudgeInitialized() ||
      !has_predict_result ||
      g_aimbot_laser_state_controller.CurrentStage() >=
          ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
    return;
  }

  const int previous_stage = g_aimbot_laser_state_controller.CurrentStage();
  const int laser_judge_class_id = ResolveLaserJudgeClassId(result);
  const int stage = stage_judge->Update(laser_judge_class_id, stage_dt);
  g_aimbot_laser_state_controller.SetCurrentStage(stage);
  if (previous_stage ==
          ImageRecognize::AerialRobotLaserLockJudge::kInitialStage &&
      stage == ImageRecognize::AerialRobotLaserLockJudge::kInitialStage + 1) {
    g_aimbot_laser_state_controller.OnStage1ToStage2Locked(
        std::chrono::steady_clock::now());
  }
  if (stage >= ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage &&
      previous_stage <
          ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
    g_aimbot_laser_state_controller.ClearSuppressWindow();
    stage_predictor_controller->RequestStage3SwitchAfterTargetLoss();
  }
}

static void
UpdateTargetLostSince(bool track_alive,
                      const std::chrono::steady_clock::time_point &now,
                      std::chrono::steady_clock::time_point *target_lost_since,
                      bool *target_lost_since_initialized) {
  if (track_alive) {
    *target_lost_since_initialized = false;
    return;
  }

  if (!*target_lost_since_initialized) {
    *target_lost_since = now;
    *target_lost_since_initialized = true;
  }
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

static void
PublishLatestFrame(cv::Mat frame,
                   const std::chrono::steady_clock::time_point &ts) {
  const int write_idx = g_write_idx.load(std::memory_order_relaxed);
  g_frame_buffers[write_idx].frame = std::move(frame);
  g_frame_buffers[write_idx].ts = ts;

  g_write_idx.store(1 - write_idx, std::memory_order_release);
  g_read_idx.store(write_idx, std::memory_order_release);
  g_frame_cv.notify_one();
}

static void EnsureOptionalSaversInitialized(
    bool enable_save_no_target_images, bool enable_save_target_videos,
    bool enable_save_full_run_video,
    std::unique_ptr<Tools::SaveImageOnNoTarget> *no_target_saver,
    std::unique_ptr<Tools::SaveVideoOnTarget> *target_video_saver,
    std::unique_ptr<Tools::SaveVideoFullRun> *full_run_video_saver) {
  if (enable_save_no_target_images && !*no_target_saver) {
    *no_target_saver =
        std::make_unique<Tools::SaveImageOnNoTarget>(5, "captures");
  }
  if (enable_save_target_videos && !*target_video_saver) {
    *target_video_saver = std::make_unique<Tools::SaveVideoOnTarget>(
        Tools::Params().target_video_fps, "target_videos", cv::Size(480, 480));
  }
  if (enable_save_full_run_video && !*full_run_video_saver) {
    *full_run_video_saver = std::make_unique<Tools::SaveVideoFullRun>(
        Tools::Params().target_video_fps, "full_run_videos", cv::Size(480, 480),
        4);
  }
}

} // namespace

void CaptureThread(CameraTask::GalaxyCamera *camera,
                   std::optional<std::string> video_path);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor,
                        Tools::ScanController &scan_controller,
                        ImagePredictCommandLineOptions command_line_options);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port, Tools::ScanController &scan_controller,
                   bool enable_send_log, bool enable_latency_profile);

int main(int argc, char **argv) {
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  std::unique_ptr<ImageRecognize::ImagePredict> predictor;
  const auto stage12_profile = StageProfile(Tools::RuntimeStage::Stage12);
  const auto stage3_profile = StageProfile(Tools::RuntimeStage::Stage3);
  const auto command_line_options =
      ImageRecognize::ParseImagePredictCommandLine(argc, argv);

  g_exposure_controller.SetExposureTimes(stage12_profile.exposure_time_us,
                                         stage3_profile.exposure_time_us);
  g_exposure_controller.LoadRuntimeParams();
  camera.setExposureTime(g_exposure_controller.GetStage12ExposureTime());
  // stage12 永远全画幅：启动时显式关闭 ROI。
  camera.setRoiEnabled(false);
  camera.setRoiKeepCentered(stage3_profile.roi.keep_centered);
  camera.setRoi(stage3_profile.roi.width, stage3_profile.roi.height,
                stage3_profile.roi.offset_x, stage3_profile.roi.offset_y);
  if (stage3_profile.roi.enabled) {
    std::cout << "[Camera] Stage3 ROI request: width="
              << stage3_profile.roi.width
              << " height=" << stage3_profile.roi.height << " keep_centered="
              << (stage3_profile.roi.keep_centered ? "true" : "false")
              << (stage3_profile.roi.keep_centered
                      ? " anchor=center"
                      : " offset_x=" +
                            std::to_string(stage3_profile.roi.offset_x) +
                            " offset_y=" +
                            std::to_string(stage3_profile.roi.offset_y))
              << std::endl;
  }

  cv::setUseOptimized(true);
  cv::setNumThreads(1);

  Tools::BindCurrentThreadToBigCores();

  try {
    predictor = std::make_unique<ImageRecognize::ImagePredict>(
        *stage12_profile.model_path, Tools::Params().openvino_device_name);
  } catch (const std::exception &e) {
    std::cerr << "初始化 OpenVINO 模型失败：" << e.what() << std::endl;
    std::cerr << "当前 stage12_model_path: " << *stage12_profile.model_path
              << std::endl;
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
  scan_controller.SetConfig(stage12_profile.scan.controller_config);
  std::thread image_predict(ImagePredictThread, std::ref(*predictor),
                            std::ref(scan_controller), command_line_options);
  std::thread imu_read(IMUReadThread, std::ref(port));
  const bool enable_send_log = ResolveOption(
      command_line_options.enable_send_log, Tools::Params().enable_send_log);
  const bool enable_latency_profile =
      ResolveOption(command_line_options.enable_latency_profile,
                    Tools::Params().enable_latency_profile);
  std::thread imu_send(IMUSendThread, std::ref(port), std::ref(scan_controller),
                       enable_send_log, enable_latency_profile);
  JoinIfNeeded(image_capture);
  JoinIfNeeded(image_predict);
  JoinIfNeeded(imu_read);
  JoinIfNeeded(imu_send);

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

      PublishLatestFrame(std::move(frame), std::chrono::steady_clock::now());

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

  const int capture_timeout_ms = Tools::Params().capture_timeout_ms;
  bool last_stage3_roi_mode = g_stage3_roi_mode.load(std::memory_order_acquire);
  auto apply_stage3_roi_mode = [&](bool stage3_roi_mode) {
    const auto total_start = std::chrono::steady_clock::now();
    const auto profile = StageProfile(stage3_roi_mode);
    const auto roi_config_start = std::chrono::steady_clock::now();
    camera->setRoiKeepCentered(profile.roi.keep_centered);
    camera->setRoi(profile.roi.width, profile.roi.height, profile.roi.offset_x,
                   profile.roi.offset_y);
    camera->setRoiEnabled(profile.roi.enabled);
    const auto roi_config_end = std::chrono::steady_clock::now();

    const auto stop_start = std::chrono::steady_clock::now();
    camera->stop();
    const auto stop_end = std::chrono::steady_clock::now();

    const auto start_start = std::chrono::steady_clock::now();
    camera->start();
    const auto start_end = std::chrono::steady_clock::now();

    std::cout << "[Camera] ROI mode switched: stage3="
              << (stage3_roi_mode ? "true" : "false")
              << " roi_enabled=" << (profile.roi.enabled ? "true" : "false")
              << " config_ms="
              << ElapsedMilliseconds(roi_config_start, roi_config_end)
              << " stop_ms=" << ElapsedMilliseconds(stop_start, stop_end)
              << " start_ms=" << ElapsedMilliseconds(start_start, start_end)
              << " total_ms=" << ElapsedMilliseconds(total_start, start_end)
              << std::endl;
  };
  apply_stage3_roi_mode(last_stage3_roi_mode);

  while (g_running) {
    const bool stage3_roi_mode =
        g_stage3_roi_mode.load(std::memory_order_acquire);
    if (stage3_roi_mode != last_stage3_roi_mode) {
      apply_stage3_roi_mode(stage3_roi_mode);
      last_stage3_roi_mode = stage3_roi_mode;
    }

    g_exposure_controller.ApplyPendingChange(camera);
    cv::Mat frame = camera->grab(capture_timeout_ms);
    if (frame.empty()) {
      continue;
    }

    // 双 buffer 避免相机线程和推理线程争用同一帧。
    PublishLatestFrame(std::move(frame), std::chrono::steady_clock::now());
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
  static Tools::DistanceCalculator distance_calculator;
  static Tools::Stage12LaserPitchCompStabilizer
      stage12_laser_pitch_comp_stabilizer;
  static ImageRecognize::TargetTrackPipeline target_track_pipeline;
  static ImageRecognize::AerialRobotLaserLockJudge aerial_robot_stage_judge;
  static const Tools::FilterType filter_type =
      Tools::AngleCalculator::ParseFilterType(
          Tools::Params().angle_filter_type);
  static ImageRecognize::TargetCampModeController target_camp_mode_controller(
      ImageRecognize::ParseTargetCampMode(Tools::Params().target_camp_mode));
  ImageRecognize::TargetCampMode target_camp_mode =
      target_camp_mode_controller.Get();
  ImageRecognize::TargetCampMode last_target_camp_mode = target_camp_mode;
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
  const bool enable_save_full_run_video =
      ResolveOption(command_line_options.enable_save_full_run_video,
                    Tools::Params().enable_save_full_run_video);
  const bool enable_latency_profile =
      ResolveOption(command_line_options.enable_latency_profile,
                    Tools::Params().enable_latency_profile);

  PrintPredictSettings(filter_type, target_camp_mode, enable_display,
                       enable_scan_mode, enable_save_no_target_images,
                       enable_save_target_videos, enable_save_full_run_video,
                       enable_latency_profile, enable_calibration_sliders,
                       enable_send_log);
  static std::unique_ptr<Tools::SaveImageOnNoTarget> no_target_saver;
  static std::unique_ptr<Tools::SaveVideoOnTarget> target_video_saver;
  static std::unique_ptr<Tools::SaveVideoFullRun> full_run_video_saver;
  EnsureOptionalSaversInitialized(enable_save_no_target_images,
                                  enable_save_target_videos,
                                  enable_save_full_run_video, &no_target_saver,
                                  &target_video_saver, &full_run_video_saver);

  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;
  LatencyStats latency_total;
  LatencyStats latency_window;
  PixelSizeStats pixel_size_stats;
  std::uint64_t ui_frame_counter = 0;
  const auto scan_trigger_delay = std::chrono::milliseconds(
      std::max(0, Tools::Params().scan_trigger_delay_ms));
  const auto stage3_profile = StageProfile(Tools::RuntimeStage::Stage3);
  const auto imu_buffer_max_age =
      std::chrono::milliseconds(Tools::Params().imu_buffer_max_age_ms);
  const double dt_max_sec = Tools::Params().dt_max_sec;
  const float minimum_angle_deg = Tools::Params().minimum_angle_deg;
  const float max_send_delta_deg = Tools::Params().max_send_delta_deg;
  const float pitch_abs_limit = Tools::Params().pitch_abs_limit;
  const float stage12_pitch_micro_deadband_deg =
      std::max(0.0f, Tools::Params().stage12_pitch_micro_deadband_deg);
  const std::uint64_t display_every_n_frames = static_cast<std::uint64_t>(
      std::max(1, Tools::Params().display_every_n_frames));
  const std::uint64_t gui_poll_every_n_frames = static_cast<std::uint64_t>(
      std::max(1, Tools::Params().gui_poll_every_n_frames));
  const std::uint64_t latency_print_interval_frames =
      static_cast<std::uint64_t>(
          std::max(1, Tools::Params().latency_print_interval_frames));
  std::chrono::steady_clock::time_point target_lost_since{};
  bool target_lost_since_initialized = false;
  bool infer_inflight = false;
  bool has_last_submitted_frame_ts = false;
  cv::Mat inflight_frame;
  std::chrono::steady_clock::time_point inflight_frame_ts{};
  std::chrono::steady_clock::time_point inflight_infer_start{};
  bool inflight_used_stage3_predictor = false;
  const double max_infer_fps = Tools::Params().max_infer_fps;
  const auto infer_submit_interval =
      max_infer_fps > 0.0
          ? std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / max_infer_fps))
          : std::chrono::steady_clock::duration::zero();
  std::chrono::steady_clock::time_point next_infer_submit_time =
      std::chrono::steady_clock::now();
  Tools::LostTargetRecoveryController lost_target_recovery_controller;

  const auto reset_tracking_state = [&]() {
    target_track_pipeline.Reset();
    distance_calculator.ResetFilter();
    stage12_laser_pitch_comp_stabilizer.Reset();
    lost_target_recovery_controller.Reset();
    ClearPendingSend();
  };
  ImageRecognize::StagePredictorController stage_predictor_controller(
      &predictor, Tools::Params().openvino_device_name,
      MaxSerialAimbotFrameHz(),
      ImageRecognize::StagePredictorController::Hooks{
          &g_exposure_controller, &g_stage3_roi_mode, &g_scan_stage3_mode,
          &scan_controller, &g_scan_controller_mutex, reset_tracking_state,
          RequestStop});

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

      const auto t_submit_wait_start = ProfileNow(enable_latency_profile);
      cv::Mat next_frame;
      std::chrono::steady_clock::time_point next_frame_ts{};
      if (!SnapshotLatestFrame(has_last_submitted_frame_ts, inflight_frame_ts,
                               &next_frame, &next_frame_ts)) {
        continue;
      }
      const auto t_snapshot_done = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::submit_wait_ns, t_submit_wait_start,
                       t_snapshot_done);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::capture_to_snapshot_ns, next_frame_ts,
                       t_snapshot_done);

      try {
        const auto t_submit_prepare_start = ProfileNow(enable_latency_profile);
        const bool submit_to_stage3 =
            stage_predictor_controller.UsingStage3Predictor();
        cv::Mat infer_frame = next_frame;
        const auto t_submit_prepare_end = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::submit_prepare_ns,
                         t_submit_prepare_start, t_submit_prepare_end);
        const auto t_async_submit_start = ProfileNow(enable_latency_profile);
        stage_predictor_controller.ActivePredictor()->startAsync(infer_frame);
        const auto t_async_submit_end = ProfileNow(enable_latency_profile);
        if (submit_to_stage3) {
          AddLatencySample(enable_latency_profile, latency_total,
                           latency_window,
                           &LatencyStats::submit_stage3_preprocess_ns,
                           t_async_submit_start, t_async_submit_end);
        }
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::submit_async_ns, t_async_submit_start,
                         t_async_submit_end);
        inflight_infer_start = t_async_submit_end;
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::capture_to_submit_ns, next_frame_ts,
                         inflight_infer_start);
        inflight_frame = std::move(infer_frame);
        inflight_frame_ts = next_frame_ts;
        inflight_used_stage3_predictor = submit_to_stage3;
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

    const auto frame_ts = inflight_frame_ts;
    ImageRecognize::OverlayData overlay_data{};

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
      result = stage_predictor_controller.ActivePredictor()->getAsyncResult();
      if (inflight_used_stage3_predictor) {
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
      if (!has_realtime_frame_dt || frame_dt <= 0.0 || frame_dt > dt_max_sec)
        frame_dt = 0.0;

      // 关联最近一次 IMU 状态并记录延迟
      const auto t_imu_match_start = ProfileNow(enable_latency_profile);
      has_matched_imu = g_imu_buffer.MatchForFrame(frame_ts, &matched_imu,
                                                   imu_buffer_max_age);
      const auto t_imu_match_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::imu_match_ns, t_imu_match_start,
                       t_imu_match_end);
    } catch (const std::exception &e) {
      infer_inflight = false;
      std::cerr << "ImagePredictThread 异常：" << e.what() << std::endl;
    }

    UpdateAerialRobotStageAndSwitchFlag(has_predict_result, result, stage_dt,
                                        &aerial_robot_stage_judge,
                                        &stage_predictor_controller);

    std::array<float, 6> tracked_box{};
    bool has_tracked_box = false;
    const auto t_select_start = ProfileNow(enable_latency_profile);
    target_camp_mode = target_camp_mode_controller.Get();
    const bool using_stage3_predictor =
        stage_predictor_controller.UsingStage3Predictor();
    if (target_camp_mode != last_target_camp_mode) {
      target_track_pipeline.Reset();
      stage12_laser_pitch_comp_stabilizer.Reset();
      last_target_camp_mode = target_camp_mode;
      std::cout << "[跟踪阵营] 切换为 "
                << ImageRecognize::ToString(target_camp_mode) << std::endl;
    }
    const auto track_pipeline_result = target_track_pipeline.Update(
        result, target_camp_mode, using_stage3_predictor, frame_dt);
    const auto t_select_end = ProfileNow(enable_latency_profile);
    AddLatencySample(enable_latency_profile, latency_total, latency_window,
                     &LatencyStats::select_box_ns, t_select_start,
                     t_select_end);
    const auto &track_result = track_pipeline_result.track_result;
    const bool track_alive = track_pipeline_result.track_alive;
    const auto now = std::chrono::steady_clock::now();
    UpdateTargetLostSince(track_alive, now, &target_lost_since,
                          &target_lost_since_initialized);
    g_target_visible.store(track_result.has_box, std::memory_order_release);
    TrackedTargetAbsoluteAngles tracked_target_absolute_angles{};
    if (track_pipeline_result.has_tracked_box && has_matched_imu) {
      const cv::Point2f tracked_center =
          ImageRecognize::BoxCenter(track_pipeline_result.tracked_box);
      const auto [tracked_absolute_yaw, tracked_absolute_pitch] =
          angle_calculator.CalculateAbsoluteAnglesWithoutFilter(
              tracked_center.x, tracked_center.y, matched_imu.yaw,
              matched_imu.pitch);
      tracked_target_absolute_angles.valid = true;
      tracked_target_absolute_angles.yaw = tracked_absolute_yaw;
      tracked_target_absolute_angles.pitch = tracked_absolute_pitch;
    }
    const bool accept_tracked_target =
        lost_target_recovery_controller.ShouldAcceptTrackedTarget(
            Tools::LostTargetRecoveryController::ReacquireCheckInput{
                using_stage3_predictor, track_pipeline_result.has_tracked_box,
                tracked_target_absolute_angles.valid,
                tracked_target_absolute_angles.pitch,
                tracked_target_absolute_angles.yaw, now,
                stage3_profile.lost_target_coast});

    if (!track_alive &&
        stage_predictor_controller.ShouldSwitchToStage3AfterLostTarget(
            target_lost_since_initialized, now, target_lost_since)) {
      stage_predictor_controller.SwitchToStage3("stage3_target_lost");
      const auto t_loop_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::loop_ns, t_loop_start, t_loop_end);
      AddLatencyFrame(enable_latency_profile, latency_total, latency_window);
      continue;
    }

    if (track_pipeline_result.has_tracked_box && accept_tracked_target) {
      tracked_box = track_pipeline_result.tracked_box;
      has_tracked_box = true;
    }

    if (has_tracked_box) {
      if (track_result.has_box) {
        overlay_data.show_detection_center = true;
        overlay_data.detection_center =
            ImageRecognize::BoxCenter(track_result.box);
      }

      const cv::Point2f tracked_center = ImageRecognize::BoxCenter(tracked_box);
      overlay_data.show_tracked_center = true;
      overlay_data.tracked_center = tracked_center;

      const float center_x = tracked_center.x;
      const float center_y = tracked_center.y;
      const auto &distance_box =
          track_result.has_box ? track_result.box : tracked_box;
      const float width = ImageRecognize::BoxWidth(distance_box);
      const float height = ImageRecognize::BoxHeight(distance_box);
      pixel_size_stats.Add(width, height);
      const auto distance_debug = distance_calculator.CalculateDistanceWithDebug(
          distance_box[0], distance_box[1], distance_box[2], distance_box[3]);
      const float distance = distance_debug.used_distance;
      g_aimbot_laser_state_controller.UpdateDistanceFlags(
          distance, &aerial_robot_stage_judge);
      if (std::isfinite(distance) && distance > 0.0f) {
        overlay_data.show_distance = true;
        overlay_data.distance = distance;
        overlay_data.show_distance_debug = true;
        overlay_data.width_distance = distance_debug.width_distance;
        overlay_data.height_distance = distance_debug.height_distance;
        overlay_data.distance_source =
            Tools::DistanceCalculator::DistanceSourceName(
                distance_debug.source);
      }

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

        const auto t_control_start = ProfileNow(enable_latency_profile);
        const float laser_pitch_comp_deg_raw =
            Tools::DistanceCalculator::CalculateLaserPitchCompensationDeg(
                distance, offset_pitch_angle);
        const float laser_pitch_comp_deg =
            using_stage3_predictor
                ? (stage12_laser_pitch_comp_stabilizer.Reset(),
                   laser_pitch_comp_deg_raw)
                : stage12_laser_pitch_comp_stabilizer.Filter(
                      laser_pitch_comp_deg_raw,
                      has_realtime_frame_dt ? frame_dt : 0.0);
        const float delta_yaw_raw =
            Tools::NormalizeDeltaDeg(static_cast<float>(offset_yaw_angle));
        float delta_pitch_raw = Tools::NormalizeDeltaDeg(
            static_cast<float>(offset_pitch_angle) + laser_pitch_comp_deg);
        if (!using_stage3_predictor) {
          delta_pitch_raw = Tools::ApplySoftDeadband(
              delta_pitch_raw, stage12_pitch_micro_deadband_deg);
        }
        if (using_stage3_predictor) {
          g_stage3_auto_scan_bounds_controller.ExpandForStage3Target(
              filtered_yaw, filtered_pitch);
        }
        if (std::abs(delta_yaw_raw) > minimum_angle_deg ||
            std::abs(delta_pitch_raw) > minimum_angle_deg) {
          const float cmd_delta_yaw = std::clamp(
              delta_yaw_raw, -max_send_delta_deg, max_send_delta_deg);
          const float cmd_delta_pitch =
              std::clamp(delta_pitch_raw, -pitch_abs_limit, pitch_abs_limit);

          const float send_abs_yaw = matched_imu.yaw + cmd_delta_yaw;
          const float send_abs_pitch = matched_imu.pitch + cmd_delta_pitch;
          const auto command_enqueue_time = std::chrono::steady_clock::now();

          StorePendingSend(AimbotSendCommand{send_abs_pitch, send_abs_yaw,
                                             cmd_delta_pitch, cmd_delta_yaw,
                                             pitch_velocity, yaw_velocity, 0x01,
                                             frame_ts, command_enqueue_time});
          if (!using_stage3_predictor) {
            g_stage3_auto_scan_bounds_controller.UpdateFromStage12Target(
                send_abs_yaw, send_abs_pitch);
          }
          if (using_stage3_predictor) {
            lost_target_recovery_controller.PrepareStage3Coast(
                true, send_abs_pitch, send_abs_yaw, pitch_velocity,
                yaw_velocity, stage3_profile.lost_target_coast);
          }
          AddLatencySample(enable_latency_profile, latency_total,
                           latency_window, &LatencyStats::result_to_control_ns,
                           infer_end_time, command_enqueue_time);
        } else {
          ClearPendingSend();
        }

        const auto t_control_end = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::control_calc_ns, t_control_start,
                         t_control_end);
      } else {
        stage12_laser_pitch_comp_stabilizer.Reset();
        ClearPendingSend();
      }
    } else {
      stage12_laser_pitch_comp_stabilizer.Reset();
      const auto recovery_result = lost_target_recovery_controller.Update(
          Tools::LostTargetRecoveryController::UpdateInput{
              using_stage3_predictor, has_matched_imu, matched_imu, track_alive,
              track_result.has_box, enable_scan_mode,
              target_lost_since_initialized, now, target_lost_since,
              scan_trigger_delay, stage3_profile.lost_target_coast,
              max_send_delta_deg, pitch_abs_limit});
      if (recovery_result.has_command) {
        const auto command_enqueue_time = std::chrono::steady_clock::now();
        StorePendingSend(
            AimbotSendCommand{recovery_result.command.absolute_pitch,
                              recovery_result.command.absolute_yaw,
                              recovery_result.command.offset_pitch,
                              recovery_result.command.offset_yaw,
                              recovery_result.command.pitch_velocity,
                              recovery_result.command.yaw_velocity, 0x01,
                              frame_ts, command_enqueue_time});
      } else if (recovery_result.pending_action ==
                 Tools::LostTargetRecoveryController::PendingAction::
                     StartScanMode) {
        StartScanMode();
      } else if (recovery_result.pending_action ==
                 Tools::LostTargetRecoveryController::PendingAction::
                     ClearPendingSend) {
        ClearPendingSend();
      }
    }

    // 可视化显示
    const double fps = fps_counter.get();

    // 渲染打点：统计显示和 GUI 轮询开销
    const auto t_render_start = ProfileNow(enable_latency_profile);
    const bool do_display =
        enable_display && (ui_frame_counter % display_every_n_frames == 0);
    const bool do_gui_poll =
        enable_display && (ui_frame_counter % gui_poll_every_n_frames == 0);
    ++ui_frame_counter;

    cv::Mat full_overlay_frame;
    const bool need_full_overlay_frame =
        do_display || (enable_save_full_run_video && full_run_video_saver);
    if (need_full_overlay_frame) {
      full_overlay_frame = inflight_frame.clone();
      ImageRecognize::DrawFullOverlay(
          full_overlay_frame, result, fps,
          g_aimbot_laser_state_controller.CurrentStage(),
          aerial_robot_stage_judge.Progress(),
          aerial_robot_stage_judge.CurrentThreshold(), overlay_data,
          has_tracked_box, tracked_box);
    }

    if (do_display) {
      if (enable_calibration_sliders) {
        Tools::CalibrationSliderPanel::Show(&g_exposure_controller,
                                            &target_camp_mode_controller);
      }
      ImageRecognize::ImageShow::ShowFrame(full_overlay_frame);
    }

    cv::Mat video_frame;
    if (enable_save_target_videos && target_video_saver) {
      video_frame = inflight_frame.clone();
      ImageRecognize::ImageShow::DrawStatusText(
          video_frame, fps,
          g_aimbot_laser_state_controller.CurrentStage(),
          aerial_robot_stage_judge.Progress(),
          aerial_robot_stage_judge.CurrentThreshold(), overlay_data);
    }

    // 当检测框数量不是 1
    // 个时，按间隔保存画框前图像，覆盖无目标/多目标异常样本。
    if (enable_save_no_target_images && no_target_saver) {
      if (inflight_used_stage3_predictor) {
        no_target_saver->UpdateStage3Raw(inflight_frame,
                                         result.boxes.size() != 1);
      } else {
        no_target_saver->Update(inflight_frame, result.boxes.size() != 1);
      }
    }
    if (enable_save_target_videos && target_video_saver) {
      target_video_saver->Update(video_frame, has_tracked_box);
    }
    if (enable_save_full_run_video && full_run_video_saver) {
      full_run_video_saver->UpdateOverlay(full_overlay_frame);
      full_run_video_saver->UpdateRaw(inflight_frame);
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
          stage_predictor_controller.SwitchToStage3("exposure_debug_stage3");
        } else {
          stage_predictor_controller.SwitchToStage12("exposure_debug_stage12");
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
        latency_window.frames >= latency_print_interval_frames) {
      PrintLatencyStats(latency_window, "窗口");
      {
        std::lock_guard<std::mutex> lk(g_send_latency_mutex);
        PrintSerialLatencyStats(g_send_latency_window, "窗口-串口链路");
        g_send_latency_window = LatencyStats{};
      }
      latency_window = LatencyStats{};
    }
  }

  if (enable_latency_profile) {
    {
      std::lock_guard<std::mutex> lk(g_send_latency_mutex);
      if (g_send_latency_window.frames > 0)
        PrintSerialLatencyStats(g_send_latency_window, "窗口尾-串口链路");
      PrintSerialLatencyStats(g_send_latency_total, "总计-串口链路");
    }
    if (latency_window.frames > 0)
      PrintLatencyStats(latency_window, "窗口尾");
    PrintLatencyStats(latency_total, "总计");
  }
  PrintPixelSizeStats(pixel_size_stats);
}

void IMUReadThread(serial::Serial &port) {
  Tools::BindCurrentThreadToAuxCore(1);
  std::vector<uint8_t> imu_read_buffer;
  const auto imu_buffer_max_age =
      std::chrono::milliseconds(Tools::Params().imu_buffer_max_age_ms);
  const auto imu_read_fail_sleep =
      std::chrono::milliseconds(Tools::Params().imu_read_fail_sleep_ms);
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
        g_imu_buffer.Add(ts, angles, imu_buffer_max_age);
      } else {
        std::this_thread::sleep_for(imu_read_fail_sleep);
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
                   bool enable_send_log, bool enable_latency_profile) {
  Tools::BindCurrentThreadToAuxCore(2);
  using Clock = Tools::ScanSendController::Clock;
  Tools::ScanSendController scan_send_controller;
  const bool stage3_scan_bounds_auto =
      g_stage3_auto_scan_bounds_controller.IsAutoMode();
  bool has_cached_tick_config = false;
  bool cached_tick_config_stage3_mode = false;
  std::uint64_t cached_tick_config_bounds_version = 0;
  Tools::ScanSendController::TickConfig cached_tick_config;
  const auto imu_send_idle_sleep =
      std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms);

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
        StopScanModeKeepPendingSend();
        scan_send_controller.ExitScanModeAndResumeNextEntry();
        continue;
      }

      const auto now = Clock::now();
      const bool scan_stage3_mode =
          g_scan_stage3_mode.load(std::memory_order_acquire);
      const std::uint64_t bounds_version =
          (scan_stage3_mode && stage3_scan_bounds_auto)
              ? g_stage3_auto_scan_bounds_controller.Version()
              : 0;
      if (!has_cached_tick_config ||
          cached_tick_config_stage3_mode != scan_stage3_mode ||
          cached_tick_config_bounds_version != bounds_version) {
        cached_tick_config = EffectiveScanTickConfig(scan_stage3_mode);
        cached_tick_config_stage3_mode = scan_stage3_mode;
        cached_tick_config_bounds_version = bounds_version;
        has_cached_tick_config = true;
      }
      const auto &tick_config = cached_tick_config;
      scan_send_controller.EnterOrStayScanMode(tick_config, &scan_controller,
                                               &g_scan_controller_mutex, now);

      const auto step_result = scan_send_controller.Step(tick_config, now);
      if (step_result.status ==
          Tools::ScanSendController::StepStatus::WaitForNextSend) {
        WaitUntilNextScanSend(step_result.wait_until);
        continue;
      }

      SerialTask::EulerAngles latest_imu{};
      if (!g_imu_buffer.GetLatest(&latest_imu)) {
        WaitForScanStateChangeFor(imu_send_idle_sleep);
        continue;
      }

      const Tools::ScanCommand scan_command = scan_send_controller.BuildCommand(
          &scan_controller, &g_scan_controller_mutex, latest_imu);

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
        scan_send_controller.ClearResumeNextEntry();
        WaitForScanStateChangeFor(imu_send_idle_sleep);
        continue;
      }
      scan_send_controller.FinishSend(tick_config, &scan_controller,
                                      &g_scan_controller_mutex, now);
      continue;
    }

    scan_send_controller.ExitScanMode();
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
        if (enable_latency_profile) {
          AddSendLatencySample(command, std::chrono::steady_clock::now());
        }
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
