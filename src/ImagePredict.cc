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

#include "CameraTask/ExposureHotkeyController.hpp"
#include "CameraTask/GetImage.hpp"
#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "ImageRecognize/ImagePredictCommandLine.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/TargetAssociation.hpp"
#include "ImageRecognize/TargetClassFilter.hpp"
#include "ImageRecognize/TargetMotionPredictor.hpp"
#include "NetworkTask/AimbotTargetReceiver.hpp"
#include "SerialTask/ImuBuffer.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/CalibrationSliderPanel.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/RuntimeParams.hpp"
#include "Tools/RuntimeStats.hpp"
#include "Tools/SaveImage.hpp"
#include "Tools/ScanController.hpp"

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
static AimbotSendCommand g_pending_send;
static bool g_has_pending_send = false;
static std::atomic<uint8_t> g_aimbot_target{0x00};
static std::atomic<bool> g_send_is_scan{false};
static std::mutex g_scan_controller_mutex;
static std::atomic<bool> g_target_visible{false};
static std::atomic<int> g_aerial_robot_stage{
    ImageRecognize::AerialRobotLaserLockJudge::kInitialStage};
static CameraTask::ExposureHotkeyController g_exposure_controller;
static std::mutex g_serial_mutex;

namespace {
static void RequestStop() {
  g_running.store(false, std::memory_order_release);
  g_frame_cv.notify_all();
}

static void ClearPendingSend() {
  std::lock_guard<std::mutex> lk(g_pending_send_mutex);
  g_send_is_scan.store(false, std::memory_order_release);
  g_has_pending_send = false;
}

static void StorePendingSend(const AimbotSendCommand &command) {
  std::lock_guard<std::mutex> lk(g_pending_send_mutex);
  g_pending_send = command;
  g_send_is_scan.store(false, std::memory_order_release);
  g_has_pending_send = true;
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
  std::lock_guard<std::mutex> lk(g_pending_send_mutex);
  g_has_pending_send = false;
  g_send_is_scan.store(true, std::memory_order_release);
}

static void SendAimbotCommand(serial::Serial &port,
                              const AimbotSendCommand &command) {
  SerialTask::SerialSend(
      port, command.absolute_pitch, command.absolute_yaw, command.offset_pitch,
      command.offset_yaw, command.pitch_velocity, command.yaw_velocity,
      command.aimbot_state, g_aimbot_target.load(std::memory_order_acquire));
}

static void HandleSerialWriteFailure(serial::Serial &port,
                                     const std::exception &e) {
  std::cerr << "Warning: IMU serial write failed, stop sending until restart: "
            << e.what() << std::endl;
  std::lock_guard<std::mutex> lk(g_serial_mutex);
  if (port.isOpen()) {
    port.close();
  }
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
    std::cerr << "Info: IMU serial reconnected." << std::endl;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Warning: IMU serial reconnect failed: " << e.what()
              << std::endl;
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
    std::cerr << "Warning: failed to open IMU serial port, continue without "
                 "serial: "
              << e.what() << std::endl;
    return false;
  }
}

static void JoinIfNeeded(std::thread &thread) {
  if (thread.joinable()) {
    thread.join();
  }
}

static void
PrintPredictSettings(Tools::FilterType filter_type,
                     ImageRecognize::TargetCampMode target_camp_mode,
                     bool enable_display, bool enable_motion_prediction,
                     bool enable_scan_mode, bool enable_save_no_target_images,
                     bool enable_latency_profile,
                     bool enable_calibration_sliders, bool enable_send_log) {
  std::cout << "[角度滤波] 类型: " << Tools::ToString(filter_type) << std::endl;
  std::cout << "[运动预测] 启用: "
            << (enable_motion_prediction ? "true" : "false") << std::endl;
  std::cout << "[跟踪阵营] 模式: " << ImageRecognize::ToString(target_camp_mode)
            << std::endl;
  std::cout << "[显示窗口] 启用: " << (enable_display ? "true" : "false")
            << std::endl;
  std::cout << "[标定滑块] 启用: "
            << (enable_calibration_sliders ? "true" : "false") << std::endl;
  std::cout << "[扫描模式] 启用: " << (enable_scan_mode ? "true" : "false")
            << std::endl;
  std::cout << "[扫描模式] 发送频率: " << Tools::Params().scan_send_hz << " Hz"
            << std::endl;
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
                             std::uint64_t LatencyStats::*bucket,
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

// 等待相机线程交付下一帧；raw_frame 保留给未命中保存逻辑使用。
static bool SnapshotLatestFrame(
    bool has_last_submitted_frame_ts,
    const std::chrono::steady_clock::time_point &last_submitted_frame_ts,
    cv::Mat *frame, cv::Mat *raw_frame,
    std::chrono::steady_clock::time_point *frame_ts, bool keep_raw_frame) {
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
  if (keep_raw_frame) {
    *raw_frame = frame->clone();
  } else {
    raw_frame->release();
  }
  return true;
}

} // namespace

void CaptureThread(CameraTask::GalaxyCamera *camera);
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
    std::cerr << "Failed to initialize OpenVINO model: " << e.what()
              << std::endl;
    std::cerr << "Configured model_path: " << Tools::Params().model_path
              << std::endl;
    std::cerr << "Configured device_name: "
              << Tools::Params().openvino_device_name << std::endl;
    return -2;
  }

  Tools::BindCurrentThreadToAllCores();

  const bool serial_enabled = OpenSerialPort(port);

  std::thread image_capture(CaptureThread, &camera);
  Tools::ScanController scan_controller;
  std::thread image_predict(ImagePredictThread, std::ref(*predictor),
                            std::ref(scan_controller), command_line_options);
  std::thread imu_read;
  std::thread imu_send;
  std::thread aimbot_target_receive;
  if (serial_enabled) {
    // 只有串口可用时才启动云台相关链路。
    aimbot_target_receive = std::thread(AimbotTargetReceiveThread);
    imu_read = std::thread(IMUReadThread, std::ref(port));
    imu_send =
        std::thread(IMUSendThread, std::ref(port), std::ref(scan_controller),
                    command_line_options.enable_send_log);
  } else {
    std::cout << "[IMU] 串口已禁用，仅运行检测/显示。" << std::endl;
  }

  JoinIfNeeded(image_capture);
  JoinIfNeeded(image_predict);
  JoinIfNeeded(imu_read);
  JoinIfNeeded(imu_send);
  JoinIfNeeded(aimbot_target_receive);

  // 只有曝光滑块属于可持久化运行参数；其他标定滑块只改内存，需手动记录。
  g_exposure_controller.SaveRuntimeParams();

  return 0;
}

void CaptureThread(CameraTask::GalaxyCamera *camera) {
  Tools::BindCurrentThreadToAuxCore(0);
  if (!camera->open()) {
    std::cerr << "Failed to open camera." << std::endl;
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
  static ImageRecognize::TargetMotionPredictor target_motion_predictor;
  static ImageRecognize::AerialRobotLaserLockJudge aerial_robot_stage_judge;
  static const Tools::FilterType filter_type =
      Tools::AngleCalculator::ParseFilterType(
          Tools::Params().angle_filter_type);
  static const ImageRecognize::TargetCampMode target_camp_mode =
      ImageRecognize::ParseTargetCampMode(Tools::Params().target_camp_mode);
  const bool enable_display = command_line_options.enable_display;
  const bool enable_calibration_sliders =
      command_line_options.enable_calibration_sliders;
  const bool enable_motion_prediction =
      ResolveOption(command_line_options.enable_motion_prediction,
                    Tools::Params().enable_motion_prediction);
  const bool enable_scan_mode = ResolveOption(
      command_line_options.enable_scan_mode, Tools::Params().enable_scan_mode);
  const bool enable_save_no_target_images =
      ResolveOption(command_line_options.enable_save_no_target_images,
                    Tools::Params().enable_save_no_target_images);
  const bool enable_latency_profile =
      ResolveOption(command_line_options.enable_latency_profile,
                    Tools::Params().enable_latency_profile);

  PrintPredictSettings(
      filter_type, target_camp_mode, enable_display, enable_motion_prediction,
      enable_scan_mode, enable_save_no_target_images, enable_latency_profile,
      enable_calibration_sliders, command_line_options.enable_send_log);
  static std::unique_ptr<Tools::SaveImageOnNoTarget> no_target_saver;
  if (enable_save_no_target_images && !no_target_saver) {
    no_target_saver =
        std::make_unique<Tools::SaveImageOnNoTarget>(5, "captures");
  }

  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;
  LatencyStats latency_total;
  LatencyStats latency_window;
  PixelHeightStats pixel_height_stats;
  std::uint64_t ui_frame_counter = 0;
  const auto scan_trigger_delay = std::chrono::milliseconds(500);
  std::chrono::steady_clock::time_point target_lost_since{};
  bool target_lost_since_initialized = false;
  bool infer_inflight = false;
  bool has_last_submitted_frame_ts = false;
  cv::Mat inflight_frame;
  cv::Mat inflight_raw_frame;
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
      cv::Mat next_raw_frame;
      std::chrono::steady_clock::time_point next_frame_ts{};
      if (!SnapshotLatestFrame(has_last_submitted_frame_ts, inflight_frame_ts,
                               &next_frame, &next_raw_frame, &next_frame_ts,
                               enable_save_no_target_images)) {
        continue;
      }

      try {
        inflight_infer_start = ProfileNow(enable_latency_profile);
        active_predictor->startAsync(next_frame);
        inflight_frame = std::move(next_frame);
        inflight_raw_frame = std::move(next_raw_frame);
        inflight_frame_ts = next_frame_ts;
        infer_inflight = true;
        has_last_submitted_frame_ts = true;
        if (infer_submit_interval !=
            std::chrono::steady_clock::duration::zero()) {
          next_infer_submit_time =
              std::chrono::steady_clock::now() + infer_submit_interval;
        }
      } catch (const std::exception &e) {
        std::cerr << "ImagePredictThread async submit exception: " << e.what()
                  << std::endl;
      }
      continue;
    }

    while (g_running && !active_predictor->isAsyncReady()) {
      std::unique_lock<std::mutex> lk(g_frame_mutex);
      g_frame_cv.wait_for(lk, std::chrono::milliseconds(1),
                          [] { return !g_running; });
    }
    if (!g_running) {
      break;
    }

    const auto t_loop_start = ProfileNow(enable_latency_profile);
    cv::Mat frame = enable_display ? inflight_frame.clone() : inflight_frame;
    cv::Mat raw_frame = inflight_raw_frame;
    const auto frame_ts = inflight_frame_ts;

    ImageRecognize::PredictResult result;
    SerialTask::EulerAngles matched_imu{};
    bool has_matched_imu = false;
    double frame_dt = 0.0;
    double stage_dt =
        ImageRecognize::AerialRobotLaserLockJudge::kDefaultDeltaSeconds;
    bool has_realtime_frame_dt = false;
    bool has_predict_result = false;
    try {
      result = active_predictor->getAsyncResult();
      if (using_stage3_predictor) {
        NormalizeStage3PredictResult(&result);
      }
      has_predict_result = true;
      const auto t_infer_end = ProfileNow(enable_latency_profile);
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
      has_matched_imu = g_imu_buffer.MatchForFrame(frame_ts, &matched_imu);
      const auto t_imu_match_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::imu_match_ns, t_imu_match_start,
                       t_imu_match_end);
    } catch (const std::exception &e) {
      infer_inflight = false;
      std::cerr << "ImagePredictThread exception: " << e.what() << std::endl;
    }

    if (has_predict_result &&
        g_aerial_robot_stage.load(std::memory_order_acquire) <
            ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
      const int previous_stage =
          g_aerial_robot_stage.load(std::memory_order_acquire);
      const int laser_judge_class_id = ResolveLaserJudgeClassId(result);
      const int stage =
          aerial_robot_stage_judge.Update(laser_judge_class_id, stage_dt);
      g_aerial_robot_stage.store(stage, std::memory_order_release);
      if (stage >= ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage &&
          previous_stage <
              ImageRecognize::AerialRobotLaserLockJudge::kFinishedStage) {
        pending_stage3_switch = true;
        std::cout << "[AerialRobotStage] stage=3, wait target lost before "
                     "switching model"
                  << std::endl;
      }
    }

    std::array<float, 6> tracked_box{};
    bool has_tracked_box = false;
    const auto track_boxes =
        ImageRecognize::FilterTrackBoxes(result.boxes, target_camp_mode);
    const auto t_select_start = ProfileNow(enable_latency_profile);
    const auto track_result = target_tracker.Update(track_boxes);
    const auto t_select_end = ProfileNow(enable_latency_profile);
    const bool track_alive =
        track_result.has_box || target_tracker.HasRecentLock();
    const auto now = std::chrono::steady_clock::now();
    g_target_visible.store(track_alive, std::memory_order_release);

    if (track_alive) {
      target_lost_since_initialized = false;
    } else if (!target_lost_since_initialized) {
      target_lost_since = now;
      target_lost_since_initialized = true;
    }

    if (pending_stage3_switch && !using_stage3_predictor && !track_alive) {
      try {
        g_exposure_controller.SetActiveMode(
            CameraTask::ExposureHotkeyController::ExposureMode::Stage3);
        stage3_predictor = std::make_unique<ImageRecognize::ImagePredict>(
            Tools::Params().stage3_model_path,
            Tools::Params().openvino_device_name);
        active_predictor = stage3_predictor.get();
        using_stage3_predictor = true;
        pending_stage3_switch = false;
        target_tracker.Reset();
        target_motion_predictor.Reset();
        ClearPendingSend();
        std::cout << "[AerialRobotStage] switched to stage3 model="
                  << Tools::Params().stage3_model_path
                  << " exposure_us=" << Tools::Params().stage3_exposure_time_us
                  << std::endl;
      } catch (const std::exception &e) {
        std::cerr << "Failed to switch to stage3 model: " << e.what()
                  << std::endl;
        RequestStop();
      }
      continue;
    }

    if (track_result.has_box) {
      if (enable_motion_prediction) {
        const auto t_motion_predict_start = ProfileNow(enable_latency_profile);
        const auto motion_prediction =
            target_motion_predictor.ObserveAndPredict(track_result.box,
                                                      frame_dt, frame.size());
        const auto t_motion_predict_end = ProfileNow(enable_latency_profile);
        AddLatencySample(enable_latency_profile, latency_total, latency_window,
                         &LatencyStats::motion_predict_ns,
                         t_motion_predict_start, t_motion_predict_end);
        tracked_box =
            motion_prediction.valid ? motion_prediction.box : track_result.box;
      } else {
        target_motion_predictor.Reset();
        tracked_box = track_result.box;
      }
      has_tracked_box = true;
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::select_box_ns, t_select_start,
                       t_select_end);
    } else if (enable_motion_prediction && track_alive &&
               target_motion_predictor.HasState()) {
      const auto t_motion_predict_start = ProfileNow(enable_latency_profile);
      const auto motion_prediction =
          target_motion_predictor.Predict(frame_dt, frame.size());
      const auto t_motion_predict_end = ProfileNow(enable_latency_profile);
      AddLatencySample(enable_latency_profile, latency_total, latency_window,
                       &LatencyStats::motion_predict_ns, t_motion_predict_start,
                       t_motion_predict_end);
      if (motion_prediction.valid) {
        tracked_box = motion_prediction.box;
        has_tracked_box = true;
      }
    } else {
      target_motion_predictor.Reset();
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
      const float height = tracked_box[3] - tracked_box[1];
      pixel_height_stats.Add(height);
      const float distance = distance_calculator.CalculateDistance(height);

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

          StorePendingSend(AimbotSendCommand{
              send_abs_pitch, send_abs_yaw, cmd_delta_pitch, cmd_delta_yaw,
              pitch_velocity, yaw_velocity, 0x01});
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
    fps_counter.tick();
    double fps = fps_counter.get();

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
      no_target_saver->Update(raw_frame, result.boxes.size() != 1);
    }

    // 处理 GUI 事件并允许按键退出
    const bool should_exit =
        do_gui_poll && g_exposure_controller.HandleGuiKey(
                           ImageRecognize::ImageShow::PollKey());
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
      latency_window = LatencyStats{};
    }
  }

  if (enable_latency_profile) {
    if (latency_window.frames > 0)
      PrintLatencyStats(latency_window, "窗口尾");
    PrintLatencyStats(latency_total, "总计");
  }
  PrintPixelHeightStats(pixel_height_stats);
}

void IMUReadThread(serial::Serial &port) {
  Tools::BindCurrentThreadToAuxCore(1);
  while (g_running) {
    try {
      SerialTask::EulerAngles angles;
      bool read_ok = false;
      {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        read_ok = SerialTask::ReadIMUData(port, angles);
      }
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
      std::cerr << "Warning: IMU serial read failed: " << e.what() << std::endl;
      {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        if (port.isOpen()) {
          port.close();
        }
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Tools::Params().imu_read_fail_sleep_ms));
    }
  }
  if (port.isOpen())
    port.close();
}

void IMUSendThread(serial::Serial &port, Tools::ScanController &scan_controller,
                   bool enable_send_log) {
  Tools::BindCurrentThreadToAuxCore(2);
  using Clock = std::chrono::steady_clock;
  const double scan_send_hz = std::max(1.0, Tools::Params().scan_send_hz);
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
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Tools::Params().imu_read_fail_sleep_ms));
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
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
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
        std::this_thread::sleep_for(next_scan_send_time - now);
        continue;
      }

      SerialTask::EulerAngles latest_imu{};
      if (!g_imu_buffer.GetLatest(&latest_imu)) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
        continue;
      }

      float gimbal_pitch_velocity = 0.0f;
      float gimbal_yaw_velocity = 0.0f;
      g_imu_buffer.GetLatestVelocity(&gimbal_pitch_velocity,
                                     &gimbal_yaw_velocity);

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
        SendAimbotCommand(
            port, AimbotSendCommand{scan_command.absolute_pitch_deg,
                                    scan_command.absolute_yaw_deg,
                                    scan_command.offset_pitch_deg,
                                    scan_command.offset_yaw_deg,
                                    gimbal_pitch_velocity, gimbal_yaw_velocity,
                                    scan_command.aimbot_state});
      } catch (const std::exception &e) {
        HandleSerialWriteFailure(port, e);
        std::this_thread::sleep_for(
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
        std::cout << std::fixed << " offset_yaw: " << command.offset_yaw
                  << "°, offset_pitch: " << command.offset_pitch << "°"
                  << std::endl;
      }
      try {
        std::lock_guard<std::mutex> lk(g_serial_mutex);
        SendAimbotCommand(port, command);
      } catch (const std::exception &e) {
        HandleSerialWriteFailure(port, e);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
        continue;
      }
    } else {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Tools::Params().imu_send_idle_sleep_ms));
    }
  }
}

void AimbotTargetReceiveThread() {
  Tools::BindCurrentThreadToAuxCore(3);
  NetworkTask::RunAimbotTargetReceiver(g_aimbot_target, []() {
    return g_running.load(std::memory_order_acquire);
  });
}
