#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <vector>

#include "CameraTask/GetImage.hpp"
#include "ImageRecognize/ImagePredictCommandLine.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/TargetAssociation.hpp"
#include "ImageRecognize/TargetMotionPredictor.hpp"
#include "NetworkTask/DeviceAServer.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/SaveImage.hpp"
#include "Tools/ScanController.hpp"

namespace {
enum class TargetCampMode {
  RedAndPurple,
  BlueAndPurple,
  All,
};

struct RuntimeParams {
  std::string model_path;
  std::string openvino_device_name;
  std::string angle_filter_type;
  std::string target_camp_mode;

  int capture_timeout_ms;
  int capture_empty_sleep_ms;
  int imu_read_fail_sleep_ms;
  int imu_send_idle_sleep_ms;
  int imu_buffer_max_age_ms;

  float minimum_angle_deg;
  float max_send_delta_deg;

  double dt_default_sec;
  double dt_max_sec;

  float pitch_abs_limit;

  bool enable_latency_profile;
  int latency_print_interval_frames;

  bool enable_display;
  bool enable_motion_prediction;
  bool enable_scan_mode;
  bool enable_save_no_target_images;

  int scan_origin_hold_ms;
  double max_infer_fps;
  double scan_send_hz;
  int display_every_n_frames;
  int gui_poll_every_n_frames;
};

struct AimbotSendCommand {
  float absolute_pitch = 0.0f;
  float absolute_yaw = 0.0f;
  float offset_pitch = 0.0f;
  float offset_yaw = 0.0f;
  float pitch_velocity = 0.0f;
  float yaw_velocity = 0.0f;
  uint8_t aimbot_state = 0x00;
};

const RuntimeParams &Params();

TargetCampMode ParseTargetCampMode(const std::string &mode);
const char *ToString(TargetCampMode mode);
bool ParseAimbotTargetMessage(const std::string &message, uint8_t &target);
bool ShouldTrackClassId(int class_id, TargetCampMode mode);
std::vector<std::array<float, 6>>
FilterTrackBoxes(const std::vector<std::array<float, 6>> &boxes,
                 TargetCampMode mode);
} // namespace

std::atomic<bool> g_running(true); // 全局运行标志
static std::mutex g_frame_mutex;   // 保护最新帧的互斥锁
static std::condition_variable g_frame_cv; // 通知预测线程有新帧到达的条件变量
                                           // IMU 数据缓冲区
static std::deque<
    std::pair<std::chrono::steady_clock::time_point, SerialTask::EulerAngles>>
    g_imu_buffer;
static std::mutex g_imu_mutex; // 保护 IMU 缓冲区的互斥锁

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

namespace {
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

static bool OpenSerialPort(serial::Serial &port) {
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

static void PrintPredictSettings(Tools::FilterType filter_type,
                                 TargetCampMode target_camp_mode,
                                 bool enable_display,
                                 bool enable_motion_prediction) {
  std::cout << "[角度滤波] 类型: " << Tools::ToString(filter_type) << std::endl;
  std::cout << "[运动预测] 启用: "
            << (enable_motion_prediction ? "true" : "false") << std::endl;
  std::cout << "[跟踪阵营] 模式: " << ToString(target_camp_mode) << std::endl;
  std::cout << "[显示窗口] 启用: " << (enable_display ? "true" : "false")
            << std::endl;
  std::cout << "[扫描模式] 启用: "
            << (Params().enable_scan_mode ? "true" : "false") << std::endl;
  std::cout << "[扫描模式] 发送频率: " << Params().scan_send_hz << " Hz"
            << std::endl;
}

// 等待相机线程交付下一帧；raw_frame 保留给未命中保存逻辑使用。
static bool SnapshotLatestFrame(
    bool has_last_submitted_frame_ts,
    const std::chrono::steady_clock::time_point &last_submitted_frame_ts,
    cv::Mat *frame, cv::Mat *raw_frame,
    std::chrono::steady_clock::time_point *frame_ts) {
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
  *raw_frame = frame->clone();
  return true;
}

struct LatencyStats {
  std::uint64_t frames = 0;
  std::uint64_t infer_ns = 0;
  std::uint64_t imu_match_ns = 0;
  std::uint64_t select_box_ns = 0;
  std::uint64_t motion_predict_ns = 0;
  std::uint64_t angle_calc_ns = 0;
  std::uint64_t control_calc_ns = 0;
  std::uint64_t render_ns = 0;
  std::uint64_t loop_ns = 0;

  void Add(std::uint64_t &bucket,
           const std::chrono::steady_clock::time_point &t0,
           const std::chrono::steady_clock::time_point &t1) {
    bucket += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  }

  void AddFrame() { ++frames; }
};

static void PrintLatencyStats(const LatencyStats &s, const char *tag) {
  if (s.frames == 0)
    return;
  const double inv = 1.0 / static_cast<double>(s.frames);
  auto ns2ms = [&](std::uint64_t ns) {
    return static_cast<double>(ns) * inv / 1e6;
  };
  auto ns2us = [&](std::uint64_t ns) {
    return static_cast<double>(ns) * inv / 1e3;
  };
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[延迟][" << tag << "] 帧数=" << s.frames << " 平均毫秒"
            << " 推理=" << ns2ms(s.infer_ns)
            << " IMU匹配=" << ns2ms(s.imu_match_ns)
            << " 选框=" << ns2ms(s.select_box_ns)
            << " 运动预测=" << ns2ms(s.motion_predict_ns)
            << " 角度=" << ns2ms(s.angle_calc_ns)
            << " 控制=" << ns2ms(s.control_calc_ns)
            << " 渲染=" << ns2ms(s.render_ns) << " 循环=" << ns2ms(s.loop_ns)
            << " | 微秒 IMU匹配=" << ns2us(s.imu_match_ns)
            << " 选框=" << ns2us(s.select_box_ns)
            << " 运动预测=" << ns2us(s.motion_predict_ns)
            << " 控制=" << ns2us(s.control_calc_ns) << std::endl;
}

static SerialTask::EulerAngles
InterpolateEulerAngles(const SerialTask::EulerAngles &lower,
                       const SerialTask::EulerAngles &upper, float alpha) {
  SerialTask::EulerAngles result{};
  result.roll = Tools::InterpolateAngleDeg(lower.roll, upper.roll, alpha);
  result.pitch = Tools::InterpolateAngleDeg(lower.pitch, upper.pitch, alpha);
  result.yaw = Tools::InterpolateAngleDeg(lower.yaw, upper.yaw, alpha);
  return result;
}

// 用图像帧时间戳匹配 IMU，必要时在相邻两帧 IMU 之间插值。
static bool
MatchImuForFrame(const std::chrono::steady_clock::time_point &frame_ts,
                 SerialTask::EulerAngles *matched_imu) {
  std::lock_guard<std::mutex> lk(g_imu_mutex);
  if (g_imu_buffer.empty()) {
    return false;
  }

  auto upper_it = std::lower_bound(
      g_imu_buffer.begin(), g_imu_buffer.end(), frame_ts,
      [](const auto &entry, const auto &ts) { return entry.first < ts; });

  if (g_imu_buffer.size() == 1 || upper_it == g_imu_buffer.begin()) {
    *matched_imu = g_imu_buffer.front().second;
    return true;
  }
  if (upper_it == g_imu_buffer.end()) {
    *matched_imu = g_imu_buffer.back().second;
    return true;
  }

  const auto lower_it = std::prev(upper_it);
  const auto span_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           upper_it->first - lower_it->first)
                           .count();
  if (span_ns <= 0) {
    *matched_imu = upper_it->second;
    return true;
  }

  const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              frame_ts - lower_it->first)
                              .count();
  const float alpha =
      static_cast<float>(elapsed_ns) / static_cast<float>(span_ns);
  *matched_imu =
      InterpolateEulerAngles(lower_it->second, upper_it->second, alpha);
  return true;
}

static bool GetLatestImu(SerialTask::EulerAngles *latest_imu) {
  std::lock_guard<std::mutex> lk(g_imu_mutex);
  if (g_imu_buffer.empty()) {
    return false;
  }

  *latest_imu = g_imu_buffer.back().second;
  return true;
}

} // namespace

void CaptureThread(CameraTask::GalaxyCamera *camera);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor,
                        Tools::ScanController &scan_controller,
                        bool enable_display);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port,
                   Tools::ScanController &scan_controller);
void AimbotTargetReceiveThread();

int main(int argc, char **argv) {
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  std::unique_ptr<ImageRecognize::ImagePredict> predictor;
  const auto command_line_options =
      ImageRecognize::ParseImagePredictCommandLine(argc, argv);

  cv::setUseOptimized(true);
  cv::setNumThreads(1);

  Tools::BindCurrentThreadToBigCores();

  try {
    predictor = std::make_unique<ImageRecognize::ImagePredict>(
        Params().model_path, Params().openvino_device_name);
  } catch (const std::exception &e) {
    std::cerr << "Failed to initialize OpenVINO model: " << e.what()
              << std::endl;
    std::cerr << "Configured model_path: " << Params().model_path << std::endl;
    std::cerr << "Configured device_name: " << Params().openvino_device_name
              << std::endl;
    return -2;
  }

  Tools::BindCurrentThreadToAllCores();

  const bool serial_enabled = OpenSerialPort(port);

  std::thread image_capture(CaptureThread, &camera);
  Tools::ScanController scan_controller;
  std::thread image_predict(ImagePredictThread, std::ref(*predictor),
                            std::ref(scan_controller),
                            command_line_options.enable_display);
  std::thread imu_read;
  std::thread imu_send;
  std::thread aimbot_target_receive;
  if (serial_enabled) {
    // 只有串口可用时才启动云台相关链路。
    aimbot_target_receive = std::thread(AimbotTargetReceiveThread);
    imu_read = std::thread(IMUReadThread, std::ref(port));
    imu_send =
        std::thread(IMUSendThread, std::ref(port), std::ref(scan_controller));
  } else {
    std::cout << "[IMU] 串口已禁用，仅运行检测/显示。" << std::endl;
  }

  JoinIfNeeded(image_capture);
  JoinIfNeeded(image_predict);
  JoinIfNeeded(imu_read);
  JoinIfNeeded(imu_send);
  JoinIfNeeded(aimbot_target_receive);

  return 0;
}

void CaptureThread(CameraTask::GalaxyCamera *camera) {
  Tools::BindCurrentThreadToAuxCore(0);
  if (!camera->open()) {
    std::cerr << "Failed to open camera." << std::endl;
    g_running = false;
    return;
  }
  while (g_running) {
    cv::Mat frame = camera->grab(Params().capture_timeout_ms);
    if (frame.empty()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Params().capture_empty_sleep_ms));
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
                        bool enable_display) {
  Tools::BindCurrentThreadToBigCores();
  FPSCounter fps_counter;
  static Tools::AngleCalculator
      angle_calculator; // 持久化 AngleCalculator，避免每次调用时重置 lastTime
  static Tools::LaserAngleCalculator laser_angle_calculator;
  static Tools::DistanceCalculator distance_calculator;
  static ImageRecognize::CrossFrameTargetTracker target_tracker;
  static ImageRecognize::TargetMotionPredictor target_motion_predictor;
  static const Tools::FilterType filter_type =
      Tools::AngleCalculator::ParseFilterType(Params().angle_filter_type);
  static const TargetCampMode target_camp_mode =
      ParseTargetCampMode(Params().target_camp_mode);
  const bool enable_motion_prediction = Params().enable_motion_prediction;

  PrintPredictSettings(filter_type, target_camp_mode, enable_display,
                       enable_motion_prediction);
  static std::unique_ptr<Tools::SaveImageOnNoTarget> no_target_saver;
  if (Params().enable_save_no_target_images && !no_target_saver) {
    no_target_saver =
        std::make_unique<Tools::SaveImageOnNoTarget>(5, "captures");
  }

  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;
  LatencyStats latency_total;
  LatencyStats latency_window;
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
  const double max_infer_fps = Params().max_infer_fps;
  const auto infer_submit_interval =
      max_infer_fps > 0.0
          ? std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / max_infer_fps))
          : std::chrono::steady_clock::duration::zero();
  std::chrono::steady_clock::time_point next_infer_submit_time =
      std::chrono::steady_clock::now();

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
                               &next_frame, &next_raw_frame, &next_frame_ts)) {
        continue;
      }

      try {
        inflight_infer_start = std::chrono::steady_clock::now();
        predictor.startAsync(next_frame);
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

    while (g_running && !predictor.isAsyncReady()) {
      std::unique_lock<std::mutex> lk(g_frame_mutex);
      g_frame_cv.wait_for(lk, std::chrono::milliseconds(1),
                          [] { return !g_running; });
    }
    if (!g_running) {
      break;
    }

    const auto t_loop_start = std::chrono::steady_clock::now();
    cv::Mat frame = inflight_frame.clone();
    cv::Mat raw_frame = inflight_raw_frame;
    const auto frame_ts = inflight_frame_ts;

    ImageRecognize::PredictResult result;
    SerialTask::EulerAngles matched_imu{};
    bool has_matched_imu = false;
    double frame_dt = 0.0;
    bool has_realtime_frame_dt = false;
    try {
      result = predictor.getAsyncResult();
      const auto t_infer_end = std::chrono::steady_clock::now();
      infer_inflight = false;
      latency_total.Add(latency_total.infer_ns, inflight_infer_start,
                        t_infer_end);
      latency_window.Add(latency_window.infer_ns, inflight_infer_start,
                         t_infer_end);

      if (has_prev_frame_ts) {
        frame_dt =
            std::chrono::duration<double>(frame_ts - prev_frame_ts).count();
        has_realtime_frame_dt = true;
      }
      prev_frame_ts = frame_ts;
      has_prev_frame_ts = true;
      if (!has_realtime_frame_dt || frame_dt <= 0.0 ||
          frame_dt > Params().dt_max_sec)
        frame_dt = 0.0;

      // 关联最近一次 IMU 状态并记录延迟
      const auto t_imu_match_start = std::chrono::steady_clock::now();
      has_matched_imu = MatchImuForFrame(frame_ts, &matched_imu);
      const auto t_imu_match_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.imu_match_ns, t_imu_match_start,
                        t_imu_match_end);
      latency_window.Add(latency_window.imu_match_ns, t_imu_match_start,
                         t_imu_match_end);
    } catch (const std::exception &e) {
      infer_inflight = false;
      std::cerr << "ImagePredictThread exception: " << e.what() << std::endl;
    }

    std::array<float, 6> tracked_box{};
    bool has_tracked_box = false;
    const auto track_boxes = FilterTrackBoxes(result.boxes, target_camp_mode);
    const auto t_select_start = std::chrono::steady_clock::now();
    const auto track_result = target_tracker.Update(track_boxes);
    const auto t_select_end = std::chrono::steady_clock::now();
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

    if (track_result.has_box) {
      if (enable_motion_prediction) {
        const auto t_motion_predict_start = std::chrono::steady_clock::now();
        const auto motion_prediction =
            target_motion_predictor.ObserveAndPredict(track_result.box,
                                                      frame_dt, frame.size());
        const auto t_motion_predict_end = std::chrono::steady_clock::now();
        latency_total.Add(latency_total.motion_predict_ns,
                          t_motion_predict_start, t_motion_predict_end);
        latency_window.Add(latency_window.motion_predict_ns,
                           t_motion_predict_start, t_motion_predict_end);
        tracked_box =
            motion_prediction.valid ? motion_prediction.box : track_result.box;
      } else {
        target_motion_predictor.Reset();
        tracked_box = track_result.box;
      }
      has_tracked_box = true;
      latency_total.Add(latency_total.select_box_ns, t_select_start,
                        t_select_end);
      latency_window.Add(latency_window.select_box_ns, t_select_start,
                         t_select_end);
    } else if (enable_motion_prediction && track_alive &&
               target_motion_predictor.HasState()) {
      const auto t_motion_predict_start = std::chrono::steady_clock::now();
      const auto motion_prediction =
          target_motion_predictor.Predict(frame_dt, frame.size());
      const auto t_motion_predict_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.motion_predict_ns, t_motion_predict_start,
                        t_motion_predict_end);
      latency_window.Add(latency_window.motion_predict_ns,
                         t_motion_predict_start, t_motion_predict_end);
      if (motion_prediction.valid) {
        tracked_box = motion_prediction.box;
        has_tracked_box = true;
      }
    } else {
      target_motion_predictor.Reset();
    }

    if (has_tracked_box) {
      if (track_result.has_box) {
        const cv::Point2f detection_center =
            ImageRecognize::BoxCenter(track_result.box);
        ImageRecognize::ImageShow::ShowDetectionCenter(
            frame, detection_center.x, detection_center.y);
      }

      const cv::Point2f tracked_center = ImageRecognize::BoxCenter(tracked_box);
      ImageRecognize::ImageShow::ShowPred(frame, tracked_center.x,
                                          tracked_center.y);

      const float center_x = tracked_center.x;
      const float center_y = tracked_center.y;
      const float width = tracked_box[2] - tracked_box[0];
      const float height = tracked_box[3] - tracked_box[1];
      const float distance =
          distance_calculator.CalculateDistance(height, width);

      if (has_matched_imu) {
        const auto t_angle_start = std::chrono::steady_clock::now();
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
        const auto t_angle_end = std::chrono::steady_clock::now();
        latency_total.Add(latency_total.angle_calc_ns, t_angle_start,
                          t_angle_end);
        latency_window.Add(latency_window.angle_calc_ns, t_angle_start,
                           t_angle_end);

        // 必须用最短角差，否则跨越 ±180° 时会出现 300° 级突变。
        const float offset_yaw_angle =
            Tools::NormalizeDeltaDeg(filtered_yaw - matched_imu.yaw);
        const float offset_pitch_angle =
            Tools::NormalizeDeltaDeg(filtered_pitch - matched_imu.pitch);

        auto [laser_yaw_angle, laser_pitch_angle] =
            laser_angle_calculator.CalculateLaserAngles(
                distance, offset_yaw_angle, offset_pitch_angle);

        const auto t_control_start = std::chrono::steady_clock::now();
        float delta_yaw_raw = Tools::NormalizeDeltaDeg(
            static_cast<float>(offset_yaw_angle + laser_yaw_angle));
        float delta_pitch_raw = Tools::NormalizeDeltaDeg(
            static_cast<float>(offset_pitch_angle + laser_pitch_angle));
        if (std::abs(delta_yaw_raw) > Params().minimum_angle_deg ||
            std::abs(delta_pitch_raw) > Params().minimum_angle_deg) {
          const float cmd_delta_yaw =
              std::clamp(delta_yaw_raw, -Params().max_send_delta_deg,
                         Params().max_send_delta_deg);
          const float cmd_delta_pitch =
              std::clamp(delta_pitch_raw, -Params().pitch_abs_limit,
                         Params().pitch_abs_limit);

          const float send_abs_yaw = matched_imu.yaw + cmd_delta_yaw;
          const float send_abs_pitch = matched_imu.pitch + cmd_delta_pitch;

          StorePendingSend(AimbotSendCommand{
              send_abs_pitch, send_abs_yaw, cmd_delta_pitch, cmd_delta_yaw,
              pitch_velocity, yaw_velocity, 0x01});
          ImageRecognize::ImageShow::ShowAngles(
              frame, send_abs_yaw, send_abs_pitch, matched_imu.yaw,
              matched_imu.pitch, cmd_delta_yaw, cmd_delta_pitch, distance);
        } else {
          ClearPendingSend();
        }

        {
          std::lock_guard<std::mutex> lk(g_scan_controller_mutex);
          scan_controller.Reset();
        }
        const auto t_control_end = std::chrono::steady_clock::now();
        latency_total.Add(latency_total.control_calc_ns, t_control_start,
                          t_control_end);
        latency_window.Add(latency_window.control_calc_ns, t_control_start,
                           t_control_end);
      } else {
        ClearPendingSend();
      }
    } else {
      if (Params().enable_scan_mode && has_matched_imu && !track_alive &&
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
    const auto t_render_start = std::chrono::steady_clock::now();
    const bool do_display =
        enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(
                                std::max(1, Params().display_every_n_frames)) ==
         0);
    const bool do_gui_poll =
        enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(std::max(
                                1, Params().gui_poll_every_n_frames)) ==
         0);
    ++ui_frame_counter;

    if (do_display) {
      ImageRecognize::ImageShow::ShowNow(frame, result, fps);
      if (has_tracked_box) {
        ImageRecognize::DrawTrackedBox(frame, tracked_box);
      }
    }

    // 当检测框数量不是 1 个时，按间隔保存画框前原图
    if (Params().enable_save_no_target_images && no_target_saver) {
      no_target_saver->Update(raw_frame, result.boxes.size() != 1);
    }

    // 处理 GUI 事件并允许按键退出
    if (do_gui_poll && ImageRecognize::ImageShow::WaitForExit()) {
      g_running = false;
      const auto t_render_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.render_ns, t_render_start, t_render_end);
      latency_window.Add(latency_window.render_ns, t_render_start,
                         t_render_end);
      const auto t_loop_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.loop_ns, t_loop_start, t_loop_end);
      latency_window.Add(latency_window.loop_ns, t_loop_start, t_loop_end);
      latency_total.AddFrame();
      latency_window.AddFrame();
      break;
    }

    const auto t_render_end = std::chrono::steady_clock::now();
    latency_total.Add(latency_total.render_ns, t_render_start, t_render_end);
    latency_window.Add(latency_window.render_ns, t_render_start, t_render_end);

    const auto t_loop_end = std::chrono::steady_clock::now();
    latency_total.Add(latency_total.loop_ns, t_loop_start, t_loop_end);
    latency_window.Add(latency_window.loop_ns, t_loop_start, t_loop_end);
    latency_total.AddFrame();
    latency_window.AddFrame();

    if (Params().enable_latency_profile &&
        latency_window.frames >= static_cast<std::uint64_t>(
                                     Params().latency_print_interval_frames)) {
      PrintLatencyStats(latency_window, "窗口");
      latency_window = LatencyStats{};
    }
  }

  if (Params().enable_latency_profile) {
    if (latency_window.frames > 0)
      PrintLatencyStats(latency_window, "窗口尾");
    PrintLatencyStats(latency_total, "总计");
  }
}

void IMUReadThread(serial::Serial &port) {
  Tools::BindCurrentThreadToAuxCore(1);
  while (g_running) {
    SerialTask::EulerAngles angles;
    if (SerialTask::ReadIMUData(port, angles)) {
      auto ts = std::chrono::steady_clock::now();

      std::lock_guard<std::mutex> lk(g_imu_mutex);
      // 添加到缓冲区末尾
      g_imu_buffer.emplace_back(ts, angles);

      // 裁剪过旧的 IMU 条目，保持缓冲区只包含最近一段时间的数据
      while (!g_imu_buffer.empty() &&
             (ts - g_imu_buffer.front().first) >
                 std::chrono::milliseconds(Params().imu_buffer_max_age_ms)) {
        g_imu_buffer.pop_front();
      }
    } else {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Params().imu_read_fail_sleep_ms));
    }
  }
  if (port.isOpen())
    port.close();
}

void IMUSendThread(serial::Serial &port,
                   Tools::ScanController &scan_controller) {
  Tools::BindCurrentThreadToAuxCore(2);
  using Clock = std::chrono::steady_clock;
  const double scan_send_hz = std::max(1.0, Params().scan_send_hz);
  const auto scan_send_interval = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / scan_send_hz));
  const auto scan_origin_hold_duration =
      std::chrono::milliseconds(std::max(0, Params().scan_origin_hold_ms));

  auto next_scan_send_time = Clock::now();
  bool last_scan_mode = false;
  bool scan_waiting_at_origin = false;
  Clock::time_point scan_origin_deadline = Clock::now();

  while (g_running) {
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
            std::chrono::milliseconds(Params().imu_send_idle_sleep_ms));
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
      if (!GetLatestImu(&latest_imu)) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Params().imu_send_idle_sleep_ms));
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

      SendAimbotCommand(port,
                        AimbotSendCommand{scan_command.absolute_pitch_deg,
                                          scan_command.absolute_yaw_deg,
                                          scan_command.offset_pitch_deg,
                                          scan_command.offset_yaw_deg, 0.0f,
                                          0.0f, scan_command.aimbot_state});
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
      std::cout << std::fixed << " offset_yaw: " << command.offset_yaw
                << "°, offset_pitch: " << command.offset_pitch << "°"
                << std::endl;
      SendAimbotCommand(port, command);
    } else {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Params().imu_send_idle_sleep_ms));
    }
  }
}

void AimbotTargetReceiveThread() {
  Tools::BindCurrentThreadToAuxCore(3);

  NetworkTask::socket_t listen_fd = NetworkTask::kInvalidSocketFd;
  if (!NetworkTask::CreateListeningSocket(listen_fd)) {
    std::cerr << "[Network] AimbotTarget 监听端口 5000 失败" << std::endl;
    return;
  }

  std::cout << "[Network] AimbotTarget 接收端已启动，监听端口：5000"
            << std::endl;

  while (g_running) {
    NetworkTask::socket_t client_fd = NetworkTask::kInvalidSocketFd;
    std::string client_ip;
    while (g_running) {
      // 超时轮询避免程序退出时卡在 accept/recv。
      if (!NetworkTask::WaitForReadable(listen_fd, 100)) {
        continue;
      }
      if (NetworkTask::AcceptClient(listen_fd, client_fd, &client_ip)) {
        break;
      }
    }

    if (!g_running || client_fd == NetworkTask::kInvalidSocketFd) {
      break;
    }

    std::cout << "[Network] AimbotTarget 发送端已连接，IP：" << client_ip
              << std::endl;

    while (g_running) {
      if (!NetworkTask::WaitForReadable(client_fd, 100)) {
        continue;
      }

      std::string received_content;
      if (!NetworkTask::ReceiveText(client_fd, received_content)) {
        std::cout << "[Network] AimbotTarget 发送端已断开" << std::endl;
        break;
      }

      uint8_t target = 0x00;
      if (!ParseAimbotTargetMessage(received_content, target)) {
        std::cerr << "[Network] 忽略非法 AimbotTarget 数据" << std::endl;
        continue;
      }

      g_aimbot_target.store(target, std::memory_order_release);
      std::cout << "[Network] AimbotTarget=" << static_cast<int>(target)
                << std::endl;
    }

    NetworkTask::CloseSocket(client_fd);
  }

  NetworkTask::CloseSocket(listen_fd);
}

namespace {
TargetCampMode ParseTargetCampMode(const std::string &mode) {
  std::string normalized = mode;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  if (normalized == "RED" || normalized == "RED_AND_PURPLE") {
    return TargetCampMode::RedAndPurple;
  }
  if (normalized == "BLUE" || normalized == "BLUE_AND_PURPLE") {
    return TargetCampMode::BlueAndPurple;
  }
  if (normalized == "ALL") {
    return TargetCampMode::All;
  }
  return TargetCampMode::RedAndPurple;
}

const char *ToString(TargetCampMode mode) {
  switch (mode) {
  case TargetCampMode::RedAndPurple:
    return "RED_AND_PURPLE";
  case TargetCampMode::BlueAndPurple:
    return "BLUE_AND_PURPLE";
  case TargetCampMode::All:
    return "ALL";
  default:
    return "UNKNOWN";
  }
}

bool ParseAimbotTargetMessage(const std::string &message, uint8_t &target) {
  std::string text;
  text.reserve(message.size());
  for (unsigned char c : message) {
    if (std::isprint(c) && !std::isspace(c)) {
      text.push_back(static_cast<char>(std::toupper(c)));
    }
  }

  if (text == "0" || text == "00" || text == "0X00") {
    target = 0x00;
    return true;
  }
  if (text == "1" || text == "01" || text == "0X01") {
    target = 0x01;
    return true;
  }
  if (!text.empty() && std::all_of(text.begin(), text.end(), [](char c) {
        return c == '0' || c == '1' || c == 'X';
      })) {
    for (auto it = text.rbegin(); it != text.rend(); ++it) {
      if (*it == '0' || *it == '1') {
        target = (*it == '1') ? 0x01 : 0x00;
        return true;
      }
    }
  }

  bool found_binary_target = false;
  uint8_t binary_target = 0x00;
  for (unsigned char c : message) {
    if (c == 0x00 || c == 0x01) {
      binary_target = c;
      found_binary_target = true;
    }
  }
  if (!found_binary_target) {
    return false;
  }

  target = binary_target;
  return true;
}

bool ShouldTrackClassId(int class_id, TargetCampMode mode) {
  if (class_id == 2 || class_id == 3) {
    return true;
  }

  switch (mode) {
  case TargetCampMode::RedAndPurple:
    return class_id == 0;
  case TargetCampMode::BlueAndPurple:
    return class_id == 1;
  case TargetCampMode::All:
    return class_id == 0 || class_id == 1 || class_id == 2 || class_id == 3;
  default:
    return false;
  }
}

std::vector<std::array<float, 6>>
FilterTrackBoxes(const std::vector<std::array<float, 6>> &boxes,
                 TargetCampMode mode) {
  std::vector<std::array<float, 6>> filtered;
  filtered.reserve(boxes.size());
  for (const auto &box : boxes) {
    const int class_id = static_cast<int>(box[5]);
    if (ShouldTrackClassId(class_id, mode)) {
      filtered.push_back(box);
    }
  }
  return filtered;
}

const RuntimeParams &Params() {
  // ===== 调参集中区（统一放在文件末尾）=====
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_all.xml", // model_path:
                                                         // 模型路径
      "CPU", // openvino_device_name: 低延迟优先，优先使用核显推理
      "ONE_EURO", // angle_filter_type:
                  // 角度滤波类型（NONE/KF/EKF/UKF/CKF/ONE_EURO）
      "ALL",      // target_camp_mode: RED/BLUE/ALL；purple 始终允许跟踪

      1000, // capture_timeout_ms: 相机取帧超时（毫秒）
      5,    // capture_empty_sleep_ms: 空帧时休眠（毫秒）
      2,    // imu_read_fail_sleep_ms: IMU读失败时休眠（毫秒）
      1,    // imu_send_idle_sleep_ms: 无目标发送线程休眠（毫秒）
      1000, // imu_buffer_max_age_ms: IMU缓冲保留时间（毫秒）

      0.0f,  // minimum_angle_deg: 最小发送角度阈值（度）
      10.0f, // max_send_delta_deg: 单帧允许最大发送角差（度）

      0.03, // dt_default_sec: 默认帧间隔（秒）
      1.0,  // dt_max_sec: dt异常上限，超出则回退默认值

      10.0f, // pitch_abs_limit: pitch绝对限幅（度）

      false, // enable_latency_profile: 是否启用阶段打点统计
      100, // latency_print_interval_frames: 每多少帧打印一次窗口统计

      false, // enable_display: 是否启用显示窗口（可用 --no-display 关闭）
      false, // enable_motion_prediction: 测试时关闭运动预测，直接用检测框
      true, // enable_scan_mode: 调试跟踪振荡时可关闭 scan，仅保留 track 下发
      true, // enable_save_no_target_images: 是否开启存图模式

      1000, // scan_origin_hold_ms: 扫描模式下保持原点的时间（毫秒）
      80.0, // max_infer_fps: 推理线程最大提交帧率，<=0 表示不限制
      1000.0, // scan_send_hz: 扫描模式下的发送频率（Hz）
      2, // display_every_n_frames: 每N帧显示1帧（2可明显降低render延迟）
      1 // gui_poll_every_n_frames: 每N帧轮询一次按键退出
  };
  return p;
}
} // namespace
