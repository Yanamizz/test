#include <iostream>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <deque>
#include <atomic>
#include <vector>
#include <cstdint>

#include "ImageRecognize/ImageShow.hpp"
#include "ImageRecognize/ImagePredict_OPENVINO.hpp"  // 切换到 OpenVINO 时，注释上一行并启用这一行
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
#include "Tools/AngleCalculate.hpp"
#include "Tools/CpuAffinity.hpp"
#include "Tools/FpsCounter.hpp"
#include "Tools/LaserAngleCalculate.hpp"
#include "Tools/SaveImage.hpp"
#include "Tools/TrackingQualityEvaluator.hpp"
#include "CameraTask/GetImage.hpp"

namespace {
struct RuntimeParams {
  std::string model_path;
  std::string angle_filter_type;

  int capture_timeout_ms;
  int capture_empty_sleep_ms;
  int imu_read_fail_sleep_ms;
  int imu_send_idle_sleep_ms;
  int imu_buffer_max_age_ms;

  float minimum_angle_deg;
  float max_send_delta_deg;

  float track_iou_min;
  int track_lost_reset_frames;

  float center_alpha_base;
  float center_alpha_gain;
  float center_alpha_min;
  float center_alpha_max;

  float pitch_center_alpha_base;
  float pitch_center_alpha_gain;
  float pitch_center_alpha_min;
  float pitch_center_alpha_max;

  double dt_default_sec;
  double dt_max_sec;

  float pitch_deadzone_deg;

  int lock_frame_count_max;
  float dynamic_limit_base;
  float dynamic_limit_gain;

  float max_step_early;
  float max_step_late;
  int max_step_switch_frames;

  float pitch_dynamic_limit_base;
  float pitch_dynamic_limit_gain;
  float pitch_max_step_early;
  float pitch_max_step_late;
  float pitch_abs_limit;

  float yaw_comp_trigger_deg;
  float yaw_comp_strength;

  bool enable_latency_profile;
  int latency_print_interval_frames;

  bool enable_display;
  int display_every_n_frames;
  int gui_poll_every_n_frames;
};

const RuntimeParams &Params();
}  // namespace

std::atomic<bool> g_running(true);          // 全局运行标志
static std::mutex g_frame_mutex;            // 保护最新帧的互斥锁
static std::condition_variable g_frame_cv;  // 通知预测线程有新帧到达的条件变量
                                            // IMU 数据缓冲区
static std::deque<std::pair<std::chrono::steady_clock::time_point, SerialTask::EulerAngles>> g_imu_buffer;
static std::mutex g_imu_mutex;  // 保护 IMU 缓冲区的互斥锁

// 全局：双buffer避免重复clone
struct FrameItem {
  cv::Mat frame;
  std::chrono::steady_clock::time_point ts{};
};

static FrameItem g_frame_buffers[2];     // 双buffer
static std::atomic<int> g_write_idx{0};  // 当前写入buffer索引
static std::atomic<int> g_read_idx{-1};  // 当前可读buffer索引，-1表示无新帧

static std::atomic<bool> has_detection{false};  // 是否有目标被检测到
static std::atomic<float> g_send_abs_yaw{0.0f};
static std::atomic<float> g_send_abs_pitch{0.0f};
static std::atomic<float> g_send_offset_yaw{0.0f};
static std::atomic<float> g_send_offset_pitch{0.0f};

static std::vector<float> g_eval_target_yaw;
static std::vector<float> g_eval_control_yaw;
static std::vector<float> g_eval_target_pitch;
static std::vector<float> g_eval_control_pitch;
static bool g_eval_has_detection = false;

namespace {
struct LatencyStats {
  std::uint64_t frames = 0;
  std::uint64_t infer_ns = 0;
  std::uint64_t imu_match_ns = 0;
  std::uint64_t select_smooth_ns = 0;
  std::uint64_t angle_calc_ns = 0;
  std::uint64_t control_calc_ns = 0;
  std::uint64_t render_ns = 0;
  std::uint64_t loop_ns = 0;

  void Add(std::uint64_t &bucket, const std::chrono::steady_clock::time_point &t0,
           const std::chrono::steady_clock::time_point &t1) {
    bucket += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  }

  void AddFrame() { ++frames; }
};

static void PrintLatencyStats(const LatencyStats &s, const char *tag) {
  if (s.frames == 0) return;
  const double inv = 1.0 / static_cast<double>(s.frames);
  auto ns2ms = [&](std::uint64_t ns) { return static_cast<double>(ns) * inv / 1e6; };
  auto ns2us = [&](std::uint64_t ns) { return static_cast<double>(ns) * inv / 1e3; };
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[Latency][" << tag << "] frames=" << s.frames << " avg_ms"
            << " infer=" << ns2ms(s.infer_ns) << " imuMatch=" << ns2ms(s.imu_match_ns)
            << " selectSmooth=" << ns2ms(s.select_smooth_ns) << " angle=" << ns2ms(s.angle_calc_ns)
            << " control=" << ns2ms(s.control_calc_ns) << " render=" << ns2ms(s.render_ns)
            << " loop=" << ns2ms(s.loop_ns) << " | tiny_us imuMatch=" << ns2us(s.imu_match_ns)
            << " selectSmooth=" << ns2us(s.select_smooth_ns) << " control=" << ns2us(s.control_calc_ns) << std::endl;
}
}  // namespace

static inline float NormalizeDeltaDeg(float delta) {
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

static inline float BoxIou(const std::array<float, 6> &a, const std::array<float, 6> &b) {
  const float xx1 = std::max(a[0], b[0]);
  const float yy1 = std::max(a[1], b[1]);
  const float xx2 = std::min(a[2], b[2]);
  const float yy2 = std::min(a[3], b[3]);
  const float w = std::max(0.0f, xx2 - xx1);
  const float h = std::max(0.0f, yy2 - yy1);
  const float inter = w * h;
  const float area_a = std::max(0.0f, a[2] - a[0]) * std::max(0.0f, a[3] - a[1]);
  const float area_b = std::max(0.0f, b[2] - b[0]) * std::max(0.0f, b[3] - b[1]);
  const float uni = area_a + area_b - inter;
  return (uni <= 0.0f) ? 0.0f : (inter / uni);
}

static inline cv::Point2f BoxCenter(const std::array<float, 6> &box) {
  return {0.5f * (box[0] + box[2]), 0.5f * (box[1] + box[3])};
}

void CaptureThread(CameraTask::GalaxyCamera *camera);
void ImagePredictThread(ImageRecognize::ImagePredict &predictor);
void IMUReadThread(serial::Serial &port);
void IMUSendThread(serial::Serial &port);

static const char *TrackingIssueToString(Tools::TrackingIssueType issue) {
  switch (issue) {
    case Tools::TrackingIssueType::Good:
      return "Good";
    case Tools::TrackingIssueType::OverOscillation:
      return "OverOscillation";
    case Tools::TrackingIssueType::OverLag:
      return "OverLag";
    case Tools::TrackingIssueType::Mixed:
      return "Mixed";
    case Tools::TrackingIssueType::TargetMissing:
      return "TargetMissing";
    case Tools::TrackingIssueType::InsufficientData:
      return "InsufficientData";
    default:
      return "Unknown";
  }
}

int main() {
  Tools::BindCurrentThreadToBigCores();
  CameraTask::GalaxyCamera camera;
  serial::Serial port;
  bool serial_enabled = false;
  std::unique_ptr<ImageRecognize::ImagePredict> predictor;
  try {
    predictor = std::make_unique<ImageRecognize::ImagePredict>(Params().model_path);
  } catch (const std::exception &e) {
    std::cerr << "Failed to initialize OpenVINO model: " << e.what() << std::endl;
    std::cerr << "Configured model_path: " << Params().model_path << std::endl;
    return -2;
  }

  if (!port.isOpen()) {
    SerialTask::DefaultConfig(port);
    try {
      port.open();
      serial_enabled = true;
    } catch (const std::exception &e) {
      std::cerr << "Warning: failed to open IMU serial port, continue without serial: " << e.what() << std::endl;
      serial_enabled = false;
    }
  } else {
    serial_enabled = true;
  }

  std::thread image_capture(CaptureThread, &camera);
  std::thread image_predict(ImagePredictThread, std::ref(*predictor));
  std::thread imu_read;
  std::thread imu_send;
  if (serial_enabled) {
    imu_read = std::thread(IMUReadThread, std::ref(port));
    imu_send = std::thread(IMUSendThread, std::ref(port));
  } else {
    std::cout << "[IMU] serial disabled, running detection/display only." << std::endl;
  }

  if (image_capture.joinable()) image_capture.join();
  if (image_predict.joinable()) image_predict.join();
  if (imu_read.joinable()) imu_read.join();
  if (imu_send.joinable()) imu_send.join();

  {
    Tools::TrackingEvalParams eval_params;
    const auto yaw_eval = Tools::EvaluateTrackingQualityWhenDetected(g_eval_target_yaw, g_eval_control_yaw,
                                                                     g_eval_has_detection, eval_params);
    const auto pitch_eval = Tools::EvaluateTrackingQualityWhenDetected(g_eval_target_pitch, g_eval_control_pitch,
                                                                       g_eval_has_detection, eval_params);

    std::cout << "[TrackingEval][Yaw] issue=" << TrackingIssueToString(yaw_eval.issue)
              << " osc=" << yaw_eval.oscillation_score << " lag=" << yaw_eval.lag_score << " rmse=" << yaw_eval.rmse
              << " bestLag=" << yaw_eval.best_lag_frames << " corr=" << yaw_eval.best_lag_corr
              << " note=" << yaw_eval.note << std::endl;

    std::cout << "[TrackingEval][Pitch] issue=" << TrackingIssueToString(pitch_eval.issue)
              << " osc=" << pitch_eval.oscillation_score << " lag=" << pitch_eval.lag_score
              << " rmse=" << pitch_eval.rmse << " bestLag=" << pitch_eval.best_lag_frames
              << " corr=" << pitch_eval.best_lag_corr << " note=" << pitch_eval.note << std::endl;
  }

  return 0;
}

void CaptureThread(CameraTask::GalaxyCamera *camera) {
  if (!camera->open()) {
    std::cerr << "Failed to open camera." << std::endl;
    g_running = false;
    return;
  }
  while (g_running) {
    cv::Mat frame = camera->grab(Params().capture_timeout_ms);
    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(Params().capture_empty_sleep_ms));
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
  static const Tools::FilterType filter_type = Tools::AngleCalculator::ParseFilterType(Params().angle_filter_type);

  std::cout << "[AngleFilter] using type: " << Tools::ToString(filter_type) << std::endl;

  // static Tools::SaveImageOnNoTarget no_target_saver(5, "captures");
  std::chrono::steady_clock::time_point prev_frame_ts{};
  bool has_prev_frame_ts = false;

  // 首次捕获目标时做软启动，抑制远距离目标首次追踪的大过冲。
  bool target_locked_last_frame = false;
  int lock_frame_count = 0;
  float last_cmd_delta_yaw = 0.0f;
  float last_cmd_delta_pitch = 0.0f;

  std::array<float, 6> last_selected_box{};
  bool has_last_selected_box = false;
  cv::Point2f smoothed_center{};
  bool has_smoothed_center = false;
  float smoothed_pitch_center_y = 0.0f;
  bool has_smoothed_pitch_center_y = false;
  int miss_frame_count = 0;
  LatencyStats latency_total;
  LatencyStats latency_window;
  std::uint64_t ui_frame_counter = 0;

  auto resetTrackingState = [&]() {
    has_detection.store(false, std::memory_order_release);
    target_locked_last_frame = false;
    lock_frame_count = 0;
  };

  while (g_running) {
    const auto t_loop_start = std::chrono::steady_clock::now();
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
      const auto t_infer_start = std::chrono::steady_clock::now();
      result = predictor.run(frame);
      const auto t_infer_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.infer_ns, t_infer_start, t_infer_end);
      latency_window.Add(latency_window.infer_ns, t_infer_start, t_infer_end);

      // 关联最近一次 IMU 状态并记录延迟
      const auto t_imu_match_start = std::chrono::steady_clock::now();
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
      const auto t_imu_match_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.imu_match_ns, t_imu_match_start, t_imu_match_end);
      latency_window.Add(latency_window.imu_match_ns, t_imu_match_start, t_imu_match_end);
    } catch (const std::exception &e) {
      std::cerr << "ImagePredictThread exception: " << e.what() << std::endl;
    }

    cv::Point2d offset_angles;
    [[maybe_unused]] const bool detected_target = !result.boxes.empty();
    if (!result.boxes.empty()) {
      const auto t_select_start = std::chrono::steady_clock::now();
      int selected_idx = 0;
      if (has_last_selected_box) {
        float best_iou = -1.0f;
        int best_iou_idx = -1;
        for (int i = 0; i < static_cast<int>(result.boxes.size()); ++i) {
          if (static_cast<int>(result.boxes[i][5]) != static_cast<int>(last_selected_box[5])) continue;
          const float iou = BoxIou(result.boxes[i], last_selected_box);
          if (iou > best_iou) {
            best_iou = iou;
            best_iou_idx = i;
          }
        }

        if (best_iou_idx >= 0 && best_iou >= Params().track_iou_min) {
          selected_idx = best_iou_idx;
        } else {
          // 没有稳定匹配时回退到最高置信度框
          selected_idx = 0;
          for (int i = 1; i < static_cast<int>(result.boxes.size()); ++i) {
            if (result.boxes[i][4] > result.boxes[selected_idx][4]) selected_idx = i;
          }
        }
      } else {
        // 首帧锁定最高置信度框
        for (int i = 1; i < static_cast<int>(result.boxes.size()); ++i) {
          if (result.boxes[i][4] > result.boxes[selected_idx][4]) selected_idx = i;
        }
      }

      const auto &selected_box = result.boxes[selected_idx];
      last_selected_box = selected_box;
      has_last_selected_box = true;
      miss_frame_count = 0;

      const cv::Point2f raw_center = BoxCenter(selected_box);
      if (!has_smoothed_center) {
        smoothed_center = raw_center;
        has_smoothed_center = true;
      } else {
        const float center_motion = cv::norm(raw_center - smoothed_center);
        const float alpha = std::clamp(Params().center_alpha_base + Params().center_alpha_gain * center_motion,
                                       Params().center_alpha_min, Params().center_alpha_max);
        smoothed_center = (1.0f - alpha) * smoothed_center + alpha * raw_center;
      }

      if (!has_smoothed_pitch_center_y) {
        smoothed_pitch_center_y = raw_center.y;
        has_smoothed_pitch_center_y = true;
      } else {
        const float pitch_motion = std::abs(raw_center.y - smoothed_pitch_center_y);
        const float pitch_alpha =
            std::clamp(Params().pitch_center_alpha_base + Params().pitch_center_alpha_gain * pitch_motion,
                       Params().pitch_center_alpha_min, Params().pitch_center_alpha_max);
        smoothed_pitch_center_y = (1.0f - pitch_alpha) * smoothed_pitch_center_y + pitch_alpha * raw_center.y;
      }

      float center_x = smoothed_center.x;
      float center_y = smoothed_pitch_center_y;
      float width = selected_box[2] - selected_box[0];
      float height = selected_box[3] - selected_box[1];
      float distance = distance_calculator.CalculateDistance(height, width);
      const auto t_select_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.select_smooth_ns, t_select_start, t_select_end);
      latency_window.Add(latency_window.select_smooth_ns, t_select_start, t_select_end);

      if (has_matched_imu) {
        double dt = Params().dt_default_sec;
        if (has_prev_frame_ts) {
          dt = std::chrono::duration<double>(frame_ts - prev_frame_ts).count();
        }
        prev_frame_ts = frame_ts;
        has_prev_frame_ts = true;
        if (dt <= 0.0 || dt > Params().dt_max_sec) dt = Params().dt_default_sec;

        const auto t_angle_start = std::chrono::steady_clock::now();
        auto [filtered_yaw, filtered_pitch] = angle_calculator.CalculateAbsoluteAngles(
            center_x, center_y, matched_imu.yaw, matched_imu.pitch, filter_type, dt);
        const auto t_angle_end = std::chrono::steady_clock::now();
        latency_total.Add(latency_total.angle_calc_ns, t_angle_start, t_angle_end);
        latency_window.Add(latency_window.angle_calc_ns, t_angle_start, t_angle_end);

        cv::Point2f pred_point =
            angle_calculator.AbsoluteAnglesToPixel(filtered_yaw, filtered_pitch, matched_imu.yaw, matched_imu.pitch);
        ImageRecognize::ImageShow::ShowPred(frame, pred_point.x, pred_point.y);

        // 必须用“最短角差”，否则跨越 ±180° 时会出现 300° 级突变
        offset_angles.x = NormalizeDeltaDeg(filtered_yaw - matched_imu.yaw);
        offset_angles.y = NormalizeDeltaDeg(filtered_pitch - matched_imu.pitch);

        auto [laser_yaw_angle, laser_pitch_angle] =
            laser_angle_calculator.CalculateLaserAngles(distance, offset_angles.x);

        const auto t_control_start = std::chrono::steady_clock::now();
        float delta_yaw_raw = NormalizeDeltaDeg(static_cast<float>(offset_angles.x + laser_yaw_angle));
        float delta_pitch_raw = NormalizeDeltaDeg(static_cast<float>(offset_angles.y + laser_pitch_angle));
        if (std::abs(delta_pitch_raw) < Params().pitch_deadzone_deg) {
          delta_pitch_raw = 0.0f;
        }
        if (std::abs(delta_pitch_raw) > Params().minimum_angle_deg ||
            std::abs(delta_yaw_raw) > Params().minimum_angle_deg) {
          // 新锁定目标时渐进放开限幅，避免第一拍打满导致过冲。
          if (!target_locked_last_frame) {
            lock_frame_count = 0;
            last_cmd_delta_yaw = 0.0f;
            last_cmd_delta_pitch = 0.0f;
          }
          target_locked_last_frame = true;
          lock_frame_count = std::min(lock_frame_count + 1, Params().lock_frame_count_max);

          const float dynamic_limit = std::min(
              Params().max_send_delta_deg,
              Params().dynamic_limit_base + Params().dynamic_limit_gain * static_cast<float>(lock_frame_count));
          float delta_yaw = std::clamp(delta_yaw_raw, -dynamic_limit, dynamic_limit);
          float delta_pitch = std::clamp(delta_pitch_raw, -dynamic_limit, dynamic_limit);

          // 进一步限制单帧命令变化量，抑制远离中心时首次追踪抖动。
          const float max_step =
              (lock_frame_count < Params().max_step_switch_frames) ? Params().max_step_early : Params().max_step_late;
          delta_yaw = std::clamp(delta_yaw, last_cmd_delta_yaw - max_step, last_cmd_delta_yaw + max_step);
          delta_pitch = std::clamp(delta_pitch, last_cmd_delta_pitch - max_step, last_cmd_delta_pitch + max_step);

          // pitch 单独再收紧一层，避免竖直方向的小噪声被放大成可见抖动。
          const float pitch_dynamic_limit =
              std::min(Params().max_send_delta_deg,
                       Params().pitch_dynamic_limit_base +
                           Params().pitch_dynamic_limit_gain * static_cast<float>(lock_frame_count));
          const float pitch_max_step = (lock_frame_count < Params().max_step_switch_frames)
                                           ? Params().pitch_max_step_early
                                           : Params().pitch_max_step_late;
          delta_pitch = std::clamp(delta_pitch, -pitch_dynamic_limit, pitch_dynamic_limit);
          delta_pitch =
              std::clamp(delta_pitch, last_cmd_delta_pitch - pitch_max_step, last_cmd_delta_pitch + pitch_max_step);
          delta_pitch = std::clamp(delta_pitch, -Params().pitch_abs_limit, Params().pitch_abs_limit);

          // 全局安全限幅
          delta_yaw = std::clamp(delta_yaw, -Params().max_send_delta_deg, Params().max_send_delta_deg);
          delta_pitch = std::clamp(delta_pitch, -Params().max_send_delta_deg, Params().max_send_delta_deg);

          const float cmd_delta_yaw = delta_yaw;
          const float cmd_delta_pitch = delta_pitch;

          const float send_abs_yaw = matched_imu.yaw + cmd_delta_yaw;
          const float send_abs_pitch = matched_imu.pitch + cmd_delta_pitch;

          const float target_abs_yaw = matched_imu.yaw + delta_yaw_raw;
          const float target_abs_pitch = matched_imu.pitch + delta_pitch_raw;

          last_cmd_delta_yaw = delta_yaw;
          last_cmd_delta_pitch = delta_pitch;

          g_send_abs_yaw.store(send_abs_yaw, std::memory_order_release);
          g_send_abs_pitch.store(send_abs_pitch, std::memory_order_release);
          g_send_offset_yaw.store(cmd_delta_yaw, std::memory_order_release);
          g_send_offset_pitch.store(cmd_delta_pitch, std::memory_order_release);

          g_eval_target_yaw.push_back(target_abs_yaw);
          g_eval_control_yaw.push_back(send_abs_yaw);
          g_eval_target_pitch.push_back(target_abs_pitch);
          g_eval_control_pitch.push_back(send_abs_pitch);
          g_eval_has_detection = true;

          has_detection.store(true, std::memory_order_release);
          ImageRecognize::ImageShow::ShowAngles(frame, send_abs_yaw, send_abs_pitch, matched_imu.yaw, matched_imu.pitch,
                                                cmd_delta_yaw, cmd_delta_pitch, distance);
        } else {
          resetTrackingState();
        }
        const auto t_control_end = std::chrono::steady_clock::now();
        latency_total.Add(latency_total.control_calc_ns, t_control_start, t_control_end);
        latency_window.Add(latency_window.control_calc_ns, t_control_start, t_control_end);
      } else {
        resetTrackingState();
      }
    } else {
      ++miss_frame_count;
      if (miss_frame_count > Params().track_lost_reset_frames) {
        has_last_selected_box = false;
        has_smoothed_center = false;
        has_smoothed_pitch_center_y = false;
      }
      resetTrackingState();
    }

    // 可视化显示
    fps_counter.tick();
    double fps = fps_counter.get();

    const auto t_render_start = std::chrono::steady_clock::now();
    const bool do_display =
        Params().enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(std::max(1, Params().display_every_n_frames)) == 0);
    const bool do_gui_poll =
        Params().enable_display &&
        (ui_frame_counter % static_cast<std::uint64_t>(std::max(1, Params().gui_poll_every_n_frames)) == 0);
    ++ui_frame_counter;

    if (do_display) {
      ImageRecognize::ImageShow::ShowNow(frame, result, fps);
    }

    // 无目标时，每隔若干帧保存图像到本次运行目录
    // no_target_saver.Update(frame, detected_target);

    // 处理 GUI 事件并允许按键退出
    if (do_gui_poll && ImageRecognize::ImageShow::WaitForExit()) {
      g_running = false;
      const auto t_render_end = std::chrono::steady_clock::now();
      latency_total.Add(latency_total.render_ns, t_render_start, t_render_end);
      latency_window.Add(latency_window.render_ns, t_render_start, t_render_end);
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
        latency_window.frames >= static_cast<std::uint64_t>(Params().latency_print_interval_frames)) {
      PrintLatencyStats(latency_window, "Window");
      latency_window = LatencyStats{};
    }
  }

  if (Params().enable_latency_profile) {
    if (latency_window.frames > 0) PrintLatencyStats(latency_window, "WindowTail");
    PrintLatencyStats(latency_total, "Total");
  }
}

void IMUReadThread(serial::Serial &port) {
  while (g_running) {
    SerialTask::EulerAngles angles;
    if (SerialTask::ReadIMUData(port, angles)) {
      auto ts = std::chrono::steady_clock::now();

      std::lock_guard<std::mutex> lk(g_imu_mutex);
      // 添加到缓冲区末尾
      g_imu_buffer.emplace_back(ts, angles);

      // 裁剪过旧的 IMU 条目，保持缓冲区只包含最近一段时间的数据
      while (!g_imu_buffer.empty() &&
             (ts - g_imu_buffer.front().first) > std::chrono::milliseconds(Params().imu_buffer_max_age_ms)) {
        g_imu_buffer.pop_front();
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(Params().imu_read_fail_sleep_ms));
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
      if (std::abs(offset_yaw) > Params().yaw_comp_trigger_deg) {
        if (offset_yaw > 0) {
          yaw += Params().yaw_comp_strength *
                 (1.0f - Params().minimum_angle_deg /
                             std::abs(offset_yaw));  // 根据偏移量大小动态调整补偿力度，越接近中心越温和
        } else {
          yaw -= Params().yaw_comp_strength * (1.0f - Params().minimum_angle_deg / std::abs(offset_yaw));
        }
      }
      SerialTask::SerialSend(port, pitch, yaw, 0x01);
      has_detection.store(false, std::memory_order_release);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(Params().imu_send_idle_sleep_ms));
    }
  }
}

namespace {
const RuntimeParams &Params() {
  // ===== 调参集中区（统一放在文件末尾）=====
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_v8n.onnx",  // model_path: 模型路径
      "CKF",                                               // angle_filter_type: 角度滤波类型（KF/EKF/UKF/CKF）

      1000,  // capture_timeout_ms: 相机取帧超时（毫秒）
      5,     // capture_empty_sleep_ms: 空帧时休眠（毫秒）
      2,     // imu_read_fail_sleep_ms: IMU读失败时休眠（毫秒）
      1,     // imu_send_idle_sleep_ms: 无目标发送线程休眠（毫秒）
      1000,  // imu_buffer_max_age_ms: IMU缓冲保留时间（毫秒）

      0.008f,  // minimum_angle_deg: 最小发送角度阈值（度）
      5.0f,    // max_send_delta_deg: 单帧允许最大发送角差（度）

      0.05f,  // track_iou_min: 目标关联最小IOU阈值
      6,      // track_lost_reset_frames: 连续丢失多少帧后重置跟踪状态

      0.18f,   // center_alpha_base: 中心点平滑基础alpha
      0.004f,  // center_alpha_gain: 中心点平滑随运动增强系数
      0.18f,   // center_alpha_min: 中心点平滑alpha下限
      0.65f,   // center_alpha_max: 中心点平滑alpha上限

      0.08f,    // pitch_center_alpha_base: pitch中心y平滑基础alpha
      0.0025f,  // pitch_center_alpha_gain: pitch中心y平滑随运动增强系数
      0.08f,    // pitch_center_alpha_min: pitch中心y平滑alpha下限
      0.32f,    // pitch_center_alpha_max: pitch中心y平滑alpha上限

      0.05,  // dt_default_sec: 默认帧间隔（秒）
      0.2,   // dt_max_sec: dt异常上限，超出则回退默认值

      0.04f,  // pitch_deadzone_deg: pitch死区（度）

      30,     // lock_frame_count_max: 锁定计数最大值
      1.2f,   // dynamic_limit_base: 动态限幅基础值
      0.45f,  // dynamic_limit_gain: 动态限幅随锁定帧增长系数

      0.5f,  // max_step_early: 锁定初期单帧最大步进
      0.9f,  // max_step_late: 锁定后期单帧最大步进
      10,    // max_step_switch_frames: 初期/后期切换帧数

      0.9f,   // pitch_dynamic_limit_base: pitch动态限幅基础值
      0.2f,   // pitch_dynamic_limit_gain: pitch动态限幅增长系数
      0.25f,  // pitch_max_step_early: pitch初期单帧步进
      0.45f,  // pitch_max_step_late: pitch后期单帧步进
      3.0f,   // pitch_abs_limit: pitch绝对限幅（度）

      0.6f,  // yaw_comp_trigger_deg: yaw补偿触发阈值（度）
      3.0f,  // yaw_comp_strength: yaw补偿力度

      true,  // enable_latency_profile: 是否启用阶段打点统计
      120,   // latency_print_interval_frames: 每多少帧打印一次窗口统计

      true,  // enable_display: 是否显示图像窗口
      2,     // display_every_n_frames: 每N帧显示1帧（2可明显降低render延迟）
      1      // gui_poll_every_n_frames: 每N帧轮询一次按键退出
  };
  return p;
}
}  // namespace