/**
 * @file    include/Tools/RuntimeParams.hpp
 * @brief   定义图像识别主流程使用的默认运行参数集合。
 */

#pragma once

#include <string>

namespace Tools {

struct RuntimeParams {
  std::string stage12_model_path; // stage1/2 使用的 OpenVINO 模型路径（xml）
  std::string stage3_model_path; // stage3 使用的 OpenVINO 模型路径（xml）
  std::string openvino_device_name; // OpenVINO 设备名，如 CPU/GPU/AUTO
  std::string angle_filter_type; // 角度滤波器类型，如 ONE_EURO/KF/UKF
  std::string target_camp_mode;  // 目标阵营筛选模式，如 RED/BLUE/ALL

  int capture_timeout_ms;          // 相机抓帧超时时间（毫秒）
  double stage12_exposure_time_us; // stage1/2 默认曝光时间（微秒）
  double stage3_exposure_time_us;  // stage3 默认曝光时间（微秒）
  int stage3_switch_target_lost_delay_ms; // 满足 stage3
                                          // 后，丢目标持续多久再切模型（毫秒）
  int capture_empty_sleep_ms; // 相机空帧时休眠时长（毫秒）
  int imu_read_fail_sleep_ms; // IMU 读取失败时休眠时长（毫秒）
  int imu_send_idle_sleep_ms; // 发送线程空闲/等待时休眠时长（毫秒）
  int imu_buffer_max_age_ms; // IMU 匹配图像帧时允许的最大时间差（毫秒）

  float minimum_angle_deg; // 小于该偏角时不发送控制（死区，单位度）
  float max_send_delta_deg; // 单次发送 yaw 偏角最大限幅（度）
  double dt_max_sec; // 帧间 dt 的有效上限，超过则视为异常（秒）
  float pitch_abs_limit; // 单次发送 pitch 偏角绝对值上限（度）
  double angle_velocity_dt_min_sec; // 速度估计使用的最小 dt（秒）
  double angle_velocity_dt_max_sec; // 速度估计使用的最大 dt（秒）
  double
      angle_velocity_yaw_abs_limit_deg_per_sec; // yaw 角速度绝对值限幅（度/秒）
  double
      angle_velocity_pitch_abs_limit_deg_per_sec; // pitch
                                                  // 角速度绝对值限幅（度/秒）
  double
      angle_velocity_yaw_max_accel_deg_per_sec2; // yaw
                                                 // 角速度变化率限幅（度/秒²）
  double
      angle_velocity_pitch_max_accel_deg_per_sec2; // pitch
                                                   // 角速度变化率限幅（度/秒²）
  double angle_velocity_yaw_cutoff_hz; // yaw 角速度低通截止频率（Hz）
  double angle_velocity_pitch_cutoff_hz; // pitch 角速度低通截止频率（Hz）
  double angle_velocity_yaw_deadband_deg_per_sec; // yaw 角速度死区（度/秒）
  double angle_velocity_pitch_deadband_deg_per_sec; // pitch 角速度死区（度/秒）

  bool enable_latency_profile;       // 是否开启时延统计
  int latency_print_interval_frames; // 时延统计打印间隔（帧）

  bool enable_scan_mode;             // 是否启用丢目标后扫描模式
  bool enable_save_no_target_images; // 是否保存“无目标”样本图
  bool enable_save_target_videos;    // 是否保存“有目标”视频片段
  bool enable_save_full_run_video;   // 是否保存程序全程视频
  bool enable_display;               // 是否启用可视化窗口
  bool enable_calibration_sliders;   // 是否启用标定滑块 UI
  bool enable_send_log;              // 是否打印串口发送日志

  int scan_origin_hold_ms; // 扫描模式起始时在原点保持时长（毫秒）
  double max_infer_fps;        // 推理提交帧率上限；<=0 表示不限速
  double scan_send_hz;         // 扫描模式串口发送频率（Hz）
  int display_every_n_frames;  // 每 N 帧刷新一次显示
  int gui_poll_every_n_frames; // 每 N 帧轮询一次 GUI 按键事件
  int target_video_fps;        // 目标视频保存帧率（fps）
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_stage12_int8_openvino_model/"
      "antidrone_stage12.xml",
      "/home/nuc/antidrone/src/model/antidrone_stage3_int8_openvino_model/"
      "antidrone_stage3.xml",
      "CPU", "ONE_EURO", "ALL",

      1000,   // capture_timeout_ms: 相机抓帧超时 1000ms
      1000.0, // stage12_exposure_time_us: stage1/2 默认曝光 1000us
      4000.0, // stage3_exposure_time_us: stage3 默认曝光 4000us
      500, // stage3_switch_target_lost_delay_ms: stage3 后丢目标 500ms 再切模型
      5,   // capture_empty_sleep_ms: 空帧休眠 5ms
      2,   // imu_read_fail_sleep_ms: IMU 读失败休眠 2ms
      1,   // imu_send_idle_sleep_ms: 发送线程空闲休眠 1ms
      100, // imu_buffer_max_age_ms: IMU/图像最大匹配窗口 100ms

      0.0f,  // minimum_angle_deg: 控制死区 0 度（不抑制微小偏角）
      10.0f, // max_send_delta_deg: yaw 单次最大发送偏角 10 度
      1.0,   // dt_max_sec: 帧间隔有效上限 1.0 秒
      5.0f,  // pitch_abs_limit: pitch 单次最大发送偏角绝对值 10 度
      0.010, // angle_velocity_dt_min_sec: 速度估计最小 dt 10ms
      0.080, // angle_velocity_dt_max_sec: 速度估计最大 dt 80ms
      480.0, // angle_velocity_yaw_abs_limit_deg_per_sec: yaw
             // 角速度限幅（调参激进档）
      520.0, // angle_velocity_pitch_abs_limit_deg_per_sec: pitch
             // 角速度限幅（调参激进档）
      3600.0, // angle_velocity_yaw_max_accel_deg_per_sec2: yaw
              // 角加速度限幅（调参激进档）
      4200.0, // angle_velocity_pitch_max_accel_deg_per_sec2: pitch
              // 角加速度限幅（调参激进档）
      28.0, // angle_velocity_yaw_cutoff_hz: yaw 角速度低通截止频率（调参激进档）
      32.0, // angle_velocity_pitch_cutoff_hz: pitch
            // 角速度低通截止频率（调参激进档）
      0.05, // angle_velocity_yaw_deadband_deg_per_sec: yaw
            // 角速度死区（抑制极小微抖）
      0.05, // angle_velocity_pitch_deadband_deg_per_sec: pitch
            // 角速度死区（仅保留极小微抖抑制）

      false, // enable_latency_profile: 默认关闭时延统计
      700,   // latency_print_interval_frames: 每 100 帧打印一次统计

      true,  // enable_scan_mode: 默认开启扫描模式
      true,  // enable_save_no_target_images: 默认不保存无目标图像
      false, // enable_save_target_videos: 默认不保存目标视频
      false, // enable_save_full_run_video: 默认不保存全程视频
      true,  // enable_display: 默认开启显示窗口
      true,  // enable_calibration_sliders: 默认开启标定滑块
      true,  // enable_send_log: 默认开启发送日志

      1000,  // scan_origin_hold_ms: 扫描起始在原点保持 1000ms
      0.0,   // max_infer_fps: 0 表示不限制推理提交帧率
      200.0, // scan_send_hz: 扫描发送频率 200Hz
      2,     // display_every_n_frames: 每 2 帧刷新一次显示
      2,     // gui_poll_every_n_frames: 每 2 帧轮询一次按键
      30     // target_video_fps: 目标视频保存 30fps
  };
  return p;
}

} // namespace Tools
