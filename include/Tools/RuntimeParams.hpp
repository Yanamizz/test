/**
 * @file    include/Tools/RuntimeParams.hpp
 * @brief   定义图像识别主流程使用的默认运行参数集合。
 *
 * 主程序命令行（`build/bin/ImagePredict`）支持以下开关：
 * - 显示窗口：
 *   `--enable-display=<bool>`
 * - 标定滑块：
 *   `--calibration-sliders=<bool>`
 * - 延迟统计：
 *   `--latency-profile=<bool>`
 * - 扫描模式：
 *   `--scan-mode=<bool>`
 * - 无目标样本保存：
 *   `--save-no-target-images=<bool>`
 * - 全程录像：
 *   `--save-full-run-video=<bool>`
 * - 发送日志：
 *   `--send-log=<bool>`
 * - 视频输入：
 *   `--video <path>`
 *   `--video=<path>`
 *
 * 其中 `<bool>` 支持：`0/1`、`false/true`、`off/on`、`no/yes`。
 */

#pragma once

#include <string>

namespace Tools {

struct RuntimeParams {
  // 注意：字段顺序需与下方 Params() 中的聚合初始化顺序严格保持一致。

  // ===== 基础模型与策略 =====
  std::string stage12_model_path; // stage1/2 使用的 OpenVINO 模型路径（xml）
  std::string stage3_model_path; // stage3 使用的 OpenVINO 模型路径（xml）
  std::string openvino_device_name; // OpenVINO 设备名，如 CPU/GPU/AUTO
  std::string angle_filter_type; // 角度滤波器类型，如 ONE_EURO/KF/UKF
  std::string target_camp_mode;  // 目标阵营筛选模式，如 RED/BLUE/ALL

  // ===== 相机采集与 ROI =====
  int capture_timeout_ms;        // 相机抓帧超时时间（毫秒）
  bool stage3_enable_camera_roi; // stage3 是否开启相机侧 ROI 裁剪（stage12
                                 // 永远关闭）
  bool stage3_roi_keep_centered; // stage3 是否保持中心点不动进行 ROI 裁剪
  int stage3_roi_width;          // stage3 ROI 宽（像素）
  int stage3_roi_height;         // stage3 ROI 高（像素）
  int stage3_roi_offset_x; // stage3 ROI 左上角 X 偏移（像素），仅在
                           // keep_centered=false 时生效
  int stage3_roi_offset_y; // stage3 ROI 左上角 Y 偏移（像素），仅在
                           // keep_centered=false 时生效
  double stage12_exposure_time_us; // stage1/2 默认曝光时间（微秒）
  double stage3_exposure_time_us;  // stage3 默认曝光时间（微秒）

  // ===== 阶段切换与 stage3 丢失恢复 =====
  int stage3_switch_target_lost_delay_ms; // 满足 stage3
                                          // 后，丢目标持续多久再切模型（毫秒）
  int stage3_lost_target_coast_ms; // stage3 丢失目标后按丢失前速度方向续行时长
                                   // （毫秒）
  int stage3_lost_target_coast_trigger_delay_ms; // stage3
                                                 // 连续丢失多久后才进入续行
                                                 // （毫秒）
  int stage3_lost_target_reacquire_confirm_ms; // stage3
                                               // 重新识别后需持续多久才退出
                                               // 续行（毫秒）
  double stage3_lost_target_reacquire_gate_deg; // stage3
                                                // 回检框与续行预测位置允许的
                                                // 最大角差（度）
  double stage3_lost_target_coast_yaw_speed_deg_per_sec; // stage3 丢目标后
                                                         // 续行固定 yaw 速度
  double
      stage3_lost_target_coast_pitch_speed_deg_per_sec; // stage3 丢目标后
                                                        // 续行固定 pitch 速度
  double stage3_fallback_min_stage2_progress; // stage2 进度达到该值后，允许
                                              // 丢目标兜底进入 stage3 候选
  int stage3_fallback_no_target_ms; // 高进度后连续空框多久触发 stage3 兜底
                                    // （毫秒）
  int stage3_fallback_recent_purple_ms; // 兜底触发前，最近一次紫色观测必须在
                                        // 该窗口内（毫秒）
  int stage3_fallback_high_progress_no_target_probe_ms; // stage2 P 曾达到阈值后
                                                        // 连续无目标多久进入
                                                        // stage3 probe（毫秒）
  int stage3_fallback_stage2_no_target_force_ms; // 进入 stage2 后连续无目标多久
                                                 // 且 P 未达阈值时，强制兜底
                                                 // 进入 stage3（毫秒）
  int stage3_probe_no_target_rollback_ms; // stage3 probe 连续无目标多久回退
                                          // stage2（毫秒）

  // ===== 线程等待与时序容差 =====
  int capture_empty_sleep_ms; // 相机空帧时休眠时长（毫秒）
  int imu_read_fail_sleep_ms; // IMU 读取失败时休眠时长（毫秒）
  int imu_send_idle_sleep_ms; // 发送线程空闲/等待时休眠时长（毫秒）
  int imu_buffer_max_age_ms; // IMU 匹配图像帧时允许的最大时间差（毫秒）

  // ===== 主控制与角速度估计 =====
  float minimum_angle_deg; // 小于该偏角时不发送控制（死区，单位度）
  float max_send_delta_deg; // 单次发送 yaw 偏角最大限幅（度）
  double dt_max_sec; // 帧间 dt 的有效上限，超过则视为异常（秒）
  float pitch_abs_limit; // 单次发送 pitch 偏角绝对值上限（度）
  float stage12_pitch_micro_deadband_deg; // stage1/2 最终发送前 pitch
                                          // 微抖死区（度）
  float aimbot_target_laser_max_distance_m; // 保留的距离配置基准（米）
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
  float
      yaw_velocity_feedforward_error_threshold_deg; // yaw
                                                    // 误差超过该阈值后追加速度前馈
  float
      pitch_velocity_feedforward_error_threshold_deg; // pitch
                                                      // 误差超过该阈值后追加速度前馈
  float yaw_error_feedforward_gain_deg_per_sec_per_deg; // yaw 误差速度前馈增益
  float pitch_error_feedforward_gain_deg_per_sec_per_deg; // pitch
                                                          // 误差速度前馈增益

  // ===== 时延统计 =====
  bool enable_latency_profile;       // 是否开启时延统计
  int latency_print_interval_frames; // 时延统计打印间隔（帧）

  // ===== 功能开关 =====
  bool enable_scan_mode;             // 是否启用丢目标后扫描模式
  bool enable_save_no_target_images; // 是否保存“无目标”样本图
  bool enable_save_target_videos;    // 是否保存“有目标”视频片段
  bool enable_save_full_run_video;   // 是否保存程序全程视频
  bool enable_display;               // 是否启用可视化窗口
  bool enable_calibration_sliders;   // 是否启用标定滑块 UI
  bool enable_send_log;              // 是否打印串口发送日志

  // ===== 扫描与推理节流 =====
  int scan_trigger_delay_ms; // 丢失目标后，延迟多久进入扫描模式（毫秒）
  int scan_origin_hold_ms; // 扫描模式起始时在原点保持时长（毫秒）
  int stage3_scan_origin_hold_ms; // stage3 扫描模式起始时在原点保持时长
                                  // （毫秒）
  std::string stage3_scan_bounds_mode; // stage3 扫描边界模式：MANUAL/AUTO
  double max_infer_fps; // 推理提交帧率上限；<=0 表示不限速
  double scan_send_hz; // stage1/2 扫描发送频率（Hz），优先用于控制平滑度
  double stage3_scan_send_hz; // stage3 扫描发送频率（Hz），优先用于控制平滑度
  double scan_yaw_speed_deg_per_sec; // stage1/2 扫描 yaw 角速度（度/秒）
  double stage3_scan_yaw_speed_deg_per_sec; // stage3 扫描 yaw 角速度（度/秒）
  float scan_pitch_wavelength_percent; // stage1/2 lambda 百分比，100% 表示往返
                                       // 轨迹刚好构成横 8 字
  float scan_pitch_amplitude_percent; // stage1/2 A 百分比，100% 表示占用
                                      // pitch 半扫描高度
  float stage3_scan_pitch_wavelength_percent; // stage3 lambda 百分比，100%
                                              // 表示往返轨迹刚好构成横 8 字
  float stage3_scan_pitch_amplitude_percent; // stage3 A 百分比，100%
                                             // 表示占用 pitch 半扫描高度

  // ===== 显示与保存输出 =====
  int display_every_n_frames;  // 每 N 帧刷新一次显示
  int gui_poll_every_n_frames; // 每 N 帧轮询一次 GUI 按键事件
  int target_video_fps;        // 目标视频保存帧率（fps）
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      // ===== 基础模型与策略 =====
      "/home/nuc/antidrone/src/model/antidrone_stage12_int8_openvino_model/"
      "antidrone_stage12.xml",
      "/home/nuc/antidrone/src/model/final_s_int8_openvino_model/final_s.xml",
      "CPU", "ONE_EURO", "ALL",

      // ===== 相机采集与 ROI =====
      1000, // capture_timeout_ms: 相机抓帧超时 1000ms
      true, // stage3_enable_camera_roi: stage3 默认开启相机侧 ROI 裁剪
      true, // stage3_roi_keep_centered: stage3 默认保持中心点不动裁剪
      1280,   // stage3_roi_width: ROI 宽
      720,    // stage3_roi_height: ROI 高
      320,    // stage3_roi_offset_x: ROI 偏移 X
      180,    // stage3_roi_offset_y: ROI 偏移 Y
      1000.0, // stage12_exposure_time_us: stage1/2 默认曝光 1000us
      4000.0, // stage3_exposure_time_us: stage3 默认曝光 4000us

      // ===== 阶段切换与 stage3 丢失恢复 =====
      200, // stage3_switch_target_lost_delay_ms: stage3 后丢目标 200ms 再切模型
      2000, // stage3_lost_target_coast_ms: stage3 丢目标后续行 2000ms
      40, // stage3_lost_target_coast_trigger_delay_ms: 丢失 40ms 后才续行
      60, // stage3_lost_target_reacquire_confirm_ms: 重识别稳定 60ms 才退出续行
      1.2, // stage3_lost_target_reacquire_gate_deg: 回检接管最大允许角差 1.2 度
      3.0, // stage3_lost_target_coast_yaw_speed_deg_per_sec: stage3 丢目标后
           // 续行固定 yaw 速度
      2.0, // stage3_lost_target_coast_pitch_speed_deg_per_sec: stage3 丢目标后
           // 续行固定 pitch 速度
      60.0, // stage3_fallback_min_stage2_progress: stage2 P>=60 后才允许兜底
      400, // stage3_fallback_no_target_ms: 连续空框 400ms 后触发兜底
      500, // stage3_fallback_recent_purple_ms: 最近 500ms 内必须见过紫色
      10000, // stage3_fallback_high_progress_no_target_probe_ms: P 曾达到阈值后
             // 连续无目标 10s，进入 stage3 probe
      60000, // stage3_fallback_stage2_no_target_force_ms: stage2 连续空框
             // 90s 且 P 未达阈值后，强制兜底进入 stage3
      20000, // stage3_probe_no_target_rollback_ms: stage3 probe 连续无目标
             // 20s 后回退 stage2

      // ===== 线程等待与时序容差 =====
      5,   // capture_empty_sleep_ms: 空帧休眠 5ms
      2,   // imu_read_fail_sleep_ms: IMU 读失败休眠 2ms
      1,   // imu_send_idle_sleep_ms: 发送线程空闲休眠 1ms
      100, // imu_buffer_max_age_ms: IMU/图像最大匹配窗口 100ms

      // ===== 主控制与角速度估计 =====
      0.0f,  // minimum_angle_deg: 控制死区 0 度（不抑制微小偏角）
      10.0f, // max_send_delta_deg: yaw 单次最大发送偏角 10 度
      1.0,   // dt_max_sec: 帧间隔有效上限 1.0 秒
      5.0f,  // pitch_abs_limit: pitch 单次最大发送偏角绝对值 5 度
      0.018f, // stage12_pitch_micro_deadband_deg: stage1/2 pitch 微抖死区 0.018
              // 度
      19.5f, // aimbot_target_laser_max_distance_m:
             // 首次开启激光/初始化阶段判断的距离上限（米）
      0.010, // angle_velocity_dt_min_sec: 速度估计最小 dt 10ms
      0.080, // angle_velocity_dt_max_sec: 速度估计最大 dt 80ms
      560.0, // angle_velocity_yaw_abs_limit_deg_per_sec: yaw
             // 角速度限幅（调参激进档）
      520.0, // angle_velocity_pitch_abs_limit_deg_per_sec: pitch
             // 角速度限幅（调参激进档）
      4600.0, // angle_velocity_yaw_max_accel_deg_per_sec2: yaw
              // 角加速度限幅（调参激进档）
      4200.0, // angle_velocity_pitch_max_accel_deg_per_sec2: pitch
              // 角加速度限幅（调参激进档）
      36.0,   // angle_velocity_yaw_cutoff_hz: yaw
              // 角速度低通截止频率（调参激进档）
      32.0,   // angle_velocity_pitch_cutoff_hz: pitch
              // 角速度低通截止频率（调参激进档）
      0.02,   // angle_velocity_yaw_deadband_deg_per_sec: yaw
              // 角速度死区（抑制极小微抖）
      0.05,   // angle_velocity_pitch_deadband_deg_per_sec: pitch
              // 角速度死区（仅保留极小微抖抑制）
      0.08f,  // yaw_velocity_feedforward_error_threshold_deg: yaw 超过
              // 0.15deg 追加误差方向速度前馈
      0.08f,  // pitch_velocity_feedforward_error_threshold_deg: pitch 超过
              // 0.10deg 追加误差方向速度前馈
      50.0f,  // yaw_error_feedforward_gain_deg_per_sec_per_deg: yaw
              // 每 1deg 误差追加速度
      15.0f,  // pitch_error_feedforward_gain_deg_per_sec_per_deg: pitch
              // 每 1deg 误差追加速度

      // ===== 时延统计 =====
      false, // enable_latency_profile: 默认关闭时延统计
      700,   // latency_print_interval_frames: 每 700 帧打印一次统计

      // ===== 功能开关 =====
      true,  // enable_scan_mode: 默认开启扫描模式
      false, // enable_save_no_target_images: 默认不保存无目标图像
      false, // enable_save_target_videos: 默认不保存目标视频
      true,  // enable_save_full_run_video: 默认不保存全程视频
      true,  // enable_display: 默认开启显示窗口
      true,  // enable_calibration_sliders: 默认开启标定滑块
      true,  // enable_send_log: 默认开启发送日志

      // ===== 扫描与推理节流 =====
      2000, // scan_trigger_delay_ms: 丢目标后 2000ms 进入扫描
      1000, // scan_origin_hold_ms: 扫描起始在原点保持 1000ms
      1500, // stage3_scan_origin_hold_ms: stage3 扫描起始在原点保持 1500ms
      "AUTO", // stage3_scan_bounds_mode: stage3 默认使用 stage12 目标角范围
      0.0, // max_infer_fps: 0 表示不限制推理提交帧率

      200.0, // scan_send_hz: stage1/2 扫描发送频率 200Hz
      200.0, // stage3_scan_send_hz: stage3 扫描发送频率 200Hz
      14.0,  // scan_yaw_speed_deg_per_sec: stage1/2 扫描 yaw 速度 16deg/s
      1.25, // stage3_scan_yaw_speed_deg_per_sec: stage3 扫描 yaw 速度 1.25deg/s

      40.0f, // scan_pitch_wavelength_percent: stage1/2 lambda 百分比
      50.0f, // scan_pitch_amplitude_percent: stage1/2 A 百分比

      40.0f,  // stage3_scan_pitch_wavelength_percent: stage3 lambda 百分比
      100.0f, // stage3_scan_pitch_amplitude_percent: stage3 A 百分比

      // ===== 显示与保存输出 =====
      2, // display_every_n_frames: 每 2 帧刷新一次显示
      2, // gui_poll_every_n_frames: 每 2 帧轮询一次按键
      30 // target_video_fps: 目标视频保存 30fps
  };
  return p;
}

} // namespace Tools
