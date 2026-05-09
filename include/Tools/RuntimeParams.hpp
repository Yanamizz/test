#pragma once

#include <string>

namespace Tools {

// 运行时调参集中区：只放跨线程/跨模块共享的参数，便于现场调试时统一修改。
struct RuntimeParams {
  std::string model_path; // OpenVINO 模型文件路径，通常为 .xml/.onnx
  std::string openvino_device_name; // OpenVINO 推理设备，如 CPU/GPU/AUTO
  std::string
      angle_filter_type; // 角度滤波类型，如 ONE_EURO/KF/EKF/UKF/CKF/NONE
  std::string target_camp_mode; // 目标阵营过滤模式，如 ALL/RED/BLUE

  int capture_timeout_ms; // 相机 grab 等待超时（毫秒）
  int capture_empty_sleep_ms; // 相机空帧后的短暂休眠（毫秒），降低空转 CPU 占用
  int imu_read_fail_sleep_ms; // IMU 读取失败后的休眠（毫秒），避免串口忙等
  int imu_send_idle_sleep_ms; // 无待发送控制量时发送线程休眠（毫秒）
  int imu_buffer_max_age_ms; // IMU 缓冲区保留的最长历史时长（毫秒）

  float minimum_angle_deg; // 小于该角度误差时不发送控制指令（度）
  float max_send_delta_deg; // yaw 单次发送的最大相对角度限幅（度）

  double dt_max_sec; // 帧间隔超过该值视为异常 dt，运动/速度计算会回退（秒）

  float pitch_abs_limit; // pitch 单次发送的最大相对角度限幅（度）

  bool enable_latency_profile;       // 是否打印各阶段平均延迟统计
  int latency_print_interval_frames; // 延迟统计窗口大小（帧）

  bool enable_motion_prediction; // 是否启用目标运动预测，减少目标移动造成的滞后
  bool enable_scan_mode; // 丢失目标后是否进入扫描发送模式
  bool enable_save_no_target_images; // 检测框数量异常时是否保存原图到 captures

  int scan_origin_hold_ms; // 进入扫描后先保持原点/起始位的时间（毫秒）
  double max_infer_fps; // 推理提交频率上限；<=0 表示不额外限速
  double scan_send_hz;  // 扫描模式下串口发送频率（Hz）
  int display_every_n_frames;  // 每隔多少帧刷新一次显示窗口
  int gui_poll_every_n_frames; // 每隔多少帧轮询一次 GUI 按键事件
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_all.xml", // model_path:
                                                         // 默认反无人机模型
      "CPU",      // openvino_device_name: CPU 推理，稳定优先
      "ONE_EURO", // angle_filter_type: OneEuro 兼顾平滑和响应速度
      "ALL", // target_camp_mode: 不按阵营过滤，跟踪所有类别目标

      1000, // capture_timeout_ms: 单次取帧最多等待 1000 ms
      5,    // capture_empty_sleep_ms: 空帧时休眠 5 ms 后重试
      2,    // imu_read_fail_sleep_ms: IMU 读失败时休眠 2 ms
      1,    // imu_send_idle_sleep_ms: 无控制量时发送线程休眠 1 ms
      100,  // imu_buffer_max_age_ms: IMU 历史最多保留 1000 ms

      0.0f,  // minimum_angle_deg: 0 表示任何非零误差都允许发送
      10.0f, // max_send_delta_deg: yaw 单次相对角度限制在 ±10°

      1.0,   // dt_max_sec: 超过 1 s 的帧间隔不用于实时 dt 计算
      10.0f, // pitch_abs_limit: pitch 单次相对角度限制在 ±10°

      false, // enable_latency_profile: 默认关闭延迟统计，减少控制台输出
      100, // latency_print_interval_frames: 每 100 帧打印一个统计窗口

      false, // enable_motion_prediction: 默认关闭预测，优先使用实测框
      true, // enable_scan_mode: 丢失目标后允许扫描搜索
      false, // enable_save_no_target_images: 保存无目标/多目标异常样本

      1000,   // scan_origin_hold_ms: 扫描前在原点保持 1000 ms
      80.0,   // max_infer_fps: 推理最多提交 80 FPS
      1000.0, // scan_send_hz: 扫描模式串口发送频率 1000 Hz
      2,      // display_every_n_frames: 每 2 帧刷新一次画面
      1};     // gui_poll_every_n_frames: 每帧轮询 GUI 按键
  return p;
}

} // namespace Tools
