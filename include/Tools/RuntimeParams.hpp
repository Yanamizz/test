/**
 * @file    include/Tools/RuntimeParams.hpp
 * @brief   定义图像识别主流程使用的默认运行参数集合。
 */

#pragma once

#include <string>

namespace Tools {

struct RuntimeParams {
  std::string model_path;            ///< 兼容/日志用的主模型路径
  std::string stage12_model_path;    ///< 初始阶段和第二阶段使用的 OpenVINO 模型
  std::string stage3_model_path;     ///< 第三阶段锁定后切换使用的 OpenVINO 模型
  std::string openvino_device_name;  ///< OpenVINO 推理设备名，例如 CPU/AUTO/GPU
  std::string angle_filter_type;     ///< 角度滤波器类型，见
                                     ///< AngleCalculator::ParseFilterType
  std::string target_camp_mode;      ///< 跟踪目标阵营，支持 RED/BLUE/ALL 等

  int capture_timeout_ms;                  ///< 相机单次抓帧超时时间（毫秒）
  double stage12_exposure_time_us;         ///< 第一/第二阶段默认曝光时间（微秒）
  double stage3_exposure_time_us;          ///< 第三阶段默认曝光时间（微秒）
  int stage3_switch_target_lost_delay_ms;  ///< 进入第三阶段后目标丢失多久再切换模型
  int capture_empty_sleep_ms;              ///< 抓到空帧后的短暂休眠时间（毫秒）
  int imu_read_fail_sleep_ms;              ///< IMU 读取失败或重连失败后的休眠时间（毫秒）
  int imu_send_idle_sleep_ms;              ///< 无待发送控制量时发送线程的休眠时间（毫秒）
  int imu_buffer_max_age_ms;               ///< IMU 缓冲区保留的最大数据年龄（毫秒）

  float minimum_angle_deg;   ///< 小于该绝对角度偏差时不发送云台修正（度）
  float max_send_delta_deg;  ///< 单次 yaw 发送偏差限幅（度）

  double dt_max_sec;  ///< 图像帧间隔超过该值时丢弃实时 dt，使用滤波器回退 dt（秒）

  float pitch_abs_limit;  ///< 单次 pitch 发送偏差限幅（度）

  bool enable_latency_profile;        ///< 是否启用各阶段耗时统计
  int latency_print_interval_frames;  ///< 耗时统计打印间隔（帧）

  bool enable_scan_mode;              ///< 丢失目标后是否启用扫描搜索
  bool enable_save_no_target_images;  ///< 是否保存无目标时的异常图片
  bool enable_display;                ///< 是否启用显示窗口
  bool enable_calibration_sliders;    ///< 是否启用标定滑块
  bool enable_send_log;               ///< 是否输出发送日志

  int scan_origin_hold_ms;      ///< 扫描开始前停在起始点的时间（毫秒）
  double max_infer_fps;         ///< 推理提交帧率上限，<=0 表示不主动限速
  double scan_send_hz;          ///< 扫描模式下云台命令发送频率（Hz）
  int display_every_n_frames;   ///< 每隔多少帧刷新一次显示窗口
  int gui_poll_every_n_frames;  ///< 每隔多少帧轮询一次 GUI 按键/滑块
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_all_int8_openvino_model/"
      "antidrone_all.xml",  // model_path
      "/home/nuc/antidrone/src/model/antidrone_all_int8_openvino_model/"
      "antidrone_all.xml",  // stage12_model_path
      "/home/nuc/antidrone/src/model/antidrone_stage3_int8_openvino_model/"
      "antidrone_stage3.xml",  // stage3_model_path
      "CPU",                   // openvino_device_name
      "ONE_EURO",              // angle_filter_type
      "ALL",                   // target_camp_mode

      1000,    // capture_timeout_ms
      1000.0,  // stage12_exposure_time_us
      4000.0,  // stage3_exposure_time_us
      500,     // stage3_switch_target_lost_delay_ms
      5,       // capture_empty_sleep_ms
      2,       // imu_read_fail_sleep_ms
      1,       // imu_send_idle_sleep_ms
      100,     // imu_buffer_max_age_ms

      0.0f,   // minimum_angle_deg
      10.0f,  // max_send_delta_deg

      1.0,  // dt_max_sec

      10.0f,  // pitch_abs_limit

      false,  // enable_latency_profile
      100,    // latency_print_interval_frames

      true,   // enable_scan_mode
      false,  // enable_save_no_target_images
      true,   // enable_display
      true,   // enable_calibration_sliders
      true,   // enable_send_log

      1000,   // scan_origin_hold_ms
      0.0,    // max_infer_fps
      200.0,  // scan_send_hz
      2,      // display_every_n_frames
      2};     // gui_poll_every_n_frames
  return p;
}

}  // namespace Tools
