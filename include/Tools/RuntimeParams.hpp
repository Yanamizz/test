/**
 * @file    include/Tools/RuntimeParams.hpp
 * @brief   定义图像识别主流程使用的默认运行参数集合。
 */

#pragma once

#include <string>

namespace Tools {

struct RuntimeParams {
  std::string model_path;
  std::string stage12_model_path;
  std::string stage3_model_path;
  std::string openvino_device_name;
  std::string angle_filter_type;
  std::string target_camp_mode;

  int capture_timeout_ms;
  double stage12_exposure_time_us;
  double stage3_exposure_time_us;
  int stage3_switch_target_lost_delay_ms;
  int capture_empty_sleep_ms;
  int imu_read_fail_sleep_ms;
  int imu_send_idle_sleep_ms;
  int imu_buffer_max_age_ms;

  float minimum_angle_deg;
  float max_send_delta_deg;
  double dt_max_sec;
  float pitch_abs_limit;

  bool enable_latency_profile;
  int latency_print_interval_frames;

  bool enable_scan_mode;
  bool enable_save_no_target_images;
  bool enable_save_target_videos;
  bool enable_display;
  bool enable_calibration_sliders;
  bool enable_send_log;

  int scan_origin_hold_ms;
  double max_infer_fps;
  double scan_send_hz;
  int display_every_n_frames;
  int gui_poll_every_n_frames;
  int target_video_fps;
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_all_int8_openvino_model/antidrone_all.xml",
      "/home/nuc/antidrone/src/model/antidrone_all_int8_openvino_model/antidrone_all.xml",
      "/home/nuc/antidrone/src/model/antidrone_stage3_int8_openvino_model/antidrone_stage3.xml",
      "CPU",
      "ONE_EURO",
      "ALL",

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
      1.0,    // dt_max_sec
      10.0f,  // pitch_abs_limit

      false,  // enable_latency_profile
      100,    // latency_print_interval_frames

      true,   // enable_scan_mode
      false,  // enable_save_no_target_images
      false,  // enable_save_target_videos
      true,   // enable_display
      true,   // enable_calibration_sliders
      true,   // enable_send_log

      1000,   // scan_origin_hold_ms
      0.0,    // max_infer_fps
      200.0,  // scan_send_hz
      2,      // display_every_n_frames
      2,      // gui_poll_every_n_frames
      30      // target_video_fps
  };
  return p;
}

}  // namespace Tools
