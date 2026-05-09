#pragma once

#include <string>

namespace Tools {

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

  double dt_max_sec;

  float pitch_abs_limit;

  bool enable_latency_profile;
  int latency_print_interval_frames;

  bool enable_motion_prediction;
  bool enable_scan_mode;
  bool enable_save_no_target_images;

  int scan_origin_hold_ms;
  double max_infer_fps;
  double scan_send_hz;
  int display_every_n_frames;
  int gui_poll_every_n_frames;
};

inline const RuntimeParams &Params() {
  static const RuntimeParams p{
      "/home/nuc/antidrone/src/model/antidrone_all.xml", // model_path
      "CPU",                                             // openvino_device_name
      "ONE_EURO",                                        // angle_filter_type
      "ALL",                                             // target_camp_mode

      1000, // capture_timeout_ms
      5,    // capture_empty_sleep_ms
      2,    // imu_read_fail_sleep_ms
      1,    // imu_send_idle_sleep_ms
      1000, // imu_buffer_max_age_ms

      0.0f,  // minimum_angle_deg
      10.0f, // max_send_delta_deg

      1.0,   // dt_max_sec
      10.0f, // pitch_abs_limit

      false, // enable_latency_profile
      100,   // latency_print_interval_frames

      false, // enable_motion_prediction
      true,  // enable_scan_mode
      true,  // enable_save_no_target_images

      1000,   // scan_origin_hold_ms
      80.0,   // max_infer_fps
      1000.0, // scan_send_hz
      2,      // display_every_n_frames
      1};     // gui_poll_every_n_frames
  return p;
}

} // namespace Tools
