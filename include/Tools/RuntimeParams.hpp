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
      "src/model/antidrone_26n_int8_openvino_model/antidrone_26n.xml",
      "src/model/antidrone_26n_int8_openvino_model/antidrone_26n.xml",
      "src/model/antidrone_stage3_int8_openvino_model/antidrone_stage3.xml",
      "CPU",
      "ONE_EURO",
      "ALL",

      1000,
      1000.0,
      1000.0,
      5,
      2,
      1,
      100,

      0.0f,
      10.0f,

      1.0,

      10.0f,

      false,
      100,

      false,
      true,
      false,

      1000,
      80.0,
      1000.0,
      2,
      1};
  return p;
}

} // namespace Tools
