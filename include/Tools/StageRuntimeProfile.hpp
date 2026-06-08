/**
 * @file    include/Tools/StageRuntimeProfile.hpp
 * @brief   收口 stage1/2 与 stage3 的运行时差异配置。
 *
 * 本文件把不同阶段会变化的运行资源整理成只读 profile，包括模型路径、
 * 曝光时间、stage3 光照预处理开关、相机 ROI 参数、扫描发送频率和
 * ScanController 配置。主流程通过 MakeStageRuntimeProfile() 获取当前
 * 阶段的统一快照，避免在 ImagePredict.cc 中反复散落 stage3 条件判断。
 * 这里不保存跨帧状态，也不直接操作相机/推理器/串口，只负责把
 * RuntimeParams 中的默认参数转换成阶段语义明确的运行配置。
 */

#pragma once

#include <algorithm>
#include <string>

#include "Tools/RuntimeParams.hpp"
#include "Tools/ScanController.hpp"

namespace Tools {

enum class RuntimeStage {
  Stage12,
  Stage3,
};

inline RuntimeStage RuntimeStageFromBool(bool stage3_mode) {
  return stage3_mode ? RuntimeStage::Stage3 : RuntimeStage::Stage12;
}

inline bool IsStage3(RuntimeStage stage) {
  return stage == RuntimeStage::Stage3;
}

inline const char *RuntimeStageDisplayName(RuntimeStage stage) {
  return IsStage3(stage) ? "stage3" : "stage1/2";
}

struct StageCameraRoiProfile {
  bool enabled = false;
  bool keep_centered = false;
  int width = 0;
  int height = 0;
  int offset_x = 0;
  int offset_y = 0;
};

struct StageScanProfile {
  double configured_send_hz = 0.0;
  double effective_send_hz = 1.0;
  double yaw_speed_deg_per_sec = 0.0;
  int origin_hold_ms = 0;
  float pitch_wavelength_percent = 100.0f;
  float pitch_amplitude_percent = 100.0f;
  ScanController::Config controller_config{};
};

struct StageRuntimeProfile {
  RuntimeStage stage = RuntimeStage::Stage12;
  const std::string *model_path = nullptr;
  double exposure_time_us = 0.0;
  bool enable_light_preprocess = false;
  int switch_target_lost_delay_ms = 0;
  StageCameraRoiProfile roi{};
  StageScanProfile scan{};

  const char *DisplayName() const { return RuntimeStageDisplayName(stage); }
  bool UsesStage3Resources() const { return IsStage3(stage); }
};

inline StageRuntimeProfile MakeStageRuntimeProfile(RuntimeStage stage,
                                                   double scan_send_hz_cap) {
  const auto &params = Params();
  const bool stage3_mode = IsStage3(stage);

  StageRuntimeProfile profile{};
  profile.stage = stage;
  profile.model_path =
      stage3_mode ? &params.stage3_model_path : &params.stage12_model_path;
  profile.exposure_time_us = stage3_mode ? params.stage3_exposure_time_us
                                         : params.stage12_exposure_time_us;
  profile.enable_light_preprocess = stage3_mode;
  profile.switch_target_lost_delay_ms =
      stage3_mode ? std::max(0, params.stage3_switch_target_lost_delay_ms) : 0;

  profile.roi.enabled = stage3_mode && params.stage3_enable_camera_roi;
  profile.roi.keep_centered = params.stage3_roi_keep_centered;
  profile.roi.width = params.stage3_roi_width;
  profile.roi.height = params.stage3_roi_height;
  profile.roi.offset_x = params.stage3_roi_offset_x;
  profile.roi.offset_y = params.stage3_roi_offset_y;

  profile.scan.configured_send_hz =
      stage3_mode ? params.stage3_scan_send_hz : params.scan_send_hz;
  profile.scan.effective_send_hz =
      std::clamp(profile.scan.configured_send_hz, 1.0,
                 std::max(1.0, scan_send_hz_cap));
  profile.scan.yaw_speed_deg_per_sec =
      std::max(0.01, stage3_mode ? params.stage3_scan_yaw_speed_deg_per_sec
                                 : params.scan_yaw_speed_deg_per_sec);
  profile.scan.origin_hold_ms =
      std::max(0, stage3_mode ? params.stage3_scan_origin_hold_ms
                              : params.scan_origin_hold_ms);
  profile.scan.pitch_wavelength_percent =
      static_cast<float>(stage3_mode ? params.stage3_scan_pitch_wavelength_percent
                                     : params.scan_pitch_wavelength_percent);
  profile.scan.pitch_amplitude_percent =
      static_cast<float>(stage3_mode ? params.stage3_scan_pitch_amplitude_percent
                                     : params.scan_pitch_amplitude_percent);
  profile.scan.controller_config =
      stage3_mode ? MakeStage3ScanControllerConfig()
                  : MakeDefaultScanControllerConfig();
  profile.scan.controller_config.tick_rate_hz =
      static_cast<float>(std::max(profile.scan.effective_send_hz, 1.0));
  profile.scan.controller_config.yaw_step_deg_per_tick =
      static_cast<float>(profile.scan.yaw_speed_deg_per_sec /
                         std::max(profile.scan.controller_config.tick_rate_hz,
                                  1.0f));
  return profile;
}

inline StageRuntimeProfile MakeStageRuntimeProfile(bool stage3_mode,
                                                   double scan_send_hz_cap) {
  return MakeStageRuntimeProfile(RuntimeStageFromBool(stage3_mode),
                                 scan_send_hz_cap);
}

} // namespace Tools
