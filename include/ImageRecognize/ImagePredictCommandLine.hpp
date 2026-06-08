/**
 * @file    include/ImageRecognize/ImagePredictCommandLine.hpp
 * @brief   解析图像识别主程序的命令行开关并生成运行选项。
 *
 * 该文件定义 ImagePredict 启动参数解析结果，覆盖显示、标定滑块、录像、
 * 扫描、发送日志、视频输入等开关。解析逻辑会把命令行覆盖值与
 * RuntimeParams 默认值合并，供 src/ImagePredict.cc 初始化各线程和功能。
 */

#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace ImageRecognize {

struct ImagePredictCommandLineOptions {
  std::optional<bool> enable_display;
  std::optional<bool> enable_calibration_sliders;
  std::optional<bool> enable_send_log;
  std::optional<bool> enable_latency_profile;
  std::optional<bool> enable_scan_mode;
  std::optional<bool> enable_save_no_target_images;
  std::optional<bool> enable_save_full_run_video;
  std::optional<std::string> video_path;
};

namespace detail {

inline bool StartsWith(std::string_view text, std::string_view prefix) {
  return text.size() >= prefix.size() &&
         text.substr(0, prefix.size()) == prefix;
}

inline bool ParseBoolValue(std::string_view value, bool fallback) {
  if (value == "0" || value == "false" || value == "off" || value == "no") {
    return false;
  }
  if (value == "1" || value == "true" || value == "on" || value == "yes") {
    return true;
  }
  return fallback;
}

inline bool ParseBoolValue(std::string_view value, std::optional<bool> fallback) {
  return ParseBoolValue(value, fallback.value_or(false));
}

} // namespace detail

inline ImagePredictCommandLineOptions
ParseImagePredictCommandLine(int argc, char **argv) {
  ImagePredictCommandLineOptions options{};
  constexpr std::string_view kEnableDisplayPrefix = "--enable-display=";
  constexpr std::string_view kDisplayPrefix = "--display=";
  constexpr std::string_view kCalibrationSlidersPrefix =
      "--calibration-sliders=";
  constexpr std::string_view kLatencyProfilePrefix = "--latency-profile=";
  constexpr std::string_view kScanModePrefix = "--scan-mode=";
  constexpr std::string_view kSaveNoTargetImagesPrefix =
      "--save-no-target-images=";
  constexpr std::string_view kSaveFullRunVideoPrefix =
      "--save-full-run-video=";
  constexpr std::string_view kSendLogPrefix = "--send-log=";
  constexpr std::string_view kVideoPrefix = "--video=";

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i] != nullptr ? argv[i] : "";

    if (arg == "--no-display" || arg == "--disable-display") {
      options.enable_display = false;
      continue;
    }

    if (arg == "--display" || arg == "--enable-display") {
      options.enable_display = true;
      continue;
    }

    if (detail::StartsWith(arg, kEnableDisplayPrefix)) {
      options.enable_display = detail::ParseBoolValue(
          arg.substr(kEnableDisplayPrefix.size()), options.enable_display);
      continue;
    }

    if (detail::StartsWith(arg, kDisplayPrefix)) {
      options.enable_display = detail::ParseBoolValue(
          arg.substr(kDisplayPrefix.size()), options.enable_display);
      continue;
    }

    if (arg == "--calibration-sliders" ||
        arg == "--enable-calibration-sliders") {
      options.enable_calibration_sliders = true;
      continue;
    }

    if (arg == "--no-calibration-sliders" ||
        arg == "--disable-calibration-sliders") {
      options.enable_calibration_sliders = false;
      continue;
    }

    if (detail::StartsWith(arg, kCalibrationSlidersPrefix)) {
      options.enable_calibration_sliders = detail::ParseBoolValue(
          arg.substr(kCalibrationSlidersPrefix.size()),
          options.enable_calibration_sliders);
      continue;
    }

    if (arg == "--latency-profile" || arg == "--enable-latency-profile") {
      options.enable_latency_profile = true;
      continue;
    }

    if (arg == "--no-latency-profile" || arg == "--disable-latency-profile") {
      options.enable_latency_profile = false;
      continue;
    }

    if (detail::StartsWith(arg, kLatencyProfilePrefix)) {
      options.enable_latency_profile = detail::ParseBoolValue(
          arg.substr(kLatencyProfilePrefix.size()),
          options.enable_latency_profile);
      continue;
    }

    if (arg == "--scan-mode" || arg == "--enable-scan-mode") {
      options.enable_scan_mode = true;
      continue;
    }

    if (arg == "--no-scan-mode" || arg == "--disable-scan-mode") {
      options.enable_scan_mode = false;
      continue;
    }

    if (detail::StartsWith(arg, kScanModePrefix)) {
      options.enable_scan_mode = detail::ParseBoolValue(
          arg.substr(kScanModePrefix.size()), options.enable_scan_mode);
      continue;
    }

    if (arg == "--save-no-target-images" ||
        arg == "--enable-save-no-target-images") {
      options.enable_save_no_target_images = true;
      continue;
    }

    if (arg == "--no-save-no-target-images" ||
        arg == "--disable-save-no-target-images") {
      options.enable_save_no_target_images = false;
      continue;
    }

    if (detail::StartsWith(arg, kSaveNoTargetImagesPrefix)) {
      options.enable_save_no_target_images = detail::ParseBoolValue(
          arg.substr(kSaveNoTargetImagesPrefix.size()),
          options.enable_save_no_target_images);
      continue;
    }

    if (arg == "--save-full-run-video" ||
        arg == "--enable-save-full-run-video") {
      options.enable_save_full_run_video = true;
      continue;
    }

    if (arg == "--no-save-full-run-video" ||
        arg == "--disable-save-full-run-video") {
      options.enable_save_full_run_video = false;
      continue;
    }

    if (detail::StartsWith(arg, kSaveFullRunVideoPrefix)) {
      options.enable_save_full_run_video = detail::ParseBoolValue(
          arg.substr(kSaveFullRunVideoPrefix.size()),
          options.enable_save_full_run_video);
      continue;
    }

    if (arg == "--send-log" || arg == "--enable-send-log") {
      options.enable_send_log = true;
      continue;
    }

    if (arg == "--no-send-log" || arg == "--disable-send-log") {
      options.enable_send_log = false;
      continue;
    }

    if (detail::StartsWith(arg, kSendLogPrefix)) {
      options.enable_send_log = detail::ParseBoolValue(
          arg.substr(kSendLogPrefix.size()), options.enable_send_log);
      continue;
    }

    if (arg == "--video") {
      if (i + 1 < argc && argv[i + 1] != nullptr) {
        options.video_path = argv[i + 1];
        ++i;
      }
      continue;
    }

    if (detail::StartsWith(arg, kVideoPrefix)) {
      options.video_path = std::string(arg.substr(kVideoPrefix.size()));
      continue;
    }
  }

  return options;
}

} // namespace ImageRecognize
