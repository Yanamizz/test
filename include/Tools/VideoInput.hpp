/**
 * @file    include/Tools/VideoInput.hpp
 * @brief   命令行视频输入的路径解析、格式判断与打开工具。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <string_view>

namespace Tools {

inline std::string NormalizeVideoPathFromCli(const std::string &raw_path) {
  const auto begin = raw_path.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = raw_path.find_last_not_of(" \t\r\n");
  std::string path = raw_path.substr(begin, end - begin + 1);
  if (path.size() >= 2) {
    const char first = path.front();
    const char last = path.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      path = path.substr(1, path.size() - 2);
    }
  }
  return path;
}

inline bool IsCommonVideoExtension(const std::filesystem::path &path) {
  static constexpr std::array<std::string_view, 12> kSupportedExtensions = {
      ".avi", ".mp4", ".mkv", ".mov", ".wmv", ".flv",
      ".webm", ".m4v", ".mpg", ".mpeg", ".ts",  ".m2ts"};

  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return std::any_of(kSupportedExtensions.begin(), kSupportedExtensions.end(),
                     [&](std::string_view supported_ext) {
                       return ext == supported_ext;
                     });
}

inline bool OpenVideoCaptureFromPath(const std::string &raw_path,
                                     cv::VideoCapture *capture,
                                     std::string *error_message) {
  if (capture == nullptr) {
    if (error_message != nullptr) {
      *error_message = "internal error: null video capture pointer";
    }
    return false;
  }

  const std::string path = NormalizeVideoPathFromCli(raw_path);
  if (path.empty()) {
    if (error_message != nullptr) {
      *error_message = "empty video path";
    }
    return false;
  }

  const std::filesystem::path fs_path(path);
  if (!std::filesystem::exists(fs_path)) {
    if (error_message != nullptr) {
      *error_message = "video file does not exist: " + path;
    }
    return false;
  }

  if (!std::filesystem::is_regular_file(fs_path)) {
    if (error_message != nullptr) {
      *error_message = "path is not a regular file: " + path;
    }
    return false;
  }

  if (!IsCommonVideoExtension(fs_path)) {
    std::cerr << "[VideoInput] warning: uncommon extension for video file: "
              << fs_path.extension().string()
              << ", trying to open with OpenCV anyway." << std::endl;
  }

  if (!capture->open(path, cv::CAP_ANY)) {
    if (error_message != nullptr) {
      *error_message = "OpenCV failed to open video: " + path;
    }
    return false;
  }

  if (!capture->isOpened()) {
    if (error_message != nullptr) {
      *error_message = "video capture is not opened after open call: " + path;
    }
    return false;
  }

  return true;
}

} // namespace Tools
