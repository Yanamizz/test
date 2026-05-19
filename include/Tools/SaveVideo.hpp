/**
 * @file    include/Tools/SaveVideo.hpp
 * @brief   Save video segments while a target is detected.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

namespace Tools {

class SaveVideoOnTarget {
 public:
  explicit SaveVideoOnTarget(int fps = 30,
                             const std::string &base_dir = "target_videos")
      : fps_(std::max(1, fps)), base_dir_(base_dir) {
    PrepareBaseFolder();
  }

  ~SaveVideoOnTarget() { StopCurrentSegment(); }

  // Call once per frame. Record while should_record=true, stop otherwise.
  void Update(const cv::Mat &frame, bool should_record) {
    if (base_dir_.empty()) return;

    if (!should_record) {
      StopCurrentSegment();
      return;
    }

    if (frame.empty()) {
      return;
    }

    if (!writer_.isOpened() || frame.size() != current_size_) {
      StopCurrentSegment();
      if (!StartNewSegment(frame.size())) {
        return;
      }
    }

    try {
      writer_.write(frame);
    } catch (const std::exception &e) {
      std::cerr << "[SaveVideo] write exception: " << e.what() << std::endl;
      StopCurrentSegment();
    }
  }

 private:
  int fps_;
  int segment_index_ = 1;
  cv::Size current_size_{};
  cv::VideoWriter writer_;
  std::filesystem::path base_dir_;
  std::filesystem::path run_folder_;

  void PrepareBaseFolder() {
    try {
      std::filesystem::create_directories(base_dir_);

      const auto now = std::chrono::system_clock::now();
      const std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm_buf{};
#ifdef _WIN32
      localtime_s(&tm_buf, &t);
#else
      localtime_r(&t, &tm_buf);
#endif

      std::ostringstream folder_name;
      folder_name << "target_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
      run_folder_ = base_dir_ / folder_name.str();
      std::filesystem::create_directories(run_folder_);

      std::cout << "[SaveVideo] run folder: " << run_folder_ << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[SaveVideo] create folder failed: " << e.what()
                << std::endl;
      run_folder_.clear();
      base_dir_.clear();
    }
  }

  std::string BuildSegmentName() {
    std::ostringstream oss;
    oss << run_folder_.filename().string() << "_" << std::setw(3)
        << std::setfill('0') << segment_index_++ << ".avi";
    return oss.str();
  }

  bool StartNewSegment(const cv::Size &size) {
    if (run_folder_.empty() || size.width <= 0 || size.height <= 0) {
      return false;
    }

    const auto file_path = run_folder_ / BuildSegmentName();
    constexpr int kFourCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    if (!writer_.open(file_path.string(), kFourCC, fps_, size, true)) {
      std::cerr << "[SaveVideo] open writer failed: " << file_path
                << std::endl;
      return false;
    }

    current_size_ = size;
    std::cout << "[SaveVideo] start segment: " << file_path << std::endl;
    return true;
  }

  void StopCurrentSegment() {
    if (writer_.isOpened()) {
      writer_.release();
      std::cout << "[SaveVideo] stop segment" << std::endl;
    }
    current_size_ = cv::Size{};
  }
};

}  // namespace Tools
