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
                             const std::string &base_dir = "target_videos",
                             cv::Size output_size = cv::Size())
      : fps_(std::max(1, fps)),
        base_dir_(base_dir),
        output_size_(output_size) {
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

    PrepareOutputFrame(frame);
    if (prepared_frame_.empty()) {
      return;
    }

    if (!writer_.isOpened() || prepared_frame_.size() != current_size_) {
      StopCurrentSegment();
      if (!StartNewSegment(prepared_frame_.size())) {
        return;
      }
    }

    try {
      writer_.write(prepared_frame_);
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
  cv::Size output_size_{};
  cv::Mat prepared_frame_;

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
    const int kFourCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

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

  void PrepareOutputFrame(const cv::Mat &frame) {
    if (output_size_.width > 0 && output_size_.height > 0 &&
        frame.size() != output_size_) {
      cv::resize(frame, prepared_frame_, output_size_);
      return;
    }
    prepared_frame_ = frame;
  }
};

class SaveVideoFullRun {
 public:
  explicit SaveVideoFullRun(int fps = 30,
                            const std::string &base_dir = "full_run_videos",
                            cv::Size output_size = cv::Size())
      : fps_(std::max(1, fps)),
        base_dir_(base_dir),
        output_size_(output_size) {
    PrepareBaseFolder();
  }

  ~SaveVideoFullRun() { Stop(); }

  void Update(const cv::Mat &frame) {
    if (base_dir_.empty() || frame.empty()) {
      return;
    }

    PrepareOutputFrame(frame);
    if (prepared_frame_.empty()) {
      return;
    }

    if (!writer_.isOpened() || prepared_frame_.size() != current_size_) {
      Stop();
      if (!Start(prepared_frame_.size())) {
        return;
      }
    }

    try {
      writer_.write(prepared_frame_);
    } catch (const std::exception &e) {
      std::cerr << "[SaveVideoFullRun] write exception: " << e.what()
                << std::endl;
      Stop();
    }
  }

 private:
  int fps_;
  int file_index_ = 1;
  cv::Size current_size_{};
  cv::VideoWriter writer_;
  std::filesystem::path base_dir_;
  std::string file_prefix_;
  cv::Size output_size_{};
  cv::Mat prepared_frame_;

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
      folder_name << "full_run_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
      file_prefix_ = folder_name.str();

      std::cout << "[SaveVideoFullRun] output dir: " << base_dir_
                << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[SaveVideoFullRun] create folder failed: " << e.what()
                << std::endl;
      file_prefix_.clear();
      base_dir_.clear();
    }
  }

  std::string BuildFileName() {
    std::ostringstream oss;
    oss << file_prefix_ << "_" << std::setw(3)
        << std::setfill('0') << file_index_++ << ".avi";
    return oss.str();
  }

  bool Start(const cv::Size &size) {
    if (base_dir_.empty() || file_prefix_.empty() || size.width <= 0 ||
        size.height <= 0) {
      return false;
    }

    const auto file_path = base_dir_ / BuildFileName();
    const int kFourCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!writer_.open(file_path.string(), kFourCC, fps_, size, true)) {
      std::cerr << "[SaveVideoFullRun] open writer failed: " << file_path
                << std::endl;
      return false;
    }

    current_size_ = size;
    std::cout << "[SaveVideoFullRun] start: " << file_path << std::endl;
    return true;
  }

  void Stop() {
    if (writer_.isOpened()) {
      writer_.release();
      std::cout << "[SaveVideoFullRun] stop" << std::endl;
    }
    current_size_ = cv::Size{};
  }

  void PrepareOutputFrame(const cv::Mat &frame) {
    if (output_size_.width > 0 && output_size_.height > 0 &&
        frame.size() != output_size_) {
      cv::resize(frame, prepared_frame_, output_size_);
      return;
    }
    prepared_frame_ = frame;
  }
};

}  // namespace Tools
