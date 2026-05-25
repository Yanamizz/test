/**
 * @file    include/Tools/SaveImage.hpp
 * @brief   在无目标场景下按间隔保存图像，便于问题回溯与样本收集。
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

class SaveImageOnNoTarget {
 public:
  explicit SaveImageOnNoTarget(int save_interval_frames = 5, const std::string &base_dir = "captures")
      : save_interval_frames_(std::max(1, save_interval_frames)), base_dir_(base_dir) {
    PrepareRunFolder();
  }

  // 每帧调用一次：当 should_save=true 时按间隔保存图像
  void Update(const cv::Mat &frame, bool should_save) {
    if (frame.empty()) return;
    if (run_folder_.empty()) return;

    if (!should_save) {
      triggered_frame_count_ = 0;
      return;
    }

    ++triggered_frame_count_;
    if (triggered_frame_count_ % save_interval_frames_ != 0) return;

    const std::string filename = BuildImageName();
    const std::filesystem::path file_path = run_folder_ / filename;

    WriteImage(file_path, frame);
  }

  // 保存实际送入 stage3 推理的 CLAHE 图像。
  void UpdateClahe(const cv::Mat &clahe_frame, bool should_save) {
    if (clahe_frame.empty()) return;
    if (run_folder_.empty()) return;

    if (!should_save) {
      triggered_frame_count_ = 0;
      return;
    }

    ++triggered_frame_count_;
    if (triggered_frame_count_ % save_interval_frames_ != 0) return;

    const std::string base_name = BuildImageName();
    WriteImage(run_folder_ / ("clahe_" + base_name), clahe_frame);
  }

  const std::filesystem::path &RunFolder() const { return run_folder_; }

 private:
  int save_interval_frames_;
  int triggered_frame_count_ = 0;
  int saved_index_ = 1;
  std::filesystem::path base_dir_;
  std::filesystem::path run_folder_;

  void WriteImage(const std::filesystem::path &file_path,
                  const cv::Mat &frame) const {
    try {
      if (!cv::imwrite(file_path.string(), frame)) {
        std::cerr << "[SaveImage] 保存失败: " << file_path << std::endl;
      }
    } catch (const std::exception &e) {
      std::cerr << "[SaveImage] 保存异常: " << e.what() << std::endl;
    }
  }

  void PrepareRunFolder() {
    try {
      std::filesystem::create_directories(base_dir_);

      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);

      std::tm tm_buf{};
#ifdef _WIN32
      localtime_s(&tm_buf, &t);
#else
      localtime_r(&t, &tm_buf);
#endif

      std::ostringstream folder_name;
      folder_name << "frame_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");

      run_folder_ = base_dir_ / folder_name.str();
      std::filesystem::create_directories(run_folder_);

      std::cout << "[SaveImage] 本次运行图像目录: " << run_folder_ << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[SaveImage] 创建目录失败: " << e.what() << std::endl;
      run_folder_.clear();
    }
  }

  std::string BuildImageName() {
    std::ostringstream oss;
    oss << run_folder_.filename().string() << "_" << std::setw(2) << std::setfill('0') << saved_index_++ << ".jpg";
    return oss.str();
  }
};

}  // namespace Tools
