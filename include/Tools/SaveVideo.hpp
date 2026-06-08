/**
 * @file    include/Tools/SaveVideo.hpp
 * @brief   在检测到目标时按片段保存视频。
 *
 * SaveVideo 提供目标片段录像与全程录像能力，内部管理编码器、异步写入队列、
 * 输出尺寸缩放和文件滚动命名。主流程只提交原始帧或叠加帧，具体磁盘写入与
 * 后台线程调度由该模块处理。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "Tools/CpuAffinity.hpp"

namespace Tools {
namespace detail {

class AsyncVideoWriterBase {
 public:
  AsyncVideoWriterBase(int fps, cv::Size output_size, const char *log_tag,
                       int worker_aux_core_index = -1,
                       std::size_t max_pending_frames = 6)
      : fps_(std::max(1, fps)),
        output_size_(output_size),
        log_tag_(log_tag == nullptr ? "[SaveVideo]" : log_tag),
        worker_aux_core_index_(worker_aux_core_index),
        max_pending_frames_(std::max<std::size_t>(1, max_pending_frames)),
        frame_interval_(std::chrono::duration<double>(1.0 / fps_)) {}

  AsyncVideoWriterBase(const AsyncVideoWriterBase &) = delete;
  AsyncVideoWriterBase &operator=(const AsyncVideoWriterBase &) = delete;

 protected:
  ~AsyncVideoWriterBase() = default;

  void SubmitFrame(const cv::Mat &frame) {
    if (frame.empty()) {
      return;
    }
    if (!ShouldSampleNow_()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (pending_frames_.size() >= max_pending_frames_) {
        pending_frames_.pop_front();
        ++dropped_frames_;
      }
      pending_frames_.push_back(frame);
    }
    cv_.notify_one();
  }

  bool ShouldSubmitFrame() const { return IsSampleDue_(); }

  void RequestStop() {
    ResetSampling();
    {
      std::lock_guard<std::mutex> lk(mutex_);
      pending_stop_ = true;
    }
    cv_.notify_one();
  }

  void ResetSampling() {
    std::lock_guard<std::mutex> lk(mutex_);
    sampling_initialized_ = false;
  }

  void ShutdownWorker() {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      shutdown_ = true;
    }
    cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  virtual bool ReadyForOutput_() const = 0;
  virtual std::filesystem::path BuildNextOutputPath_() = 0;

  void StartWorker() {
    worker_ = std::thread([this]() { WorkerLoop_(); });
  }

 private:
  bool ShouldSampleNow_() {
    std::lock_guard<std::mutex> lk(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!sampling_initialized_) {
      sampling_initialized_ = true;
      next_sample_time_ = now + ToSteadyDuration_(frame_interval_);
      return true;
    }
    if (now < next_sample_time_) {
      return false;
    }
    next_sample_time_ = now + ToSteadyDuration_(frame_interval_);
    return true;
  }

  bool IsSampleDue_() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return IsSampleDueLocked_();
  }

  bool IsSampleDueLocked_() const {
    if (!sampling_initialized_) {
      return true;
    }
    return std::chrono::steady_clock::now() >= next_sample_time_;
  }

  static std::chrono::steady_clock::duration
  ToSteadyDuration_(const std::chrono::duration<double> &duration) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        duration);
  }

  void WorkerLoop_() {
    if (worker_aux_core_index_ >= 0) {
      Tools::BindCurrentThreadToAuxCore(
          static_cast<size_t>(worker_aux_core_index_));
    } else {
      Tools::BindCurrentThreadToAuxCores();
    }

    while (true) {
      cv::Mat frame;
      bool should_stop = false;
      bool should_shutdown = false;
      {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [&]() {
          return shutdown_ || pending_stop_ || !pending_frames_.empty();
        });
        should_shutdown = shutdown_;
        should_stop = pending_stop_;
        pending_stop_ = false;
        if (should_stop) {
          pending_frames_.clear();
        } else if (!pending_frames_.empty()) {
          frame = pending_frames_.front();
          pending_frames_.pop_front();
          should_shutdown = shutdown_ && pending_frames_.empty();
        }
      }

      if (should_stop) {
        StopWriter_();
      }

      if (!frame.empty()) {
        PrepareOutputFrame_(frame);
        if (!prepared_frame_.empty() && EnsureWriterOpen_(prepared_frame_.size())) {
          try {
            writer_.write(prepared_frame_);
          } catch (const std::exception &e) {
            std::cerr << log_tag_ << " write exception: " << e.what()
                      << std::endl;
            StopWriter_();
          }
        }
      }

      if (should_shutdown) {
        break;
      }
    }

    StopWriter_();
    if (dropped_frames_ > 0) {
      std::cout << log_tag_ << " dropped frames=" << dropped_frames_
                << std::endl;
    }
  }

  bool EnsureWriterOpen_(const cv::Size &size) {
    if (writer_.isOpened() && size == current_size_) {
      return true;
    }

    StopWriter_();
    if (!ReadyForOutput_() || size.width <= 0 || size.height <= 0) {
      return false;
    }

    const auto file_path = BuildNextOutputPath_();
    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!writer_.open(file_path.string(), fourcc, fps_, size, true)) {
      std::cerr << log_tag_ << " open writer failed: " << file_path
                << std::endl;
      return false;
    }

    current_size_ = size;
    std::cout << log_tag_ << " start: " << file_path << std::endl;
    return true;
  }

  void StopWriter_() {
    if (writer_.isOpened()) {
      writer_.release();
      std::cout << log_tag_ << " stop" << std::endl;
    }
    current_size_ = cv::Size{};
  }

  void PrepareOutputFrame_(const cv::Mat &frame) {
    const cv::Size target_size = OutputSizeForFrame_(frame);
    if (target_size.width <= 0 || target_size.height <= 0) {
      prepared_frame_ = cv::Mat{};
      return;
    }

    if (frame.size() != target_size) {
      cv::resize(frame, prepared_frame_, target_size);
      return;
    }
    prepared_frame_ = frame;
  }

  cv::Size OutputSizeForFrame_(const cv::Mat &frame) {
    if (output_size_.width > 0 && output_size_.height > 0) {
      return output_size_;
    }
    if (locked_output_size_.width <= 0 || locked_output_size_.height <= 0) {
      locked_output_size_ = frame.size();
    }
    return locked_output_size_;
  }

  int fps_;
  cv::Size output_size_{};
  std::string log_tag_;
  int worker_aux_core_index_ = -1;
  std::size_t max_pending_frames_ = 6;
  std::chrono::duration<double> frame_interval_;
  std::chrono::steady_clock::time_point next_sample_time_{};
  bool sampling_initialized_ = false;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<cv::Mat> pending_frames_;
  bool pending_stop_ = false;
  bool shutdown_ = false;
  std::thread worker_;

  cv::VideoWriter writer_;
  cv::Size current_size_{};
  cv::Size locked_output_size_{};
  cv::Mat prepared_frame_;
  std::uint64_t dropped_frames_ = 0;
};

} // namespace detail

class SaveVideoOnTarget : private detail::AsyncVideoWriterBase {
 public:
  explicit SaveVideoOnTarget(int fps = 30,
                             const std::string &base_dir = "target_videos",
                             cv::Size output_size = cv::Size())
      : detail::AsyncVideoWriterBase(fps, output_size, "[SaveVideo]"),
        base_dir_(base_dir) {
    PrepareBaseFolder();
    StartWorker();
  }

  ~SaveVideoOnTarget() {
    RequestStop();
    ShutdownWorker();
  }

  // 每帧调用一次：should_record=true 时持续录制，否则停止录制。
  void Update(const cv::Mat &frame, bool should_record) {
    if (base_dir_.empty()) {
      return;
    }

    if (!should_record) {
      if (recording_requested_) {
        recording_requested_ = false;
        RequestStop();
      }
      return;
    }

    recording_requested_ = true;
    SubmitFrame(frame);
  }

 private:
  int segment_index_ = 1;
  std::filesystem::path base_dir_;
  std::filesystem::path run_folder_;
  bool recording_requested_ = false;

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

  bool ReadyForOutput_() const override { return !run_folder_.empty(); }

  std::filesystem::path BuildNextOutputPath_() override {
    std::ostringstream oss;
    oss << run_folder_.filename().string() << "_" << std::setw(3)
        << std::setfill('0') << segment_index_++ << ".avi";
    return run_folder_ / oss.str();
  }
};

class SaveVideoFullRunStream : private detail::AsyncVideoWriterBase {
 public:
  SaveVideoFullRunStream(int fps, std::filesystem::path base_dir,
                         std::string file_prefix, std::string stream_suffix,
                         const char *log_tag, cv::Size output_size,
                         int worker_aux_core_index)
      : detail::AsyncVideoWriterBase(fps, output_size, log_tag,
                                     worker_aux_core_index),
        base_dir_(std::move(base_dir)),
        file_prefix_(std::move(file_prefix)),
        stream_suffix_(std::move(stream_suffix)) {
    StartWorker();
  }

  ~SaveVideoFullRunStream() { ShutdownWorker(); }

  bool WantsFrame() const { return ShouldSubmitFrame(); }

  void Update(const cv::Mat &frame) {
    if (base_dir_.empty() || frame.empty()) {
      return;
    }
    SubmitFrame(frame);
  }

 private:
  int file_index_ = 1;
  std::filesystem::path base_dir_;
  std::string file_prefix_;
  std::string stream_suffix_;

  bool ReadyForOutput_() const override {
    return !base_dir_.empty() && !file_prefix_.empty() &&
           !stream_suffix_.empty();
  }

  std::filesystem::path BuildNextOutputPath_() override {
    std::ostringstream oss;
    oss << file_prefix_ << "_" << stream_suffix_ << "_" << std::setw(3)
        << std::setfill('0') << file_index_++ << ".avi";
    return base_dir_ / oss.str();
  }
};

class SaveVideoFullRun {
 public:
  explicit SaveVideoFullRun(int fps = 30,
                            const std::string &base_dir = "full_run_videos",
                            cv::Size output_size = cv::Size(),
                            int worker_aux_core_index = 4)
      : fps_(std::max(1, fps)),
        base_dir_(base_dir),
        output_size_(output_size),
        worker_aux_core_index_(worker_aux_core_index) {
    PrepareBaseFolder();
    if (ReadyForOutput_()) {
      const int raw_worker_aux_core_index = std::max(0, worker_aux_core_index_);
      const int overlay_worker_aux_core_index = raw_worker_aux_core_index + 1;
      raw_stream_ = std::make_unique<SaveVideoFullRunStream>(
          fps_, base_dir_, file_prefix_, "raw", "[SaveVideoFullRunRaw]",
          output_size_, raw_worker_aux_core_index);
      overlay_stream_ = std::make_unique<SaveVideoFullRunStream>(
          fps_, base_dir_, file_prefix_, "overlay",
          "[SaveVideoFullRunOverlay]", output_size_,
          overlay_worker_aux_core_index);
    }
  }

  bool WantsOverlayFrame() const {
    return overlay_stream_ && overlay_stream_->WantsFrame();
  }

  void UpdateRaw(const cv::Mat &frame) {
    if (raw_stream_) {
      raw_stream_->Update(frame);
    }
  }

  void UpdateOverlay(const cv::Mat &frame) {
    if (overlay_stream_) {
      overlay_stream_->Update(frame);
    }
  }

  void Update(const cv::Mat &frame) { UpdateOverlay(frame); }

 private:
  int fps_ = 30;
  std::filesystem::path base_dir_;
  cv::Size output_size_{};
  int worker_aux_core_index_ = 4;
  std::string file_prefix_;
  std::unique_ptr<SaveVideoFullRunStream> raw_stream_;
  std::unique_ptr<SaveVideoFullRunStream> overlay_stream_;

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

  bool ReadyForOutput_() const {
    return !base_dir_.empty() && !file_prefix_.empty();
  }
};

} // namespace Tools
