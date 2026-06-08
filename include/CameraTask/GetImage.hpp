/**
 * @file    include/CameraTask/GetImage.hpp
 * @brief   封装 Galaxy 相机的打开、取流、参数设置与运行时曝光配置能力。
 *
 * GalaxyCamera 负责大恒相机 SDK 生命周期、设备打开/关闭、开始/停止取流、
 * 图像 buffer 管理、曝光参数写入和相机侧 ROI 应用。该文件也维护 ROI
 * 切换后 payload 尺寸刷新与全画幅恢复逻辑，是实机相机接入的主要边界。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <DxImageProc.h>
#include <GxIAPI.h>
#include <GxPixelFormat.h>

#include "Tools/CameraRoiRuntime.hpp"

namespace CameraTask {

class GalaxyCamera {
 public:
  GalaxyCamera() = default;
  ~GalaxyCamera() { close(); }

  GalaxyCamera(const GalaxyCamera &) = delete;
  GalaxyCamera &operator=(const GalaxyCamera &) = delete;

  // 手动参数统一放到文件末尾，这里只保留操作接口。
  // 手动设置接口（优先使用这些设置函数）。
  void setWhiteBalanceAuto(bool enable) { enable_auto_white_balance = enable; }
  void setWhiteBalanceChannel(const std::string &channel_name) { wb_channel_name_ = channel_name; }
  void setWhiteBalanceChannelIndex(int idx) { wb_channel_index_ = idx; }
  void setWhiteBalanceRatio(double ratio) { white_balance_red = ratio; }
  void setExposureAuto(bool enable) { enable_auto_exposure = enable; }
  void setExposureTime(double us) { exposure_time_us = us; }
  void setGainAuto(bool enable) { enable_auto_gain = enable; }
  void setGain(double db) { gain_db = db; }
  void setRoiEnabled(bool enable) { enable_roi_ = enable; }
  void setRoi(int width, int height, int offset_x = 0, int offset_y = 0) {
    roi_width_ = width;
    roi_height_ = height;
    roi_offset_x_ = offset_x;
    roi_offset_y_ = offset_y;
  }
  void setRoiKeepCentered(bool enable) { roi_keep_centered_ = enable; }
  bool isRoiEnabled() const { return enable_roi_; }

  double getExposureTime() const { return exposure_time_us; }

  bool applyExposureTime(double us) {
    enable_auto_exposure = false;
    if (device_handle_) {
      GX_STATUS off_status = GXSetEnumValueByString(device_handle_, "ExposureAuto", "Off");
      if (off_status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(ExposureAuto:Off)");
    }
    return applyManualExposureTime(us);
  }

  bool loadRuntimeParams(const std::string &path = "camera_runtime_params.ini") {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    bool loaded = false;
    std::string line;
    while (std::getline(file, line)) {
      const auto key_pos = line.find("exposure_time_us");
      if (key_pos == std::string::npos) continue;

      const auto eq_pos = line.find('=', key_pos);
      if (eq_pos == std::string::npos) continue;

      try {
        exposure_time_us = std::stod(line.substr(eq_pos + 1));
        loaded = true;
      } catch (...) {
        std::cerr << "[GalaxyCamera] invalid exposure_time_us in " << path << ": " << line << std::endl;
      }
    }
    return loaded;
  }

  bool saveRuntimeParams(const std::string &path = "camera_runtime_params.ini") const {
    std::ofstream file(path, std::ios::trunc);
    if (!file.is_open()) {
      std::cerr << "[GalaxyCamera] failed to open runtime params file for write: " << path << std::endl;
      return false;
    }

    file << "# Runtime camera parameters\n";
    file << std::fixed << std::setprecision(1) << "exposure_time_us=" << exposure_time_us << '\n';
    return true;
  }

  bool open() {
    if (opened_) return true;

    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXInitLib");
      return false;
    }
    lib_initialized_ = true;

    uint32_t device_number = 0;
    status = GXUpdateAllDeviceList(&device_number, 1000);
    if (status != GX_STATUS_SUCCESS || device_number == 0) {
      if (status != GX_STATUS_SUCCESS) logLastError("GXUpdateAllDeviceList");
      close();
      return false;
    }

    GX_DEVICE_INFO device_info{};
    status = GXGetDeviceInfo(1, &device_info);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXGetDeviceInfo");
      close();
      return false;
    }

    GX_OPEN_PARAM open_param{};
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;

    if (device_info.emDevType == GX_DEVICE_CLASS_GEV) {
      GX_DEVICE_IP_INFO device_ip_info;
      status = GXGetDeviceIPInfo(1, &device_ip_info);
      if (status != GX_STATUS_SUCCESS) {
        logLastError("GXGetDeviceIPInfo");
        close();
        return false;
      }
      open_param.openMode = GX_OPEN_MAC;
      open_param.pszContent = device_ip_info.szMAC;
    } else {
      // U3V/USB 设备按索引打开。
      index_str_ = "1";
      open_param.openMode = GX_OPEN_INDEX;
      open_param.pszContent = const_cast<char *>(index_str_.c_str());
    }

    status = GXOpenDevice(&open_param, &device_handle_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXOpenDevice");
      close();
      return false;
    }

    // 查询颜色滤波器信息，供 Bayer 转 RGB 使用。
    GX_NODE_ACCESS_MODE access_mode;
    status = GXGetNodeAccessMode(device_handle_, "PixelColorFilter", &access_mode);
    const bool has_color_filter = (status == GX_STATUS_SUCCESS) &&
                                  ((access_mode == GX_NODE_ACCESS_MODE_WO) || (access_mode == GX_NODE_ACCESS_MODE_RO) ||
                                   (access_mode == GX_NODE_ACCESS_MODE_RW));
    if (has_color_filter) {
      GX_ENUM_VALUE emValue;
      status = GXGetEnumValue(device_handle_, "PixelColorFilter", &emValue);
      if (status == GX_STATUS_SUCCESS) {
        color_filter_ = emValue.stCurValue.nCurValue;
      } else {
        logLastError("GXGetEnumValue(PixelColorFilter)");
      }
    }

    uint32_t ds_num = 0;
    status = GXGetDataStreamNumFromDev(device_handle_, &ds_num);
    if (status != GX_STATUS_SUCCESS || ds_num < 1) {
      if (status != GX_STATUS_SUCCESS) logLastError("GXGetDataStreamNumFromDev");
      close();
      return false;
    }
    status = GXGetDataStreamHandleFromDev(device_handle_, 1, &ds_handle_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXGetDataStreamHandleFromDev");
      close();
      return false;
    }
    if (!RefreshPayloadBuffer()) {
      close();
      return false;
    }
    CacheFullFrameGeometry();

    opened_ = true;
    Tools::SetCameraRoiRuntime(false, 0, 0);
    return true;
  }

  bool start() {
    if (!opened_ || started_) return opened_;

    applyCameraParams();
    if (!RefreshPayloadBuffer()) {
      return false;
    }
    GX_STATUS status = GXSetCommandValue(device_handle_, "AcquisitionStart");
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetCommandValue(AcquisitionStart)");
      return false;
    }
    started_ = true;
    return true;
  }

  void stop() {
    if (!opened_ || !started_) return;
    GX_STATUS status = GXSetCommandValue(device_handle_, "AcquisitionStop");
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetCommandValue(AcquisitionStop)");
    }
    started_ = false;
  }

  void close() {
    stop();
    if (device_handle_) {
      GXCloseDevice(device_handle_);
      device_handle_ = nullptr;
    }
    if (lib_initialized_) {
      GXCloseLib();
      lib_initialized_ = false;
    }
    image_buffer_.reset();
    ds_handle_ = nullptr;
    payload_size_bytes_ = 0;
    opened_ = false;
    started_ = false;
    Tools::SetCameraRoiRuntime(false, 0, 0);
  }

  cv::Mat grab(int timeout_ms = 1000) {
    if (!opened_) return {};
    if (!started_ && !start()) return {};

    GX_STATUS status = GXGetImage(device_handle_, &frame_data_, timeout_ms);
    if (status != GX_STATUS_SUCCESS) {
      if (status == GX_STATUS_TIMEOUT) {
        // 超时是正常现象（相机暂时无新帧），静默重试；连续超时过多则重启采集
        if (++consecutive_timeouts_ >= 30) {
          std::cerr << "[GalaxyCamera] " << consecutive_timeouts_ << " consecutive timeouts, restarting acquisition..."
                    << std::endl;
          stop();
          start();
          consecutive_timeouts_ = 0;
        }
        return {};
      }
      consecutive_timeouts_ = 0;
      logLastError("GXGetImage");
      return {};
    }
    consecutive_timeouts_ = 0;
    if (frame_data_.nStatus != GX_FRAME_STATUS_SUCCESS) return {};

    return convertToMat();
  }

 private:
  bool RefreshPayloadBuffer() {
    if (ds_handle_ == nullptr) {
      return false;
    }

    uint32_t payload_size = 0;
    const GX_STATUS status = GXGetPayLoadSize(ds_handle_, &payload_size);
    if (status != GX_STATUS_SUCCESS || payload_size == 0) {
      if (status != GX_STATUS_SUCCESS) {
        logLastError("GXGetPayLoadSize");
      }
      return false;
    }

    if (!image_buffer_ || payload_size_bytes_ != payload_size) {
      image_buffer_.reset(new uint8_t[static_cast<size_t>(payload_size)]);
      payload_size_bytes_ = payload_size;
    }
    std::memset(&frame_data_, 0, sizeof(frame_data_));
    frame_data_.pImgBuf = image_buffer_.get();
    frame_data_.nImgSize = static_cast<int32_t>(payload_size);
    return true;
  }

	  cv::Mat convertToMat() {
    const int width = frame_data_.nWidth;
    const int height = frame_data_.nHeight;
    if (width <= 0 || height <= 0 || frame_data_.pImgBuf == nullptr) return {};

    const auto pixel_format = static_cast<int32_t>(frame_data_.nPixelFormat);
    const auto img_size = static_cast<size_t>(width) * static_cast<size_t>(height);

    if (pixel_format == GX_PIXEL_FORMAT_MONO8) {
      cv::Mat mono(height, width, CV_8UC1, frame_data_.pImgBuf);
      cv::Mat out = mono.clone();
      return postProcess(out);
    }

    if (pixel_format == GX_PIXEL_FORMAT_MONO10 || pixel_format == GX_PIXEL_FORMAT_MONO12) {
      raw8_buffer_.resize(img_size);
      if (DxRaw16toRaw8(static_cast<unsigned char *>(frame_data_.pImgBuf), raw8_buffer_.data(), width, height,
                        DX_BIT_2_9) != DX_OK) {
        return {};
      }
      cv::Mat mono(height, width, CV_8UC1, raw8_buffer_.data());
      cv::Mat out = mono.clone();
      return postProcess(out);
    }

    if (isBayer8(pixel_format)) {
      rgb_buffer_.resize(img_size * 3);
      if (DxRaw8toRGB24(static_cast<unsigned char *>(frame_data_.pImgBuf), rgb_buffer_.data(), width, height,
                        RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(color_filter_), false) != DX_OK) {
        return {};
      }
      cv::Mat rgb(height, width, CV_8UC3, rgb_buffer_.data());
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return postProcess(bgr);
    }

    if (isBayer16(pixel_format)) {
      raw8_buffer_.resize(img_size);
      if (DxRaw16toRaw8(static_cast<unsigned char *>(frame_data_.pImgBuf), raw8_buffer_.data(), width, height,
                        DX_BIT_2_9) != DX_OK) {
        return {};
      }
      rgb_buffer_.resize(img_size * 3);
      if (DxRaw8toRGB24(raw8_buffer_.data(), rgb_buffer_.data(), width, height, RAW2RGB_NEIGHBOUR,
                        DX_PIXEL_COLOR_FILTER(color_filter_), false) != DX_OK) {
        return {};
      }
      cv::Mat rgb(height, width, CV_8UC3, rgb_buffer_.data());
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return postProcess(bgr);
    }

    return {};
  }

  static bool isBayer8(int32_t fmt) {
    switch (fmt) {
      case GX_PIXEL_FORMAT_BAYER_GR8:
      case GX_PIXEL_FORMAT_BAYER_RG8:
      case GX_PIXEL_FORMAT_BAYER_GB8:
      case GX_PIXEL_FORMAT_BAYER_BG8:
        return true;
      default:
        return false;
    }
  }

  static bool isBayer16(int32_t fmt) {
    switch (fmt) {
      case GX_PIXEL_FORMAT_BAYER_GR10:
      case GX_PIXEL_FORMAT_BAYER_RG10:
      case GX_PIXEL_FORMAT_BAYER_GB10:
      case GX_PIXEL_FORMAT_BAYER_BG10:
      case GX_PIXEL_FORMAT_BAYER_GR12:
      case GX_PIXEL_FORMAT_BAYER_RG12:
      case GX_PIXEL_FORMAT_BAYER_GB12:
      case GX_PIXEL_FORMAT_BAYER_BG12:
        return true;
      default:
        return false;
    }
  }

  void logLastError(const char *where) const {
    GX_STATUS error_code = GX_STATUS_SUCCESS;
    char err_text[1024] = {0};
    size_t size = sizeof(err_text);
    const GX_STATUS ret = GXGetLastError(&error_code, err_text, &size);
    if (ret == GX_STATUS_SUCCESS) {
      std::cerr << "[GalaxyCamera] " << where << " failed: " << err_text << " (" << error_code << ")" << std::endl;
    } else {
      std::cerr << "[GalaxyCamera] " << where << " failed: GXGetLastError failed (" << ret << ")" << std::endl;
    }
  }

  void applyCameraParams() {
    if (!device_handle_) return;
    // 白平衡设置：支持自动/手动；手动时先选择通道，再写入 BalanceRatio。
    if (enable_auto_white_balance) {
      GX_STATUS status = GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", "Continuous");
      if (status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(BalanceWhiteAuto:Continuous)");
    } else {
      GX_STATUS off_status = GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", "Off");
      if (off_status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(BalanceWhiteAuto:Off)");

      // 先设置通道：优先使用名称（如 "Red"/"Green"/"Blue"），否则使用索引。
      if (!wb_channel_name_.empty()) {
        GX_STATUS sel_status = GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", wb_channel_name_.c_str());
        if (sel_status != GX_STATUS_SUCCESS) {
          logLastError("GXSetEnumValueByString(BalanceRatioSelector)");
        }
      } else {
        GX_STATUS sel_status = GXSetIntValue(device_handle_, "BalanceRatioSelector", wb_channel_index_);
        if (sel_status != GX_STATUS_SUCCESS) {
          logLastError("GXSetIntValue(BalanceRatioSelector)");
        }
      }

      // 再写入白平衡比率（节点名为 BalanceRatio）。
      if (white_balance_red > 0.0) {
        GX_STATUS ratio_status = GXSetFloatValue(device_handle_, "BalanceRatio", white_balance_red);
        if (ratio_status != GX_STATUS_SUCCESS) logLastError("GXSetFloatValue(BalanceRatio)");
      }
    }

    if (enable_auto_exposure) {
      GX_STATUS status = GXSetEnumValueByString(device_handle_, "ExposureAuto", "Continuous");
      if (status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(ExposureAuto:Continuous)");
    } else {
      GX_STATUS off_status = GXSetEnumValueByString(device_handle_, "ExposureAuto", "Off");
      if (off_status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(ExposureAuto:Off)");
      if (exposure_time_us > 0.0) {
        applyManualExposureTime(exposure_time_us);
      }
    }

    if (enable_auto_gain) {
      GX_STATUS status = GXSetEnumValueByString(device_handle_, "GainAuto", "Once");
      if (status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(GainAuto:Once)");
    } else {
      GX_STATUS off_status = GXSetEnumValueByString(device_handle_, "GainAuto", "Off");
      if (off_status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(GainAuto:Off)");
      if (gain_db >= 0.0) {
        GX_STATUS status = GXSetFloatValue(device_handle_, "Gain", gain_db);
        if (status != GX_STATUS_SUCCESS) logLastError("GXSetFloatValue(Gain)");
      }
    }

    if (enable_roi_) {
      applyRoiIfEnabled();
    } else {
      RestoreFullFrame();
    }
  }

  static int64_t ClampAlignInt(int64_t value, const GX_INT_VALUE &range) {
    int64_t v = std::clamp(value, range.nMin, range.nMax);
    if (range.nInc > 0) {
      v = range.nMin + ((v - range.nMin) / range.nInc) * range.nInc;
      v = std::clamp(v, range.nMin, range.nMax);
    }
    return v;
  }

  bool QueryIntNode(const char *name, GX_INT_VALUE *out) {
    const GX_STATUS status = GXGetIntValue(device_handle_, name, out);
    if (status != GX_STATUS_SUCCESS) {
      logLastError(name);
      return false;
    }
    return true;
  }

  void CacheFullFrameGeometry() {
    int64_t offset_x_min = 0;
    int64_t offset_y_min = 0;
    if (!ResetOffsetsToMin(&offset_x_min, &offset_y_min)) {
      return;
    }

    GX_INT_VALUE width_range{};
    GX_INT_VALUE height_range{};
    if (!QueryIntNode("Width", &width_range) ||
        !QueryIntNode("Height", &height_range)) {
      return;
    }

    full_frame_width_ = width_range.nMax;
    full_frame_height_ = height_range.nMax;
    full_frame_offset_x_min_ = offset_x_min;
    full_frame_offset_y_min_ = offset_y_min;
  }

  bool ResetOffsetsToMin(int64_t *offset_x_min = nullptr,
                         int64_t *offset_y_min = nullptr) {
    GX_INT_VALUE offset_x_range{};
    GX_INT_VALUE offset_y_range{};
    if (!QueryIntNode("OffsetX", &offset_x_range) ||
        !QueryIntNode("OffsetY", &offset_y_range)) {
      return false;
    }

    GX_STATUS status =
        GXSetIntValue(device_handle_, "OffsetX", offset_x_range.nMin);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(OffsetX@min)");
      return false;
    }
    status = GXSetIntValue(device_handle_, "OffsetY", offset_y_range.nMin);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(OffsetY@min)");
      return false;
    }

    if (offset_x_min != nullptr) {
      *offset_x_min = offset_x_range.nMin;
    }
    if (offset_y_min != nullptr) {
      *offset_y_min = offset_y_range.nMin;
    }
    return true;
  }

  void RestoreFullFrame() {
    if (!ResetOffsetsToMin()) {
      Tools::SetCameraRoiRuntime(false, 0, 0);
      return;
    }

    if (full_frame_width_ <= 0 || full_frame_height_ <= 0) {
      CacheFullFrameGeometry();
    }
    if (full_frame_width_ <= 0 || full_frame_height_ <= 0) {
      std::cerr << "[GalaxyCamera] full-frame geometry unavailable, skip restore."
                << std::endl;
      Tools::SetCameraRoiRuntime(false, 0, 0);
      return;
    }

    GX_STATUS status = GXSetIntValue(device_handle_, "Width", full_frame_width_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(Width@max)");
    }
    status = GXSetIntValue(device_handle_, "Height", full_frame_height_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(Height@max)");
    }
    status = GXSetIntValue(device_handle_, "OffsetX", full_frame_offset_x_min_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(OffsetX@restore)");
    }
    status = GXSetIntValue(device_handle_, "OffsetY", full_frame_offset_y_min_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetIntValue(OffsetY@restore)");
    }
    std::cout << "[GalaxyCamera] ROI disabled: width=" << full_frame_width_
              << " height=" << full_frame_height_
              << " offset_x=" << full_frame_offset_x_min_
              << " offset_y=" << full_frame_offset_y_min_ << std::endl;
    Tools::SetCameraRoiRuntime(false, 0, 0);
  }

  void applyRoiIfEnabled() {
    GX_INT_VALUE width_range{};
    GX_INT_VALUE height_range{};
    if (!ResetOffsetsToMin() ||
        !QueryIntNode("Width", &width_range) ||
        !QueryIntNode("Height", &height_range)) {
      std::cerr << "[GalaxyCamera] ROI nodes unavailable, skip ROI config." << std::endl;
      return;
    }

    int64_t width = ClampAlignInt(static_cast<int64_t>(std::max(1, roi_width_)), width_range);
    int64_t height = ClampAlignInt(static_cast<int64_t>(std::max(1, roi_height_)), height_range);
    int64_t offset_x = static_cast<int64_t>(std::max(0, roi_offset_x_));
    int64_t offset_y = static_cast<int64_t>(std::max(0, roi_offset_y_));

    if ((full_frame_width_ <= 0 || full_frame_height_ <= 0) &&
        roi_keep_centered_) {
      CacheFullFrameGeometry();
    }
    GX_STATUS status = GXSetIntValue(device_handle_, "Width", width);
    if (status != GX_STATUS_SUCCESS) logLastError("GXSetIntValue(Width)");
    status = GXSetIntValue(device_handle_, "Height", height);
    if (status != GX_STATUS_SUCCESS) logLastError("GXSetIntValue(Height)");

    GX_INT_VALUE offset_x_range{};
    GX_INT_VALUE offset_y_range{};
    if (!QueryIntNode("OffsetX", &offset_x_range) ||
        !QueryIntNode("OffsetY", &offset_y_range)) {
      std::cerr << "[GalaxyCamera] ROI offset nodes unavailable after resize, skip ROI offset config."
                << std::endl;
      Tools::SetCameraRoiRuntime(true, 0, 0);
      return;
    }

    if (roi_keep_centered_ && full_frame_width_ > 0 && full_frame_height_ > 0) {
      const int64_t centered_x =
          full_frame_offset_x_min_ + ((full_frame_width_ - width) / 2);
      const int64_t centered_y =
          full_frame_offset_y_min_ + ((full_frame_height_ - height) / 2);
      offset_x = centered_x;
      offset_y = centered_y;
    }

    offset_x = ClampAlignInt(offset_x, offset_x_range);
    offset_y = ClampAlignInt(offset_y, offset_y_range);

    // 使用整幅画面缓存的尺寸做边界限制，避免误用 Width/Height 当前节点的新上限。
    const int64_t full_frame_max_x =
        full_frame_offset_x_min_ + std::max<int64_t>(0, full_frame_width_ - width);
    const int64_t full_frame_max_y =
        full_frame_offset_y_min_ + std::max<int64_t>(0, full_frame_height_ - height);
    offset_x = std::clamp(offset_x, full_frame_offset_x_min_, full_frame_max_x);
    offset_y = std::clamp(offset_y, full_frame_offset_y_min_, full_frame_max_y);
    offset_x = ClampAlignInt(offset_x, offset_x_range);
    offset_y = ClampAlignInt(offset_y, offset_y_range);

    status = GXSetIntValue(device_handle_, "OffsetX", offset_x);
    if (status != GX_STATUS_SUCCESS) logLastError("GXSetIntValue(OffsetX)");
    status = GXSetIntValue(device_handle_, "OffsetY", offset_y);
    if (status != GX_STATUS_SUCCESS) logLastError("GXSetIntValue(OffsetY)");

    std::cout << "[GalaxyCamera] ROI enabled: width=" << width
              << " height=" << height
              << " offset_x=" << offset_x
              << " offset_y=" << offset_y << std::endl;
    Tools::SetCameraRoiRuntime(true, static_cast<int>(offset_x),
                               static_cast<int>(offset_y));
  }

  double normalizeExposureTime(double us) const {
    double value = std::max(1.0, us);
    if (!device_handle_) return value;

    GX_FLOAT_VALUE exposure_range{};
    const GX_STATUS status = GXGetFloatValue(device_handle_, "ExposureTime", &exposure_range);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXGetFloatValue(ExposureTime)");
      return value;
    }

    value = std::clamp(value, exposure_range.dMin, exposure_range.dMax);
    if (exposure_range.bIncIsValid && exposure_range.dInc > 0.0) {
      value =
          exposure_range.dMin + std::round((value - exposure_range.dMin) / exposure_range.dInc) * exposure_range.dInc;
      value = std::clamp(value, exposure_range.dMin, exposure_range.dMax);
    }
    return value;
  }

  bool applyManualExposureTime(double us) {
    const double normalized_us = normalizeExposureTime(us);
    if (!device_handle_) {
      exposure_time_us = normalized_us;
      return true;
    }

    GX_STATUS status = GXSetFloatValue(device_handle_, "ExposureTime", normalized_us);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetFloatValue(ExposureTime)");
      return false;
    }

    exposure_time_us = normalized_us;
    return true;
  }

  cv::Mat postProcess(cv::Mat &img) {
    if (enable_invert) {
      cv::flip(img, img, -1);  // flip both vertically and horizontally
    }
    return img;
  }

 private:
  struct BufferDeleter {
    void operator()(uint8_t *p) const { delete[] p; }
  };

  std::unique_ptr<uint8_t[], BufferDeleter> image_buffer_;
  std::vector<uint8_t> raw8_buffer_;
  std::vector<uint8_t> rgb_buffer_;

  GX_DEV_HANDLE device_handle_ = nullptr;
  GX_DS_HANDLE ds_handle_ = nullptr;
  GX_FRAME_DATA frame_data_{};
  std::string index_str_;
  bool lib_initialized_ = false;
  bool opened_ = false;
  bool started_ = false;
  int64_t color_filter_ = GX_COLOR_FILTER_NONE;
  int consecutive_timeouts_ = 0;  ///< 连续超时帧计数，用于触发自动重启
  uint32_t payload_size_bytes_ = 0;
  int64_t full_frame_width_ = 0;
  int64_t full_frame_height_ = 0;
  int64_t full_frame_offset_x_min_ = 0;
  int64_t full_frame_offset_y_min_ = 0;

 public:
  // ===== 手动配置区（统一放在文件末尾）=====
  bool enable_invert = false;              // 是否翻转图像
  bool enable_auto_white_balance = false;  // 是否持续自动白平衡
  bool enable_auto_exposure = false;       // 启动时是否执行一次自动曝光
  bool enable_auto_gain = false;           // 启动时是否执行一次自动增益
  double white_balance_red = 1.75;         // 手动白平衡红通道比例
  double exposure_time_us = 1000.0;        // 手动曝光时间（微秒）
  double gain_db = 24.0;                   // 手动增益（dB）
  std::string wb_channel_name_ = "Red";    // 白平衡通道名，可改为 "Green" / "Blue"
  int wb_channel_index_ = 0;               // 白平衡通道索引，通道名为空时使用
  bool enable_roi_ = false;                // 是否开启相机侧 ROI（裁剪画幅）
  bool roi_keep_centered_ = true;          // 是否保持中心点不动进行 ROI 裁剪
  int roi_width_ = 1280;                   // ROI 宽（像素）
  int roi_height_ = 720;                   // ROI 高（像素）
  int roi_offset_x_ = 320;                 // ROI 左上角 X 偏移（像素）
  int roi_offset_y_ = 180;                 // ROI 左上角 Y 偏移（像素）
};

}  // namespace CameraTask
