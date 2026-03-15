
#pragma once

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <DxImageProc.h>
#include <GxIAPI.h>
#include <GxPixelFormat.h>

namespace CameraTask {

class GalaxyCamera {
 public:
  GalaxyCamera() = default;
  ~GalaxyCamera() { close(); }

  GalaxyCamera(const GalaxyCamera &) = delete;
  GalaxyCamera &operator=(const GalaxyCamera &) = delete;

  // User-configurable settings (set before open/start)
  bool enable_invert = true;              ///< Whether to flip image vertically and horizontally
  bool enable_auto_white_balance = true;  ///< Use one-time auto white balance on start
  bool enable_auto_exposure = false;      ///< Use one-time auto exposure on start
  bool enable_auto_gain = false;          ///< Use one-time auto gain on start
  double white_balance_red = 1.75;        ///< Red balance ratio, used when auto white balance is off
  double exposure_time_us = 4500.0;       ///< Exposure time (microseconds), used when auto exposure is off
  double gain_db = 0.0;                   ///< Gain value (dB), used when auto gain is off
  // 手动设置接口（优先使用这些 setter）
  void setWhiteBalanceAuto(bool enable) { enable_auto_white_balance = enable; }
  void setWhiteBalanceChannel(const std::string &channel_name) { wb_channel_name_ = channel_name; }
  void setWhiteBalanceChannelIndex(int idx) { wb_channel_index_ = idx; }
  void setWhiteBalanceRatio(double ratio) { white_balance_red = ratio; }
  void setExposureAuto(bool enable) { enable_auto_exposure = enable; }
  void setExposureTime(double us) { exposure_time_us = us; }
  void setGainAuto(bool enable) { enable_auto_gain = enable; }
  void setGain(double db) { gain_db = db; }

  bool open() {
    if (opened_) return true;

    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXInitLib");
      return false;
    }

    uint32_t device_number = 0;
    status = GXUpdateAllDeviceList(&device_number, 1000);
    if (status != GX_STATUS_SUCCESS || device_number == 0) {
      if (status != GX_STATUS_SUCCESS) logLastError("GXUpdateAllDeviceList");
      GXCloseLib();
      return false;
    }

    GX_DEVICE_INFO device_info{};
    status = GXGetDeviceInfo(1, &device_info);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXGetDeviceInfo");
      GXCloseLib();
      return false;
    }

    GX_OPEN_PARAM open_param{};
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;

    if (device_info.emDevType == GX_DEVICE_CLASS_GEV) {
      GX_DEVICE_IP_INFO device_ip_info;
      status = GXGetDeviceIPInfo(1, &device_ip_info);
      if (status != GX_STATUS_SUCCESS) {
        logLastError("GXGetDeviceIPInfo");
        GXCloseLib();
        return false;
      }
      open_param.openMode = GX_OPEN_MAC;
      open_param.pszContent = device_ip_info.szMAC;
    } else {
      // U3V/USB devices: open by index
      index_str_ = "1";
      open_param.openMode = GX_OPEN_INDEX;
      open_param.pszContent = const_cast<char *>(index_str_.c_str());
    }

    status = GXOpenDevice(&open_param, &device_handle_);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXOpenDevice");
      GXCloseLib();
      device_handle_ = nullptr;
      return false;
    }

    // Query color filter (for Bayer to RGB)
    GX_NODE_ACCESS_MODE access_mode;
    status = GXGetNodeAccessMode(device_handle_, "PixelColorFilter", &access_mode);
    has_color_filter_ = (status == GX_STATUS_SUCCESS) &&
                        ((access_mode == GX_NODE_ACCESS_MODE_WO) || (access_mode == GX_NODE_ACCESS_MODE_RO) ||
                         (access_mode == GX_NODE_ACCESS_MODE_RW));
    if (has_color_filter_) {
      GX_ENUM_VALUE emValue;
      status = GXGetEnumValue(device_handle_, "PixelColorFilter", &emValue);
      if (status == GX_STATUS_SUCCESS) {
        color_filter_ = emValue.stCurValue.nCurValue;
      } else {
        logLastError("GXGetEnumValue(PixelColorFilter)");
      }
    }

    GX_DS_HANDLE ds_handle = nullptr;
    uint32_t ds_num = 0;
    status = GXGetDataStreamNumFromDev(device_handle_, &ds_num);
    if (status != GX_STATUS_SUCCESS || ds_num < 1) {
      if (status != GX_STATUS_SUCCESS) logLastError("GXGetDataStreamNumFromDev");
      close();
      return false;
    }
    status = GXGetDataStreamHandleFromDev(device_handle_, 1, &ds_handle);
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXGetDataStreamHandleFromDev");
      close();
      return false;
    }

    uint32_t payload_size = 0;
    status = GXGetPayLoadSize(ds_handle, &payload_size);
    if (status != GX_STATUS_SUCCESS || payload_size == 0) {
      if (status != GX_STATUS_SUCCESS) logLastError("GXGetPayLoadSize");
      close();
      return false;
    }

    image_buffer_.reset(new uint8_t[static_cast<size_t>(payload_size)]);
    std::memset(&frame_data_, 0, sizeof(frame_data_));
    frame_data_.pImgBuf = image_buffer_.get();
    frame_data_.nImgSize = static_cast<int32_t>(payload_size);

    opened_ = true;
    return true;
  }

  bool start() {
    if (!opened_ || started_) return opened_;

    applyCameraParams();
    GX_STATUS status = GXSetCommandValue(device_handle_, "AcquisitionStart");
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetCommandValue(AcquisitionStart)");
      return false;
    }
    started_ = true;
    return true;
  }

  // 性能调优接口：允许外部开启/关闭去畸变（去畸变较耗时）
  void setUndistort(bool enable) { enable_undistort_ = enable; }

  void stop() {
    if (!opened_ || !started_) return;
    GX_STATUS status = GXSetCommandValue(device_handle_, "AcquisitionStop");
    if (status != GX_STATUS_SUCCESS) {
      logLastError("GXSetCommandValue(AcquisitionStop)");
    }
    started_ = false;
  }

  void close() {
    if (!opened_) return;
    stop();
    if (device_handle_) {
      GXCloseDevice(device_handle_);
      device_handle_ = nullptr;
    }
    GXCloseLib();
    image_buffer_.reset();
    opened_ = false;
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
      cv::Mat out = bgr.clone();
      return postProcess(out);
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
      cv::Mat out = bgr.clone();
      return postProcess(out);
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
    // 白平衡设置：支持自动/手动；手动时先选择通道（selector），再设置 BalanceRatio
    if (enable_auto_white_balance) {
      GX_STATUS status = GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", "Once");
      if (status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(BalanceWhiteAuto:Once)");
    } else {
      GX_STATUS off_status = GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", "Off");
      if (off_status != GX_STATUS_SUCCESS) logLastError("GXSetEnumValueByString(BalanceWhiteAuto:Off)");

      // 先设置通道：优先使用名称（如 "Red"/"Green"/"Blue"），否则使用索引
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

      // 再写入比率（节点名 BalanceRatio）
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
        GX_STATUS status = GXSetFloatValue(device_handle_, "ExposureTime", exposure_time_us);
        if (status != GX_STATUS_SUCCESS) logLastError("GXSetFloatValue(ExposureTime)");
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
  }

  cv::Mat postProcess(cv::Mat &img) {
    // 去畸变（如果启用且内参已设置）
    if (enable_undistort_ && !img.empty() && !camera_matrix_.empty()) {
      cv::Mat undistorted;
      cv::undistort(img, undistorted, camera_matrix_, dist_coeffs_);
      img = undistorted;
    }

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
  GX_FRAME_DATA frame_data_{};
  std::string index_str_;
  bool opened_ = false;
  bool started_ = false;
  bool has_color_filter_ = false;
  int64_t color_filter_ = GX_COLOR_FILTER_NONE;
  int consecutive_timeouts_ = 0;  ///< 连续超时帧计数，用于触发自动重启

  // 相机内参与畸变系数（用于去畸变）
  cv::Mat camera_matrix_ =
      (cv::Mat_<double>(3, 3) << 1576.303044, 0.0, 952.451125, 0.0, 1578.069737, 599.901423, 0.0, 0.0, 1.0);
  cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.275212, 0.210437, -0.000083, 0.000589, 0.0);
  bool enable_undistort_ = true;  // 默认启用去畸变
  // -------- 手动默认设置 (可直接在此处修改) --------
  // 白平衡：优先使用通道名（wb_channel_name_），若置空则使用索引(wb_channel_index_)
  std::string wb_channel_name_ = "Red";  // 可改为 "Green"/"Blue" 或置空使用索引
  int wb_channel_index_ = 0;             // 通道索引，若使用索引则修改此值
  // 曝光与增益的默认手动值（已对外暴露为 public 成员）
  // white_balance_red, exposure_time_us, gain_db 已在 public 区定义，可直接修改
  // -----------------------------------------------
};

}  // namespace CameraTask
