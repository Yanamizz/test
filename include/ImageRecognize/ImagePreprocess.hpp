/**
 * @file    include/ImageRecognize/ImagePreprocess.hpp
 * @brief   提供图像预处理工具，将输入的cv::Mat标准化为ONNX模型可用的张量数据。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace ImagePreprocess {

/**
 * @brief 预处理结果的容器，包含平铺的数据和对应的形状信息。
 */
struct PreprocessResult {
  std::vector<float> data;                       ///< 预处理后的平铺数据，按CHW顺序存储
  std::array<int64_t, 4> shape{1, 3, 640, 640};  ///< 图像的形状：{batch, channel, height, width}
};

/**
 * @brief 图像预处理类：负责尺寸变换、颜色空间转换、归一化以及HWC->CHW的重排。
 */
class ImagePreprocess {
 public:
  ImagePreprocess() = default;

  explicit ImagePreprocess(cv::Size inputSize) : inputSize_(inputSize) {}

  /**
   * @brief 对输入图像执行预处理并返回张量化结果。
   * @param[in] input 输入的BGR格式图像，使用cv::Mat存储。
   * @returns 包含CHW顺序数据和形状信息的预处理结果；若输入为空则返回空数据和默认形状。
   */
  PreprocessResult run(const cv::Mat &input) const {
    PreprocessResult result{};
    result.shape = {1, 3, inputSize_.height, inputSize_.width};

    if (input.empty()) {
      return result;
    }

    cv::Mat resized_;
    cv::resize(input, resized_, inputSize_);  // 调整图像大小

    cv::Mat rgbImage_;
    cv::cvtColor(resized_, rgbImage_, cv::COLOR_BGR2RGB);  // 转换颜色空间 BGR -> RGB
    cv::Mat floatImage_;
    rgbImage_.convertTo(floatImage_, CV_32F, 1.0 / 255.0);  // 归一化处理

    const int channels_ = floatImage_.channels();
    const int height_ = floatImage_.rows;
    const int width_ = floatImage_.cols;

    result.data.resize(static_cast<size_t>(channels_ * height_ * width_));

    std::vector<cv::Mat> splitChannels_;
    cv::split(floatImage_, splitChannels_);
    size_t offset_ = 0;
    for (int c = 0; c < channels_; ++c) {
      const float *channelPtr_ = splitChannels_[c].ptr<float>(0);
      const size_t channelSize_ = static_cast<size_t>(height_ * width_);
      std::copy(channelPtr_, channelPtr_ + channelSize_, result.data.begin() + offset_);  // HWC 转 CHW
      offset_ += channelSize_;
    }

    return result;
  }

 private:
  cv::Size inputSize_{640, 640};  ///< 模型期望的输入尺寸
};

}  // namespace ImagePreprocess
