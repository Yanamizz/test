/**
 * @file    include\ImageRecognize\ImagePreprocess.hpp
 * @brief   本文件功能图像预处理。
 *
 * @date    2026-01-18
 *
 * @brief   主要实现功能：
 * @brief   传入图像（cv::Mat），进行预处理，输出处理后的数组{batch, channel, height, width}作为onnx模型的输入。
 *
 *
 *
 * @brief   预处理包括：
 * @brief   1. 调整图像大小（1280*720）。
 * @brief   2. 归一化处理（像素值缩放到0-1之间）。
 * @brief   3. 转换颜色空间（BGR转RGB）。
 * @brief   4. 转换数据格式（HWC转CHW）。
 * @brief   5. 增加batch维度。
 *

 */

#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace ImagePreprocess {

struct PreprocessResult {
  std::vector<float> data;
  std::array<int64_t, 4> shape{1, 3, 640, 640};  ///< 图像的形状：{batch, channel, height, width}
};

class ImagePreprocess {
 public:
  ImagePreprocess() = default;

  explicit ImagePreprocess(cv::Size inputSize) : inputSize_(inputSize) {}

  PreprocessResult run(const cv::Mat& input) const {
    PreprocessResult result{};
    result.shape = {1, 3, inputSize_.height, inputSize_.width};

    if (input.empty()) {
      return result;
    }

    cv::Mat resized;
    cv::resize(input, resized, inputSize_);  // 调整图像大小

    cv::Mat floatImage;
    resized.convertTo(floatImage, CV_32F, 1.0 / 255.0);  // 归一化处理

    const int channels = floatImage.channels();
    const int height = floatImage.rows;
    const int width = floatImage.cols;

    result.data.resize(static_cast<size_t>(channels * height * width));

    std::vector<cv::Mat> splitChannels;
    cv::split(floatImage, splitChannels);
    size_t offset = 0;
    for (int c = 0; c < channels; ++c) {
      const float* channelPtr = splitChannels[c].ptr<float>(0);
      const size_t channelSize = static_cast<size_t>(height * width);
      std::copy(channelPtr, channelPtr + channelSize, result.data.begin() + offset);  // HWC 转 CHW
      offset += channelSize;
    }

    return result;
  }

 private:
  cv::Size inputSize_{640, 640};
};

}  // namespace ImagePreprocess
