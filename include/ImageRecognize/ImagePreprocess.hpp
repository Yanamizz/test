/**
 * @file    include/ImageRecognize/ImagePreprocess.hpp
 * @brief   执行图像缩放、颜色转换、归一化和张量重排等推理前预处理。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ImageRecognize {

/**
 * @brief 预处理结果的容器，包含平铺的数据和对应的形状信息。
 */
struct PreprocessResult {
  std::vector<float> data; ///< 预处理后的平铺数据，按 CHW 顺序存储
  std::array<int64_t, 4> shape{}; ///< 图像形状：{batch, channel, height, width}
  float scale = 1.0f; ///< 原图等比例缩放到 letterbox 内容区域时使用的缩放系数
  int pad_x = 0; ///< letterbox 左侧填充像素
  int pad_y = 0; ///< letterbox 顶部填充像素
  int content_width = 0; ///< letterbox 中有效内容宽度
  int content_height = 0; ///< letterbox 中有效内容高度
};

/**
 * @brief 图像预处理类：负责尺寸变换、颜色空间转换、归一化以及 HWC→CHW 的重排。
 */
class ImagePreprocess {
public:
  ImagePreprocess() : inputSize_(DefaultInputSize()) {}

  /**
   * @brief 通过指定输入尺寸构造。
   * @param[in] inputSize  模型期望的输入宽高。
   */
  explicit ImagePreprocess(cv::Size inputSize) : inputSize_(inputSize) {}

  /**
   * @brief 对输入图像执行预处理并返回张量化结果。
   * @param[in] input  输入图像（BGR，HWC）。
   * @returns          CHW
   * 顺序数据与对应形状；若输入为空则返回空数据与默认形状。
   */
  PreprocessResult run(const cv::Mat &input) const {
    PreprocessResult result{};
    result.shape = {1, 3, inputSize_.height, inputSize_.width};

    if (input.empty()) {
      return result;
    }

    const float scale_x =
        static_cast<float>(inputSize_.width) / static_cast<float>(input.cols);
    const float scale_y =
        static_cast<float>(inputSize_.height) / static_cast<float>(input.rows);
    result.scale = std::min(scale_x, scale_y);

    result.content_width = std::max(
        1, static_cast<int>(std::round(input.cols * result.scale)));
    result.content_height = std::max(
        1, static_cast<int>(std::round(input.rows * result.scale)));
    result.content_width = std::min(result.content_width, inputSize_.width);
    result.content_height = std::min(result.content_height, inputSize_.height);

    result.pad_x = (inputSize_.width - result.content_width) / 2;
    result.pad_y = (inputSize_.height - result.content_height) / 2;

    cv::Mat resized_;
    cv::resize(input, resized_,
               cv::Size(result.content_width, result.content_height));

    cv::Mat letterboxed_(inputSize_, input.type(), cv::Scalar(114, 114, 114));
    resized_.copyTo(letterboxed_(
        cv::Rect(result.pad_x, result.pad_y, result.content_width,
                 result.content_height)));

    cv::Mat rgbImage_;
    cv::cvtColor(letterboxed_, rgbImage_, cv::COLOR_BGR2RGB); // BGR -> RGB

    cv::Mat floatImage_;
    rgbImage_.convertTo(floatImage_, CV_32F, 1.0 / 255.0); // 归一化到 [0,1]

    const int channels_ = 3;
    const int height_ = floatImage_.rows;
    const int width_ = floatImage_.cols;
    const int plane_size_ = height_ * width_;

    result.data.resize(static_cast<size_t>(channels_ * height_ * width_));
    float *output = result.data.data();

    // 低延迟优先：避免并行调度开销，直接按行顺序写入 CHW 缓冲区。
    for (int h = 0; h < height_; ++h) {
      const cv::Vec3f *src_row = floatImage_.ptr<cv::Vec3f>(h);
      float *dst_r = output + 0 * plane_size_ + h * width_;
      float *dst_g = output + 1 * plane_size_ + h * width_;
      float *dst_b = output + 2 * plane_size_ + h * width_;

      for (int w = 0; w < width_; ++w) {
        const cv::Vec3f &pixel = src_row[w];
        dst_r[w] = pixel[0];
        dst_g[w] = pixel[1];
        dst_b[w] = pixel[2];
      }
    }

    return result;
  }

private:
  static cv::Size DefaultInputSize();

  cv::Size inputSize_; ///< 模型期望的输入尺寸
};

inline cv::Size ImagePreprocess::DefaultInputSize() {
  // ===== 手动配置区（统一放在文件末尾）=====
  return {480, 480}; // 模型默认输入尺寸
}

} // namespace ImageRecognize
