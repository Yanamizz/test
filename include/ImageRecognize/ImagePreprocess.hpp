/**
 * @file    include/ImageRecognize/ImagePreprocess.hpp
 * @brief   执行图像缩放、颜色转换、归一化和张量重排等推理前预处理。
 *
 * 该文件把 OpenCV 图像转换为模型输入张量需要的 layout 和数值范围，
 * 包括 letterbox/resize、BGR/RGB 转换、归一化和 CHW 排布。预处理结果
 * 同时保留缩放与填充信息，供后处理阶段把模型坐标映射回原图坐标。
 */

#pragma once

#include "ImageRecognize/YoloLightPreprocess.hpp"

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
  ImagePreprocess() : ImagePreprocess(DefaultInputSize()) {}

  /**
   * @brief 通过指定输入尺寸构造。
   * @param[in] inputSize  模型期望的输入宽高。
   */
  explicit ImagePreprocess(cv::Size inputSize,
                           bool enable_light_preprocess = false)
      : inputSize_(inputSize),
        enable_light_preprocess_(enable_light_preprocess) {}

  /**
   * @brief 对输入图像执行预处理并写入输出结果（就地复用 out->data 容量）。
   */
  void run(const cv::Mat &input, PreprocessResult *out) {
    if (out == nullptr) {
      return;
    }

    out->shape = {1, 3, inputSize_.height, inputSize_.width};
    out->scale = 1.0f;
    out->pad_x = 0;
    out->pad_y = 0;
    out->content_width = 0;
    out->content_height = 0;

    if (input.empty()) {
      out->data.clear();
      return;
    }

    const float scale_x =
        static_cast<float>(inputSize_.width) / static_cast<float>(input.cols);
    const float scale_y =
        static_cast<float>(inputSize_.height) / static_cast<float>(input.rows);
    out->scale = std::min(scale_x, scale_y);

    out->content_width = std::max(
        1, static_cast<int>(std::round(input.cols * out->scale)));
    out->content_height = std::max(
        1, static_cast<int>(std::round(input.rows * out->scale)));
    out->content_width = std::min(out->content_width, inputSize_.width);
    out->content_height = std::min(out->content_height, inputSize_.height);

    out->pad_x = (inputSize_.width - out->content_width) / 2;
    out->pad_y = (inputSize_.height - out->content_height) / 2;

    cv::resize(input, resized_,
               cv::Size(out->content_width, out->content_height));
    if (enable_light_preprocess_) {
      light_preprocessor_.PreprocessForYolo(resized_, &resized_);
    }

    if (letterboxed_.size() != inputSize_ || letterboxed_.type() != input.type()) {
      letterboxed_ = cv::Mat(inputSize_, input.type());
    }
    letterboxed_.setTo(cv::Scalar(114, 114, 114));
    resized_.copyTo(letterboxed_(
        cv::Rect(out->pad_x, out->pad_y, out->content_width,
                 out->content_height)));

    cv::cvtColor(letterboxed_, rgbImage_, cv::COLOR_BGR2RGB); // BGR -> RGB
    rgbImage_.convertTo(floatImage_, CV_32F, 1.0 / 255.0);    // 归一化到 [0,1]

    constexpr int kChannels = 3;
    const int height = floatImage_.rows;
    const int width = floatImage_.cols;
    const int plane_size = height * width;
    const std::size_t total_values =
        static_cast<std::size_t>(kChannels * height * width);
    out->data.resize(total_values);
    float *output = out->data.data();

    // 低延迟优先：避免并行调度开销，直接按行顺序写入 CHW 缓冲区。
    for (int h = 0; h < height; ++h) {
      const cv::Vec3f *src_row = floatImage_.ptr<cv::Vec3f>(h);
      float *dst_r = output + h * width;
      float *dst_g = output + plane_size + h * width;
      float *dst_b = output + 2 * plane_size + h * width;

      for (int w = 0; w < width; ++w) {
        const cv::Vec3f &pixel = src_row[w];
        dst_r[w] = pixel[0];
        dst_g[w] = pixel[1];
        dst_b[w] = pixel[2];
      }
    }
  }

  PreprocessResult run(const cv::Mat &input) {
    PreprocessResult result{};
    run(input, &result);
    return result;
  }

private:
  static cv::Size DefaultInputSize();

  cv::Mat resized_;
  cv::Mat letterboxed_;
  cv::Mat rgbImage_;
  cv::Mat floatImage_;
  cv::Size inputSize_; ///< 模型期望的输入尺寸
  bool enable_light_preprocess_ = false;
  YoloLightPreprocessor light_preprocessor_;
};

inline cv::Size ImagePreprocess::DefaultInputSize() {
  // ===== 手动配置区（统一放在文件末尾）=====
  return {480, 480}; // 模型默认输入尺寸
}

} // namespace ImageRecognize
