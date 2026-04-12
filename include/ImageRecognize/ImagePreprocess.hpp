/**
 * @file    include/ImageRecognize/ImagePreprocess.hpp
 * @brief   图像预处理：resize、颜色转换、归一化与 HWC→CHW 重排。
 *
 * @details
 * - 将 `cv::Mat`（BGR，HWC）调整为模型期望尺寸（默认 640×640）。
 * - 转换为 RGB，归一化到 [0,1]，并展平为 NCHW 连续向量。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace ImageRecognize {

/**
 * @brief 预处理结果的容器，包含平铺的数据和对应的形状信息。
 */
struct PreprocessResult {
  std::vector<float> data;         ///< 预处理后的平铺数据，按 CHW 顺序存储
  std::array<int64_t, 4> shape{};  ///< 图像形状：{batch, channel, height, width}
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
   * @returns          CHW 顺序数据与对应形状；若输入为空则返回空数据与默认形状。
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
    cv::cvtColor(resized_, rgbImage_, cv::COLOR_BGR2RGB);  // BGR -> RGB

    cv::Mat floatImage_;
    rgbImage_.convertTo(floatImage_, CV_32F, 1.0 / 255.0);  // 归一化到 [0,1]

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

  cv::Size inputSize_;  ///< 模型期望的输入尺寸
};

inline cv::Size ImagePreprocess::DefaultInputSize() {
  // ===== 手动配置区（统一放在文件末尾）=====
  return {640, 640};  // 模型默认输入尺寸
}

}  // namespace ImageRecognize
