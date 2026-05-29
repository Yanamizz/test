/**
 * @file    include/ImageRecognize/YoloLightPreprocess.hpp
 * @brief   提供 stage3 YOLO 的光照归一化预处理。
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

namespace ImageRecognize {

/**
 * @brief 对 YOLO 输入执行轻量 CLAHE 预处理。
 *
 * 输入与输出均为尺寸和类型一致的 BGR 图像帧。CLAHE 仅作用于 LAB 空间的
 * L 通道，从而避免直接对颜色通道做均衡化。
 */
class YoloLightPreprocessor {
 public:
  explicit YoloLightPreprocessor(double clip_limit = 2.0,
                                  cv::Size tile_grid_size = cv::Size(8, 8))
      : clahe_(cv::createCLAHE(clip_limit, tile_grid_size)),
        lab_channels_(3) {}

  void PreprocessForYolo(const cv::Mat &frame, cv::Mat *output) {
    if (output == nullptr) {
      return;
    }
    if (frame.empty()) {
      output->release();
      return;
    }

    if (frame.type() != CV_8UC3) {
      // 主流程中的相机图像默认应为 BGR8。若有非预期格式传入，这里直接拷贝，
      // 避免发生难以察觉的颜色空间转换。
      frame.copyTo(*output);
      return;
    }

    cv::cvtColor(frame, lab_image_, cv::COLOR_BGR2Lab);
    cv::split(lab_image_, lab_channels_);
    clahe_->apply(lab_channels_[0], lab_channels_[0]);
    cv::merge(lab_channels_, lab_image_);
    cv::cvtColor(lab_image_, *output, cv::COLOR_Lab2BGR);
  }

  cv::Mat PreprocessForYolo(const cv::Mat &frame) {
    cv::Mat output;
    PreprocessForYolo(frame, &output);
    return output;
  }

 private:
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat lab_image_;
  std::vector<cv::Mat> lab_channels_;
};

} // namespace ImageRecognize
