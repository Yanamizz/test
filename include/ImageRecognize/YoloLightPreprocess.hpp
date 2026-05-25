/**
 * @file    include/ImageRecognize/YoloLightPreprocess.hpp
 * @brief   Stage3 YOLO light-normalization preprocessing.
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

namespace ImageRecognize {

/**
 * @brief Mild CLAHE preprocessing for YOLO input.
 *
 * The input and output are BGR frames with the same size and type. CLAHE is
 * applied only on the L channel in LAB space so that color channels are not
 * equalized directly.
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
      // Camera frames in the main pipeline are expected to be BGR8. Avoid
      // surprising color conversions if an unexpected format reaches here.
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
