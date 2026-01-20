/**
 * @file    include\ImageRecognize\ImagePredict.hpp
 * @brief   本文件功能onnx模型图像识别。
 *
 * @date    2026-01-19
 *
 * @brief   主要实现功能：
 * @brief   传入图像（cv::Mat），识别后输出识别结果数组。
 *
 * @brief   传入图像（cv::Mat），使用预处理头文件（ImagePreprocess.hpp）进行预处理
 * @brief   每次传入一张图片，传出一组数据
 * @brief   处理识别后的数组output_tensor,获得识别框的数据
 * @brief   output_shape : [1, 5, 8400]   {[batch_size], num_detection[x,y,w,h,score], detections_size}
 * @brief   传出的数据为识别框比例恢复后的坐标数据及置信度[x1,y1,x2,y2,score]
 *

 */
#pragma once

#include "ImageRecognize/ImagePreprocess.hpp"
#include <array>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace ImagePredict {

struct PredictResult {
  std::vector<std::array<float, 5>> boxes;  // {x1,y1,x2,y2,score}
};

class ImagePredict {
 public:
  ImagePredict() = default;

  PredictResult run(const cv::Mat& origin_image) const {
    PredictResult result{};
    auto pre_image = preprocessor_.run(origin_image);

    return result;
  }

 private:
  ImagePreprocess::ImagePreprocess preprocessor_{cv::Size(640, 640)};
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "predict"};
  Ort::SessionOptions session_options_;
  // Ort::Session session_{env_, model_path.c_str(), session_options_}; // 需要 model 路径时再初始化
};

}  // namespace ImagePredict