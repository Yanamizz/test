/**
 * @file include/ImageRecognize/OutputDataProcess.hpp
 *
 * @brief 本文件功能为处理图像识别模型的输出数据。
 *
 * @brief 主要实现功能：
 * @brief 处理识别后的数组output_tensor_，获得识别框的数据。
 *
 *
 *
 *
 *
 */

#pragma once
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace OutputDataProcess {
struct DataProcessResult {
  std::vector<std::array<float, 5>> boxes;  // {x1,y1,x2,y2,score}
};

class OutputDataProcess {
 public:
  OutputDataProcess() = default;

  DataProcessResult run(Ort::Value &output_tensor_, cv::Size original_image_size_) const {
    DataProcessResult result_{};

    float *output_data_ = output_tensor_.GetTensorMutableData<float>();
    Ort::TensorTypeAndShapeInfo output_info_ = output_tensor_.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_shape_ = output_info_.GetShape();

    int num_detections_ = output_shape_[1];
    int detections_size_ = output_shape_[2];

    float scale_x_ = static_cast<float>(original_image_size_.width) / static_cast<float>(cut_size_.width);
    float scale_y_ = static_cast<float>(original_image_size_.height) / static_cast<float>(cut_size_.height);

    for (int i = 0; i < num_detections_; ++i) {
      float cx = output_data_[i * detections_size_ + 0];
      float cy = output_data_[i * detections_size_ + 1];
      float w = output_data_[i * detections_size_ + 2];
      float h = output_data_[i * detections_size_ + 3];
      float score = output_data_[i * detections_size_ + 4];

      if (score > set_score_) {
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        float x2 = cx + w / 2.0f;
        float y2 = cy + h / 2.0f;

        x1 *= scale_x_;
        y1 *= scale_y_;
        x2 *= scale_x_;
        y2 *= scale_y_;

        result_.boxes.push_back({x1, y1, x2, y2, score});
      }
    }
    return result_;
  }

 private:
  cv::Size cut_size_{640.f, 640.f};
  float set_score_{0.9f};
};
}  // namespace OutputDataProcess