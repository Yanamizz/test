#include <opencv2/opencv.hpp>
#include <iostream>
#include "ImageRecognize/ImagePredict.hpp"
#include <string>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <chrono>
#include <vector>

struct boxs {
  std::vector<std::array<float, 5>> boxes;  // 每个检测框包含 [x1, y1, x2, y2, score]
};

int main() {
  try {
    // 加载测试图像
    std::string image_path = "/home/hanni/code/rm/test/ImageRecognize/data/test.jpg";
    std::string model_path = "/home/hanni/code/rm/test/ImageRecognize/model/best.onnx";
    cv::Mat image = cv::imread(image_path);

    if (image.empty()) {
      std::cerr << "无法加载图像: " << image_path << std::endl;
      return -1;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    boxs boxes = ImagePredict::ImagePredict predictor.predict(image, model_path);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::string window_name = std::to_string(duration.count()) + "ms";
    std::cout << "推理时间: " << duration.count() << " 毫秒" << std::endl;

    for (const auto &box : boxes.boxes) {
      cv::rectangle(image, cv::Point(static_cast<int>(box[0]), static_cast<int>(box[1])),
                    cv::Point(static_cast<int>(box[2]), static_cast<int>(box[3])), cv::Scalar(0, 255, 0), 2);
      std::string score_text = std::to_string(box[4]);
      cv::putText(image, score_text, cv::Point(static_cast<int>(box[0]), static_cast<int>(box[1]) - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow(window_name, image);
    cv::waitKey(0);
  }

  catch (const Ort::Exception &e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}