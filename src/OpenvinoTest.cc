#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

#include "ImageRecognize/ImagePredict_OPENVINO.hpp"
#include "ImageRecognize/ImageShow.hpp"

namespace {

struct RuntimeParams {
  std::string image_path;
  std::string model_path;
  std::string device_name;
};

RuntimeParams ParseArgs(int argc, char **argv) {
  RuntimeParams params{
      "", "/home/nuc/antidrone/src/model/antidrone_26n.onnx", "CPU"};

  if (argc < 2) {
    throw std::runtime_error(
        "Usage: ./bin/OpenvinoTest <image_path> [model_path] [device_name]");
  }

  params.image_path = argv[1];
  if (argc >= 3) {
    params.model_path = argv[2];
  }
  if (argc >= 4) {
    params.device_name = argv[3];
  }
  return params;
}

} // namespace

int main(int argc, char **argv) {
  try {
    const RuntimeParams params = ParseArgs(argc, argv);

    cv::Mat image = cv::imread(params.image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      std::cerr << "Failed to read image: " << params.image_path << std::endl;
      return -1;
    }

    ImageRecognize::ImagePredict predictor(params.model_path,
                                           params.device_name);
    ImageRecognize::PredictResult result = predictor.run(image);

    ImageRecognize::ImageShow::DrawNow(image, result, 0.0);
    ImageRecognize::ImageShow::ShowFrame(image);
    while (!ImageRecognize::ImageShow::WaitForExit()) {
    }
  } catch (const std::exception &e) {
    std::cerr << "OpenvinoTest exception: " << e.what() << std::endl;
    return -2;
  }

  return 0;
}
