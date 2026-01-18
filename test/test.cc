#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  std::string path = "data/test.jpg";
  if (argc > 1) {
    path = argv[1];
  }

  cv::Mat image = cv::imread(path);
  if (image.empty()) {
    std::cerr << "无法读取图片: " << path << " (请检查路径或文件是否存在)" << std::endl;
    return 1;
  }

  cv::imshow("Image", image);
  cv::waitKey(0);
  return 0;
}