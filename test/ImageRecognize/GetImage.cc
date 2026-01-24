#include <opencv2/opencv.hpp>

#include <iostream>

#include "CameraTask/Getimage.hpp"

int main() {
  CameraTask::GalaxyCamera camera;
  if (!camera.open()) {
    std::cerr << "无法打开 Galaxy 相机" << std::endl;
    return 1;
  }

  if (!camera.start()) {
    std::cerr << "无法开始采集" << std::endl;
    return 1;
  }

  std::cout << "按 ESC 或 q 退出" << std::endl;

  while (true) {
    cv::Mat frame = camera.grab(1000);
    if (frame.empty()) {
      continue;
    }

    cv::imshow("Galaxy Camera", frame);
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    }
  }

  camera.stop();
  camera.close();
  return 0;
}
