#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv) {
  if (argc != 2) {
    printf("用法：DisplayImage.out <图像路径>\n");
    return -1;
  }

  Mat image;
  image = imread(argv[1], IMREAD_COLOR);

  if (!image.data) {
    printf("没有图像数据 \n");
    return -1;
  }
  namedWindow("显示图像", WINDOW_AUTOSIZE);
  imshow("显示图像", image);

  waitKey(0);

  return 0;
}