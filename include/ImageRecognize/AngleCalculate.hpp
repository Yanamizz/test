#pragma once
#include <cmath>
#include <iostream>

#include "SerialTask/Common.hpp"

struct CameraParams {
  double fx;  // 焦距 fx
  double fy;  // 焦距 fy
  double cx;  // 主点坐标 cx
  double cy;  // 主点坐标 cy
};

struct DetectionResult {
  double x;         // 检测框左上角 x
  double y;         // 检测框左上角 y
  double w;         // 检测框宽度
  double h;         // 检测框高度
  double center_x;  // 检测框中心 x
  double center_y;  // 检测框中心 y
};

struct GimbalAngles {
  double pitch;  // 俯仰角
  double yaw;    // 偏航角
};

#define RAD_TO_DEG 57.29577951308232  // 180 / PI

class AngleCalculator {
 public:
  inline GimbalAngles CalculateGimbalAngles(const DetectionResult& detection) {
    GimbalAngles target;

    double x_norm = (detection.center_x - DEFAULT_CAMERA_PARAMS.cx) / DEFAULT_CAMERA_PARAMS.fx;
    double y_norm = (detection.center_y - DEFAULT_CAMERA_PARAMS.cy) / DEFAULT_CAMERA_PARAMS.fy;

    target.yaw = std::atan(x_norm) * RAD_TO_DEG;

    target.pitch = std::atan(y_norm) * RAD_TO_DEG;

    return target;
  }

 private:
  const CameraParams DEFAULT_CAMERA_PARAMS = {1903.0, 1713.7, 641.35, 372.89};
  const double DEFAULT_OBJECT_LENGTH = 0.2;  // 米
};