/**
 * @class OffsetAngles
 * @brief 目标偏移角度计算工具类。
 *
 * 用于根据检测结果和预测中心点，计算云台需要调整的角度。
 * 主要供视觉推理线程调用，实现目标跟踪与角度输出。
 */

#pragma once

#include <Eigen/Dense>

#include "ImageRecognize/AngleCalculate.hpp"
class OffsetAngles {
 public:
  /**
   * @brief 计算目标偏移角度。
   * @param last_predict_center 上一帧预测的目标中心点坐标（像素）
   * @param last_w 上一帧目标宽度（像素）
   * @param last_h 上一帧目标高度（像素）
   * @param has_detection 是否检测到目标
   * @return GimbalAngles 计算得到的云台角度（pitch/yaw）
   *
   * 算法流程：
   * 1. 若检测到目标，则构造DetectionResult结构体，中心点和宽高赋值。
   * 2. 若未检测到目标，则所有值置零。
   * 3. 调用AngleCalculator::CalculateGimbalAngles进行角度计算。
   * 4. 若无目标，角度输出为0。
   */
  GimbalAngles CalculateOffsetAngles(Eigen::Vector2d &last_predict_center, float last_w, float last_h,
                                     bool has_detection) {
    DetectionResult predict_detection;
    if (has_detection) {
      // 计算目标左上角坐标和中心点
      predict_detection.x = last_predict_center[0] - last_w / 2;
      predict_detection.y = last_predict_center[1] - last_h / 2;
      predict_detection.w = last_w;
      predict_detection.h = last_h;
      predict_detection.center_x = last_predict_center[0];
      predict_detection.center_y = last_predict_center[1];
    } else {
      // 未检测到目标，所有值清零
      predict_detection.x = predict_detection.y = predict_detection.w = predict_detection.h = 0;
      predict_detection.center_x = predict_detection.center_y = 0;
    }
    AngleCalculator angle_calculator;
    GimbalAngles angles;
    if (has_detection) {
      // 计算云台角度
      angles = angle_calculator.CalculateGimbalAngles(predict_detection);
    } else {
      // 无目标，角度为0
      angles.pitch = 0.0;
      angles.yaw = 0.0;
    }
    return angles;
  }
};