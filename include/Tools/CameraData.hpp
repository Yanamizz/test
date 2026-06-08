/**
 * @file    include/Tools/CameraData.hpp
 * @brief   保存相机内参常量，供角度和距离计算模块复用。
 *
 * CameraData 集中定义焦距像素值和主点，是像素坐标到角度/距离换算的基础
 * 数据源。当前项目约定不依赖不可靠畸变矫正，因此这里只保留主链路实际使用
 * 的 fx/fy/cx/cy 常量。
 */

#pragma once

namespace Tools {
struct CameraData {
  static constexpr double kFocalX = 18213.568586;
  static constexpr double kFocalY = 18211.286085;
  static constexpr double kPrincipalX = 960.548600;
  static constexpr double kPrincipalY = 600.710214;
};
} // namespace Tools
