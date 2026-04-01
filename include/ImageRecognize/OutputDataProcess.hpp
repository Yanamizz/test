/**
 * @file    include/ImageRecognize/OutputDataProcess.hpp
 * @brief   共享的检测结果类型定义。
 */

#pragma once
#include <array>
#include <vector>

namespace ImageRecognize {

/**
 * @brief 后处理结果结构体。
 * @note  `boxes` 中每个元素为 {x1, y1, x2, y2, score}。
 */
struct DataProcessResult {
  std::vector<std::array<float, 5>> boxes;  ///< 检测框列表：{x1,y1,x2,y2,score}
};

using PredictResult = DataProcessResult;

}  // namespace ImageRecognize