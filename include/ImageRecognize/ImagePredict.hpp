/**
 * @file    include\ImageRecognize\ImagePredict.hpp
 * @brief   本文件功能onnx模型图像识别。
 *
 * @date    2026-01-19
 *
 * @brief   主要实现功能：
 * @brief   传入图像（cv::Mat），识别后输出识别结果数组。
 *
 * @brief   传入图像（cv::Mat），使用预处理头文件（ImagePreprocess.hpp）进行预处理
 * @brief   输出处理后的数组{batch, channel, height, width}作为onnx模型的输入。
 *
 *

 */