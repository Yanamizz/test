/**
 * @file    include/ImageRecognize/ImagePredict.hpp
 * @brief   模型推理入口：加载 ONNX 模型并执行单张图像的推理。
 *
 * @details
 * - 使用 `ImagePreprocess` 将输入图像标准化为模型需要的 NCHW 张量。
 * - 调用 ONNX Runtime 执行推理，固定一个输入与一个输出。
 * - 将输出交由 `OutputDataProcess` 解析为用于绘制的检测框结果。
 */
#pragma once

#include "ImageRecognize/ImagePreprocess.hpp"
#include "ImageRecognize/OutputDataProcess.hpp"
#include <array>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <thread>

namespace ImageRecognize {

using PredictResult = ImageRecognize::DataProcessResult;  ///< 推理的结果别名，复用后处理的结果结构

/**
 * @brief 图像预测器：封装单次推理流程（预处理→推理→后处理）。
 */
class ImagePredict {
 public:
  ImagePredict() = default;

  // 基于模型路径构造，创建并缓存 ORT 会话与 I/O 名称，便于实时推理复用
  explicit ImagePredict(const std::string& model_path) {
    // 优化：配置会话选项（线程数与图优化）并复用该会话以降低每帧开销
    int cpu_threads = std::max(1u, std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() : 1u);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(cpu_threads);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
    Ort::AllocatorWithDefaultOptions allocator{};
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      throw std::runtime_error("Model must have exactly one input and one output");
    }
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
  }

  /**
   * @brief 执行一次图像推理，返回检测结果。
   * @param[in] origin_image_  原始输入图像（BGR，HWC）。
   * @param[in] model_path_    ONNX 模型文件路径。
   * @returns  PredictResult   检测框集合（角点坐标与分数）。
   */
  PredictResult run(const cv::Mat& origin_image_, std::string model_path_) const {
    ImageRecognize::ImagePreprocess preprocessor_{cv::Size(width_, height_)};
    auto pre_image_ = preprocessor_.run(origin_image_);

    Ort::SessionOptions opts{};
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    opts.SetIntraOpNumThreads(4);  // 根据 CPU 核心数调整

    Ort::Session session_{env_, model_path_.c_str(), opts};

    Ort::AllocatorWithDefaultOptions allocator_{};
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      throw std::runtime_error("Model must have exactly one input and one output");
    }

    std::array<const char*, 1> input_names = {"images"};
    std::array<const char*, 1> output_names = {"output0"};

    std::array<int64_t, 4> input_shape_ = pre_image_.shape;  // Use actual tensor shape
    auto memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor_ = Ort::Value::CreateTensor<float>(
        memory_info_, pre_image_.data.data(), pre_image_.data.size(), input_shape_.data(), input_shape_.size());

    auto output_tensors_ =
        session_.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor_, 1, output_names.data(), 1);
    ImageRecognize::OutputDataProcess output_processor_;
    return output_processor_.run(output_tensors_[0], origin_image_.size());
  }

  // 复用已创建的会话进行推理（实时/视频流推荐使用）
  PredictResult run(const cv::Mat& origin_image_) {
    if (!session_) {
      throw std::runtime_error("Session not initialized. Use ImagePredict(model_path) constructor.");
    }
    ImageRecognize::ImagePreprocess preprocessor_{cv::Size(width_, height_)};
    auto pre_image_ = preprocessor_.run(origin_image_);

    std::array<int64_t, 4> input_shape_ = pre_image_.shape;
    auto memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor_ = Ort::Value::CreateTensor<float>(
        memory_info_, pre_image_.data.data(), pre_image_.data.size(), input_shape_.data(), input_shape_.size());

    const char* in_names[] = {input_name_.c_str()};
    const char* out_names[] = {output_name_.c_str()};
    auto output_tensors_ = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor_, 1, out_names, 1);
    ImageRecognize::OutputDataProcess output_processor_;
    return output_processor_.run(output_tensors_[0], origin_image_.size());
  }

 private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "predict"};  ///< ORT 环境对象
  Ort::SessionOptions session_options_;                 ///< ORT 会话选项（如需可扩展）
  static constexpr int width_ = 480;                    ///< 模型输入宽
  static constexpr int height_ = 480;                   ///< 模型输入高
  std::unique_ptr<Ort::Session> session_;               ///< 复用的 ORT 会话
  std::string input_name_;                              ///< 模型输入名称
  std::string output_name_;                             ///< 模型输出名称

  // Ort::Session session_{env_, model_path.c_str(), session_options_}; // 需要 model 路径时再初始化
};

}  // namespace ImageRecognize