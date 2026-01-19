#include <opencv2/opencv.hpp>
#include <iostream>
#include "ImageRecognize/ImagePreprocess.hpp"
#include <string>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <chrono>

int main() {
  try {
    // 加载测试图像
    std::string imagePath = "/home/hanni/code/rm/test/ImageRecognize/data/test.jpg";
    std::string model_path = "/home/hanni/code/rm/test/ImageRecognize/model/best.onnx";
    cv::Mat image = cv::imread(imagePath);

    if (image.empty()) {
      std::cerr << "无法读取图片: " << imagePath << std::endl;
      return -1;
    }

    std::cout << "原始图像尺寸: " << image.cols << "x" << image.rows << std::endl;

    // 创建预处理器（模型需要 640x640 输入）
    ImagePreprocess::ImagePreprocess preprocessor(cv::Size(640, 640));

    // 执行预处理
    ImagePreprocess::PreprocessResult pre_image = preprocessor.run(image);

    // Allocate ONNXRuntime session
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Session session{env, model_path.c_str(), session_options};

    // 获取模型的输入输出节点名称
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = session.GetInputCount();
    size_t num_output_nodes = session.GetOutputCount();

    std::cout << "模型有 " << num_input_nodes << " 个输入节点和 " << num_output_nodes << " 个输出节点" << std::endl;

    // 获取输入节点名称（保存字符串副本）
    std::vector<std::string> input_names_str;
    std::vector<const char*> input_node_names;
    for (size_t i = 0; i < num_input_nodes; i++) {
      auto input_name = session.GetInputNameAllocated(i, allocator);
      input_names_str.push_back(std::string(input_name.get()));
      std::cout << "输入节点 " << i << ": " << input_names_str.back() << std::endl;
    }
    for (const auto& name : input_names_str) {
      input_node_names.push_back(name.c_str());
    }

    // 获取输出节点名称（保存字符串副本）
    std::vector<std::string> output_names_str;
    std::vector<const char*> output_node_names;
    for (size_t i = 0; i < num_output_nodes; i++) {
      auto output_name = session.GetOutputNameAllocated(i, allocator);
      output_names_str.push_back(std::string(output_name.get()));
      std::cout << "输出节点 " << i << ": " << output_names_str.back() << std::endl;
    }
    for (const auto& name : output_names_str) {
      output_node_names.push_back(name.c_str());
    }

    // Allocate model inputs: fill in shape and size (使用vector避免栈溢出)
    // 640 * 640 * 3 * 1 = 1,228,800
    std::vector<float> input(1228800);

    // 将预处理的图像数据复制到输入数组
    if (pre_image.data.size() == input.size()) {
      std::copy(pre_image.data.begin(), pre_image.data.end(), input.begin());
    } else {
      std::cerr << "预处理数据大小不匹配: " << pre_image.data.size() << " != " << input.size() << std::endl;
      return -1;
    }

    std::array<int64_t, 4> input_shape{1, 3, 640, 640};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input.data(), input.size(),
                                                              input_shape.data(), input_shape.size());

    // 记录推理开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    // Run the model (使用实际的节点名称，让模型自动分配输出)
    auto output_tensors =
        session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);

    // 记录推理结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::string window_name = std::to_string(duration.count()) + "ms";

    // 获取输出张量信息
    auto& output_tensor = output_tensors[0];
    auto output_shape_info = output_tensor.GetTensorTypeAndShapeInfo();
    auto output_shape = output_shape_info.GetShape();

    std::cout << "输出形状: [";
    for (size_t i = 0; i < output_shape.size(); ++i) {
      std::cout << output_shape[i];
      if (i < output_shape.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 获取输出数据
    float* output_data = output_tensor.GetTensorMutableData<float>();

    // 解析检测结果并画框
    cv::Mat display_image = image.clone();

    // YOLOv8/YOLOv5 输出格式通常是 [batch, num_detections, 5+num_classes]
    // 或者 [batch, 84/85, 8400] (转置格式)
    if (output_shape.size() >= 2) {
      int num_detections = 0;
      int detection_size = 0;

      // 判断输出格式
      if (output_shape.size() == 3 && output_shape[2] > output_shape[1]) {
        // 格式: [1, num_detections, detection_size]
        num_detections = output_shape[1];
        detection_size = output_shape[2];
      } else if (output_shape.size() == 3 && output_shape[1] > output_shape[2]) {
        // 格式: [1, detection_size, num_detections] - 需要转置
        detection_size = output_shape[1];
        num_detections = output_shape[2];
      }

      std::cout << "检测数量: " << num_detections << ", 检测维度: " << detection_size << std::endl;

      float conf_threshold = 0.25;  // 置信度阈值
      int detection_count = 0;

      // 计算缩放比例（从 640x640 回到原始图像尺寸）
      float scale_x = static_cast<float>(image.cols) / 640.0f;
      float scale_y = static_cast<float>(image.rows) / 640.0f;

      // 遍历检测结果
      for (int i = 0; i < num_detections; ++i) {
        float* detection;
        if (output_shape.size() == 3 && output_shape[2] > output_shape[1]) {
          detection = output_data + i * detection_size;
        } else {
          detection = output_data + i;
        }

        // YOLO 格式: [x_center, y_center, width, height, confidence, class_scores...]
        float x_center = detection[0] * scale_x;
        float y_center = detection[1] * scale_y;
        float width = detection[2] * scale_x;
        float height = detection[3] * scale_y;
        float confidence = detection[4];

        if (confidence > conf_threshold) {
          // 转换为边界框坐标
          int x1 = static_cast<int>(x_center - width / 2);
          int y1 = static_cast<int>(y_center - height / 2);
          int x2 = static_cast<int>(x_center + width / 2);
          int y2 = static_cast<int>(y_center + height / 2);

          // 画框
          cv::rectangle(display_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

          // 显示置信度
          std::string label = cv::format("%.2f", confidence);
          cv::putText(display_image, label, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0),
                      1);

          detection_count++;
        }
      }

      std::cout << "检测到 " << detection_count << " 个目标" << std::endl;
    }

    // 添加推理时间标注
    cv::putText(display_image, window_name, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    cv::imshow(window_name, display_image);
    std::cout << "推理时间: " << window_name << std::endl;
    cv::waitKey(0);

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "异常: " << e.what() << std::endl;
    return -1;
  }
}