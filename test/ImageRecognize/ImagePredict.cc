#include <opencv2/opencv.hpp>
#include <iostream>
#include "ImageRecognize/ImagePreprocess.hpp"
#include <string>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <chrono>
#include <vector>

void print_output_info(Ort::Session& session) {
  Ort::AllocatorWithDefaultOptions allocator;
  size_t out_count = session.GetOutputCount();
  for (size_t i = 0; i < out_count; ++i) {
    char* name = session.GetOutputNameAllocated(i, allocator).get();
    auto type_info = session.GetOutputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto elem_type = tensor_info.GetElementType();
    auto dims = tensor_info.GetShape();
    std::cout << "Output " << i << " name: " << name << "\n";
    std::cout << "  dtype: " << elem_type << "\n";
    std::cout << "  shape: [";
    for (size_t j = 0; j < dims.size(); ++j) {
      std::cout << dims[j] << (j + 1 < dims.size() ? ", " : "");
    }
  }
}

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

    // Run the model (使用实际的节点名称，让模型自动分配输出)
    auto output_tensors =
        session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);

    print_output_info(session);
    std::cout << std::endl;
    auto& output_tensor = output_tensors[0];
    auto output_shape_info = output_tensor.GetTensorTypeAndShapeInfo();
    auto output_shape = output_shape_info.GetShape();
    for (size_t i = 0; i < output_shape.size(); ++i) {
      std::cout << output_shape[i];
      if (i < output_shape.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 获取输出数据
    float* output_data = output_tensor.GetTensorMutableData<float>();
    int num_detections = 0;
    int detection_size = 0;
    if (output_shape.size() == 3 && output_shape[2] > output_shape[1]) {
      // 格式: [1, num_detections, detection_size]
      num_detections = output_shape[1];
      detection_size = output_shape[2];
    } else if (output_shape.size() == 3 && output_shape[1] > output_shape[2]) {
      // 格式: [1, detection_size, num_detections] - 需要转置
      detection_size = output_shape[1];
      num_detections = output_shape[2];
    }

    return 0;
  }

  catch (const Ort::Exception& e) {
    std::cerr << "ONNX Runtime 错误: " << e.what() << " 状态码: " << e.GetOrtErrorCode() << std::endl;
    return -1;
  }
}