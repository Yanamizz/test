/**
 * @file    src/RawToClahe.cc
 * @brief   Offline tool to convert saved raw images into CLAHE-processedimages.
# 批量生成训练用 CLAHE 图，默认输出到 xxx_train_clahe
./build/bin/RawToClahe captures/run_xxx

  # 指定输出目录
./build/bin/RawToClahe captures/run_xxx dataset_stage3_train

  # 明确模型输入尺寸
./build/bin/RawToClahe --size 480x480 captures/run_xxx dataset_stage3_train

  # 如果你真的想要原尺寸 CLAHE 图
./build/bin/RawToClahe --full-size captures/run_xxx captures/run_xxx_full_clahe
 */

#include "ImageRecognize/YoloLightPreprocess.hpp"

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
  bool model_input_mode = true;
  cv::Size input_size{480, 480};
  std::filesystem::path input_path;
  std::filesystem::path output_path;
};

bool IsImageFile(const std::filesystem::path &path) {
  if (!path.has_extension()) {
    return false;
  }

  std::string ext = path.extension().string();
  for (char &c : ext) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }

  return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp";
}

std::filesystem::path DefaultOutputPath(const std::filesystem::path &input) {
  if (std::filesystem::is_directory(input)) {
    return input.parent_path() / (input.filename().string() + "_train_clahe");
  }

  const std::filesystem::path parent = input.has_parent_path()
                                           ? input.parent_path()
                                           : std::filesystem::path(".");
  return parent /
         (input.stem().string() + "_train_clahe" + input.extension().string());
}

std::filesystem::path OutputFileFor(const std::filesystem::path &input_file,
                                    const std::filesystem::path &input_root,
                                    const std::filesystem::path &output_root,
                                    bool batch_mode) {
  if (!batch_mode) {
    return output_root;
  }

  const auto relative = std::filesystem::relative(input_file, input_root);
  return output_root / relative;
}

cv::Mat
BuildModelInputClahe(const cv::Mat &input, const cv::Size &input_size,
                     ImageRecognize::YoloLightPreprocessor *preprocessor) {
  const float scale_x =
      static_cast<float>(input_size.width) / static_cast<float>(input.cols);
  const float scale_y =
      static_cast<float>(input_size.height) / static_cast<float>(input.rows);
  const float scale = std::min(scale_x, scale_y);

  int content_width =
      std::max(1, static_cast<int>(std::round(input.cols * scale)));
  int content_height =
      std::max(1, static_cast<int>(std::round(input.rows * scale)));
  content_width = std::min(content_width, input_size.width);
  content_height = std::min(content_height, input_size.height);

  const int pad_x = (input_size.width - content_width) / 2;
  const int pad_y = (input_size.height - content_height) / 2;

  cv::Mat resized;
  cv::resize(input, resized, cv::Size(content_width, content_height));
  preprocessor->PreprocessForYolo(resized, &resized);

  cv::Mat letterboxed(input_size, input.type(), cv::Scalar(114, 114, 114));
  resized.copyTo(
      letterboxed(cv::Rect(pad_x, pad_y, content_width, content_height)));
  return letterboxed;
}

bool ConvertOne(const std::filesystem::path &input_file,
                const std::filesystem::path &output_file,
                const Options &options,
                ImageRecognize::YoloLightPreprocessor *preprocessor) {
  const cv::Mat input = cv::imread(input_file.string(), cv::IMREAD_COLOR);
  if (input.empty()) {
    std::cerr << "[RawToClahe] 读取失败: " << input_file << std::endl;
    return false;
  }

  cv::Mat output;
  if (options.model_input_mode) {
    output = BuildModelInputClahe(input, options.input_size, preprocessor);
  } else {
    preprocessor->PreprocessForYolo(input, &output);
  }
  std::filesystem::create_directories(output_file.parent_path());
  if (!cv::imwrite(output_file.string(), output)) {
    std::cerr << "[RawToClahe] 保存失败: " << output_file << std::endl;
    return false;
  }

  return true;
}

void PrintUsage(const char *program) {
  std::cerr << "用法: " << program
            << " [--model-input|--full-size] [--size WxH] <输入图片或目录> "
               "[输出图片或目录]\n"
            << "示例: " << program
            << " captures/run_xxx captures/run_xxx_train\n"
            << "示例: " << program
            << " --size 480x480 captures/run_xxx/stage3_raw_000001.jpg\n"
            << "示例: " << program
            << " --full-size captures/run_xxx captures/run_xxx_full_clahe"
            << std::endl;
}

cv::Size ParseSize(const std::string &text) {
  const std::size_t x_pos = text.find('x');
  if (x_pos == std::string::npos) {
    throw std::invalid_argument("size must be WxH");
  }

  const int width = std::stoi(text.substr(0, x_pos));
  const int height = std::stoi(text.substr(x_pos + 1));
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("size must be positive");
  }
  return {width, height};
}

bool ParseArgs(int argc, char **argv, Options *options) {
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--model-input") {
      options->model_input_mode = true;
    } else if (arg == "--full-size") {
      options->model_input_mode = false;
    } else if (arg == "--size") {
      if (i + 1 >= argc) {
        std::cerr << "[RawToClahe] --size 缺少参数。" << std::endl;
        return false;
      }
      try {
        options->input_size = ParseSize(argv[++i]);
      } catch (const std::exception &e) {
        std::cerr << "[RawToClahe] --size 无效: " << e.what() << std::endl;
        return false;
      }
    } else {
      positional.push_back(arg);
    }
  }

  if (positional.empty() || positional.size() > 2) {
    return false;
  }

  options->input_path = positional[0];
  options->output_path = positional.size() == 2
                             ? std::filesystem::path(positional[1])
                             : DefaultOutputPath(options->input_path);
  return true;
}

} // namespace

int main(int argc, char **argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  if (!std::filesystem::exists(options.input_path)) {
    std::cerr << "[RawToClahe] 输入不存在: " << options.input_path << std::endl;
    return 1;
  }

  const bool batch_mode = std::filesystem::is_directory(options.input_path);

  ImageRecognize::YoloLightPreprocessor preprocessor;
  std::vector<std::filesystem::path> inputs;
  if (batch_mode) {
    for (const auto &entry :
         std::filesystem::recursive_directory_iterator(options.input_path)) {
      if (entry.is_regular_file() && IsImageFile(entry.path())) {
        inputs.push_back(entry.path());
      }
    }
  } else if (IsImageFile(options.input_path)) {
    inputs.push_back(options.input_path);
  }

  if (inputs.empty()) {
    std::cerr << "[RawToClahe] 没有找到可处理的图片。" << std::endl;
    return 1;
  }

  int ok_count = 0;
  for (const auto &input_file : inputs) {
    const std::filesystem::path output_file = OutputFileFor(
        input_file, options.input_path, options.output_path, batch_mode);
    if (ConvertOne(input_file, output_file, options, &preprocessor)) {
      ++ok_count;
    }
  }

  std::cout << "[RawToClahe] 完成: " << ok_count << "/" << inputs.size()
            << " 模式="
            << (options.model_input_mode ? "model-input" : "full-size")
            << " 尺寸=" << options.input_size.width << "x"
            << options.input_size.height << " 输出=" << options.output_path
            << std::endl;
  return ok_count == static_cast<int>(inputs.size()) ? 0 : 2;
}
