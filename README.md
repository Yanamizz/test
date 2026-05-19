# antidrone

> 上下文权威入口（给新 Agent）：`AGENT.md`  
> 本文件面向人类使用说明；接手项目上下文请统一从 `AGENT.md` 开始。

雷达站目标识别与云台控制项目，核心能力包含：

- 相机采集 + OpenVINO 推理
- 多线程目标跟踪与姿态解算
- 串口 IMU 读写与云台控制帧发送
- TCP 接收外部 `AimbotTarget` 触发信息

## 目录结构

```text
.
├── CMakeLists.txt
├── include/                 # 头文件（CameraTask / ImageRecognize / SerialTask / NetworkTask / Tools）
├── src/                     # 主程序与测试程序
├── third_lib/               # 第三方依赖（serial / kalman / openvino / Galaxy SDK 等）
├── camera_runtime_params.ini
├── run                      # 快速运行脚本（默认关闭显示与调试项）
└── test                     # 调试运行脚本（默认开启显示）
```

## 环境要求

- Ubuntu 24.04 LTS（推荐）
- CMake >= 3.10
- C++17 编译器（g++/clang++）
- OpenCV 4.x（`videoio/imgcodecs/imgproc/core/dnn/highgui/calib3d`）
- Eigen3（CMake 包：`find_package(Eigen3 REQUIRED)`）
- 串口库 `serial`（系统安装或使用仓库内 `third_lib/serial` 回退）
- 大恒相机 SDK（`gxiapi`，`ImagePredict` 目标链接需要）
- OpenVINO（可选）
  - `ImagePredict`/`OpenvinoTest` 在未找到 OpenVINO 时仍可编译，但运行会走报错路径

## 构建

首次配置并构建：

```bash
cmake -S . -B build
cmake --build build -j
```

仅增量编译：

```bash
cmake --build build -j
```

可执行文件默认输出到：

```text
build/bin/
```

## 运行

### 1) 主程序（推荐脚本）

低干扰运行（关闭显示、标定滑块、延迟统计、发送日志等）：

```bash
./run
```

调试运行（开启显示与标定滑块，关闭扫描模式）：

```bash
./test
```

### 2) 直接运行主程序

```bash
./build/bin/ImagePredict [options]
```

常用开关（可组合）：

- `--display` / `--no-display`
- `--calibration-sliders` / `--no-calibration-sliders`
- `--latency-profile` / `--no-latency-profile`
- `--motion-prediction` / `--no-motion-prediction`
- `--scan-mode` / `--no-scan-mode`
- `--save-no-target-images` / `--no-save-no-target-images`
- `--send-log` / `--no-send-log`

## 网络联调（AimbotTarget）

TCP 端口默认 `5000`。完整联调说明见：

- [src/README.md](src/README.md)

当前主程序中的 `AimbotTarget` 规则：

1. 初始值 `0`
2. 每收到一次网络 `1`，计数 `+1`，上限 `3`
3. 每当锁定流程 `stage` 发生一次变化，计数 `-1`，下限 `0`
4. 串口发送时采用二值化：`AimbotTarget >= 1` 则发送 `1`，否则发送 `0`

## 运行参数文件

运行时相机曝光参数保存于：

- `camera_runtime_params.ini`

示例：

```ini
stage12_exposure_time_us=700.0
stage3_exposure_time_us=4000.0
```

## 常见问题

### 1) `build.sh: 没有那个文件或目录`

本仓库根目录不使用 `./build.sh`。请使用：

```bash
cmake -S . -B build
cmake --build build -j
```

### 2) `CMakeCache.txt` 指向旧路径导致构建异常

清理并重新配置：

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j
```

### 3) 找不到 `Eigen3Config.cmake`

安装 Eigen3 开发包后重新配置（例如 Ubuntu）：

```bash
sudo apt-get update
sudo apt-get install -y libeigen3-dev
```

### 4) OpenVINO 未找到

确认 OpenVINO 环境已安装并能被 CMake 发现，或在仓库内提供可用的 `third_lib/openvino/build/OpenVINOConfig.cmake`。





