# antidrone

> 上下文权威入口（给新 Agent）：`AGENT.md`  
> 本文件面向人类使用说明；接手项目上下文请统一从 `AGENT.md` 开始。

雷达站目标识别与云台控制项目，核心能力包含：

- 相机采集 + OpenVINO 推理
- 多线程目标跟踪与姿态解算
- 串口 IMU 读写与云台控制帧发送

## 目录结构

```text
.
├── CMakeLists.txt
├── include/                 # 头文件（CameraTask / ImageRecognize / SerialTask / Tools）
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
- OpenCV 4.x（'sudo apt install -y libopencv-dev'）
- Eigen3（'sudo apt install libeigen3-dev'）
- third_lib
  - 串口库 `serial`（<https://github.com/wjwwood/serial.git>）
  - 大恒相机 SDK（<https://www.daheng-imaging.com/downloads/>）
  - kalman (<https://github.com/mherb/kalman.git>)
  - OneEuroFilter (<https://github.com/casiez/OneEuroFilter.git>)
  - OpenVINO（可选）
    - `ImagePredict`/`OpenvinoTest` 在未找到 OpenVINO 时仍可编译，但运行会走报错路径
    - <https://www.intel.cn/content/www/cn/zh/developer/tools/openvino-toolkit/overview.html>
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
- `--scan-mode` / `--no-scan-mode`
- `--save-no-target-images` / `--no-save-no-target-images`
- `--send-log` / `--no-send-log`

## AimbotTarget 激光控制

主程序不再通过 TCP 网络通信控制激光开关；旧 NetworkTask 联调示例已删除。

当前主程序中的 `AimbotTarget` 规则：

1. 首次获得有效目标距离时，触发激光开启 flag，串口 `AimbotTarget` 开始保持 `0x01`
2. 首次获得有效目标距离时，阶段判断也初始化/重置唯一一次；目标初始在 25m 以内也会执行
3. `stage1 -> stage2` 完成后进入 55s 激光关闭窗口，串口帧 `AimbotTarget` 发送 `0x00`
4. 关闭窗口结束后恢复发送 `0x01`
5. 进入 `stage3` 时清除关闭窗口并固定发送 `0x01`
6. 距离不再直接关闭激光；串口 `AimbotTarget` 只会被 `stage1 -> stage2` 后的关闭窗口压到 `0x00`

## 运行参数文件

运行时相机曝光参数保存于：

- `camera_runtime_params.ini`

示例：

```ini
stage12_exposure_time_us=700.0
stage3_exposure_time_us=4000.0
```

默认运行参数中的 `stage3_scan_bounds_mode` 为 `AUTO`：stage3 扫描边界会使用
stage12 有目标控制发送时收集到的 yaw/pitch 最大最小值，并限制在手动扫描边界内。
改为非 `AUTO` 时沿用手动扫描边界。

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
