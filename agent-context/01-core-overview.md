# Core Overview

用途：给新 Agent 提供项目全局认知与主链路。  
更新时间：2026-05-19  
适用场景：首次接手、需要快速建立系统心智模型。

## 项目定位

`antidrone` 是目标识别与云台控制项目，核心链路：

1. 相机采集图像
2. OpenVINO 推理目标
3. 跟踪与角度/激光补偿计算
4. 串口发送云台控制帧
5. 网络线程接收外部 `AimbotTarget` 触发

## 代码分层

- 主流程与线程编排：`src/ImagePredict.cc`
- 识别与跟踪：`include/ImageRecognize/`
- 串口协议与收发：`include/SerialTask/`
- 网络 TCP 接收：`include/NetworkTask/`
- 工具与控制算法：`include/Tools/`
- 第三方依赖：`third_lib/`

## 重点文件

- `src/ImagePredict.cc`
- `include/NetworkTask/AimbotTargetReceiver.hpp`
- `include/SerialTask/SerialSend.hpp`
- `include/Tools/LaserAngleCalculate.hpp`
- `include/Tools/RuntimeParams.hpp`
