# Change Entrypoints

用途：按任务类型快速定位改动入口。  
更新时间：2026-07-03
适用场景：准备改代码、评估影响面、做交接定位。

## 任务到入口映射

1. 主流程与线程协作  
- `src/ImagePredict.cc`
- 阶段资源组切换与副作用应用：`include/ImageRecognize/StagePredictorController.hpp`
- 目标控制/扫描发送仲裁：`include/Tools/AimbotCommandArbiter.hpp`
- 发送命令数据结构：`include/Tools/AimbotCommand.hpp`
- 显示/录像叠加帧生成：`include/ImageRecognize/OverlayFrameRenderer.hpp`

2. CPU 亲和性与核心分配  
- `include/Tools/CpuAffinity.hpp`

3. 推理引擎线程策略（OpenVINO）  
- `include/ImageRecognize/ImagePredict_OPENVINO.hpp`

4. 串口收发语义  
- `include/SerialTask/Common.hpp`
- `include/SerialTask/SerialSend.hpp`

5. TCP 网络控制链路
- `include/Tools/TcpStageSignalReceiver.hpp`
- `include/Tools/AimbotLaserStateController.hpp`
- `src/ImagePredict.cc` 中 `TCPStageThread`
- `src/TcpStageRecvTest.cc`
- `src/TcpStageClientRecvTest.cc`
- `src/TcpStageSendTest.cc`

6. 角度/激光补偿与扫描控制  
- `include/Tools/AngleCalculate.hpp`
- `include/Tools/LaserAngleCalculate.hpp`
- `include/Tools/ScanController.hpp`

7. 运行参数默认值  
- `include/Tools/RuntimeParams.hpp`
- 常用运行参数快照：`include/Tools/RuntimeParamProfiles.hpp`

8. 人类使用文档  
- `README.md`

## 变更范围约束

- 避免误改 `third_lib/` 外部依赖代码。  
- 涉及线程时序改动时，优先做关键路径影响分析。  
- 变更核心行为后，必须同步更新 `06-latest-handoff.md`。
