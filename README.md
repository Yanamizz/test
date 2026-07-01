# Radar

本项目当前是一条面向 RoboMaster 雷达站的“接收 -> 处理 -> 发送”通信主链路，而不是视觉识别项目。

它的当前职责是：

- 接收裁判系统常规链路主协议
- 接收信息波链路与敌方密钥链路数据
- 维护协议结构体状态
- 生成并发送 `0x0305` 小地图位置数据
- 生成并发送 `0x0301(0x0121 RadarCMD)` 自主决策指令
- 记录结构体日志、原始二进制日志与运行时指标

当前不把系统重心放在：

- 视觉识别
- 相机目标检测
- 反无人机功能

## 当前主链路

主入口在 [src/main.cc](/home/hanni/Radar/src/main.cc:45)。

当前运行链路如下：

1. 串口常规链路通过 `SerialPort` 接收真实裁判系统字节流，或通过 `ReplayInputSource` 从文件回放。
2. 信息波链路通过 `TcpClient` 主动连接 `8001`，接收真实 TCP 字节流，或通过 `ReplayInputSource` 从文件回放。
3. 敌方一级/二级密钥链路通过 `TcpClient` 主动连接 `8002/8003`，接收完整 `0x0A06` 协议帧。
4. 所有输入字节流逐字节喂给各自的 `rm::device::Referee<revision>` 实例维护状态。
5. `MapRobotRelay` 根据 `0x0A01 + 0x020B` 维护 `0x0305` 状态，并按 5Hz 发送最新地图数据。
6. `RadarDecisionTree` 根据 `0x020E` 生成自主决策，由 `RadarCommandSender` 组包为 `0x0301(0x0121)`。
7. `RefereeTxScheduler` 统一调度 `0x0301` 与 `0x0305` 的串口发送节奏。
8. [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58) 统一负责真实输入、文件回放、串口重连、TCP 重连、周期任务和日志指标刷新。

## 关键模块

- [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:32)
  - 唯一手动配置入口
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)
  - 主循环、poll、自动重连、周期任务
- [include/referee/serial_port.hpp](/home/hanni/Radar/include/referee/serial_port.hpp:75)
  - 裁判系统串口读写
- [include/referee/tcp_client.hpp](/home/hanni/Radar/include/referee/tcp_client.hpp:46)
  - `8001/8002/8003` 非阻塞 TCP 客户端
- [include/referee/map_robot_relay.hpp](/home/hanni/Radar/include/referee/map_robot_relay.hpp:254)
  - `0x0A01 -> 0x0305` 状态维护、过期处理、发送日志
- [include/referee/radar_decision_tree.hpp](/home/hanni/Radar/include/referee/radar_decision_tree.hpp:45)
  - 基于 `0x020E` 的自主决策
- [include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:85)
  - `0x0121` 组包、队列、10 秒冷却、日志
- [include/referee/enemy_key_receiver.hpp](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:72)
  - `8002/8003` 上的 `0x0A06` 解包与校验
- [include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33)
  - `0x0301` FIFO 与 `0x0305` latest-only 发送调度
- [include/log/log_backend.hpp](/home/hanni/Radar/include/log/log_backend.hpp:129)
  - 异步日志后端与 `runtime_metrics.log`
- [include/log/referee_main_log.hpp](/home/hanni/Radar/include/log/referee_main_log.hpp:739)
  - 主协议结构体日志格式化

## 配置入口

所有手动配置统一放在 [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:32)。

常用配置项：

- `kRefereeRevision`
  - 当前裁判系统协议版本
- `kRadarLogMode`
  - `kDebug` / `kMatch`
- `kDebugAllowMissingInterfaces`
  - 调试模式下允许接口缺失，缺失链路对应状态按 0 值处理
- `kSerialRefereeInputMode`
  - 串口主协议使用真实输入或文件回放
- `kInfoWaveInputMode`
  - 信息波 `8001` 使用真实输入或文件回放
- `kSerialRefereeReplayFile` / `kInfoWaveReplayFile`
  - 回放文件路径
- `kSerialRefereeReplayRateHz` / `kInfoWaveReplayRateHz`
  - 回放频率
- `kTcpServerAddress`
  - `8001/8002/8003` 的对端服务端地址
- `kTcpLocalBindAddress`
  - 本机绑定地址，留空表示让系统自动选路由出口
- `kInfoWaveTcpServerPort` / `kEnemyLevel1KeyTcpServerPort` / `kEnemyLevel2KeyTcpServerPort`
  - 三条 TCP 客户端链路端口
- `kSerialReconnectIntervalMs` / `kTcpReconnectIntervalMs`
  - 自动重连周期
- `kInfoWaveTcpIdleTimeoutMs` / `kEnemyKeyTcpIdleTimeoutMs`
  - TCP 空闲断开策略

当前代码语义上是 TCP client-only，本机不再监听 `8001/8002/8003`。

## 输入模式

当前支持两类输入源：

- `kReal`
  - 串口来自真实裁判系统
  - 信息波来自真实 TCP `8001`
  - 敌方密钥来自真实 TCP `8002/8003`
- `kFile`
  - 串口与信息波 `8001` 可分别从预制 `.bin` 文件回放完整协议帧

当前约束：

- 串口主协议与信息波 `8001` 可以独立切换 `kReal/kFile`
- `8002/8003` 目前只走真实 TCP，不走文件回放
- 即使串口输入改成文件回放，发送仍尝试走真实串口

样例文件位于：

- [test/info/outputInfo.bin](/home/hanni/Radar/test/info/outputInfo.bin)
- [test/info/outputInfo2.bin](/home/hanni/Radar/test/info/outputInfo2.bin)
- [test/info/map_robot_sender_sample.bin](/home/hanni/Radar/test/info/map_robot_sender_sample.bin)

## 构建与运行

首次配置：

```bash
cmake -S . -B build
```

构建主程序：

```bash
cmake --build build --target radar -j2
```

运行：

```bash
./build/bin/radar
```

辅助脚本：

```bash
python3 test/serialtest.py
```

辅助脚本文件：

- [test/serialtest.py](/home/hanni/Radar/test/serialtest.py:1)

当前仓库不再维护独立的 C++ 测试可执行程序；测试能力已经回收到主程序、回放链路和日志链路中。

## 日志与可观测性

默认构建下，每次运行都会在 `test/logs/` 下创建新的时间戳目录。

目录结构大致如下：

- `test/logs/<timestamp>/main/*.log`
  - 结构体日志、发送日志、连接状态日志、运行指标
- `test/logs/<timestamp>/raw/*.bin`
  - 原始串口/TCP 输入字节流

常见日志文件包括：

- `main/serial_state.log`
- `main/tcp_startup.log`
- `main/tcp_channel_state.log`
- `main/tcp_client_state.log`
- `main/runtime_metrics.log`
- `main/0x0305_map_robot_data.log`
- `main/0x0305_map_robot_data_skipped.log`

当前代码不再创建 `latest/` 快照目录，所有证据链都保留在当次运行目录中。

## 调试注意事项

- `kDebug` 模式下允许串口或 TCP 接口缺失，相关状态会按 0 值处理，便于单链路调试。
- 常规链路与信息波链路分别使用独立 `Referee` 实例，因此各自维护独立的 `seq` 与 `loss_rate`。
- `loss_rate()` 来自 `librm` 的通用实现，假设发送端 `seq` 按 256 周期滚动；对信息波 TCP 流来说，它不一定等价于真实网络丢包率。

## 相关文档

- [AGENTS.md](/home/hanni/Radar/AGENTS.md:1)
  - 仓库协作约束与修改边界
- [.agent/handoff.md](/home/hanni/Radar/.agent/handoff.md:1)
  - 当前代码状态的接手说明
- [.agent/full_pipeline.md](/home/hanni/Radar/.agent/full_pipeline.md:1)
  - 从接收到发送的完整链路说明
- [.agent/radar_stability_review.md](/home/hanni/Radar/.agent/radar_stability_review.md:1)
  - 稳定性现状、已完成保护与剩余风险
