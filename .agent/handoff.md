# Handoff

本文档用于帮助后续 Agent 或开发者快速接手当前仓库，重点只覆盖现在真实存在的 Linux 主程序链路。

## 1. 当前项目快照

当前项目不是旧的 STM32 `main.cc` 方案，也不是视觉识别工程，而是一条项目侧通信主链路：

- 接收裁判系统串口主协议
- 接收信息波 TCP `8001`
- 接收敌方密钥 TCP `8002/8003`
- 维护协议结构体状态
- 发送 `0x0305` 与 `0x0301(0x0121)`
- 记录结构体日志、原始字节流和运行指标

当前主入口：

- [src/main.cc](/home/hanni/Radar/src/main.cc:45)

当前唯一手动配置入口：

- [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:32)

## 2. 主程序对象关系

[src/main.cc](/home/hanni/Radar/src/main.cc:45) 启动时做的事情可以概括为：

1. 创建本轮运行日志目录。
2. 尝试打开裁判系统串口。
3. 按配置主动连接 TCP `8001/8002/8003`。
4. 创建两个主协议解包器：
   - `serial_referee`
   - `info_wave_referee`
5. 创建发送与业务模块：
   - `RefereeTxScheduler`
   - `RadarCommandSender`
   - `RadarDecisionTree`
   - `MapRobotRelay`
   - 两个 `EnemyKeyReceiver`
6. 给串口链路与信息波链路挂回调。
7. 进入 [`RunSerialInfoWaveAndKeyTcpLoop(...)`](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)。

这意味着当前项目的真实主链并不在 [src/main.cc](/home/hanni/Radar/src/main.cc:45) 里展开，而是分散在 `include/referee/*.hpp` 的项目侧封装中。

## 3. 当前输入职责

### 3.1 串口常规链路

文件：

- [include/referee/serial_port.hpp](/home/hanni/Radar/include/referee/serial_port.hpp:75)
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

职责：

- 读取真实裁判系统字节流
- 逐字节喂给 `serial_referee`
- 维护 `0x020B`、`0x020E`、`0x0201` 等常规链路状态
- 作为所有发包的真实输出口

串口掉线后不会直接退出进程，而是进入后台重连。

### 3.2 信息波链路 `8001`

文件：

- [include/referee/tcp_client.hpp](/home/hanni/Radar/include/referee/tcp_client.hpp:46)
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

职责：

- 本端只作为 TCP 客户端
- 主动连接配置中的 `kTcpServerAddress:8001`
- 逐字节喂给独立的 `info_wave_referee`
- 维护 `0x0A01~0x0A06` 相关结构体状态

当前 `8001` 的 idle timeout 可在 [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:76) 中单独配置；当前代码支持设为 `<=0` 从而保持连接。

### 3.3 敌方密钥链路 `8002/8003`

文件：

- [include/referee/enemy_key_receiver.hpp](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:72)
- [include/referee/tcp_client.hpp](/home/hanni/Radar/include/referee/tcp_client.hpp:46)

职责：

- 本端只作为 TCP 客户端
- 主动连接 `8002/8003`
- 每个端口内部维护一个独立的 `Referee` 解包器
- 只接受完整 `0x0A06` 帧，而不是裸 6 字节
- 成功拿到一次合法密钥后，将其送入 `RadarCommandSender` 的待发队列

说明：

- 两个端口当前语义是“一级密钥 / 二级密钥”两个独立来源。
- 当前实现完成一次合法接收后会关闭该端口连接，不再持续保持。

### 3.4 文件回放

文件：

- [include/referee/replay_input_source.hpp](/home/hanni/Radar/include/referee/replay_input_source.hpp:43)

当前支持：

- 串口主协议回放
- 信息波 `8001` 回放

当前不支持：

- `8002/8003` 文件回放

## 4. 当前发送职责

### 4.1 `0x0305` 小地图位置

文件：

- [include/referee/map_robot_relay.hpp](/home/hanni/Radar/include/referee/map_robot_relay.hpp:254)

职责：

- 从信息波 `0x0A01` 维护敌方位置
- 从串口 `0x020B` 维护己方地面机器人位置
- 按 5Hz latest-only 节奏发送 `0x0305`
- 状态过期后自动置 0
- 记录成功发送和跳过发送原因

调试模式下若允许缺失接口，则即便 `0x0A01` 或 `0x020B` 缺失，也会按 0 值继续跑主链路。

### 4.2 `0x0301(0x0121 RadarCMD)`

文件：

- [include/referee/radar_decision_tree.hpp](/home/hanni/Radar/include/referee/radar_decision_tree.hpp:45)
- [include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:85)

职责：

- `RadarDecisionTree` 读取串口 `0x020E`
- 自动决定是否申请双倍易伤
- 自动决定是否切换到下一组预置己方密钥
- `RadarCommandSender` 负责组包、日志、排队和冷却

敌方密钥提交 `password_cmd=2` 的特点：

- 走 FIFO 队列
- 有 10 秒冷却
- 真正发送后才推进下一条

## 5. 统一发送调度

文件：

- [include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33)

当前语义：

- `0x0301`：FIFO，高优先级，统一限频
- `0x0305`：latest-only，只保留最新一帧
- 真正写串口失败时，会把串口打回断开状态，由主循环自动重连

因此当前项目里不应再让各业务模块直接写串口，而应走调度器。

## 6. 主循环真实职责

文件：

- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

当前主循环负责：

- 串口自动重连
- TCP 自动重连
- TCP 连接超时与 idle timeout
- 真实输入读取
- 文件回放推进
- 周期任务推进：
  - `MapRobotRelay::ProcessPeriodic()`
  - `RadarCommandSender::ProcessPending(...)`
  - `RefereeTxScheduler::Process()`
  - `RuntimeMetrics::MaybeFlush()`

当前主程序的“运行感受”主要由这个文件决定。

## 7. 日志与观测点

默认构建下，每次运行会在 `test/logs/<timestamp>/` 下创建新目录。

当前常见日志：

- `main/serial_state.log`
  - 串口打开失败、掉线、重连
- `main/tcp_startup.log`
  - 启动阶段各 TCP 客户端是否拉起
- `main/tcp_channel_state.log`
  - 连接通道的状态变化
- `main/tcp_client_state.log`
  - TCP 客户端连接完成、断开、超时
- `main/runtime_metrics.log`
  - 主循环耗时、日志队列压力、loss_rate 快照
- `main/0x0305_map_robot_data.log`
  - 成功发出的 `0x0305`
- `main/0x0305_map_robot_data_skipped.log`
  - 因 stale、rate limit 等原因未发送
- `main/0x0121_*.log`
  - 自主决策、敌方密钥提交、拒绝原因
- `raw/*.bin`
  - 原始串口/TCP 输入字节流

当前代码已经不再创建 `latest/` 快照目录。

## 8. `seq` 与 `loss_rate` 的现实含义

当前至少有四类 `Referee` 解包器实例：

- `serial_referee`
- `info_wave_referee`
- `enemy_level1_key_receiver` 内部的 `referee_`
- `enemy_level2_key_receiver` 内部的 `referee_`

它们彼此独立维护：

- `seq`
- `received_packets`
- `loss_rate()`

这意味着：

- 串口和信息波不是共用一组 `seq`
- `8002/8003` 也不是和主信息波共用同一组 `seq`

需要特别注意的是：

- `loss_rate()` 来自 [include/radar/libs/librm/src/librm/device/referee/referee.hpp](/home/hanni/Radar/include/radar/libs/librm/src/librm/device/referee/referee.hpp:1)
- 它默认假设发送端 `seq` 按 256 周期滚动
- 对信息波 TCP 流来说，这个值不一定等于真实网络丢包率
- 如果发送端 `seq` 语义不符合这个假设，`loss_rate` 甚至可能出现负值或明显失真

因此排查 TCP 丢包时，应优先结合：

- 原始 `raw/*.bin`
- `seq` 序列是否跳变
- `0x0A01` 实际到达间隔
- `runtime_metrics.log`

不要只盯 `loss_rate()` 一个指标。

## 9. 当前修改边界

优先改：

- `src/`
- `include/config/`
- `include/referee/`
- `include/log/`
- `.agent/`

尽量不要广泛改动：

- `include/radar/`

原因：

- `include/radar/` 是协议层和第三方 `librm` 子树
- 当前仓库对 `include/radar` 的改动应该尽量少且明确

## 10. 接手时最先读什么

如果你是第一次接手当前仓库，建议阅读顺序：

1. [README.md](/home/hanni/Radar/README.md:1)
2. [src/main.cc](/home/hanni/Radar/src/main.cc:45)
3. [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:32)
4. [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)
5. [include/referee/map_robot_relay.hpp](/home/hanni/Radar/include/referee/map_robot_relay.hpp:254)
6. [include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:85)
7. [include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33)
8. [.agent/radar_stability_review.md](/home/hanni/Radar/.agent/radar_stability_review.md:1)

如果问题是：

- TCP 连不上：优先看 [include/referee/tcp_client.hpp](/home/hanni/Radar/include/referee/tcp_client.hpp:46)、[include/referee/tcp_connection_log.hpp](/home/hanni/Radar/include/referee/tcp_connection_log.hpp:23) 和 `main/tcp_*.log`
- `0x0305` 发送异常：优先看 [include/referee/map_robot_relay.hpp](/home/hanni/Radar/include/referee/map_robot_relay.hpp:254)、[include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33)
- `0x0121` 发送异常：优先看 [include/referee/radar_decision_tree.hpp](/home/hanni/Radar/include/referee/radar_decision_tree.hpp:45)、[include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:85)
- 运行卡顿或日志过多：优先看 [include/log/log_backend.hpp](/home/hanni/Radar/include/log/log_backend.hpp:129) 与 `runtime_metrics.log`
