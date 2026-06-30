# Radar - Project Agent Guide

本文件描述仓库根目录下代码的项目级协作约束。  
若进入 `include/radar/libs/librm/` 子树，需同时遵守该目录下已有的 [AGENTS.md](include/radar/libs/librm/AGENTS.md)。

## 1. 项目定位

本项目当前是一条“接收 -> 处理 -> 发送”的雷达通信主链路，而不是视觉识别项目。

当前主职责：

- 接收裁判系统常规链路主协议
- 接收信息波链路数据
- 维护协议对应结构体状态
- 根据规则生成 `0x0305` 和 `0x0121` 等发送数据
- 记录结构体日志、原始二进制日志、运行时指标

当前不应把系统重心放在：

- 视觉识别
- 相机目标检测
- 在本项目内实现反无人机功能

## 2. 代码边界

优先修改以下项目侧目录：

- `src/`
- `include/config/`
- `include/referee/`
- `include/log/`
- `test/`
- `.agent/`

尽量不要改动以下目录：

- `include/radar/`

说明：

- `include/radar/` 下主要是外部协议层、已有业务层和 `librm` 代码。
- 如果只是增加项目功能，优先在 `include/referee/` 或 `include/log/` 做封装，不要把项目逻辑直接塞回协议头。
- 如果未来规则升级必须改协议结构体、命令码或 memory map，再最小化修改 `include/radar` 中对应协议文件。

## 3. 单一配置入口

协议版本、输入源模式、日志模式、回放参数、端口和超时，统一以：

- `include/config/config.hpp`

为唯一手动配置入口。

重要约定：

- 版本切换优先只改 `radar::config::kRefereeRevision`
- 串口/信息波真实输入或文件回放，只改 `config.hpp`
- 不要在 `src/main.cc`、测试程序或各业务模块中重复硬编码版本名、端口、路径、频率

## 4. 当前主链路

主入口：

- `src/main.cc`

主程序保持尽量简洁，只保留主链：

1. 初始化日志、串口、TCP、状态对象
2. 给常规链路与信息波链路挂回调
3. 进入统一输入循环

主链细节尽量放在头文件封装中。

当前关键模块：

- `include/referee/referee_input_loop.hpp`
  - 统一主循环
  - 负责真实输入与文件回放输入调度
  - 负责串口自动重连
  - 负责周期任务推进
- `include/referee/serial_port.hpp`
  - 裁判系统串口读写
- `include/referee/tcp_server.hpp`
  - `8001/8002/8003` TCP 接入
- `include/referee/replay_input_source.hpp`
  - 预制 `.bin` 文件按频率回放
- `include/referee/map_robot_relay.hpp`
  - `0x0A01 -> 0x0305` 状态维护与发送
- `include/referee/radar_decision_tree.hpp`
  - 基于 `0x020E` 的自主决策
- `include/referee/radar_command_sender.hpp`
  - `0x0121` 组包、冷却和日志
- `include/referee/referee_tx_scheduler.hpp`
  - `0x0301` / `0x0305` 统一发送调度

## 5. 输入源约定

当前支持两类输入源：

- `kReal`
  - 串口主协议来自真实串口
  - 信息波来自真实 TCP `8001`
- `kFile`
  - 从预制二进制文件回放原始字节流

当前约束：

- 串口主协议与信息波 `8001` 可独立切换真实/文件模式
- 串口与信息波各自独立配置回放频率
- `8002/8003` 目前保持真实 TCP 输入，不走文件回放
- 即使串口输入改为文件回放，发送仍继续走真实串口

样例文件位于：

- `test/info/outputInfo.bin`
- `test/info/outputInfo2.bin`
- `test/info/map_robot_sender_sample.bin`

## 6. 日志与可观测性

日志是比赛后复盘证据链的一部分，不能随意删除。

当前日志分三类：

1. 结构体日志
2. 原始二进制日志
3. 运行时指标日志

相关模块：

- `include/log/log_backend.hpp`
- `include/log/referee_main_log.hpp`

约定：

- 每次运行应创建新的时间戳日志目录
- 原始二进制日志优先保留
- `latest/*.json` 属于快照，不能替代原始日志
- 性能优化优先通过异步写入、限频和模式区分完成，不要直接删除关键日志

## 7. 协议与规则变更

当规则更新时，优先按以下顺序处理：

1. 检查 `include/radar/libs/librm/src/librm/device/referee/` 下协议头是否已支持新版本
2. 若未支持，补协议结构体、命令码和映射
3. 回到 `include/config/config.hpp` 切换 `kRefereeRevision`
4. 检查 `include/referee/` 中是否存在版本差异需要适配
5. 验证 `0x0305`、`0x0121`、日志链路是否仍正确

原则：

- 主程序不要直接写协议版本分支
- 版本差异优先封装在协议层或 `include/referee/` 的适配层

## 8. 测试与调试

当前常用目标：

- `radar`
- `referee_serial_logger`
- `referee_map_robot_sender`

构建命令：

```bash
cmake --build build -j2
```

如果只编主程序：

```bash
cmake --build build --target radar -j2
```

测试程序用途：

- `test/referee_serial_logger.cc`
  - 主协议解包与日志验证
- `test/referee_map_robot_sender.cc`
  - `0x0305` 最小发送测试
- `test/serialtest.py`
  - 简单串口辅助测试

## 9. 修改风格

推荐做法：

- 保持 `src/main.cc` 简洁
- 复杂逻辑放进 `include/referee/` 或 `include/log/`
- 新增功能优先做项目侧封装，不直接污染协议头
- 先维护状态，再做决策，再统一发送
- 修改后至少重新构建一次

不推荐做法：

- 在多个文件重复写同一套版本/端口/路径常量
- 把日志打印散落到主程序各处
- 为临时调试把关键逻辑写死在 `main.cc`
- 没有必要时大范围改动 `include/radar/`

## 10. 相关文档

建议优先参考：

- `README.md`
- `.agent/handoff.md`
- `.agent/full_pipeline.md`
- `.agent/radar_stability_review.md`

这些文档分别对应：

- 项目定位和范围
- 历史交接上下文
- 从接收到发送的全流程说明
- 稳定性问题与优化记录
