# Integration And Troubleshooting

用途：联调与高频故障快速定位。
更新时间：2026-07-03
适用场景：构建失败、串口异常、阶段切换异常。

## TCP 网络控制链路

旧 `NetworkTask` 激光控制计数链路已删除。当前主程序默认作为 TCP client 连接 `192.168.50.75:9001`，由阶段 server 发送命令；TCP 阶段输入线程接收 `0x91` 并维护 `game_progress/stage_remain_time`，接收 `0x92` 的低 `1 bit` 后按 `0->1` 上升沿推进五阶段业务状态，不维护网络 AimbotTarget 计数。

## 高频排障清单

1. 编译失败：先检查 Eigen3/OpenVINO/Galaxy SDK 依赖
2. 串口行为异常：检查 `SerialSend.hpp` 线值消费、`ImagePredict.cc` 中距离触发激光 flag 与 `stage4/5` 强制开激光逻辑
3. 锁定阶段异常：检查 `src/ImagePredict.cc` 中 `TCPStageThread`、`sync_tcp_stage()` 与外部 `0x92` 发送是否存在缺失的 `0->1` 边沿
4. 比赛状态异常：检查 `0x91` 的低 `4 bit` 是否正确落到 `game_progress`，以及后 `2Byte` 是否按大端发送 `stage_remain_time`
