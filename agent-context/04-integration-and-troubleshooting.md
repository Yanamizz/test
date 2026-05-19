# Integration And Troubleshooting

用途：网络联调与高频故障快速定位。  
更新时间：2026-05-19  
适用场景：A/B 设备联调、构建失败、串口异常、阶段切换异常。

## 网络联调基础

- 默认 TCP 端口：`5000`
- 默认对端：`NetworkTask::kDefaultPeerIp`
- 详细联调示例：`src/README.md`

建议先做：

1. 双方 IP 配置在同网段
2. 双向 `ping` 连通
3. 先启动收方再启动发方（或发方重试连接）

## 高频排障清单

1. 编译失败：先检查 Eigen3/OpenVINO/Galaxy SDK 依赖
2. 串口行为异常：检查 `SerialSend.hpp` 二值化发送与接收计数逻辑
3. 锁定阶段异常：检查 `src/ImagePredict.cc` 中阶段更新与切换路径
4. 网络收发异常：检查 `AimbotTargetReceiver.hpp` 与 socket 超时/断链处理
