# Integration And Troubleshooting

用途：联调与高频故障快速定位。
更新时间：2026-05-30
适用场景：构建失败、串口异常、阶段切换异常。

## TCP 网络控制链路

旧 `NetworkTask` TCP 联调示例已删除。当前主程序不再通过 TCP 网络通信控制激光开关，也不再维护网络 AimbotTarget 计数。

## 高频排障清单

1. 编译失败：先检查 Eigen3/OpenVINO/Galaxy SDK 依赖
2. 串口行为异常：检查 `SerialSend.hpp` 线值消费、`ImagePredict.cc` 中距离触发阶段判断 flag、`stage1->stage2` 关闭窗口与 `stage3` 强制开激光逻辑
3. 锁定阶段异常：检查 `src/ImagePredict.cc` 中阶段更新与切换路径
