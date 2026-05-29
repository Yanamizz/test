# Business Contracts

用途：定义关键业务语义，避免跨模块理解偏差。  
更新时间：2026-05-30
适用场景：串口/网络语义相关改动、联调问题分析。

## AimbotTarget 语义（权威）

线协议常量位于 `include/SerialTask/Common.hpp`：

- `kAimbotTargetMin = 0`
- `kAimbotTargetActiveThreshold = 1`

行为约定：

1. 主程序不再通过 TCP 网络通信控制激光开关，也不再维护 AimbotTarget 网络计数
2. 首次获得“有效目标距离”时，触发激光开启 flag，串口 `AimbotTarget` 开始保持 `0x01`；当前“有效”定义为 `0 < distance <= aimbot_target_laser_max_distance_m`
3. 首次获得有效目标距离时，阶段判断也初始化/重置唯一一次；目标初始就在该距离阈值以内也会执行
4. `stage1 -> stage2` 完成后进入 55s 激光关闭窗口，串口帧 `AimbotTarget` 发送 `0x00`
5. 关闭窗口结束后恢复发送 `0x01`；进入 `stage3` 时清除关闭窗口并固定发送 `0x01`
6. 距离不再直接关闭激光；串口 `AimbotTarget` 只会被 `stage1 -> stage2` 后的关闭窗口压到 `0x00`

## 运行参数约定

- 默认运行参数定义在 `include/Tools/RuntimeParams.hpp`
- 相机运行时曝光参数文件：`camera_runtime_params.ini`
- 关键行为变更时，需同步更新交接摘要与本约定文档
