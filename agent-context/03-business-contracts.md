# Business Contracts

用途：定义关键业务语义，避免跨模块理解偏差。  
更新时间：2026-06-01
适用场景：串口/网络语义相关改动、联调问题分析。

## AimbotTarget 语义（权威）

线协议常量位于 `include/SerialTask/Common.hpp`：

- `kAimbotTargetMin = 0`
- `kAimbotTargetActiveThreshold = 1`

行为约定：

1. 主程序不再通过 TCP 网络通信控制激光开关，也不再维护 AimbotTarget 网络计数
2. 首次获得“有效目标距离”时，触发激光开启 flag，串口 `AimbotTarget` 开始保持 `0x01`；当前“有效”定义为 `0 < distance <= aimbot_target_laser_max_distance_m`
3. 首次获得有效目标距离时，阶段判断也初始化/重置唯一一次；目标初始就在该距离阈值以内也会执行
4. 不再设计或执行 `stage1 -> stage2` 后的激光关闭窗口；阶段切换不会主动把串口 `AimbotTarget` 压到 `0x00`
5. 进入 `stage3` 时仍固定发送 `0x01`
6. 距离不再直接关闭激光；触发开启 flag 后，串口 `AimbotTarget` 保持 `0x01`

## 运行参数约定

- 默认运行参数定义在 `include/Tools/RuntimeParams.hpp`
- 相机运行时曝光参数文件：`camera_runtime_params.ini`
- 关键行为变更时，需同步更新交接摘要与本约定文档
