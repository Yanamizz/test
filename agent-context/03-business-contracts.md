# Business Contracts

用途：定义关键业务语义，避免跨模块理解偏差。  
更新时间：2026-06-27
适用场景：串口/网络语义相关改动、联调问题分析。

## 阶段语义（权威）

当前业务阶段已从三阶段升级为五阶段：

- `stage1 / stage2 / stage3`：复用当前 `stage12` 资源组
- `stage4 / stage5`：复用当前 `stage3` 资源组

阶段推进规则：

1. 主程序通过 TCP 监听单字节 `0x00/0x01`
2. 每次 `0x00 -> 0x01` 上升沿，业务阶段推进一次
3. `0x01` 持续保持不会重复推进，必须先回到 `0x00`
4. 阶段上限为 `stage5`
5. 锁定进度 `P` 规则如下：
   未被激光照射时，`P` 以 `0.5/s` 匀速衰减，不低于 `0`，并立即清零本次连续照射的 `t/n`
   被激光照射时，`P` 停止衰减；每累计 `0.1s` 连续照射，触发一次增长
   第 `1` 个 `0.1s`：`P += 0.6`
   第 `2` 个 `0.1s`：`P += 1.2`
   第 `n` 个 `0.1s`：`P += 0.6 * n`
   若单次连续照射不足 `0.1s` 即中断，则 `t/n` 立即归零

## AimbotTarget 语义（权威）

线协议常量位于 `include/SerialTask/Common.hpp`：

- `kAimbotTargetMin = 0`
- `kAimbotTargetActiveThreshold = 1`

行为约定：

1. 主程序不再通过 TCP 维护 AimbotTarget 网络计数；TCP 仅负责推进业务阶段
2. 首次获得“有效目标距离”时，触发激光开启 flag，串口 `AimbotTarget` 开始保持 `0x01`；当前“有效”定义为 `0 < distance <= aimbot_target_laser_max_distance_m`
3. 进入业务 `stage4/stage5` 时仍固定发送 `0x01`
4. 距离不再直接关闭激光；触发开启 flag 后，串口 `AimbotTarget` 保持 `0x01`

## 运行参数约定

- 默认运行参数定义在 `include/Tools/RuntimeParams.hpp`
- 相机运行时曝光参数文件：`camera_runtime_params.ini`
- 关键行为变更时，需同步更新交接摘要与本约定文档
