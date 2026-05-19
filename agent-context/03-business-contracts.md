# Business Contracts

用途：定义关键业务语义，避免跨模块理解偏差。  
更新时间：2026-05-19  
适用场景：串口/网络语义相关改动、联调问题分析。

## AimbotTarget 语义（权威）

常量位于 `include/SerialTask/Common.hpp`：

- `kAimbotTargetMin = 0`
- `kAimbotTargetActiveThreshold = 1`
- `kAimbotTargetMax = 3`

行为约定：

1. 初始值为 `0`
2. 每收到一次网络 `1`，内部计数 `+1`，上限 `3`
3. 锁定流程 `stage` 每变化一次，内部计数 `-1`，下限 `0`
4. 串口发送时采用二值化：`AimbotTarget >= 1` 发送 `1`，否则 `0`

## 运行参数约定

- 默认运行参数定义在 `include/Tools/RuntimeParams.hpp`
- 相机运行时曝光参数文件：`camera_runtime_params.ini`
- 关键行为变更时，需同步更新交接摘要与本约定文档
