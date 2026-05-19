# Latest Handoff

用途：记录最近一次交接结论与后续建议。  
更新时间：2026-05-19  
适用场景：新 Agent 快速接续、避免重复排查。

## 本轮完成事项（延迟优先）

1. CPU 大核识别修正（12900H 重点）  
- 文件：`include/Tools/CpuAffinity.hpp`  
- 变更：`detectBigCoresByType()` 从“`core_type >= 1`”改为“取观测到的最大 `core_type` 作为性能核组”。  
- 目的：避免把 E 核误归为大核，降低推理线程调度抖动。

2. 增加主线程辅助核绑定接口  
- 文件：`include/Tools/CpuAffinity.hpp`  
- 变更：新增 `BindCurrentThreadToAuxCores()`。

3. 主线程初始化后绑核策略调整  
- 文件：`src/ImagePredict.cc`  
- 变更：模型初始化完成后，主线程从全核改为辅助核集合。  
- 目的：减少与推理关键路径争抢性能核。

4. OpenVINO 推理线程策略改为延迟导向  
- 文件：`include/ImageRecognize/ImagePredict_OPENVINO.hpp`  
- 变更：参数字段改名 `hw_threads_reserved -> latency_threads_cap`；  
  `infer_threads` 由“总线程减保留”改为 `min(hw_threads, latency_threads_cap)`，并带最小保护。  
- 目的：抑制过高并发导致的尾延迟与抖动。

## 当前风险与注意事项

1. 设备限制  
- 当前环境仅编辑器，无法编译与运行验证；本轮为静态改动与一致性检查。

2. 工作区状态  
- 仓库存在历史改动，交接时不要回退与本轮无关的变更。

3. 已知文本编码问题  
- 个别旧文件曾出现中文注释乱码；后续若继续维护，建议统一 UTF-8 编码并复查注释行是否影响代码可读性。

## 下一步建议（上线前）

1. 实机验证指标  
- p50/p95/p99 推理时延  
- 串口发送周期抖动  
- 丢帧率与目标锁定稳定性

2. 若延迟仍波动较大  
- 在不影响精确度前提下，继续下调 `latency_threads_cap` 做 A/B 对比。  
- 复核 `IMUSendThread` 是否需要更高优先级或更强隔离策略。
