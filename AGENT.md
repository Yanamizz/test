# AGENT

本仓库的上下文单入口文件。  
新 Agent 接手时，只需要执行一句话：`阅读 AGENT.md 来获取项目上下文`。

## 接手契约（强制）

1. 先完整阅读本文件。  
2. 必须读取 `my-skill/my_skill/SKILL.md`。  
3. 读取后必须按 `my_skill` 的规则执行本次会话。  
4. 再按“固定阅读顺序”读取 `agent-context/`。  
5. 若与其他文档冲突，以 `AGENT.md` 与其索引的 `agent-context/` 为准。  
6. 新增关键规则时，先更新 `agent-context/` 对应主题，再回写本索引。

## 固定阅读顺序（必读）

1. `my-skill/my_skill/SKILL.md`
2. `agent-context/01-core-overview.md`
3. `agent-context/02-build-and-run.md`
4. `agent-context/03-business-contracts.md`
5. `agent-context/04-integration-and-troubleshooting.md`
6. `agent-context/05-change-entrypoints.md`
7. `agent-context/06-latest-handoff.md`

## 按任务扩展阅读

- 主流程/线程/时序改动：优先读 `03` + `05` + `06`
- 构建或环境问题：优先读 `02` + `04`
- 串口/网络语义问题：优先读 `03` + `04`
- 扫描发送/阶段切换/丢目标恢复重构：优先读 `05` + `06`
- 不确定入口文件：优先读 `05`

## 当前项目关键约定（摘要）

- 性能优先级：`精确度 > 延迟 > CPU 利用率`（在不破坏功能前提下优化）。  
- 去畸变策略：由于畸变参数不准确，项目约定可去除去畸变操作。  
- 上下文维护：最新交接统一追加到 `agent-context/06-latest-handoff.md`。  
- 代码改动前，先核对 `agent-context/05-change-entrypoints.md` 的入口映射。
- 当前主流程已新增 4 个关键 seam：`TargetTrackPipeline`、`StageRuntimeProfile`、`StagePredictorController`、`LostTargetRecoveryController`；扫描发送线程已新增 `ScanSendController`。后续优先沿这些 seam 深化，避免把状态重新摊回 `src/ImagePredict.cc`。

## 提交前校对清单

1. 关键业务语义是否与 `03-business-contracts.md` 一致。  
2. 默认端口/IP、运行方式是否与 `02`/`04` 一致。  
3. 本次改动入口是否与 `05-change-entrypoints.md` 对齐。  
4. 若变更关键行为，是否已更新 `06-latest-handoff.md`。

## 兼容说明

- 若存在 `AGENTS.md`，应仅作为兼容跳转，不承载业务细节。  
- `README.md` 面向人类使用说明；接手上下文仍以本文件为准。
