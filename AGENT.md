# AGENT

本文件用于让新接入模型快速理解 `antidrone` 项目结构、运行方式和推荐工作流。

## 1. 项目是什么

`antidrone` 是雷达站目标识别与云台控制项目，主链路是：

1. 相机采集图像
2. OpenVINO 推理目标
3. 跨帧跟踪 + 角度/激光补偿计算
4. 串口发送云台控制帧
5. 网络线程接收外部 `AimbotTarget` 触发

## 2. 代码主要分布

- 主流程与线程编排：
  - `src/ImagePredict.cc`
- 识别与跟踪：
  - `include/ImageRecognize/`
- 串口协议与收发：
  - `include/SerialTask/`
- 网络 TCP 接收：
  - `include/NetworkTask/`
- 工具与控制算法（角度、扫描、补偿、参数）：
  - `include/Tools/`
- 第三方依赖：
  - `third_lib/`

重点文件：

- `src/ImagePredict.cc`：主循环 + 线程 + 阶段切换 + 串口发送联动
- `include/NetworkTask/AimbotTargetReceiver.hpp`：TCP 接收与 `AimbotTarget` 更新
- `include/SerialTask/SerialSend.hpp`：串口帧封包发送（二值化 `AimbotTarget`）
- `include/Tools/LaserAngleCalculate.hpp`：距离与激光 pitch 补偿
- `include/Tools/RuntimeParams.hpp`：默认运行参数

## 3. 构建与运行

构建：

```bash
cmake -S . -B build
cmake --build build -j
```

运行：

```bash
./run    # 默认低干扰运行
./test   # 默认调试运行（显示窗口）
```

主程序二进制：

```text
build/bin/ImagePredict
```

## 4. 当前关键业务约定（必读）

### AimbotTarget 语义

常量定义在 `include/SerialTask/Common.hpp`（`SerialTask` 命名空间）：

- `kAimbotTargetMin = 0`
- `kAimbotTargetActiveThreshold = 1`
- `kAimbotTargetMax = 3`

行为：

1. 初始值为 `0`
2. 网络每收到一次 `1`，内部计数 `+1`，上限 `3`
3. 锁定流程 `stage` 每变化一次，内部计数 `-1`，下限 `0`
4. 串口发送时：`AimbotTarget >= 1` 则线协议发送 `1`，否则 `0`

## 5. 推荐 Skill（本仓已落地）

本仓库内路径：`my-skill/`

- `my-skill/caveman/`
- `my-skill/diagnose/`
- `my-skill/grill-me/`
- `my-skill/improve-codebase-architecture/`
- `my-skill/zoom-out/`

建议使用顺序：

1. `zoom-out`：先给目标区域画模块地图和调用链
2. `diagnose`：若是 bug/性能问题，先建立可重复反馈回路
3. `grill-me`：在改设计或拆分前逐题确认关键决策
4. `improve-codebase-architecture`：做结构优化时用，优先深模块与可测试 seam
5. `caveman`：需要极简高密度沟通时启用

## 6. 给新模型的工作规则

1. 优先做“低风险、可编译验证”的保守修改。
2. 不要回滚用户已有改动（当前工作区常常非干净）。
3. 每次改动后至少执行一次：
   - `cmake --build build -j`
4. 涉及 `src/ImagePredict.cc` 的修改，先确认不会破坏线程协作和发送时序。
5. 文档与实现保持同步，尤其是 `AimbotTarget` 与运行参数语义。

## 7. 高频风险点

1. `ImagePredict.cc` 体量大，跨线程共享状态多，改动容易引入时序问题。
2. `third_lib` 下存在大量外部代码，避免无关编辑。
3. 环境依赖不齐时，可能出现 OpenVINO / Eigen / SDK 相关构建问题。

## 8. 快速排障清单

1. 编译失败先看是否为依赖缺失（Eigen3/OpenVINO/Galaxy SDK）。
2. 串口行为异常先看：
   - `SerialSend.hpp` 的二值化是否符合预期
   - `AimbotTargetReceiver.hpp` 是否收到并累加
3. 锁定阶段行为异常先看：
   - `ImagePredict.cc` 中 `g_aerial_robot_stage` 更新与切换逻辑
4. 网络联调先看：
   - `src/README.md`

## 9. 变更入口索引

按任务类型定位文件：

1. 改主流程/线程协作：
   - `src/ImagePredict.cc`
2. 改网络收发协议与连接：
   - `include/NetworkTask/AimbotTargetReceiver.hpp`
   - `include/NetworkTask/DeviceAServer.hpp`
   - `include/NetworkTask/DeviceBClient.hpp`
   - `include/NetworkTask/SocketCommon.hpp`
3. 改串口帧语义与发送：
   - `include/SerialTask/Common.hpp`
   - `include/SerialTask/SerialSend.hpp`
4. 改角度/激光补偿：
   - `include/Tools/AngleCalculate.hpp`
   - `include/Tools/LaserAngleCalculate.hpp`
   - `include/Tools/ScanController.hpp`
5. 改默认运行参数：
   - `include/Tools/RuntimeParams.hpp`
6. 改联调文档：
   - `README.md`
   - `src/README.md`

## 10. 提交前检查表

1. 编译通过：
   - `cmake --build build -j`
2. 文档是否与行为一致（尤其 `AimbotTarget`、默认端口/IP、运行命令）。
3. 是否误改 `third_lib/` 外部代码。
4. 是否保留并解释了必要日志，移除了调试临时代码。
5. `git status` 检查变更范围是否只覆盖本次任务目标。

## 11. 当前会话交接摘要（2026-05-18）

详细 handoff 文件：

- `/tmp/handoff-oOAMUY.md`

本轮关键新增：

1. `AimbotTarget` 语义已在主流程、网络、串口、示例统一：
   - 收到网络 `1` 计数 +1（上限 3）
   - stage 变化计数 -1（下限 0）
   - 串口按 `>=1` 二值发送
2. 网络默认配置已集中：
   - `NetworkTask::kDefaultTcpPort`
   - `NetworkTask::kDefaultPeerIp`
3. `ImagePredict.cc` 第一层拆分已做：
   - `UpdateAerialRobotStageAndSwitchFlag`
   - `UpdateTargetVisibleAndLostSince`
   - `Stage3SwitchTargetLostLongEnough`
4. 延迟统计关键修复已完成：
   - stage3 切换 `continue` 前补 `loop_ns + frame`
   - `loop_ns` 计时覆盖 `isAsyncReady` 等待
5. 开关优先级已统一：
   - 静态开关 + CLI 同时有效
   - CLI 冲突优先
6. 曝光配置文件已统一到同一路径机制：
   - 显式 path > 环境变量 `ANTIDRONE_RUNTIME_PARAMS_PATH` > 编译期项目根路径
   - `run/test` 均显式注入同一路径

注意事项：

1. 用户明确 `Antidrone Run.desktop` / `Antidrone Test.desktop` 的删除是主动行为，不要恢复。
2. 工作区非干净，`third_lib/*` 存在本地状态，后续不要误回滚。

建议下一步：

1. 给延迟统计增加 bucket 覆盖率计数（避免“部分路径记账”的均值误导）。
2. 继续 `ImagePredict.cc` 轻拆分（发送决策块、渲染块再抽 helper，不改线程模型）。
3. 曝光加载/保存时增加实际路径打印（debug 级），便于现场确认读取来源。

建议技能顺序：

1. `zoom-out`
2. `diagnose`
3. `grill-me`
4. `improve-codebase-architecture`
5. `caveman`
