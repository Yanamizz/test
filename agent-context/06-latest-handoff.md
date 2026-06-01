# Latest Handoff

用途：记录最近一次交接结论与后续建议。  
更新时间：2026-06-02
适用场景：新 Agent 快速接续、避免重复排查。

## 本轮完成事项（延迟优先）

28. 四个架构候选全部完成
- 文件：`include/ImageRecognize/Stage3FallbackController.hpp`
- 文件：`include/Tools/AimbotCommand.hpp`
- 文件：`include/Tools/AimbotCommandArbiter.hpp`
- 文件：`include/Tools/RuntimeParamProfiles.hpp`
- 文件：`include/ImageRecognize/OverlayFrameRenderer.hpp`
- 文件：`src/ImagePredict.cc`
- 文件：`agent-context/05-change-entrypoints.md`
- 变更：阶段切换异常兜底、stage3 probe 回退与异常切换前电机响应探测收口到 `Stage3FallbackController`；主流程只负责严格 100ms tick、完成切 stage3 的全局副作用，以及消费 controller 输出。
- 变更：目标控制帧、扫描模式和 pending-send 条件变量仲裁收口到 `AimbotCommandArbiter`，发送命令结构收口为 `AimbotSendCommand`；保留原有 `ClearPendingSend / StorePendingSend / StartScanMode / StopScanModeKeepPendingSend` 包装入口，避免扩大调用面。
- 变更：把阶段兜底、控制限幅/前馈和显示节流这几组常一起读取的 `RuntimeParams` 组装到 `RuntimeParamProfiles`，降低主流程中散落的参数读取。
- 变更：显示窗口、全程 overlay 录像和目标状态录像的帧生成收口到 `OverlayFrameRenderer`；仍保持显示/全程 overlay 画完整检测信息，目标录像只画左上角状态信息。
- 风险判断：中低风险架构收口。该轮触及阶段兜底、发送仲裁和录像渲染，但保持 stage12/stage3 行为分离，不改阈值、阶段规则、扫描参数、串口线协议或激光开启语义。
- 当前状态：已于 2026-06-02 本地执行 `cmake --build build -j --target ImagePredict` 编译通过，仅有 Galaxy SDK `GXDef.h` 既有 typedef warning。

12. target_camp_mode 支持滑块面板运行时切换
- 文件：`include/ImageRecognize/TargetClassFilter.hpp`
- 文件：`include/Tools/CalibrationSliderPanel.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：在现有标定滑块面板增加 `target camp 0R 1B 2All` 三档滑块，用于运行时切换 `RED_AND_PURPLE / BLUE_AND_PURPLE / ALL`。
- 变更：`target_camp_mode` 从预测线程启动时的固定值改为运行时读取；切换后会重置 `TargetTrackPipeline`，避免跨阵营沿用旧 tracker 历史框。
- 设计说明：没有新增单独头文件；运行时控制器放在已有 `TargetClassFilter.hpp`，因为该 Module 已拥有 `TargetCampMode` 的解析、命名和筛选语义，滑块面板只作为写入口。
- 风险判断：低风险。该改动只影响目标类别筛选入口，不改变 stage12/stage3 阶段切换、扫描发送、ROI 或推理链路。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过。

13. AimbotTarget 串口线协议激光开启控制
- 文件：`include/SerialTask/Common.hpp`
- 文件：`include/SerialTask/SerialSend.hpp`
- 文件：`src/ImagePredict.cc`
- 文件：`agent-context/03-business-contracts.md`
- 文件：`README.md`
- 变更：`SerialSend.hpp` 中 `AimbotFrame_SCM_t::AimbotTarget` 恢复消费调用方传入线值，并通过 `ToWireAimbotTarget()` 统一规整为 `0x00/0x01`。
- 变更：已删除锁定流程 `stage1 -> stage2` 完成后的 55s 激光关闭窗口；阶段切换不会主动把串口帧 `AimbotTarget` 压到 `0x00`。
- 变更：进入 `stage3` 时仍固定发送 `AimbotTarget=0x01`。
- 变更：首次获得有效目标距离时触发激光开启 flag，串口 `AimbotTarget` 开始保持 `0x01`；避免目标初始就在 24m 以内时激光一直不开。
- 变更：首次获得有效目标距离时，阶段判断也初始化/重置唯一一次；避免目标初始就在 25m 以内时阶段判断不启动。距离不再直接关闭激光；触发开启 flag 后，串口 `AimbotTarget` 保持 `0x01`。
- 变更：主程序不再通过 TCP 网络通信控制激光开关，也不再维护 AimbotTarget 网络计数；旧 `NetworkTask` 示例后续已删除。
- 风险判断：低风险业务语义变化。该改动只移除激光关闭窗口状态，不合并 stage12/stage3 行为，也不改变 `ToWireAimbotTarget()` 的二值规整。
- 当前状态：已于 2026-06-01 本地重新清理代码链路，等待本轮构建复验。

14. 低风险架构/延迟/鲁棒性巡检收尾
- 文件：`src/ImagePredict.cc`
- 文件：`include/ImageRecognize/TargetClassFilter.hpp`
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`agent-context/04-integration-and-troubleshooting.md`
- 变更：扫描发送线程中 `TickConfig` 生成改为一次 `StageRuntimeProfile` 快照，避免高频扫描路径每轮重复构造 3 次 profile；stage12/stage3 仍各自走独立 profile，不改变发送频率、原点保持或扫描控制器配置。
- 变更：`TargetCampModeController::SetModeIndex()` 从 CAS 循环收敛为单次原子 `exchange`，降低接口实现复杂度；UI 写、推理线程读语义不变。
- 变更：修正 `ImagePredict.cc` 无目标恢复分支缩进，修正 `RuntimeParams.hpp` 中与真实默认值不一致的注释。
- 变更：同步 `04-integration-and-troubleshooting.md`，避免继续提示旧的 AimbotTarget 二值化发送契约。
- 风险判断：低风险。以上改动不改变主程序逻辑链条，不合并 stage12/stage3 行为。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过。

15. 录像叠加左上角状态信息但不录制识别框
- 文件：`include/ImageRecognize/ImageShow.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `ImageShow::DrawStatusText()`，仅绘制 FPS、锁定 Stage/进度和可用距离文字，不绘制检测框、检测中心、预测点或跟踪框。
- 变更：目标录像与全程录像保存前单独 clone 一份视频帧并绘制上述状态文字；显示窗口仍沿用原有画框逻辑。
- 风险判断：低风险。该改动只影响录像输出画面内容，不改变推理、跟踪、串口发送或保存触发条件。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

16. stage3 扫描边界支持 manual/auto
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `stage3_scan_bounds_mode`，默认 `AUTO`；非 `AUTO` 时沿用原有手动 yaw/pitch 扫描上下限。
- 变更：stage12 有目标且实际发送控制命令时，收集发送绝对 yaw/pitch 的最小/最大值；stage3 扫描进入 auto 模式时，用这组范围作为扫描上下限。
- 变更：stage3 auto 模式下，stage3 识别到目标时会用视觉解算出的目标绝对 yaw/pitch 检查是否超出当前 auto 扫描上下限；若超出，则扩张 auto 上下限并触发扫描线程刷新配置。
- 变更：auto 范围会被 clamp 到原手动上下限内；除 yaw/pitch 上下限外，stage3 扫描发送频率、yaw 速度、原点保持、lambda/A 等参数保持不变。若尚未收集到 stage12 控制发送范围，则自动回退手动边界。
- 变更：`ScanSendController` 在扫描态内检测 tick config 变化并刷新 `ScanController` 配置，确保切入 stage3 或 auto 范围更新后立即应用新边界；配置刷新会保留当前扫描 yaw 进度并夹到新边界内。
- 变更：扫描过程中识别到目标并退出扫描后，下一次丢目标重新进入扫描时不再回到原点，而是从上次扫描位置继续；首次进入扫描或非目标打断的扫描仍按原逻辑回原点并执行原点保持。
- 风险判断：低风险。该改动只影响 stage3 扫描边界选择，不改变阶段切换、串口帧格式或 stage12 扫描配置。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

17. stage3 auto 扫描边界延迟优化
- 文件：`src/ImagePredict.cc`
- 变更：为 stage3 auto 扫描目标角范围增加轻量版本号，只有范围真实扩张时才递增；扫描发送线程据此判断 auto 边界是否需要重算。
- 变更：`IMUSendThread` 缓存 `TickConfig`，仅在扫描 stage 模式变化或 auto 角范围版本变化时重新构造 `StageRuntimeProfile` 与扫描配置，避免高频扫描循环每 tick 重算 profile 和抢边界 mutex。
- 风险判断：低风险。该优化不改变 stage12 采样条件、stage3 auto/manual 边界语义、扫描发送频率或阶段切换逻辑。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

18. 全逻辑链低风险延迟优化
- 文件：`src/ImagePredict.cc`
- 变更：预测主循环缓存静态运行参数，包括 IMU 匹配窗口、dt 上限、控制限幅、显示/GUI/延迟统计周期，避免每帧重复读取 `RuntimeParams` 与重复转换 duration/count。
- 变更：主循环每帧缓存 `UsingStage3Predictor()` 结果，跟踪、重识别接管、stage12 角范围采样、stage3 丢目标续行共享同一个 stage 状态快照，减少重复函数调用且保持本帧逻辑一致。
- 变更：`ClearPendingSend()` 与 `StartScanMode()` 仅在状态变化可能唤醒发送线程时通知条件变量；目标可见状态更新去掉同帧重复 atomic 写。
- 变更：相机采集线程缓存 `capture_timeout_ms`，IMU 读/发线程缓存 IMU buffer 最大年龄、读失败休眠、发送空闲休眠；阶段启动距离阈值缓存为静态值。
- 变更：串口发送线程只有在 `enable_latency_profile` 开启时才更新发送延迟统计，默认运行时避免每次正常发送都抢 `g_send_latency_mutex`。
- 风险判断：低风险。该轮只减少重复读取、重复通知和默认关闭调试统计时的锁开销，不改变采集、推理、跟踪、阶段切换、扫描或串口线协议语义。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

19. 旧激光偏移角补偿下线
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 文件：`include/Tools/CalibrationSliderPanel.hpp`
- 文件：`include/ImageRecognize/StagePredictorController.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：删除旧 `LaserAngleCalculator` 及其基于距离/参考距离/固定激光高度的 pitch 偏移角补偿算法；控制链路不再把 `laser_yaw_angle/laser_pitch_angle` 叠加到发送角。
- 变更：保留 `DistanceCalculator`、目标高度标定、距离滤波和 stage12/stage3 距离标定阶段切换；当前首次有效目标距离触发激光开启 flag 并初始化/重置阶段判断，不再按距离直接关闭激光。
- 变更：标定滑块面板移除 `near/far pitch dist` 与 `near/far pitch comp` 控件和显示，仅保留目标高度、曝光和 target camp 切换。
- 风险判断：中低风险业务变更。该改动明确关闭旧激光偏移角补偿，但不改变目标检测、跟踪、距离估计、阶段切换、扫描或串口协议格式。
- 当前状态：已于 2026-05-29 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。下一步需要重新设计激光偏移角算法。
- 后续重构待确认问题：1) 激光相对相机光心的左右/上下/前后物理偏移；2) 激光应打目标框中心、几何中心、某个比例点还是锁定区域中心；3) 补偿模型建议采用“物理几何基础补偿 + 距离标定残差表”；4) stage12 与 stage3 是否使用独立补偿参数；5) 是否已有距离、yaw 误差、pitch 误差、stage、目标位置说明等实测数据。
- 历史说明：本条下线后曾短暂回到纯视觉 yaw/pitch；当前已被 2026-05-30 条目 23 的第一版激光 pitch 几何补偿覆盖。

20. 全程录像改为 raw/overlay 双路输出
- 文件：`include/Tools/SaveVideo.hpp`
- 文件：`include/ImageRecognize/ImageShow.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：启用全程录像后同时写两路 AVI：`full_run_YYYYmmdd_HHMMSS_raw_001.avi` 保存原始 `inflight_frame`，`full_run_YYYYmmdd_HHMMSS_overlay_001.avi` 保存与显示窗口一致的检测框、中心点、跟踪框、FPS、stage/进度和距离文字。
- 变更：新增 `DrawFullOverlay()`，显示窗口和 overlay 全程录像复用同一套叠加绘制逻辑；overlay 信息不再依赖 `enable_display`，关闭显示窗口时仍能完整录下可视化信息。
- 变更：目标片段录像继续只绘制左上角状态文字，不录制识别框、检测中心、预测点或跟踪框，保留 2026-05-29 条目 15 的行为。
- 变更：全程录像 raw/overlay 两个写盘线程都绑定到辅助核索引 `4`，在 12900H 上优先落到 E-core 集合中的单个辅助核，避免占用推理线程使用的大核；若辅助核不足会自动夹到最后一个辅助核。
- 风险判断：低风险。该改动只影响全程录像输出内容、文件命名和录像写盘线程亲和性，不改变采集、推理、跟踪、阶段切换或串口发送链路。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

21. 修复有目标时仍持续扫描
- 文件：`src/ImagePredict.cc`
- 现象：扫描状态下识别到目标后，发送线程可能继续清掉刚生成的目标控制帧，表现为“有目标也不锁定，而是持续扫描”。
- 根因：`IMUSendThread` 在 `scan_mode && target_visible` 分支调用 `ClearPendingSend()`，该函数会同时退出扫描并清空 pending 控制命令；与识别线程 `StorePendingSend()` 竞争时会吞掉锁定命令。
- 变更：新增 `StopScanModeKeepPendingSend()`，扫描线程因目标可见退出扫描时只关闭 `g_send_is_scan`，不清空 pending 目标控制帧；仍保留 `ExitScanModeAndResumeNextEntry()`，所以下次丢目标重新扫描会从上次扫描位置继续。
- 风险判断：低风险。该修复只改变“扫描被目标打断”这一状态转换，不改变扫描参数、目标角度解算、阶段切换或串口帧格式。

22. 删除网络通信控制激光开关链路
- 文件：`src/ImagePredict.cc`
- 历史文件：`include/NetworkTask/AimbotTargetReceiver.hpp`（后续已删除）
- 文件：`include/SerialTask/Common.hpp`
- 变更：主程序不再 include `NetworkTask/NetworkTask.hpp`，不再启动 `AimbotTargetReceiveThread`，不再维护 `g_aimbot_target` 网络计数，也不再在 stage 切换时扣减该计数。
- 历史说明：当时 `AimbotTargetReceiver` 曾保留为独立网络联调工具；当前已被 2026-05-30 条目 25 删除。
- 变更：删除 `kAimbotTargetMax`、`SaturatingIncrementAimbotTarget()`、`SaturatingDecrementAimbotTarget()` 计数 helper；串口 `AimbotTarget` 线值仍经 `ToWireAimbotTarget()` 规整为 `0x00/0x01`。
- 风险判断：低风险。该改动删除网络到激光的控制链，不改变图像识别、阶段切换、stage3 强制开激光或串口帧格式。

23. 第一版激光 pitch 几何补偿模型
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 文件：`include/Tools/CalibrationSliderPanel.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：`DistanceCalculator` 扩展为宽度优先距离估计；bbox 宽度有效时按八棱柱对边宽 `0.05m` 与 `fx` 估距，宽度不可用时回退原高度距离。stage12/stage3 分别保留独立 `near/far_width_pixel` 调参入口，当前默认 `0` 表示尚未完成像素宽度标定。
- 变更：新增第一版激光 pitch 几何补偿：激光在相机上方 `0.09m`，与相机光轴在 `14.313m` 前向距离交汇；补偿只在 `10m <= distance <= 24m` 内生效，距离外退回纯视觉 pitch。
- 变更：补偿计算带入当前视觉 pitch 偏角，避免目标位于画面斜上/斜下时只按中心点距离套用补偿；yaw 仍保持纯视觉偏移，不做激光补偿。
- 变更：`ImagePredict.cc` 在发送前将 `laser_pitch_comp_deg` 叠加到 `delta_pitch_raw`；不改变 stage 判断、扫描、AimbotTarget 激光开启控制或串口帧格式。
- 变更：滑块面板提示更新为 `laser pitch comp: width-first, 10-24m`，暂不新增宽度/激光参数滑块，参数集中在 `LaserAngleCalculate.hpp` 调参区。
- 风险判断：中低风险业务变更。该改动恢复激光 pitch 补偿并把距离来源切到宽度优先，需要实机确认宽度估距稳定性和补偿符号；代码影响面保持在角度/距离计算与发送前叠加。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

24. 宽度像素标定滑块与宽高统计
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 文件：`include/Tools/CalibrationSliderPanel.hpp`
- 文件：`include/Tools/RuntimeStats.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：`DistanceCalculator` 新增 `GetCalibrationTargetWidths()` 与 `SetCalibrationTargetWidths()`，用于读取/调整当前编辑阶段的 `near/far_calibration_target_width`；`near_width_pixel / far_width_pixel` 保留在代码调参区，作为识别框观测像素值。
- 变更：标定滑块面板新增 `near target width` 与 `far target width` 两个滑块，随 stage12/stage3 编辑模式切换同步不同阶段的等效目标宽度标定值；高度滑块、曝光滑块和 target camp 滑块语义不变。
- 变更：主流程获取 bbox 高度时同步获取 bbox 宽度，并把收尾打印从 `[像素高度]` 升级为 `[像素尺寸]`，同时输出宽/高平均、最小、最大值，方便直接用运行日志确定宽度滑块应填的近/远像素中位数。
- 风险判断：低风险。该改动只增加宽度标定入口和统计输出，不改变阶段切换、扫描、串口协议或激光补偿公式。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

24.1 宽度滑块语义修正与临时距离调试显示
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 文件：`include/Tools/CalibrationSliderPanel.hpp`
- 文件：`include/ImageRecognize/ImageShow.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：宽度滑块从调 `near/far_width_pixel` 改为调 `near/far_calibration_target_width`；`near/far_width_pixel` 继续作为代码调参区中的识别框观测像素值，不再由滑块修改。
- 变更：`DistanceCalculator` 新增 `DistanceDebugInfo` 与 `CalculateDistanceWithDebug()`，同时给出 `width_distance / height_distance / used_distance / source`，主控制仍沿用宽度优先、宽度无效回退高度的逻辑。
- 变更：显示/录像状态文字临时增加 `Dw / Dh / Src`，用于标定阶段判断高度与宽度补偿是否有效；标定完成后可删除这组调试显示。
- 变更：按“`Dw` 对应水平 10m”回推 near 等效宽度：stage12 `near_width_pixel=105.029` 对应 `near_calibration_target_width=0.05767m`，stage3 `near_width_pixel=85.880` 对应 `near_calibration_target_width=0.04715m`。这两个 near width 当前不再使用理论 `0.05m`，而是使用近点实测等效宽度。
- 风险判断：低风险标定辅助。该改动只修正调参入口语义并增加临时可视化，不改变阶段切换、扫描、串口协议或激光补偿公式。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

25. 删除旧 TCP NetworkTask 死代码
- 删除文件：`include/NetworkTask/AimbotTargetReceiver.hpp`
- 删除文件：`include/NetworkTask/DeviceAServer.hpp`
- 删除文件：`include/NetworkTask/DeviceBClient.hpp`
- 删除文件：`include/NetworkTask/NetworkTask.hpp`
- 删除文件：`include/NetworkTask/SocketCommon.hpp`
- 删除文件：`src/device_a_server.cc`
- 删除文件：`src/device_b_client.cc`
- 删除文件：`src/README.md`
- 变更：取消 TCP 控制激光后，旧 NetworkTask 头文件和两个独立 TCP 示例程序已无主链路引用；删除后 CMake 不再生成 `device_a_server` / `device_b_client` 示例目标。
- 变更：同步 `README.md`、`AGENT.md`、`01-core-overview.md`、`04-integration-and-troubleshooting.md`、`05-change-entrypoints.md`，避免继续把 `NetworkTask` 或 `src/README.md` 作为有效入口。
- 风险判断：低风险死代码清理。该改动不触碰 `ImagePredict` 主程序、串口发送、阶段切换、扫描或激光补偿逻辑。

26. stage1/2 激光 pitch 补偿项抑抖
- 文件：`src/ImagePredict.cc`
- 变更：新增 `Stage12LaserPitchCompStabilizer`，复用现有 `OneEuroFilter` 对 `stage1/2` 的 `laser_pitch_comp_deg` 做单独平滑，并叠加变化率限幅，降低识别框抖动导致的补偿角快速跳变。
- 变更：`stage3` 不使用该抑抖器，仍保持原有补偿行为；此次改动不钝化 `visual_pitch` 主角链，只抑制补偿项本身。
- 变更：在丢目标、无匹配 IMU、阵营切换、stage 切换复位等场景下重置补偿抑抖状态，避免把旧目标或旧 stage 的滤波尾迹带入下一段控制。
- 风险判断：低风险。该改动只影响 `stage1/2` 发送前的激光 pitch 补偿平滑，不改变距离估计、扫描、阶段切换、串口帧格式或 `stage3` 行为。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

27. 默认 calibration 改为由 pixel 自动反推
- 文件：`include/Tools/CameraData.hpp`
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 变更：提取 `CameraData` 中的焦距常量 `kFocalX / kFocalY`，供距离标定默认值计算复用，避免在多个位置手抄内参数值。
- 变更：`DistanceCalculator` 新增 `CalibrationTargetMetersFromPixel()`，默认 `near/far_calibration_target_width/height` 不再手填固定结果，而是按 `10m/24m` 水平距离、当前 `near/far pixel` 与焦距自动反推。
- 变更：保留运行时滑块 `SetCalibrationTargetHeights/Widths()` 覆盖能力；因此“自动反推”只作用于默认初始值，不影响你在标定面板上的手调逻辑。
- 风险判断：低风险。该改动只消除重复手算和默认值不一致风险，不改变距离估计公式、阶段切换、扫描或串口协议。
- 当前状态：已于 2026-05-30 本地重新 `cmake --build build -j` 编译通过，仅有 Galaxy SDK 头文件既有 warning。

0. 目标框处理链路收口为 `TargetTrackPipeline`
- 文件：`include/ImageRecognize/TargetTrackPipeline.hpp`
- 文件：`include/ImageRecognize/TargetTracking.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `TargetTrackPipeline`，将“候选框 class filter -> 跨帧关联 -> stage3 框稳定化”从 `ImagePredictThread` 主流程中抽离为独立 Module。
- 变更：`ImagePredict.cc` 不再直接持有 `CrossFrameTargetTracker` 与 `TemporalBoxStabilizer`，而是通过统一的 `TargetTrackPipelineResult` 获取 `track_result / tracked_box / track_alive`。
- 变更：该次抽离不改变现有算法参数和业务语义，目标是降低主流程耦合，为后续继续收口状态机与后处理算法提供更清晰 seam。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过。

0. 阶段运行配置收口为 `StageRuntimeProfile`
- 文件：`include/Tools/StageRuntimeProfile.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `StageRuntimeProfile` Module，把 `stage1/2` 与 `stage3` 的模型路径、曝光值、ROI 开关与参数、扫描频率限幅结果、扫描控制器配置、stage3 丢目标续行参数统一组织为一个只读运行时 profile。
- 变更：`ImagePredict.cc` 中原本散落的 `stage12/stage3` 条件分支，已在以下场景切换为 profile 访问：启动日志打印、相机 ROI 初始化、扫描控制器初始化、ROI 模式切换、stage3/stage1/2 模型切换、副作用日志打印。
- 变更：该次重构不修改扫描算法、切阶段时机、ROI 语义、曝光语义和模型推理逻辑，目标是减少主流程 shallow condition，提升阶段差异逻辑的 locality，给后续继续收口状态机与线程协作创造更清晰 seam。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过。

0. 阶段预测器切换收口为 `StagePredictorController`
- 文件：`include/ImageRecognize/StagePredictorController.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `StagePredictorController` Module，把以下原本分散在 `ImagePredictThread` 内的阶段切换逻辑收口：`stage3` 待切换标记、目标丢失延时后切换判断、`stage1/2 <-> stage3` 预测器切换、切换时曝光/ROI/扫描模式/标定阶段/扫描控制器配置等副作用应用。
- 变更：`ImagePredict.cc` 不再直接维护 `active_predictor / using_stage3_predictor / pending_stage3_switch / switch_to_stage*_predictor lambda` 这组状态与逻辑，而是通过 `StagePredictorController` 统一完成异步推理入口选择、自动切换和 GUI 热键切换。
- 变更：该次抽离不改变切阶段时机、切换日志语义、ROI 切换语义、曝光模式切换语义和 stage3 模型懒加载行为，目标是进一步降低主循环 shallow state handling，提升阶段切换逻辑的 locality，为后续继续拆扫描状态机创造更深的 seam。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过。

0. 丢目标恢复流程收口为 `LostTargetRecoveryController`
- 文件：`include/Tools/LostTargetRecoveryController.hpp`
- 文件：`src/ImagePredict.cc`
- 文件：`include/SerialTask/SerialSend.hpp`
- 变更：新增 `LostTargetRecoveryController` Module，把以下原本散落在 `ImagePredictThread` 无目标分支中的逻辑收口：`stage3` 丢目标后固定速度续行准备、续行时长判断、续行控制量生成、续行结束后回退到扫描或清空 pending send。
- 变更：`ImagePredict.cc` 不再直接维护 `stage3_lost_target_coast` 的内部状态，而是通过 `PrepareStage3Coast()` 和 `Update()` 统一处理恢复流程，主循环只消费 `has_command / pending_action` 结果。
- 历史说明：本条中 `SerialSend.hpp` 使用 `ToWireAimbotTarget(AimbotTarget)` 的描述已被 2026-06-01 条目 13 覆盖；当前线协议由首次有效距离激光开启 flag 与 `stage3` 强制开激光规则决定，TCP 内部计数逻辑和 `stage1->stage2` 激光关闭窗口均已删除。
- 变更：该次抽离不改变 `stage3` 续行触发条件、续行固定速度语义、进入扫描时机和 `ClearPendingSend/StartScanMode` 的既有时机，目标是继续降低主循环 shallow recovery handling，提升丢目标恢复逻辑的 locality。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过。

0. 扫描发送状态机收口为 `ScanSendController`
- 文件：`include/Tools/ScanSendController.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `ScanSendController` Module，把 `IMUSendThread` 内部原本局部维护的扫描发送状态机收口：扫描模式首次进入、原点等待、下一次发送时刻、扫描命令构造、原点等待结束后 `scan_controller.Reset()`。
- 变更：`IMUSendThread` 不再直接维护 `last_scan_mode / scan_waiting_at_origin / next_scan_send_time / scan_origin_deadline` 这组局部状态，而是通过 `EnterOrStayScanMode / Step / BuildCommand / FinishSend / ExitScanMode` 统一驱动扫描发送流程。
- 风险说明：该次属于中风险重构，因为它触及串口发送线程的局部状态机；但本轮仍刻意保持原有等待条件、发送频率计算、扫描配置切换和 `WaitUntilNextScanSend/WaitForScanStateChangeFor` 的外部时序不变，没有改扫描算法和共享变量语义。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过。

0. Galaxy 相机 ROI 裁剪开关（以视野换目标占比）  
- 文件：`include/CameraTask/GetImage.hpp`  
- 文件：`include/Tools/RuntimeParams.hpp`  
- 文件：`src/ImagePredict.cc`  
- 文件：`include/Tools/AngleCalculate.hpp`  
- 变更：在 `GalaxyCamera` 增加 ROI 配置接口 `setRoiEnabled/setRoi`，并在 `applyCameraParams()` 中使用 `Width/Height/OffsetX/OffsetY` 节点应用相机侧裁剪；默认关闭。  
- 变更：ROI 应用时加入节点范围与步进对齐，并采用“先 `Offset=0`、再改 `Width/Height`、最后设置目标 `Offset`”顺序，降低节点写入失败概率。  
- 变更：主流程在启动时将 `RuntimeParams` 中 ROI 参数下发给相机，并打印 ROI 请求日志。  
- 变更：角度解算加入 ROI 主点补偿（`cx/cy` 减 `offset_x/offset_y`），避免开启 ROI 后产生系统性角度偏移。  
- 目的：在不改镜头的前提下，通过缩小取景视野提升目标在模型输入中的像素占比（等效数字变焦），用于“小目标”场景调优。  

0. ROI 控制范围收敛为“仅 stage3 生效”  
- 文件：`include/Tools/RuntimeParams.hpp`  
- 文件：`src/ImagePredict.cc`  
- 变更：ROI 参数重命名为 `stage3_*`，并改为“`stage12` 永远关闭 ROI、`stage3` 由开关控制（默认开启）”。  
- 变更：ROI 切换信号由预测线程发布，实际相机 ROI 节点写入统一在采集线程执行，避免跨线程直接操作相机对象。  
- 变更：`CaptureThread` 在 stage 切换时执行 `stop/start` 后应用对应 ROI 模式；切回 stage12 时回归全画幅。  
- 目的：满足“仅 stage3 裁剪”的业务诉求，并保持主流程线程边界清晰。  

0. ROI 居中基准与全画幅恢复修正  
- 文件：`include/CameraTask/GetImage.hpp`  
- 变更：新增 `ResetOffsetsToMin()` 与 `RestoreFullFrame()`，关闭 ROI 时不再只清运行时标记，而是实际将 `Width/Height` 恢复到最大值并把 `OffsetX/OffsetY` 恢复到最小值。  
- 变更：居中裁剪前先把 offset 归零，再读取 `Width/Height` 范围并按 full-frame 计算中心偏移，避免把上一次 ROI 窗口误当成新裁剪基准。  
- 目的：修复“stage3 看起来保持左上角不变”和“stage12 未恢复原始全画幅”的行为错误。  

0. ROI 切换后 payload buffer 刷新与半开状态收尾修正
- 文件：`include/CameraTask/GetImage.hpp`
- 变更：新增 `ds_handle_` 与 `payload_size_bytes_`，在 `open()` 和 `start()` 前通过 `GXGetPayLoadSize()` 刷新 `frame_data_.nImgSize` 与 `image_buffer_`，避免 ROI/全画幅切换后仍复用旧尺寸 buffer。
- 变更：新增 `full_frame_width_ / full_frame_height_ / full_frame_offset_x_min_ / full_frame_offset_y_min_`，在首次打开相机时缓存 full-frame 基准，用于后续居中裁剪与恢复全画幅。
- 变更：`close()` 不再依赖 `opened_` 才执行收尾；即便 `open()` 半途中失败，也会关闭设备句柄、关闭 SDK 库并清空 ROI 运行时状态，避免半开状态残留。
- 变更：恢复全画幅时增加日志 `[GalaxyCamera] ROI disabled: ...`，便于实机确认 stage12 是否真的回到全画幅。
- 诊断判断：用户实机日志中 ROI 切换后紧跟 `munmap_chunk(): invalid pointer`，高概率不是普通 ROI 计算错误，而是切换后 payload size 变化、旧图像 buffer 被 SDK 越界写坏，最终在 glibc 释放时崩溃。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过，待实机复现验证。

0. ROI 居中偏移写入顺序修正 + stage3 慢速扫描模式
- 文件：`include/CameraTask/GetImage.hpp`
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`include/Tools/ScanController.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：`applyRoiIfEnabled()` 改为“先归零 offset、先写目标 `Width/Height`、再重新读取 `OffsetX/OffsetY` 范围、最后按 full-frame 中心写偏移”。
- 诊断判断：之前虽然数学上计算了居中 offset，但 offset 节点范围是在 full-frame 状态下提前读取的；某些 Galaxy 相机此时 `OffsetX/OffsetY` 上限就是 `0`，于是居中值被再次 clamp 到 `0`，表现成“左上角不动”。
- 变更：扫描模式新增 stage3 独立参数：`stage3_scan_origin_hold_ms`、`stage3_scan_send_hz`、`stage3_scan_yaw_speed_deg_per_sec`。切到 stage3 时，扫描频率和 yaw 扫描角速度都会自动切慢；切回 stage1/2 时自动恢复原配置。
- 当前默认值：`stage1/2` 扫描发送 `200 Hz`、yaw 扫描速度 `16 deg/s`；`stage3` 扫描发送 `120 Hz`、yaw 扫描速度 `8 deg/s`。
- 当前状态：已于 2026-05-27 本地重新 `cmake --build build -j` 编译通过，待实机验证日志是否出现更接近中心的 `offset_x/offset_y`。

0. stage3 丢目标后按上一帧速度方向短暂续行，再退回扫描
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `stage3_lost_target_coast_ms` 与 `stage3_lost_target_velocity_scale`，仅在 `stage3` 丢失目标后启用。
- 变更：当 stage3 仍有目标时，缓存最近一次已发送控制命令的绝对角和 yaw/pitch 角速度；一旦下一段时间检测框消失，则先按这组速度方向续行一小段时间，再回退到原有扫描模式。
- 变更：`g_target_visible` 改为表示“当前帧是否真实有检测框”，避免 `HasRecentLock()` 仍为真时阻塞扫描/续行状态切换。
- 当前默认值：续行时长 `220 ms`，速度倍率 `1.0`。
- 目的：缓解 stage3 缩画幅后，对较快目标因短时失配而立刻丢失的问题，优先向目标离开方向补一小段位移，争取重新把目标拉回视野。

0. AimbotTarget 语义收敛（历史：计数/二值化发送一致）
- 文件：`include/SerialTask/Common.hpp`
- 文件：`include/SerialTask/SerialSend.hpp`
- 历史文件：`include/NetworkTask/AimbotTargetReceiver.hpp`（后续已删除）
- 文件：`src/ImagePredict.cc`
- 历史说明：本条记录的是旧计数契约；当前已被 2026-05-30 条目 13/22 覆盖。`SerialSend.hpp` 仍用 `ToWireAimbotTarget()` 规整线值，但网络接收增量、stage 切换扣减和内部计数 helper 均已删除。

0. Yaw/Pitch 速度滤波调参激进档
- 文件：`include/Tools/RuntimeParams.hpp`
- 变更：进入调参激进档，yaw 速度参数为 `480.0`、`3600.0`、`28.0`、`0.05`，pitch 速度参数为 `520.0`、`4200.0`、`32.0`、`0.05`，分别对应速度限幅、角加速度限幅、低通截止频率、死区。
- 目的：提高 yaw/pitch 速度响应，减少跟随滞后；若实机出现小抖，优先回落 cutoff 或 max_accel。

1. 增加全程录像开关
- 文件：`include/Tools/SaveVideo.hpp`
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`include/ImageRecognize/ImagePredictCommandLine.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `enable_save_full_run_video`，默认关闭；命令行可用 `--save-full-run-video`/`--no-save-full-run-video` 控制。
- 变更：启用后从程序运行到结束持续保存视频到 `full_run_videos/full_run_YYYYmmdd_HHMMSS_001.avi`，复用 `target_video_fps` 和 MJPG AVI 编码。
- 变更：为降低录像开销，目标录像与全程录像统一改为保存不带 `putText`/框绘制的 `inflight_frame` 原始帧，并在保存器内部缩放到 `480x480`。

2. Pitch 速度滤波调参记录
- 文件：`include/Tools/RuntimeParams.hpp`
- 变更：曾将 `angle_velocity_pitch_max_accel_deg_per_sec2` 从 `3000.0` 收到 `2600.0`、`angle_velocity_pitch_cutoff_hz` 从 `22.0` 收到 `18.0`、`angle_velocity_pitch_deadband_deg_per_sec` 从 `0.0` 加到 `0.25` 来压制微抖。
- 变更：因实机反馈 `pitch` 仍略滞后，当前推进到激进跟手档 `3200.0`、`24.0`、`0.05`，比原始速度滤波更跟手但仍保留极小微抖抑制。
- 目的：在抑制 `pitch` 小抖的前提下减少跟随滞后。

3. Stage3 推理端轻量光照归一化
- 文件：`include/ImageRecognize/YoloLightPreprocess.hpp`
- 文件：`src/ImagePredict.cc`
- 文件：`include/Tools/SaveImage.hpp`
- 变更：仅 stage3 在 OpenVINO 输入预处理内部执行 LAB 色彩空间 L 通道 CLAHE（clipLimit=2.0，tileGridSize=8x8），并且增强位置已从相机原图后移到 resize 后的模型输入内容区域，以降低预处理延迟。
- 变更：stage3 无目标/多目标样本保存输出 `stage3_raw_*.jpg`，对应进入 stage3 预测器前的原始帧；实际 CLAHE 在模型输入尺寸上执行。
- 注意：所有阶段异常样本保存条件保持“检测框数量不是 1 个”，即无目标或多目标。

4. CPU 大核识别修正（12900H 重点）  
- 文件：`include/Tools/CpuAffinity.hpp`  
- 变更：`detectBigCoresByType()` 从“`core_type >= 1`”改为“取观测到的最大 `core_type` 作为性能核组”。  
- 目的：避免把 E 核误归为大核，降低推理线程调度抖动。

5. 增加主线程辅助核绑定接口  
- 文件：`include/Tools/CpuAffinity.hpp`  
- 变更：新增 `BindCurrentThreadToAuxCores()`。

6. 主线程初始化后绑核策略调整  
- 文件：`src/ImagePredict.cc`  
- 变更：模型初始化完成后，主线程从全核改为辅助核集合。  
- 目的：减少与推理关键路径争抢性能核。

7. OpenVINO 推理线程策略改为延迟导向  
- 文件：`include/ImageRecognize/ImagePredict_OPENVINO.hpp`  
- 变更：参数字段改名 `hw_threads_reserved -> latency_threads_cap`；  
  `infer_threads` 由“总线程减保留”改为 `min(hw_threads, latency_threads_cap)`，并带最小保护。  
- 目的：抑制过高并发导致的尾延迟与抖动。

8. stage3 检测框稳定化升级为带门控的 One Euro 滤波
- 文件：`include/ImageRecognize/TemporalBoxStabilizer.hpp`
- 文件：`include/ImageRecognize/TargetTrackPipeline.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：保留原有 IoU / 中心偏移 / 面积变化门控，仅当可确认仍是同一目标时，才对 `stage3` 的框中心与框尺寸应用 One Euro Filter。
- 变更：`TargetTrackPipeline::Update()` 新增 `dt` 输入，复用主循环实时帧间隔；`stage1/2` 仍不启用该稳框路径。
- 变更：原先固定权重的自适应 EMA 稳框逻辑已替换为速度自适应滤波，目标低速时更稳，高速时自动提高截止频率以减小跟随滞后。
- 默认参数：`center_min_cutoff_hz=1.6`、`center_beta=0.28`、`size_min_cutoff_hz=1.2`、`size_beta=0.16`、`derivative_cutoff_hz=1.0`。
- 目的：针对 stage3 检测框抖动，在尽量不损失精度和动态响应的前提下压制高频抖动；风险低于直接改检测阈值、NMS 或主控制滤波。

9. stage3 丢失续行增加轻量滞回，抑制“短暂丢失 -> 续行 -> 回检 -> 再续行”抖动
- 文件：`include/Tools/RuntimeParams.hpp`
- 文件：`include/Tools/StageRuntimeProfile.hpp`
- 文件：`include/Tools/LostTargetRecoveryController.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：新增 `stage3_lost_target_coast_trigger_delay_ms` 与 `stage3_lost_target_reacquire_confirm_ms`，分别控制“连续丢失多久后才进入续行”和“重新识别稳定多久后才退出续行”。
- 变更：`LostTargetRecoveryController` 内部新增轻量滞回与单次丢失续行消费逻辑：同一次丢失只触发一次续行；续行中重新识别到目标时不会立刻抢回控制，而是经过一个很短的确认窗口。
- 变更：`ImagePredict.cc` 主循环在消费 `tracked_box` 前调用 `ShouldAcceptTrackedTarget()`；当 stage3 正在续行且回检尚未稳定时，继续沿用续行而不是立即切回检测控制。
- 默认参数：`trigger_delay_ms=40`、`reacquire_confirm_ms=60`。
- 目的：在不取消 stage3 续行机制的前提下，抑制因小画幅短时漏检导致的控制来回切换抖动，同时尽量不引入明显额外滞后。

10. 阶段切换延迟优化：stage3 模型后台预热
- 文件：`include/ImageRecognize/StagePredictorController.hpp`
- 诊断结论：首次切换到 `stage3` 的主要延迟来源，大概率不是阶段判定本身，而是 `stage3` 模型首次构造时的 OpenVINO 读模型与编译模型开销；这也解释了“第一次切换明显慢，后续切换快很多”的现象。
- 变更：在 `RequestStage3SwitchAfterTargetLoss()` 中，一旦进入“达到 stage=3、等待丢目标后切换”的待切换窗口，就异步启动 `stage3` 模型后台预热，而不是等真正切换那一刻再同步构造。
- 变更：后台预热线程绑定到辅助核，尽量减少与主推理线程争抢性能核；真正切换时通过 `EnsureStage3PredictorReady_()` 接管预热结果，若预热失败则自动回退到原来的同步加载路径。
- 变更：新增阶段切换日志，分别输出 `stage3` 预热启动、预热完成耗时、预热未完成需等待、同步回退加载耗时，以及真正“切换接管耗时”。
- 风险判断：低风险。该改动没有改变阶段切换判定时机、ROI/曝光/扫描副作用时机，也没有改变模型选择逻辑，只是把首次 `stage3` 模型构造前移到等待窗口中。
- 当前状态：已于 2026-05-28 本地重新 `cmake --build build -j` 编译通过。

11. 激光/距离补偿热路径快照优化 + ROI/阶段切换分段耗时打点
- 文件：`include/Tools/LaserAngleCalculate.hpp`
- 文件：`include/ImageRecognize/StagePredictorController.hpp`
- 文件：`src/ImagePredict.cc`
- 历史说明：当时曾优化 `DistanceCalculator` 与旧 `LaserAngleCalculator` 的热路径快照；截至 2026-05-29 条目 19，旧 `LaserAngleCalculator` 已下线，当前只保留 `DistanceCalculator` 距离估计与目标高度标定。
- 变更：`StagePredictorController::ApplyStageSideEffects_()` 新增分段耗时日志，拆出 `exposure / roi_flag / calibration / scan_config / total`，便于判断阶段切换剩余开销是否还卡在副作用应用链上。
- 变更：`CaptureThread` 中 `apply_stage3_roi_mode()` 新增分段耗时日志，拆出 `config_ms / stop_ms / start_ms / total_ms`，用于确认 ROI 切换延迟是否主要来自 `camera->stop()` / `camera->start()`。
- 变更：把 `RuntimeStats` 里原本只打印未采样的 `submit_stage3_preprocess_ns` 接上真实统计，当前记为 `stage3` 路径 `startAsync()` 时间，避免延迟日志出现误导性的长期 `0ms`。
- 风险判断：低风险。以上改动均不改变主流程行为，只补诊断信息并减少热路径的轻微读配置开销。

## 当前风险与注意事项

1. 设备限制  
- 当前环境无法直接连接实机相机复现；但截至 2026-05-27，修复后的 `ImagePredict` 已完成本地编译验证。

2. 工作区状态  
- 仓库存在历史改动，交接时不要回退与本轮无关的变更。

3. 已知文本编码问题  
- 个别旧文件曾出现中文注释乱码；后续若继续维护，建议统一 UTF-8 编码并复查注释行是否影响代码可读性。

## 下一步建议（上线前）

1. 实机验证指标  
- p50/p95/p99 推理时延  
- 串口发送周期抖动  
- 丢帧率与目标锁定稳定性
- stage3 日志是否接近居中偏移：
  预期 `1920x1200 -> 1280x720` 时，`offset_x` 应接近 `320`，`offset_y` 应接近 `240`（受相机节点步进约束可略有偏差）
- stage12 切回时是否打印全画幅恢复日志：
  预期出现 `[GalaxyCamera] ROI disabled: width=<max> height=<max> offset_x=<min> offset_y=<min>`
- 原始复现命令 `./bin/ImagePredict --no-save-no-target-images` 下，切换 stage3/stage12 后不再出现 `munmap_chunk(): invalid pointer`

2. 若延迟仍波动较大  
- 在不影响精确度前提下，继续下调 `latency_threads_cap` 做 A/B 对比。  
- 复核 `IMUSendThread` 是否需要更高优先级或更强隔离策略。

3. 若实机切阶段后仍然崩溃  
- 第一优先方向：将 `CaptureThread` 中的 ROI 切换从 `camera->stop(); camera->start();` 升级为完整的 `camera->close(); camera->open(); camera->start();`，牺牲一点切换时延换 SDK 兼容性。
- 第二优先方向：在 `RefreshPayloadBuffer()` 中临时加带标签的 payload 日志，记录 ROI 开关前后的旧/新 `payload_size_bytes_`，验证是否仍有尺寸漂移未被刷新。
