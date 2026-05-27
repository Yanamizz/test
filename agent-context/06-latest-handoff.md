# Latest Handoff

用途：记录最近一次交接结论与后续建议。  
更新时间：2026-05-27  
适用场景：新 Agent 快速接续、避免重复排查。

## 本轮完成事项（延迟优先）

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
- 变更：顺手修正 `SerialSend.hpp` 中 `AimbotTarget` 实际发送值，改为真正使用 `ToWireAimbotTarget(AimbotTarget)`，同时消除未使用变量警告；这与既有二值化协议语义一致，不改变设计意图。
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

0. AimbotTarget 语义收敛（计数/二值化发送一致）
- 文件：`include/SerialTask/Common.hpp`
- 文件：`include/SerialTask/SerialSend.hpp`
- 文件：`include/NetworkTask/AimbotTargetReceiver.hpp`
- 文件：`src/ImagePredict.cc`
- 变更：在 `SerialTask/Common.hpp` 新增 `ToWireAimbotTarget`、`SaturatingIncrementAimbotTarget`、`SaturatingDecrementAimbotTarget`，统一 AimbotTarget 计数与线协议二值化规则。
- 变更：网络接收线程增量逻辑改为复用 `SaturatingIncrementAimbotTarget`；阶段切换扣减逻辑改为复用 `SaturatingDecrementAimbotTarget`，移除分散重复实现。
- 变更：`SerialSend.hpp` 中 `AimbotFrame_SCM_t::AimbotTarget` 从固定 `0x01` 修正为使用 `ToWireAimbotTarget` 的二值化结果，避免“接口传参与实际发送语义不一致”。
- 目的：与 `03-business-contracts.md` 第 4 条契约对齐，减少联调歧义，提升语义可维护性和测试可覆盖性。

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
- 变更：`DistanceCalculator` 与 `LaserAngleCalculator` 的热路径改为每次计算只快照一次当前 runtime stage 参数，避免每帧多次读取阶段配置带来的额外锁/原子访问。
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
