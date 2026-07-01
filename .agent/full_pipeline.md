# 从接收到发送的完整主链路

本文档只描述当前仓库真实存在的 Linux 主程序链路，不展开旧 STM32 主程序，也不讨论视觉识别支线。

## 1. 全流程总览

当前主链路可以概括为：

```text
真实串口 or 串口回放
-> serial_referee
-> 维护常规链路状态
-> RadarDecisionTree 读取 0x020E
-> RadarCommandSender 生成 0x0301(0x0121)
-> RefereeTxScheduler
-> 串口发送

真实 TCP 8001 or 文件回放
-> info_wave_referee
-> 维护 0x0A01~0x0A06 状态
-> MapRobotRelay 读取 0x0A01
-> 结合串口侧 0x020B
-> 生成 0x0305
-> RefereeTxScheduler
-> 串口发送

真实 TCP 8002/8003
-> EnemyKeyReceiver
-> 解析完整 0x0A06
-> RadarCommandSender 等待队列
-> RefereeTxScheduler
-> 串口发送
```

同时，整条链路持续输出：

- 结构体日志
- 原始二进制日志
- 串口/TCP 状态日志
- 运行时指标

## 2. 启动阶段

主入口：

- [src/main.cc](/home/hanni/Radar/src/main.cc:45)

启动阶段当前做的事情：

1. [`CreateRunLogRoot()`](/home/hanni/Radar/include/log/referee_main_log.hpp:79) 创建本轮日志目录。
2. 初始化 [include/referee/tcp_connection_log.hpp](/home/hanni/Radar/include/referee/tcp_connection_log.hpp:23) 中的 `TcpConnectionLog`。
3. 尝试打开裁判系统串口。
4. 如果 `8001` 配置为真实输入，则尝试拉起信息波 TCP 客户端。
5. 尝试拉起 `8002/8003` 两个敌方密钥 TCP 客户端。
6. 创建两个主协议解包器：
   - `serial_referee`
   - `info_wave_referee`
7. 创建发送和业务模块：
   - `RefereeTxScheduler`
   - `RadarCommandSender`
   - `RadarDecisionTree`
   - `MapRobotRelay`
   - `EnemyKeyReceiver(enemy_level1_key)`
   - `EnemyKeyReceiver(enemy_level2_key)`
8. 如果配置为文件回放，则创建 [include/referee/replay_input_source.hpp](/home/hanni/Radar/include/referee/replay_input_source.hpp:43) 中的 `ReplayInputSource`。
9. 记录 `main/input_mode.log`。
10. 挂接串口/信息波回调并进入统一主循环。

主程序本身只负责接线，不在 [src/main.cc](/home/hanni/Radar/src/main.cc:45) 中堆业务逻辑。

当前主入口的关键代码如下：

```cpp
int main() {
  const auto run_log_root = radar::log::CreateRunLogRoot();
  radar::referee::TcpConnectionLog tcp_log(run_log_root);

  radar::referee::SerialPort serial;
  radar::referee::TcpClient info_wave_tcp;
  radar::referee::TcpClient enemy_level1_key_tcp;
  radar::referee::TcpClient enemy_level2_key_tcp;

  Referee serial_referee;
  Referee info_wave_referee;
  radar::log::BinaryLogStore raw_log_store(run_log_root);
  radar::referee::RefereeTxScheduler tx_scheduler(serial, run_log_root);
  radar::referee::RadarCommandSender radar_command_sender(tx_scheduler, run_log_root);
  radar::referee::RadarDecisionTree<kRevision> radar_decision_tree(radar_command_sender);
  radar::referee::MapRobotRelay<kRevision> relay(tx_scheduler, run_log_root);
  radar::referee::EnemyKeyReceiver<kRevision> enemy_level1_key_receiver(
      "enemy_level1_key", radar::config::kEnemyLevel1KeyTcpServerPort, radar_command_sender, run_log_root);
  radar::referee::EnemyKeyReceiver<kRevision> enemy_level2_key_receiver(
      "enemy_level2_key", radar::config::kEnemyLevel2KeyTcpServerPort, radar_command_sender, run_log_root);

  serial_referee.AttachCallback([&](rm::u16 cmd_id, rm::u8 seq) {
    relay.ProcessSerial(cmd_id, seq, serial_referee);
    radar_decision_tree.ProcessSerial(cmd_id, seq, serial_referee);
  });
  info_wave_referee.AttachCallback([&](rm::u16 cmd_id, rm::u8 seq) {
    relay.ProcessInfoWaveTcp(cmd_id, seq, info_wave_referee, serial_referee.data());
  });

  radar::referee::RunSerialInfoWaveAndKeyTcpLoop(
      serial, info_wave_tcp, enemy_level1_key_tcp, enemy_level2_key_tcp, serial_referee, info_wave_referee,
      serial_replay_source ? &*serial_replay_source : nullptr,
      info_wave_replay_source ? &*info_wave_replay_source : nullptr, enemy_level1_key_receiver,
      enemy_level2_key_receiver, radar_command_sender, relay, tx_scheduler, raw_log_store, g_running);
}
```

对应当前实现位置：

- [`main()` in src/main.cc](/home/hanni/Radar/src/main.cc:45)

## 3. 配置如何影响链路

唯一手动配置入口：

- [include/config/config.hpp](/home/hanni/Radar/include/config/config.hpp:32)

当前最影响运行行为的配置包括：

- `kRefereeRevision`
  - 当前使用的裁判协议版本
- `kRadarLogMode`
  - `kDebug` / `kMatch`
- `kDebugAllowMissingInterfaces`
  - 调试模式下缺失接口按 0 值继续运行
- `kSerialRefereeInputMode`
  - 串口真实输入或文件回放
- `kInfoWaveInputMode`
  - `8001` 真实输入或文件回放
- `kTcpServerAddress`
  - 三条 TCP 客户端连接的对端 IP
- `kTcpLocalBindAddress`
  - 本机绑定地址，留空则自动选路由出口
- `kInfoWaveTcpIdleTimeoutMs`
  - `8001` 空闲多久主动断开
- `kEnemyKeyTcpIdleTimeoutMs`
  - `8002/8003` 空闲多久主动断开
- `kMapRobotMinSendIntervalMs`
  - `0x0305` 限频
- `kRadarPasswordVerifyCooldownMs`
  - `password_cmd=2` 冷却

## 4. 字节流是如何进入解包器的

### 4.1 真实串口

文件：

- [include/referee/serial_port.hpp](/home/hanni/Radar/include/referee/serial_port.hpp:75)
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

行为：

- 主循环使用 `poll` 监听串口 fd
- 读取到一段字节后写入 `raw/serial_referee_rx.bin`
- 然后调用 [`FeedBytes(serial_referee, bytes, size)`](/home/hanni/Radar/include/referee/referee_input_loop.hpp:42)
- `FeedBytes` 再逐字节执行 `referee << byte`

当前实现非常直接：

```cpp
template <typename Referee>
void FeedBytes(Referee &referee, const rm::u8 *data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    referee << data[i];
  }
}
```

### 4.2 真实 TCP `8001`

文件：

- [include/referee/tcp_client.hpp](/home/hanni/Radar/include/referee/tcp_client.hpp:46)
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

行为：

- 主循环监听信息波 TCP 客户端 fd
- 若处于 `connecting` 状态，则在 `POLLOUT` 时调用 `FinishConnect()`
- 若处于 `connected` 状态，则在 `POLLIN` 时读取字节流
- 读取到的字节写入 `raw/tcp_8001_info_wave_rx.bin`
- 再逐字节喂给 `info_wave_referee`

### 4.3 真实 TCP `8002/8003`

文件：

- [include/referee/enemy_key_receiver.hpp](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:72)
- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

行为：

- 同样由 `TcpClient` 拉起非阻塞连接
- 收到字节流后分别写入：
  - `raw/tcp_8002_enemy_level1_key_rx.bin`
  - `raw/tcp_8003_enemy_level2_key_rx.bin`
- 然后交给各自的 [`EnemyKeyReceiver::ProcessBytes(...)`](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:104)
- 每个接收器内部都有独立的 `Referee` 解析完整 `0x0A06`

关键代码如下：

```cpp
template <typename SerialProtocol>
bool ProcessBytes(const rm::u8 *data, std::size_t size, const SerialProtocol &) {
  if (completed_) {
    return true;
  }

  for (std::size_t i = 0; i < size; ++i) {
    referee_ << data[i];
  }
  return completed_;
}

void OnFrame(rm::u16 cmd_id, rm::u8 seq) {
  ++valid_frame_count_;
  radar::log::GetRuntimeMetrics(log_store_.root()).RecordLossRate(name_, referee_.loss_rate());

  if (cmd_id != Cmd::kRadar5) {
    LogUnexpectedCmd(cmd_id, seq);
    return;
  }

  for (std::size_t i = 0; i < kKeyBytes; ++i) {
    latest_key_[i] = referee_.data().radar5.key[i];
  }

  if (!IsRadarKey(latest_key_)) {
    rejected_ = true;
    command_sender_.LogRejectedKey(name_, source_port_, latest_key_.data(), latest_key_.size(),
                                   "0x0a06_key_must_be_6_ascii_letters_or_digits");
    LogRejectedFrame(seq);
    return;
  }

  has_key_ = true;
  completed_ = true;
  command_sender_.QueueOpponentKey(name_, source_port_, latest_key_);
  LogAcceptedFrame(seq);
}
```

对应实现位置：

- [`EnemyKeyReceiver::ProcessBytes(...)`](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:103)
- [`EnemyKeyReceiver::OnFrame(...)`](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:128)

### 4.4 文件回放

文件：

- [include/referee/replay_input_source.hpp](/home/hanni/Radar/include/referee/replay_input_source.hpp:43)

行为：

- 回放文件会先被切成完整裁判协议帧
- 主循环按配置频率释放完整帧
- 释放出的字节写入对应 `raw/file_*.bin`
- 然后逐字节喂给对应 `Referee`

当前只支持：

- 串口回放
- 信息波 `8001` 回放

## 5. 解包器之后发生什么

当前项目里的所有主协议解析都依赖：

- `rm::device::Referee<revision>`

也就是说：

- 主循环只负责“把字节流喂进去”
- 真正的 SOF、长度、`seq`、CRC8、CRC16、`cmd_id` 解析由 `librm` 完成
- 完整帧落地到 `RefereeProtocol<revision>` 后，再触发项目侧回调

当前主程序里挂了两组主要回调：

### 5.1 串口回调

在 [src/main.cc](/home/hanni/Radar/src/main.cc:123) 里，`serial_referee.AttachCallback(...)` 当前做两件事：

1. `relay.ProcessSerial(cmd_id, seq, serial_referee)`
2. `radar_decision_tree.ProcessSerial(cmd_id, seq, serial_referee)`

也就是说串口帧一旦落地：

- 一方面进入结构体日志与 `0x020B` 状态维护
- 另一方面若是 `0x020E`，会进入自主决策树

### 5.2 信息波回调

在 [src/main.cc](/home/hanni/Radar/src/main.cc:128) 里，`info_wave_referee.AttachCallback(...)` 当前做的事：

1. `relay.ProcessInfoWaveTcp(cmd_id, seq, info_wave_referee, serial_referee.data())`

也就是说信息波帧一旦落地：

- 会被记入主协议结构体日志
- 若是 `0x0A01`，更新敌方位置与到达时间
- `MapRobotRelay` 在周期任务中再决定是否产出最新 `0x0305`

## 6. `0x0305` 是如何生成的

文件：

- [include/referee/map_robot_relay.hpp](/home/hanni/Radar/include/referee/map_robot_relay.hpp:254)

当前逻辑分成两部分：

### 6.1 状态维护

- [`ProcessSerial(...)`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:290)
  - 记录串口结构体日志
  - 如果 `cmd_id == 0x020B`，更新己方地面机器人位置与时间戳
- [`ProcessInfoWaveTcp(...)`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:309)
  - 记录信息波结构体日志
  - 如果 `cmd_id == 0x0A01`，更新敌方位置、到达间隔与时间戳

当前实现片段如下：

```cpp
void ProcessSerial(rm::u16 cmd_id, rm::u8 seq, const Referee &serial_referee) {
  main_logger_.LogFrame(cmd_id, seq, serial_referee.data(), serial_referee.loss_rate());
  radar::log::GetRuntimeMetrics(main_logger_.root()).RecordLossRate("serial", serial_referee.loss_rate());

  if (cmd_id == Cmd::kGroundRobotPosition) {
    has_ground_robot_position_ = true;
    ++ground_seen_count_;
    latest_ally_protocol_ = serial_referee.data();
    last_ground_robot_position_time_ = Clock::now();
  }
}

void ProcessInfoWaveTcp(rm::u16 cmd_id, rm::u8 seq, const Referee &info_wave_referee,
                        const Protocol &serial_protocol) {
  main_logger_.LogFrame(cmd_id, seq, info_wave_referee.data(), info_wave_referee.loss_rate());
  radar::log::GetRuntimeMetrics(main_logger_.root()).RecordLossRate("info_wave_tcp", info_wave_referee.loss_rate());

  if (cmd_id == Cmd::kRadar0) {
    if (last_radar0_time_.has_value()) {
      last_radar0_interval_ms_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *last_radar0_time_).count();
    }
    latest_opponent_protocol_ = info_wave_referee.data();
    latest_ally_protocol_ = serial_protocol;
    last_radar0_time_ = Clock::now();
  }
}
```

### 6.2 周期发送

[`ProcessPeriodic()`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:329) 每轮都会检查：

- `0x0A01` 是否还在有效期内
- `0x020B` 是否还在有效期内
- 当前是否已经到达 `0x0305` 最小发送间隔

然后：

- 对 stale 的敌方/己方状态自动置 0
- 组装 `MapRobotPosition`
- 通过 `RefereePrepare(...)` 打包主协议 `0x0305`
- 交给 [`RefereeTxScheduler::UpdateLatestMapRobotFrame(...)`](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:78)

当前核心代码如下：

```cpp
void ProcessPeriodic() {
  const auto now = Clock::now();
  const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
  const bool ground_fresh = IsFreshAt(last_ground_robot_position_time_, now, RefereeTimeout());
  if (!opponent_fresh && !AllowMissingInterfaces()) {
    tx_scheduler_.ClearLatestMapRobotFrame();
    if (last_radar0_time_.has_value()) {
      LogMapRobotSkip(now, opponent_fresh, ground_fresh, "opponent_stale");
    } else {
      LogMapRobotSkip(now, opponent_fresh, ground_fresh, "opponent_missing");
    }
    return;
  }
  if (!tx_scheduler_.IsMapRobotSendDue(now)) {
    LogMapRobotSkip(now, opponent_fresh, ground_fresh, "rate_limited");
  }

  QueueMapRobotPosition(latest_opponent_protocol_, latest_ally_protocol_, now);
}

void QueueMapRobotPosition(const Protocol &opponent_protocol, const Protocol &ally_protocol, TimePoint now) {
  Protocol opponent_zero_filled_protocol = opponent_protocol;
  Protocol ground_protocol = ally_protocol;
  const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
  const bool ground_fresh = IsFreshAt(last_ground_robot_position_time_, now, RefereeTimeout());
  if (!opponent_fresh) {
    opponent_zero_filled_protocol.radar0 = {};
  }
  if (!ground_fresh) {
    ground_protocol.ground_robot_position = {};
  }

  const auto map = BuildMapRobotPosition<revision>(opponent_zero_filled_protocol, ground_protocol);
  std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> tx_buffer{};
  const rm::u8 frame_len = rm::device::RefereePrepare(tx_buffer.data(), 0, map);
  std::vector<rm::u8> frame(tx_buffer.begin(), tx_buffer.begin() + frame_len);
  tx_scheduler_.UpdateLatestMapRobotFrame(frame, [this, map, frame, opponent_fresh, ground_fresh, enqueue_time =
                                                                       Clock::now()]() {
    LogMapRobotData(map, frame.data(), frame.size(), Clock::now(), opponent_fresh, ground_fresh,
                    std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - enqueue_time).count(), "");
  });
}
```

对应实现位置：

- [`MapRobotRelay::ProcessPeriodic()`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:329)
- [`MapRobotRelay::QueueMapRobotPosition(...)`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:398)

当前 `0x0305` 不是“收到一次 `0x0A01` 就立即直发”，而是：

- 输入 10Hz 只负责更新状态
- 输出 5Hz 发送最新状态

## 7. `0x0121` 是如何生成的

文件：

- [include/referee/radar_decision_tree.hpp](/home/hanni/Radar/include/referee/radar_decision_tree.hpp:45)
- [include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:85)

### 7.1 来自 `0x020E` 的自主决策

`RadarDecisionTree` 只关心串口主协议中的 `0x020E radar_info`：

- 提取双倍易伤次数
- 提取对方是否正在双倍易伤
- 提取己方当前加密等级
- 提取当前是否允许修改己方密钥

然后决定：

- 是否触发一次新的双倍易伤请求
- 是否切换到下一组预置己方密钥

### 7.2 来自 `8002/8003` 的敌方密钥提交

[`EnemyKeyReceiver`](/home/hanni/Radar/include/referee/enemy_key_receiver.hpp:72) 成功解析到合法 `0x0A06` 后：

- 不直接写串口
- 只调用 [`RadarCommandSender::QueueOpponentKey(...)`](/home/hanni/Radar/include/referee/radar_command_sender.hpp:164)

之后由 [`RadarCommandSender::ProcessPending(...)`](/home/hanni/Radar/include/referee/radar_command_sender.hpp:179) 周期推进：

- 若还在 10 秒冷却期，则等待
- 若允许发送，则组装 `password_cmd=2`
- 通过 `Referee0x301Prepare(...)` 打包 `0x0301(0x0121)`
- 交给统一发送调度器

当前这段逻辑的关键实现如下：

```cpp
void QueueOpponentKey(std::string name, int source_port, const std::array<rm::u8, 6> &key) {
  PendingOpponentKey pending;
  pending.name = std::move(name);
  pending.source_port = source_port;
  pending.key = key;
  pending_opponent_keys_.push_back(pending);
  LogQueuedKey(pending);
}

template <typename Protocol>
void ProcessPending(const Protocol &serial_protocol) {
  if (password_verify_in_flight_ || pending_opponent_keys_.empty()) {
    return;
  }

  const auto now = Clock::now();
  if (password_verify_next_allowed_time_.has_value() && now < *password_verify_next_allowed_time_) {
    LogWaitingCooldown(pending_opponent_keys_.front(), now);
    return;
  }

  const auto pending = pending_opponent_keys_.front();
  auto cmd = MakeCommand();
  cmd.password_cmd = kRadarVerifyOpponentKeyCommand;
  cmd.password_1 = pending.key[0];
  cmd.password_2 = pending.key[1];
  cmd.password_3 = pending.key[2];
  cmd.password_4 = pending.key[3];
  cmd.password_5 = pending.key[4];
  cmd.password_6 = pending.key[5];

  RadarCommandContext context;
  context.name = pending.name;
  context.source = "tcp";
  context.source_port = pending.source_port;
  context.decision = "sent";
  const bool queued = Send(cmd, serial_protocol, context, [this]() {
    password_verify_in_flight_ = false;
    password_verify_next_allowed_time_ = Clock::now() + PasswordVerifyCooldown();
  });
  if (!queued) {
    return;
  }
  password_verify_in_flight_ = true;
  pending_opponent_keys_.pop_front();
}
```

对应实现位置：

- [`RadarCommandSender::QueueOpponentKey(...)`](/home/hanni/Radar/include/referee/radar_command_sender.hpp:164)
- [`RadarCommandSender::ProcessPending(...)`](/home/hanni/Radar/include/referee/radar_command_sender.hpp:179)

## 8. 统一发送调度如何工作

文件：

- [include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33)

当前策略：

- `0x0301`
  - FIFO 队列
  - 高优先级
  - 统一限频
- `0x0305`
  - latest-only
  - 队列里只保留当前最新一帧

[`Process()`](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:105) 的行为是：

1. 如果串口未打开，本轮不发，只保留队列状态。
2. 若 `0x0301` 到了发送窗口，先发 `0x0301`。
3. 然后若 `0x0305` 到了发送窗口，再发 `0x0305`。
4. 真正写串口失败时，记录日志并关闭串口，等待主循环自动重连。

这保证了：

- 关键决策包不会被 `0x0305` 抢占
- 地图位置始终以最新状态发送

当前调度器的关键代码如下：

```cpp
void UpdateLatestMapRobotFrame(std::vector<rm::u8> frame, SentCallback on_sent) {
  latest_map_robot_frame_.emplace();
  latest_map_robot_frame_->frame = std::move(frame);
  latest_map_robot_frame_->on_sent = std::move(on_sent);
}

void Process() {
  const auto now = Clock::now();
  if (!serial_.is_open()) {
    return;
  }

  if (!radar_command_queue_.empty() && IsRadarCommandSendDue(now)) {
    auto &task = radar_command_queue_.front();
    std::string error;
    if (!serial_.TryWriteAll(task.frame.data(), task.frame.size(), &error)) {
      HandleSerialWriteFailure(error);
      return;
    }
    auto sent_task = std::move(task);
    radar_command_queue_.pop_front();
    last_radar_command_send_time_ = now;
    if (sent_task.on_sent) {
      sent_task.on_sent();
    }
  }

  if (!serial_.is_open()) {
    return;
  }
  if (latest_map_robot_frame_.has_value() && IsMapRobotSendDue(now)) {
    std::string error;
    if (!serial_.TryWriteAll(latest_map_robot_frame_->frame.data(), latest_map_robot_frame_->frame.size(), &error)) {
      HandleSerialWriteFailure(error);
      return;
    }
    last_map_robot_send_time_ = now;
    if (latest_map_robot_frame_->on_sent) {
      latest_map_robot_frame_->on_sent();
    }
  }
}
```

对应实现位置：

- [`RefereeTxScheduler::UpdateLatestMapRobotFrame(...)`](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:78)
- [`RefereeTxScheduler::Process()`](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:105)

## 9. 重连、超时与周期任务

文件：

- [include/referee/referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:58)

主循环每轮都会做这些周期工作：

- `try_reconnect_serial()`
  - 串口断开后按配置周期重试
- `try_reconnect_tcp_client(...)`
  - `8001/8002/8003` 断开后按配置周期重试
- `CloseIfConnectTimedOut(...)`
  - 非阻塞 connect 太久未完成则关闭重试
- `CloseIdleClientIfTimedOut(...)`
  - TCP 空闲超时则关闭
- `map_robot_relay.ProcessPeriodic()`
  - 推进 `0x0305`
- `radar_command_sender.ProcessPending(...)`
  - 推进 `password_cmd=2`
- `tx_scheduler.Process()`
  - 推进串口发送
- [`runtime_metrics.MaybeFlush()`](/home/hanni/Radar/include/log/log_backend.hpp:309)
  - 刷新运行时指标

主循环里与重连和周期任务最相关的实现片段如下：

```cpp
const auto try_reconnect_tcp_client = [&](TcpClient &client, const std::string &name, int port, bool enable,
                                          bool allow_debug_disable,
                                          std::optional<LoopClock::time_point> *last_attempt_time,
                                          bool *retry_state_logged) {
  if (!enable || client.is_open()) {
    return;
  }

  const auto now = LoopClock::now();
  if (!*retry_state_logged) {
    tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "reconnecting",
                            std::string("remote=") + radar::config::kTcpServerAddress);
    *retry_state_logged = true;
  }
  if (last_attempt_time->has_value() &&
      now - **last_attempt_time < std::chrono::milliseconds(radar::config::kTcpReconnectIntervalMs)) {
    return;
  }

  *last_attempt_time = now;
  std::string error;
  if (client.TryOpen(radar::config::kTcpServerAddress, port, radar::config::kTcpLocalBindAddress, &error)) {
    *retry_state_logged = false;
    return;
  }

  tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "connect_failed", error);
};

const auto service_periodic_tasks = [&](const LoopClock::time_point &loop_start) {
  if (info_wave_tcp.CloseIdleClientIfTimedOut(radar::config::kInfoWaveTcpIdleTimeoutMs)) {
    metrics.RecordTcpIdleDisconnect(info_wave_tcp.port());
  }
  if (!enemy_level1_key_receiver.completed() &&
      enemy_level1_key_tcp.CloseIdleClientIfTimedOut(radar::config::kEnemyKeyTcpIdleTimeoutMs)) {
    metrics.RecordTcpIdleDisconnect(enemy_level1_key_tcp.port());
  }
  if (!enemy_level2_key_receiver.completed() &&
      enemy_level2_key_tcp.CloseIdleClientIfTimedOut(radar::config::kEnemyKeyTcpIdleTimeoutMs)) {
    metrics.RecordTcpIdleDisconnect(enemy_level2_key_tcp.port());
  }
  map_robot_relay.ProcessPeriodic();
  radar_command_sender.ProcessPending(serial_referee.data());
  tx_scheduler.Process();
  const auto loop_end = LoopClock::now();
  metrics.RecordLoopIteration(std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start),
                              std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start));
  metrics.MaybeFlush();
};
```

对应实现位置：

- [`try_reconnect_tcp_client` in referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:176)
- [`service_periodic_tasks` in referee_input_loop.hpp](/home/hanni/Radar/include/referee/referee_input_loop.hpp:220)

## 10. 日志链路

当前日志根目录为每次运行独立创建的时间戳目录，常见输出包括：

- `main/0xNNNN_*.log`
  - 主协议结构体日志
- `main/0x0305_map_robot_data.log`
  - 成功发出的地图数据
- `main/0x0305_map_robot_data_skipped.log`
  - 跳过原因
- `main/0x0121_*.log`
  - 决策/密钥相关发送日志
- `main/serial_state.log`
  - 串口状态变化
- `main/tcp_startup.log`
  - 启动阶段 TCP 拉起结果
- `main/tcp_channel_state.log`
  - 通道状态变化
- `main/tcp_client_state.log`
  - 客户端连接状态变化
- `main/runtime_metrics.log`
  - 主循环耗时、日志压力、loss_rate 汇总
- `raw/*.bin`
  - 原始串口与 TCP 字节流

当前代码已经删除 `latest/` 快照目录，所有排查都应直接看当次运行目录。

## 11. 现在最容易看错的点

### 11.1 TCP 角色

当前本端是 TCP client-only，不再监听端口。

### 11.2 `loss_rate`

`loss_rate()` 是 `librm` 提供的通用指标，不保证等于真实网络丢包率。  
对信息波 TCP 流尤其如此。

### 11.3 `0x020B`

当前 `0x0305` 的己方地面机器人位置仍主要来自串口 `0x020B`。  
如果实机链路里雷达收不到 `0x020B`，己方坐标会长期为 0。

### 11.4 调试模式

`kDebug` 模式下允许接口缺失并按 0 值继续跑主链路；这属于调试便利，不代表比赛模式的真实数据完整性。
