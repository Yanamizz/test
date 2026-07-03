# 雷达通信稳定性审查

本文只审查当前主程序的通信稳定性，不讨论视觉识别、UI 或反制扩展功能。

依据：

- 当前主链路实现：[src/main.cc](/home/hanni/Radar/src/main.cc:45)
- 当前项目侧通信封装：`include/referee/*`
- 当前日志与运行指标：`include/log/*`
- 当前协议解析基础：`include/radar/libs/librm/src/librm/device/referee/*`

## 1. 规则侧最关键的约束

当前最影响稳定性的协议约束主要有：

- `0x0305` 发送上限 `5Hz`
- `0x0A01~0x0A06` 信息波通常为 `10Hz`
- `0x0301` 属于总带宽受限的机器人交互链路
- `0x0121` 的 `radar_cmd` 必须单调递增
- `password_cmd=2` 每次提交后 10 秒内再次提交无效
- `0x0305` 中坐标 `x=y=0` 代表该机器人未发送
- `0x0A06` 的 6 字节密钥必须是 ASCII 字母或数字

因此真正影响比赛稳定性的重点不是“能不能收包”，而是：

- 输入和输出频率是否匹配
- 状态是否有新鲜度管理
- 决策发送是否受统一调度
- 串口/TCP 短暂异常后是否能自动恢复

## 2. 当前已经落地的稳定性保护

### 2.1 `0x0305` 已从“输入触发直发”改成“5Hz latest-only”

当前实现：

- `0x0A01` 到达时只更新敌方状态
- [`MapRobotRelay::ProcessPeriodic()`](/home/hanni/Radar/include/referee/map_robot_relay.hpp:329) 周期检查是否到达发送窗口
- [include/referee/referee_tx_scheduler.hpp](/home/hanni/Radar/include/referee/referee_tx_scheduler.hpp:33) 只保留最新一帧 `0x0305`

这避免了：

- `10Hz` 信息波直接把 `0x0305` 顶到超频
- 串口发送队列被过期地图帧堆满

### 2.2 状态过期已加入主链路

当前实现：

- `0x0A01` 有独立时间戳和超时窗口
- `0x0301/0x0200` 有独立时间戳和超时窗口
- stale 后自动置 0 再组包 `0x0305`

这避免了：

- TCP 断流后继续发旧敌方位置
- 串口侧掉状态后继续发旧己方位置

### 2.3 `password_cmd=2` 已进入 FIFO + 10 秒冷却

当前实现：

- `8002/8003` 只负责把合法密钥送入队列
- [include/referee/radar_command_sender.hpp](/home/hanni/Radar/include/referee/radar_command_sender.hpp:179) 周期检查是否到达可发送时间
- 只有真正发出后才推进下一条

这避免了：

- 短时间内连续上报敌方密钥导致后面的包无效
- 多路来源直接抢占串口

### 2.4 已有统一发送调度器

当前实现：

- `0x0301` 走 FIFO
- `0x0305` 走 latest-only
- `0x0301` 优先于 `0x0305`

这避免了：

- 各业务模块各自直写串口
- 高频地图数据把关键决策包挤掉

### 2.5 串口已支持自动重连

当前实现：

- 串口打开失败不会直接让程序退出
- 串口读写失败会进入断开状态
- 主循环按固定周期后台重试打开

这使得：

- USB 瞬断后不必手动重启整个进程
- TCP 接收与周期任务在串口掉线期间仍可继续推进

### 2.6 主接收 TCP 仍是 client-only，另外新增可选 server 通道

当前实现：

- 本端对 `8001/8002/8003` 仍不监听，只主动连接配置里的对端服务端
- 同时新增一条可选外部设备 server 通道，负责监听、accept 与保活
- 客户端链路仍支持连接超时、断开后重试、空闲超时关闭

这使得：

- 主接收链路的端口冲突和误绑定风险仍然较低
- 若后续要通过 server 通道向外发数据，底层会话已经准备好，不必再重写 socket 层

### 2.7 `8001` 可以保持长连接

当前配置支持：

- 将 `kInfoWaveTcpIdleTimeoutMs` 设为 `<=0`

这样 `8001` 在无数据停顿期间也不会被本端主动断开，有利于对端发送节奏不稳定时维持长连接。

### 2.8 日志后端已异步化并带运行指标

当前实现：

- 文本日志与原始二进制日志走异步后端
- Best-effort 日志允许在高压下丢弃
- `runtime_metrics.log` 周期记录主循环耗时、日志队列压力、loss_rate 快照

这比同步逐条刷盘更适合比赛场景。

## 3. 当前仍需特别注意的风险

### 3.1 `loss_rate()` 不一定等于真实 TCP 丢包率

当前 `loss_rate()` 来自 [include/radar/libs/librm/src/librm/device/referee/referee.hpp](/home/hanni/Radar/include/radar/libs/librm/src/librm/device/referee/referee.hpp:1) 的通用实现，默认假设：

- `seq` 按 256 周期滚动
- 每轮应看到 256 个不同序号

但信息波 TCP 流可能不满足这个假设。结果是：

- `loss_rate` 可能明显失真
- 甚至可能出现负值

因此对信息波链路来说，`loss_rate()` 更像“按 seq 规则计算出的帧填充度指标”，不能直接当作真实网络丢包率。

### 3.2 `0x0301/0x0200` 是否稳定送到雷达仍要实机确认

当前 `0x0305` 的己方坐标仍主要来自：

- 串口 `0x0301` 下的 `0x0200 AllyRobotPosition`

如果正式链路中雷达并不能稳定收到 `0x0301/0x0200`，那：

- 敌方位置仍可更新
- 己方坐标会长期为 0

这不会让主程序崩，但会影响地图语义完整性。

### 3.3 `8002/8003` 当前是一锤子买卖

当前 `EnemyKeyReceiver` 在成功接收一次合法 `0x0A06` 后会：

- 标记 `completed`
- 停止该 TCP 链路

这适合“拿到一次密钥就够”的当前需求，但如果未来希望：

- 长时间持续接收多次密钥更新
- 同一端口重复验证不同来源

就需要重新设计这部分生命周期。

### 3.4 业务值域校验仍然不够强

当前已经有：

- 协议层 CRC 校验
- `0x0A06` ASCII 校验

但还缺少更强的业务校验，例如：

- 地图坐标是否超出赛场范围
- 血量/发弹量是否异常
- `0x020E` 是否出现保留位异常

这类问题不会破坏收发链路本身，但会让“语义错误的数据”被当成正常状态继续传递。

### 3.5 若未来接入更长帧，当前通用 `Referee` 边界要重审

当前主项目依赖 `librm` 的通用 `Referee` 解析器。

若后续要接入：

- 图传链路
- 更长的自定义帧

就需要重新确认：

- `kRefProtocolFrameMaxLen`
- 是否仍应复用同一类 parser

这不是当前主链路的即时问题，但属于未来扩展时的明确边界。

## 4. 当前排障时最有价值的观测点

排查“通信不稳定”时，建议优先看：

### 4.1 连接层

- `main/serial_state.log`
- `main/tcp_startup.log`
- `main/tcp_channel_state.log`
- `main/tcp_client_state.log`

判断：

- 是根本没连上
- 还是连上后频繁断开
- 还是因 idle timeout / connect timeout 被本端主动关闭

### 4.2 原始输入层

- `raw/serial_referee_rx.bin`
- `raw/tcp_8001_info_wave_rx.bin`
- `raw/tcp_8002_enemy_level1_key_rx.bin`
- `raw/tcp_8003_enemy_level2_key_rx.bin`

判断：

- 对端到底有没有把字节送过来
- 是完全没流量，还是流量到了但协议没解出来

### 4.3 主协议更新层

- `main/0x0a01_radar0.log`
- `main/0x020b_ground_robot_position.log`
- `main/0x020e_radar_info.log`
- `main/0x0a06_*.log`

判断：

- 帧有没有解成功
- `seq` 是否连续
- 状态更新频率是否符合预期

### 4.4 发送层

- `main/0x0305_map_robot_data.log`
- `main/0x0305_map_robot_data_skipped.log`
- `main/0x0121_*.log`
- `main/tx_scheduler.log`

判断：

- 是因为 stale 被清 0
- 是因为 rate limit 未发送
- 还是因为冷却、队列、串口写失败被推迟

### 4.5 运行时层

- `main/runtime_metrics.log`

判断：

- 主循环是否被阻塞
- 日志队列是否积压
- 连接断开或 skip 是否集中出现

## 5. 对当前版本的总体判断

当前项目最关键的稳定性缺口，已经从“功能没接上”转移到了“语义是否可靠”：

- 收发主链已经具备自动重连、限频、状态过期和统一调度
- 原始日志与运行指标也足够支撑复盘
- 真正剩下的高价值问题，主要是 `loss_rate` 解释、`0x0301/0x0200` 来源确认，以及更细的业务值域校验

换句话说，当前版本已经具备“能连续跑、出问题能留痕、关键规则不明显违规”的基础稳定性。

## 6. 后续建议优先级

第一优先级：

1. 实机确认雷达链路下 `0x0301/0x0200` 的可用性。
2. 给信息波链路建立比 `loss_rate()` 更可信的频率/延迟观测方式。
3. 对关键业务字段补充值域校验与拒绝日志。

第二优先级：

1. 视对端协议情况决定是否补应用层心跳或握手。
2. 若 `8002/8003` 未来需要重复使用，重新设计 `completed` 生命周期。
3. 若未来接入更长帧，重审 parser 边界。
