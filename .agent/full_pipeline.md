# 从接收字节流到维护结构体到发送打包数据的全流程

## 文档目的

本文档基于当前仓库实际代码，按“接收字节流 -> 协议解包 -> 结构体维护 -> 应用层读取 -> 重新打包发送”的顺序，解释整条通信链路如何工作。

和上一版不同，这一版在每个关键功能点下面都直接附上当前仓库里的原代码节选，方便对照阅读，不容易和真实实现脱节。

本文只描述当前项目保留的通信主线，不展开视觉识别、敌方定位识别和反无人机功能。

## 1. 全流程总览

当前工程的完整数据流可以概括为：

```text
UART/DMA 收到一段字节
-> RefereeUART::RxCallback 逐字节喂给 Referee<kNewV120>
-> Referee<kNewV120> 有限状态机分帧
-> CRC8 校验帧头，CRC16 校验整帧
-> 按 cmd_id 把 payload memcpy 到 RefereeProtocol<kNewV120> 对应成员
-> 触发回调
-> RefereeUser<kNewV120> 若发现 cmd_id == 0x301，则继续把 user_data 拆到 RefereeSubProtocol
-> 应用层从 referee->data() / subReferee->data() 读取最新状态
-> 应用层生成待发送结构体
-> Referee0x301Prepare() 或 RefereePrepare() 打包
-> 通过串口发送给裁判系统/服务器端
```

下面把这个流程拆开看。

## 2. 工程中这条链路是怎么接起来的

先看初始化接线。当前工程在 [main.cc](/include/radar/app/main.cc#L51) 里已经把主协议、子协议和串口桥接对象串起来了。

源码节选：

```cpp
void GlobalWarehouse::Init() {
  buzzer = new Buzzer;

  can1 = new rm::hal::Can{hcan1};
  // can_communicator = new rm::device::AimbotCanCommunicator(*can2);
  dbus = new rm::hal::Serial{huart1, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  refereeUart = new rm::hal::Serial{huart3, 127, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma};

  rc = new rm::device::DR16{*dbus};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> //
      {*can1, {0x04, 0x03, 3.141593f, 420.0f, 5.0f, {0.f, 500.f}, {0.f, 5.f}}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> //
      {*can1, {0x02, 0x01, 3.141593f, 420.0f, 5.0f, {0.f, 500.f}, {0.f, 5.f}}};
  yaw_encoder = new rm::device::JyMe02Can{*can1, 0x60, 0.002f};
  pitch_encoder = new rm::device::JyMe02Can{*can1, 0x55, 0.002f};
  referee = new rm::device::Referee<device::RefereeRevision::kNewV120>;
  subReferee = new rm::device::RefereeUser(*referee);
  referee->AttachCallback(std::bind(&rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120>::AttachCallback,
                                    subReferee, std::placeholders::_1,
                                    std::placeholders::_2));
  refereeUART = new rm::device::RefereeUART{*refereeUart, *referee};

  can1->SetFilter(0, 0);
  can1->Begin();
  rc->Begin();
  buzzer->Init();
  refereeUART->Begin();
```

这里的关键关系是：

1. `referee`
   - 主协议解析器
   - 负责把裁判系统原始字节流拆成 `RefereeProtocol<kNewV120>`
2. `subReferee`
   - `0x301` 用户子协议解析器
   - 负责把主协议中的 `robot_interaction_data.user_data` 继续拆成 `RefereeSubProtocol`
3. `refereeUART`
   - 串口到协议解析器的桥
   - 负责把串口收到的每个字节喂给 `referee`
4. `AttachCallback(...)`
   - 把 `subReferee` 挂到 `referee` 的收包回调后面
   - 因此每当主协议收到一帧，子协议都会有机会检查是不是 `0x301`

可以把这一层理解成：

```text
huart3
-> rm::hal::Serial
-> RefereeUART
-> Referee<kNewV120>
-> RefereeUser<kNewV120>
```

## 3. 字节流是怎样进入主协议解析器的

入口在 [RefereeUart.cc](/include/radar/app/RefereeUart.cc#L38)。

源码节选：

```cpp
RefereeUART::RefereeUART(hal::SerialInterface &serial, Referee<RefereeRevision::kNewV120> &referee) : serial_(&serial), referee_(referee) {
  static hal::SerialRxCallbackFunction rx_callback =
      std::bind(&RefereeUART::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

void RefereeUART::Begin() { this->serial_->Begin(); }

void RefereeUART::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (auto i = 0; i < rx_len; i++) {
    referee_ << data.at(i);
  }
}
```

这段代码说明了三件事：

1. `RefereeUART` 在构造时把自己的 `RxCallback` 注册给串口对象
2. `Begin()` 最终调用的是底层串口的 `Begin()`
3. 每次串口收到一批数据后，不是整批直接交给 `Referee`，而是逐字节执行：

```cpp
referee_ << data.at(i);
```

这意味着 `Referee` 的设计本质是：

- 流式字节解析器
- 不要求一次必须拿到完整帧
- 只要后续字节继续进来，就能沿着状态机继续走

所以无论串口回调一次给：

- 3 字节
- 17 字节
- 64 字节
- 127 字节

这套代码都能继续累计解包。

## 4. 主协议 `Referee<kNewV120>` 是如何解包的

主协议核心实现在 [referee.hpp](/include/radar/libs/librm/src/librm/device/referee/referee.hpp#L53)。

### 4.1 有限状态机本体

源码节选：

```cpp
template <RefereeRevision revision, usize MaxRxCallbacks = 10>
class Referee : public Device {
  enum class DeserializeFsmState {
    kSof,
    kLenLsb,
    kLenMsb,
    kSeq,
    kCrc8,
    kCrc16,
  } deserialize_fsm_state_{DeserializeFsmState::kSof};

 public:
  using RxCallback = std::function<void(u16,  ///< cmd_id
                                        u8    ///< packet seq
                                        )>;

  Referee() = default;

  void operator<<(u8 data) {
    switch (deserialize_fsm_state_) {
      case DeserializeFsmState::kSof: {
        if (data == kRefProtocolHeaderSof) {
          deserialize_fsm_state_ = DeserializeFsmState::kLenLsb;
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        } else {
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kLenLsb: {
        data_len_this_time_ = data;
        valid_data_so_far_[valid_data_so_far_idx_++] = data;
        deserialize_fsm_state_ = DeserializeFsmState::kLenMsb;
        break;
      }

      case DeserializeFsmState::kLenMsb: {
        data_len_this_time_ |= (data << 8);
        valid_data_so_far_[valid_data_so_far_idx_++] = data;

        if (data_len_this_time_ < (kRefProtocolFrameMaxLen - kRefProtocolAllMetadataLen)) {
          deserialize_fsm_state_ = DeserializeFsmState::kSeq;
        } else {
          deserialize_fsm_state_ = DeserializeFsmState::kSof;
          valid_data_so_far_idx_ = 0;
        }
        break;
      }

      case DeserializeFsmState::kSeq: {
        seq_this_time_ = data;
        valid_data_so_far_[valid_data_so_far_idx_++] = data;
        deserialize_fsm_state_ = DeserializeFsmState::kCrc8;
        break;
      }

      case DeserializeFsmState::kCrc8: {
        valid_data_so_far_[valid_data_so_far_idx_++] = data;

        if (valid_data_so_far_idx_ == kRefProtocolHeaderLen) {
          if (modules::Crc8(valid_data_so_far_.data(), kRefProtocolHeaderLen - 1, modules::CRC8_INIT) ==
              valid_data_so_far_[4]) {
            deserialize_fsm_state_ = DeserializeFsmState::kCrc16;
          } else {
            deserialize_fsm_state_ = DeserializeFsmState::kSof;
            valid_data_so_far_idx_ = 0;
          }
        }
        break;
      }
```

这段代码展示了前半段状态机：

1. `kSof`
   - 等待帧头 `0xA5`
   - 如果不是 `0xA5`，当前字节直接丢弃
2. `kLenLsb`
   - 读取长度低字节
3. `kLenMsb`
   - 读取长度高字节
   - 同时检查长度是否合法
4. `kSeq`
   - 读取本帧序号
5. `kCrc8`
   - 在帧头够长后，立刻对帧头做 CRC8 校验
   - CRC8 错了就直接丢帧并回到 `kSof`

这意味着：

- 错帧会尽早被截断
- 不会带着错误帧继续往后累积
- 对串口噪声有基本抗性

### 4.2 接收完整帧并做 CRC16

继续看后半段：

```cpp
      case DeserializeFsmState::kCrc16: {
        if (valid_data_so_far_idx_ < (kRefProtocolAllMetadataLen + data_len_this_time_)) {
          valid_data_so_far_[valid_data_so_far_idx_++] = data;
        }
        if (valid_data_so_far_idx_ >= (kRefProtocolAllMetadataLen + data_len_this_time_)) {
          deserialize_fsm_state_ = DeserializeFsmState::kSof;
          valid_data_so_far_idx_ = 0;
          crc16_this_time_ = (valid_data_so_far_[kRefProtocolAllMetadataLen + data_len_this_time_ - 1] << 8) |
                             valid_data_so_far_[kRefProtocolAllMetadataLen + data_len_this_time_ - 2];

          if (modules::Crc16(valid_data_so_far_.data(), kRefProtocolAllMetadataLen + data_len_this_time_ - 2,
                             modules::CRC16_INIT) == crc16_this_time_) {
            cmdid_this_time_ = (valid_data_so_far_[6] << 8) | valid_data_so_far_[5];

            // 整包接收完+CRC校验通过
            // 裁判系统仍然在线
            ReportStatus(kOk);
            // 把数据拷贝到反序列化缓冲区对应的结构体中
            const usize member_offset = referee_protocol_memory_map_.map.at(cmdid_this_time_);
            u8 *dest_ptr = reinterpret_cast<u8 *>(&deserialize_buffer_) + member_offset;
            u8 *src_ptr = valid_data_so_far_.data() + kRefProtocolHeaderLen + kRefProtocolCmdIdLen;
            std::memcpy(dest_ptr, src_ptr, data_len_this_time_);

            //  触发用户注册的回调函数
            for (auto &cb : rx_callbacks_) {
              if (cb) {
                cb(cmdid_this_time_, seq_this_time_);
              }
            }
            received_packets_++;
            // 如果这一包的seq比上一包小，说明本次256包周期结束，计算丢包率
            if (seq_this_time_ < last_seq_) {
              loss_rate_smooth_.add((1.f - static_cast<f32>(received_packets_) / 256.f) * 100.f);
              received_packets_ = 0;
            }
            last_seq_ = seq_this_time_;
          }
        }
        break;
      }
```

这段代码就是“完整收包后的落地逻辑”。

真实执行顺序是：

1. 继续收后续字节，直到累计长度达到：

```cpp
kRefProtocolAllMetadataLen + data_len_this_time_
```

2. 从帧末尾取出 CRC16
3. 对整帧前半部分重新计算 CRC16
4. 若 CRC16 正确，则从固定位置提取 `cmd_id`
5. 用 `cmd_id` 查内存映射
6. 把 payload 复制进 `deserialize_buffer_` 对应字段
7. 触发所有回调

这里最关键的三行是：

```cpp
const usize member_offset = referee_protocol_memory_map_.map.at(cmdid_this_time_);
u8 *dest_ptr = reinterpret_cast<u8 *>(&deserialize_buffer_) + member_offset;
std::memcpy(dest_ptr, src_ptr, data_len_this_time_);
```

它的含义是：

- `deserialize_buffer_` 是整份主协议状态
- 不同 `cmd_id` 对应这份大结构体里的不同成员偏移
- 某帧来了，就只覆盖对应成员那一段内存

因此 `Referee` 并不是“每帧构造一个新对象”，而是：

- 一直维护一份最新状态
- 每来一帧，只更新对应字段

### 4.3 `Referee` 内部到底维护了哪些核心状态

继续看成员定义：

```cpp
 private:
  RefereeProtocol<revision> deserialize_buffer_{};
  RefereeProtocolMemoryMap<revision> referee_protocol_memory_map_;
  usize valid_data_so_far_idx_{0};
  usize data_len_this_time_{0};
  usize cmdid_this_time_{0};
  u8 seq_this_time_{0};
  u8 last_seq_{0};          ///< 上一包的seq，用于检测256包周期边界
  u8 received_packets_{0};  ///< 本轮seq内正常接收到的数据包数量
  u16 crc16_this_time_{0};
  std::array<u8, kRefProtocolFrameMaxLen> valid_data_so_far_{};

  etl::vector<RxCallback, MaxRxCallbacks> rx_callbacks_;       ///< 数据包接收回调列表
  etl::pseudo_moving_average<f32, 10> loss_rate_smooth_{0.f};  ///< 近10轮平均丢包率
};
```

这些成员分别承担下面的职责：

1. `valid_data_so_far_`
   - 当前正在累计的完整原始帧缓存
2. `valid_data_so_far_idx_`
   - 当前已经收到了多少字节
3. `data_len_this_time_`
   - 本帧 payload 长度
4. `cmdid_this_time_`
   - 本帧命令字
5. `deserialize_buffer_`
   - 已经维护好的“最新主协议状态”
6. `rx_callbacks_`
   - 成功收帧后的回调列表
7. `loss_rate_smooth_`
   - 根据 `seq` 估计出的平滑丢包率

因此 `Referee<kNewV120>` 不仅是解包器，也是“主协议状态仓库”。

## 5. `RefereeProtocol<kNewV120>` 到底长什么样

真正存放主协议数据的结构在 [protocol_new_v120.hpp](/include/radar/libs/librm/src/librm/device/referee/protocol_new_v120.hpp#L39)。

### 5.1 命令字定义

源码节选：

```cpp
template <>
struct RefereeCmdId<RefereeRevision::kNewV120> {
  constexpr static u16 kGameStatus = 0x1;
  constexpr static u16 kGameResult = 0x2;
  constexpr static u16 kGameRobotHp = 0x3;
  constexpr static u16 kEventData = 0x101;
  constexpr static u16 kRefereeWarning = 0x104;
  constexpr static u16 kDartInformation = 0x105;
  constexpr static u16 kRobotStatus = 0x201;
  constexpr static u16 kPowerHeatData = 0x202;
  constexpr static u16 kRobotPos = 0x203;
  constexpr static u16 kBuff = 0x204;
  constexpr static u16 kHurtData = 0x206;
  constexpr static u16 kShootData = 0x207;
  constexpr static u16 kProjectileAllowance = 0x208;
  constexpr static u16 kRfidStatus = 0x209;
  constexpr static u16 kDartClientCmd = 0x20a;
  constexpr static u16 kGroundRobotPosition = 0x20b;
  constexpr static u16 kRadarMarkData = 0x20c;
  constexpr static u16 kSentryInfo = 0x20d;
  constexpr static u16 kRadarInfo = 0x20e;
  constexpr static u16 kRobotInteractionData = 0x301;
  constexpr static u16 kCustomRobotData = 0x302;
  constexpr static u16 kMapCommand = 0x303;
  constexpr static u16 kMapRobotData = 0x305;
  constexpr static u16 kCustomClientData = 0x306;
  constexpr static u16 kMapData = 0x307;
  constexpr static u16 kCustomInfo = 0x308;
  constexpr static u16 kRobotCustomData = 0x309;
  constexpr static u16 kRobotCustomData2 = 0x310;
  constexpr static u16 kRobotCustomData3 = 0x311;
  constexpr static u16 kRadar0 = 0xa01;
  constexpr static u16 kRadar1 = 0xa02;
  constexpr static u16 kRadar2 = 0xa03;
  constexpr static u16 kRadar3 = 0xa04;
  constexpr static u16 kRadar4 = 0xa05;
  constexpr static u16 kRadar5 = 0xa06;
};
```

这段代码告诉我们：

- 当前项目主版本是 `kNewV120`
- 不仅支持常规裁判系统命令
- 也已经支持 `0x0A01 ~ 0x0A06` 这组信息波相关命令

### 5.2 主结构体中的关键成员

源码节选：

```cpp
template <>
struct RefereeProtocol<RefereeRevision::kNewV120> {
  struct {
    u8 robot_id;
    u8 robot_level;
    u16 current_HP;
    u16 maximum_HP;
    u16 shooter_barrel_cooling_value;
    u16 shooter_barrel_heat_limit;
    u16 chassis_power_limit;
    u8 power_management_gimbal_output : 1;
    u8 power_management_chassis_output : 1;
    u8 power_management_shooter_output : 1;
  } robot_status;
  struct {
    u16 reserved;
    u16 reserved_2;
    f32 reserved_3;
    u16 buffer_energy;
    u16 shooter_17mm_1_barrel_heat;
    u16 shooter_42mm_barrel_heat;
  } power_heat_data;
  struct {
    f32 x;
    f32 y;
    f32 angle;
  } robot_pos;
  struct {
    u8 recovery_buff;
    u16 cooling_buff;
    u8 defence_buff;
    u8 vulnerability_buff;
    u16 attack_buff;
    u8 remaining_energy;
  } buff;
  struct {
    u16 data_cmd_id;
    u16 sender_id;
    u16 receiver_id;
    u8 user_data[112];
  } robot_interaction_data;
  struct {
    u16 enemy_hero_position_x;
    u16 enemy_hero_position_y;
    u16 enemy_engineer_position_x;
    u16 enemy_engineer_position_y;
    u16 enemy_infantry_3_position_x;
    u16 enemy_infantry_3_position_y;
    u16 enemy_infantry_4_position_x;
    u16 enemy_infantry_4_position_y;
    u16 enymy_drone_position_x;
    u16 enymy_drone_position_y;
    u16 enemy_sentry_position_x;
    u16 enemy_sentry_position_y;
  } radar0;
  struct {
    u16 enemy_hero_HP;
    u16 enemy_engineer_HP;
    u16 enemy_infantry_3_HP;
    u16 enemy_infantry_4_HP;
    u16 reserved;
    u16 enemy_sentry_HP;
  } radar1;
  struct {
    u8 key[6];
  } radar5;
};
```

这段不是完整结构体，只截了当前主线最重要的几个成员。它们大致分成四类：

1. 常规链路状态
   - `robot_status`
   - `power_heat_data`
   - `robot_pos`
   - `buff`
2. `0x301` 容器
   - `robot_interaction_data`
3. 信息波扩展输入
   - `radar0`
   - `radar1`
   - `radar2`
   - `radar3`
   - `radar4`
   - `radar5`
4. 其余比赛状态字段
   - 比如 `game_status`、`projectile_allowance`、`rfid_status` 等

应用层调用：

```cpp
globals->referee->data()
```

拿到的就是这整个大结构体的当前快照。

### 5.3 主协议内存映射是如何建立的

源码节选：

```cpp
template <>
struct RefereeProtocolMemoryMap<RefereeRevision::kNewV120> {
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>({
      {RefereeCmdId<RefereeRevision::kNewV120>::kGameStatus, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, game_status)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kGameResult, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, game_result)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kGameRobotHp, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, game_robot_HP)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kEventData, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, event_data)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRobotStatus, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, robot_status)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kPowerHeatData, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, power_heat_data)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRobotPos, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, robot_pos)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kBuff, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, buff)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRobotInteractionData, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, robot_interaction_data)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar0, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar0)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar1, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar1)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar2, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar2)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar3, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar3)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar4, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar4)},
      {RefereeCmdId<RefereeRevision::kNewV120>::kRadar5, offsetof(RefereeProtocol<RefereeRevision::kNewV120>, radar5)},
  });
};
```

这就是前面 `Referee::operator<<` 里那句：

```cpp
referee_protocol_memory_map_.map.at(cmdid_this_time_)
```

到底在查什么。

它的本质是：

- 左边：命令字 `cmd_id`
- 右边：`RefereeProtocol<kNewV120>` 某成员在结构体里的偏移量

所以如果后面要新增、修正某个命令的自动落地，必须同时改两处：

1. `RefereeProtocol<kNewV120>` 里补字段
2. `RefereeProtocolMemoryMap<kNewV120>` 里补映射

只改其中一处都不够。

## 6. `0x301` 用户子协议是怎样继续拆包的

主协议 `0x301` 并不直接对应某个业务结构，它只是一个容器帧。

### 6.1 `0x301` 容器结构

再看一次主结构中的这部分：

```cpp
struct {
  u16 data_cmd_id;
  u16 sender_id;
  u16 receiver_id;
  u8 user_data[112];
} robot_interaction_data;
```

这表示：

1. 主协议先收到一帧 `0x301`
2. 这帧内部再告诉你真正的子命令 `data_cmd_id`
3. 真实业务内容存放在 `user_data[112]`

因此对于 `0x301`，解析还没结束，还得继续下钻一层。

### 6.2 `RefereeUser<kNewV120>` 的拆包逻辑

源码节选：

```cpp
template <RefereeRevision revision>
class RefereeUser final : public Device {
 private:
  RefereeSubProtocol deserialize_buffer_{};
  RefereeSubProtocolMemoryMap referee_protocol_memory_map_;
  Referee<revision> &referee_;

 public:
  RefereeUser() = delete;

  explicit RefereeUser(Referee<revision> &referee) : referee_(referee) {}

  const RefereeSubProtocol &data() const { return deserialize_buffer_; }

  void AttachCallback(u16 cmd_id_, u8 seq_) {
    ReportStatus(kOk);
    u16 subCmdID = referee_.data().robot_interaction_data.data_cmd_id;
    if (cmd_id_ != 0x301) return;
    if (!rm::device::RefereeSubProtocolMemoryMap::map.contains(subCmdID)) return;
    const usize member_offset = rm::device::RefereeSubProtocolMemoryMap::map.at(subCmdID);
    u8 *dest_ptr = reinterpret_cast<u8 *>(&deserialize_buffer_) + member_offset;
    u8 *src_ptr = const_cast<u8 *>(referee_.data().robot_interaction_data.user_data);
    std::memcpy(dest_ptr, src_ptr, rm::device::RefereeSubProtocolMemoryMap::mapSize.at(subCmdID));
  }
};
```

这段代码的真实执行流程是：

1. 主协议成功收帧后，触发回调
2. `RefereeUser::AttachCallback(...)` 被调用
3. 如果这帧不是 `0x301`，立刻返回
4. 如果是 `0x301`，就从：

```cpp
referee_.data().robot_interaction_data.data_cmd_id
```

拿到子命令号

5. 用子命令号去查 `RefereeSubProtocolMemoryMap`
6. 从：

```cpp
referee_.data().robot_interaction_data.user_data
```

取出真正 payload

7. 再 `memcpy` 到 `deserialize_buffer_` 对应成员

所以 `RefereeUser` 可以理解成：

- 专门服务于 `0x301` 的第二层解析器
- 它维护的是另一份“最新子协议状态”

读取方式就是：

```cpp
globals->subReferee->data()
```

## 7. `RefereeSubProtocol` 到底维护了什么

真正的子协议结构和映射在 [protocol_user.hpp](/include/radar/app/subReferee/protocol_user.hpp#L407)。

### 7.1 子命令定义

源码节选：

```cpp
struct RefereeSubCmdId {
  // 对应0x20B，哨兵机器人接收位置信息
  constexpr static u16 kAllyRobotPosition = 0x0200;
  // 对应雷达0x0A01~0x0A05信息波，包含敌方信息
  constexpr static u16 kEnemyRobotPosition = 0x0201;
  constexpr static u16 kEnemyRobotHP = 0x0202;
  constexpr static u16 kEnemyRobotProjectileAllowance = 0x0203;
  constexpr static u16 kEnemyGoldCoinRFID = 0x0204;
  constexpr static u16 kEnemyRobotBuff = 0x0205;
  constexpr static u16 kAllRadarInfo = 0x0206;
  // 云台手大吊射
  constexpr static u16 kHero2Drone = 0x0207;
  constexpr static u16 kDrone2Hero = 0x0208;
  // UI
  constexpr static u16 kUILayer = 0x0100;
  constexpr static u16 kUIFigure1 = 0x0101;
  constexpr static u16 kUIFigure2 = 0x0102;
  constexpr static u16 kUIFigure5 = 0x0103;
  constexpr static u16 kUIFigure7 = 0x0104;
  constexpr static u16 kUICharacter = 0x0110;
  constexpr static u16 kSentryCMD = 0x0120;
  constexpr static u16 kRadarCMD = 0x0121;
};
```

这段代码说明当前项目把 `0x301` 里的子协议进一步抽象成了这些业务类型。

当前主线最关键的是：

1. 信息波/敌方状态抽象
   - `0x0201 ~ 0x0206`
2. 云台与雷达联动
   - `0x0207`
   - `0x0208`
3. 指令输出
   - `0x0120`
   - `0x0121`

其中：

- `0x0121 RadarCMD`
  - 当前最接近“双倍易伤触发、破解密钥上报”
- `0x0201 ~ 0x0205`
  - 当前最接近把 `0x0A01 ~ 0x0A05` 整理成项目内部稳定格式

### 7.2 子协议状态结构体

源码节选：

```cpp
struct RefereeSubProtocol {
  struct AllyRobotPosition ally_robot_position{};                           // 0x0200
  struct EnemyRobotPosition enemy_robot_position{};                         // 0x0201
  struct EnemyRobotHP enemy_robot_HP{};                                     // 0x0202
  struct EnemyRobotProjectileAllowance enemy_robot_projectile_allowance{};  // 0x0203
  struct EnemyGoldCoinRFID enemy_gold_coin_RFID{};                          // 0x0204
  struct EnemyRobotBuff enemy_robot_buff{};                                 // 0x0205
  struct Hero2Drone hero_2_drone{};
  struct Drone2Hero drone_2_hero{};
  struct SentryCMD sentry_cmd{};                                            // 0x0120
  struct RadarCMD radar_cmd{};                                              // 0x0121
};
```

这就是 `subReferee->data()` 返回的那份状态结构。

也就是说：

- `referee->data()`
  - 更像“原始主协议状态”
- `subReferee->data()`
  - 更像“当前工程定义的用户子协议状态”

### 7.3 子协议内存映射

源码节选：

```cpp
struct RefereeSubProtocolMemoryMap {
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>(
      {{RefereeSubCmdId::kAllyRobotPosition, offsetof(RefereeSubProtocol, ally_robot_position)},
       {RefereeSubCmdId::kEnemyRobotPosition, offsetof(RefereeSubProtocol, enemy_robot_position)},
       {RefereeSubCmdId::kEnemyRobotHP, offsetof(RefereeSubProtocol, enemy_robot_HP)},
       {RefereeSubCmdId::kEnemyRobotProjectileAllowance,
        offsetof(RefereeSubProtocol, enemy_robot_projectile_allowance)},
       {RefereeSubCmdId::kEnemyGoldCoinRFID, offsetof(RefereeSubProtocol, enemy_gold_coin_RFID)},
       {RefereeSubCmdId::kEnemyRobotBuff, offsetof(RefereeSubProtocol, enemy_robot_buff)},
       {RefereeSubCmdId::kAllRadarInfo, offsetof(RefereeSubProtocol, enemy_robot_HP)},
       {RefereeSubCmdId::kHero2Drone, offsetof(RefereeSubProtocol, hero_2_drone)},
       {RefereeSubCmdId::kDrone2Hero, offsetof(RefereeSubProtocol, drone_2_hero)},
       {RefereeSubCmdId::kSentryCMD, offsetof(RefereeSubProtocol, sentry_cmd)},
       {RefereeSubCmdId::kRadarCMD, offsetof(RefereeSubProtocol, radar_cmd)}});
  static MAPBOX_ETERNAL_CONSTEXPR const auto mapSize = mapbox::eternal::map<u16, usize>(
      {{RefereeSubCmdId::kAllRadarInfo, sizeof(RefereeSubProtocol) - sizeof(RefereeSubProtocol::ally_robot_position) -
                                            sizeof(RefereeSubProtocol::enemy_robot_position)},
       {RefereeSubCmdId::kAllyRobotPosition, sizeof(RefereeSubProtocol::ally_robot_position)},
       {RefereeSubCmdId::kEnemyRobotPosition, sizeof(RefereeSubProtocol::enemy_robot_position)},
       {RefereeSubCmdId::kEnemyRobotHP, sizeof(RefereeSubProtocol::enemy_robot_HP)},
       {RefereeSubCmdId::kEnemyRobotProjectileAllowance, sizeof(RefereeSubProtocol::enemy_robot_projectile_allowance)},
       {RefereeSubCmdId::kEnemyGoldCoinRFID, sizeof(RefereeSubProtocol::enemy_gold_coin_RFID)},
       {RefereeSubCmdId::kEnemyRobotBuff, sizeof(RefereeSubProtocol::enemy_robot_buff)},
       {RefereeSubCmdId::kHero2Drone, sizeof(RefereeSubProtocol::hero_2_drone)},
       {RefereeSubCmdId::kDrone2Hero, sizeof(RefereeSubProtocol::drone_2_hero)},
       {RefereeSubCmdId::kSentryCMD, sizeof(RefereeSubProtocol::sentry_cmd)},
       {RefereeSubCmdId::kRadarCMD, sizeof(RefereeSubProtocol::radar_cmd)}});
};
```

这和主协议层的逻辑完全对应：

- `map`
  - 负责根据子命令找到子结构体偏移
- `mapSize`
  - 负责告诉 `memcpy` 到底拷多少字节

因此后续若新增子协议字段，必须同步维护：

1. `RefereeSubProtocol`
2. `RefereeSubProtocolMemoryMap::map`
3. `RefereeSubProtocolMemoryMap::mapSize`

## 8. `0x0A01 ~ 0x0A06` 与内部 `0x0201 ~ 0x0206` 的关系

当前代码里这两套东西是并存的，但职责不完全一样。

### 8.1 主协议层的 `0x0A01 ~ 0x0A06`

在 [protocol_new_v120.hpp](/include/radar/libs/librm/src/librm/device/referee/protocol_new_v120.hpp#L39) 里，它们是主协议的一部分：

```cpp
constexpr static u16 kRadar0 = 0xa01;
constexpr static u16 kRadar1 = 0xa02;
constexpr static u16 kRadar2 = 0xa03;
constexpr static u16 kRadar3 = 0xa04;
constexpr static u16 kRadar4 = 0xa05;
constexpr static u16 kRadar5 = 0xa06;
```

并且有对应结构体：

```cpp
struct {
  u16 enemy_hero_position_x;
  u16 enemy_hero_position_y;
  ...
} radar0;

struct {
  u16 enemy_hero_HP;
  ...
} radar1;

struct {
  u8 key[6];
} radar5;
```

这表示：

- 如果串口直接收到 `0x0A01 ~ 0x0A06`
- 那么这些内容会直接落到：

```cpp
referee->data().radar0
referee->data().radar1
...
referee->data().radar5
```

### 8.2 子协议层的 `0x0201 ~ 0x0206`

在 [protocol_user.hpp](/include/radar/app/subReferee/protocol_user.hpp#L407) 里，这套编号更像“项目内部统一抽象”：

```cpp
constexpr static u16 kEnemyRobotPosition = 0x0201;
constexpr static u16 kEnemyRobotHP = 0x0202;
constexpr static u16 kEnemyRobotProjectileAllowance = 0x0203;
constexpr static u16 kEnemyGoldCoinRFID = 0x0204;
constexpr static u16 kEnemyRobotBuff = 0x0205;
constexpr static u16 kAllRadarInfo = 0x0206;
```

建议这样理解：

1. `0x0A01 ~ 0x0A06`
   - 更接近常规链路收到的原始协议帧
2. `0x0201 ~ 0x0206`
   - 更接近项目内部对敌方信息的统一表示

所以应用层常见的合理做法是：

1. 先收原始主协议
2. 需要时从 `radar0 ~ radar5` 提炼整理
3. 再转成内部统一业务结构，或者封成 `0x301` 子协议对外发送

## 9. 应用层如何消费这些“已维护好的结构体”

当前项目不是“来一帧就直接在回调里做全部业务”，而是更接近：

- 接收层负责把结构体维护好
- 主循环层负责周期性读取结构体做业务

看当前主循环里的实际代码。源码节选：

```cpp
device::Drone2Hero d2h;
device::Hero2Drone h2d;

void GlobalWarehouse::SubLoop500Hz() {
  imu_time = HAL_GetTick();

  globals->RCStateUpdate();
  gimbal->GimbalTask();

  globals->yaw_motor->SetMitCommand(0, 0, gimbal->yaw_speed_pid_.out(), 0, 0);
  globals->pitch_motor->SetMitCommand(0, 0, gimbal->pitch_speed_pid_.out(), 0, 0);

  // 调试信息
  d_aimbot_yaw = globals->Aimbot.Yaw;
  d_aimbot_pitch = globals->Aimbot.Pitch;
  d_aimbot_yaw_vel = globals->Aimbot.YawVelocity;
  d_aimbot_pitch_vel = globals->Aimbot.PitchVelocity;
  d_aim_state = globals->Aimbot.AimbotState;
  d_encoder_pitch = gimbal->pitch_angle_encoder_;
  d_encoder_yaw = gimbal->yaw_angle_encoder_;
  d_vel_pitch = gimbal->pitch_vel_;
  d_vel_yaw = gimbal->yaw_vel_;
  d_yaw_motor_t = gimbal->yaw_speed_pid_.out();
  d_pitch_motor_t = gimbal->pitch_speed_pid_.out();
  d_yaw_target = gimbal->yaw_target_encoder_;
  d_pitch_target = gimbal->pitch_target_encoder_;
  d_referee_pos_X = globals->referee->data().robot_pos.x;
  d_referee_pos_y = globals->referee->data().robot_pos.x;
  d_referee_ammo_direction = globals->referee->data().robot_pos.angle;
  memcpy(&d2h, &globals->subReferee->data().drone_2_hero, sizeof(d2h));
  memcpy(&h2d, &globals->subReferee->data().hero_2_drone, sizeof(h2d));
}
```

这段代码非常能说明当前架构思想：

1. 主循环不是自己解包
2. 它直接读取：
   - `globals->referee->data()`
   - `globals->subReferee->data()`
3. 说明在进入主循环之前，接收层已经把状态维护好了

因此当前工程推荐的职责划分是：

1. 接收层
   - 只负责收字节、校验、分发、更新结构体
2. 业务层
   - 周期性读取结构体
   - 做融合、决策、重组
3. 发送层
   - 把业务结果重新打包发出

不要把大段决策逻辑塞进串口回调里。

## 10. 发送端是怎么打包的

当前仓库已经提供了两个现成打包接口，位于 [referee_user.hpp](/include/radar/app/subReferee/referee_user.hpp#L74)。

### 10.1 打包 `0x301` 用户交互帧

源码节选：

```cpp
template <typename T>
[[nodiscard]] inline u8 Referee0x301Prepare(u8 *data, const u16 start_index, T &info, const u16 sender, const u16 receiver) {
  static u8 seq_ = 0;
  u16 index_ = start_index;
  data[index_++] = kRefProtocolHeaderSof;
  constexpr u16 data_len = sizeof(info) + 6;
  data[index_++] = data_len & 0xff;
  data[index_++] = data_len >> 8;
  data[index_++] = seq_++;
  data[index_++] = modules::Crc8(&data[start_index], kRefProtocolHeaderLen - 1, modules::CRC8_INIT);
  data[index_++] = 0x301 & 0xff;
  data[index_++] = 0x301 >> 8;
  data[index_++] = getCmd(info) & 0xff;
  data[index_++] = getCmd(info) >> 8;
  data[index_++] = sender & 0xff;
  data[index_++] = sender >> 8;
  data[index_++] = receiver & 0xff;
  data[index_++] = receiver >> 8;
  std::memcpy(&data[index_], &info, sizeof(info));
  index_ += sizeof(info);
  const u16 CRC16_ = modules::Crc16(&data[start_index], index_ - start_index, modules::CRC16_INIT);
  data[index_++] = CRC16_ & 0xff;
  data[index_++] = CRC16_ >> 8;
  return index_ - start_index;
};
```

这段代码实际上把一整帧都封好了。它自动完成：

1. 写 `SOF`
2. 写长度
3. 写 `seq`
4. 写 CRC8
5. 写主命令字 `0x301`
6. 写子命令字 `getCmd(info)`
7. 写 `sender_id`
8. 写 `receiver_id`
9. 把结构体内容 `memcpy` 到 payload
10. 写 CRC16

因此应用层只需要：

1. 准备一个业务结构体
2. 填它的字段
3. 调一次 `Referee0x301Prepare(...)`
4. 把生成的字节发出去

### 10.2 `getCmd(info)` 是如何自动匹配子命令的

这背后依赖 [protocol_user.hpp](/include/radar/app/subReferee/protocol_user.hpp#L459) 中的类型映射。源码节选：

```cpp
template <typename T>
struct TypeToCmd;

#define DEFINE_TYPE_TO_CMD(TypeName, CmdName)                 \
  template <>                                                 \
  struct TypeToCmd<TypeName> {                                \
    static constexpr u16 value = RefereeSubCmdId::k##CmdName; \
  };

DEFINE_TYPE_TO_CMD(EnemyRobotPosition, EnemyRobotPosition)
DEFINE_TYPE_TO_CMD(EnemyRobotHP, EnemyRobotHP)
DEFINE_TYPE_TO_CMD(EnemyRobotProjectileAllowance, EnemyRobotProjectileAllowance)
DEFINE_TYPE_TO_CMD(EnemyGoldCoinRFID, EnemyGoldCoinRFID)
DEFINE_TYPE_TO_CMD(EnemyRobotBuff, EnemyRobotBuff)
DEFINE_TYPE_TO_CMD(AllRadarInfo, AllRadarInfo)
DEFINE_TYPE_TO_CMD(Hero2Drone, Hero2Drone)
DEFINE_TYPE_TO_CMD(Drone2Hero, Drone2Hero)
DEFINE_TYPE_TO_CMD(UILayer, UILayer)
DEFINE_TYPE_TO_CMD(UIFigure1, UIFigure1)
DEFINE_TYPE_TO_CMD(UIFigure2, UIFigure2)
DEFINE_TYPE_TO_CMD(UIFigure5, UIFigure5)
DEFINE_TYPE_TO_CMD(UIFigure7, UIFigure7)
DEFINE_TYPE_TO_CMD(UICharacter, UICharacter)

template <typename T>
constexpr int getCmd(const T &obj) {
  return TypeToCmd<T>::value;
}
```

这意味着：

- 你把什么类型的结构体传给 `Referee0x301Prepare(...)`
- 它就会自动知道对应的子命令字是什么

比如：

- `RadarCMD`
  - 会打成 `0x0121`
- `Hero2Drone`
  - 会打成 `0x0207`
- `EnemyRobotHP`
  - 会打成 `0x0202`

### 10.3 打包 `0x0305 MapRobotPosition`

第二个接口是：

```cpp
inline u8 RefereePrepare(u8 *data, const u16 start_index, const struct MapRobotPosition &structInfo) {
  static u8 seq_ = 0;
  u16 index_ = start_index;
  data[index_++] = kRefProtocolHeaderSof;
  constexpr u16 data_len = sizeof(structInfo);
  data[index_++] = data_len & 0xff;
  data[index_++] = data_len >> 8;
  data[index_++] = seq_++;
  data[index_++] = modules::Crc8(&data[start_index], kRefProtocolHeaderLen - 1, modules::CRC8_INIT);
  data[index_++] = getCmd(structInfo) & 0xff;
  data[index_++] = getCmd(structInfo) >> 8;
  std::memcpy(&data[index_], &structInfo, sizeof(structInfo));
  index_ += sizeof(structInfo);
  const u16 CRC16_ = modules::Crc16(&data[start_index], index_ - start_index, modules::CRC16_INIT);
  data[index_++] = CRC16_ & 0xff;
  data[index_++] = CRC16_ >> 8;
  return index_ - start_index;
};
```

它主要用于：

- 直接按 `0x0305` 打包 `MapRobotPosition`

和 `0x301` 打包器相比，它少了一层“子协议头”，更接近主协议直发。

## 11. 一个完整的发送思路示例

比如要向外发送 `RadarCMD`，推荐流程是：

1. 准备业务结构体
2. 填字段
3. 准备发送缓冲区
4. 打包
5. 调串口发送接口

示意代码：

```cpp
rm::device::RadarCMD cmd{};
cmd.radar_cmd = 1;
cmd.password_cmd = 1;
cmd.password_1 = key0;
cmd.password_2 = key1;
cmd.password_3 = key2;
cmd.password_4 = key3;
cmd.password_5 = key4;
cmd.password_6 = key5;

u8 tx_buf[128]{};
u8 len = rm::device::Referee0x301Prepare(tx_buf, 0, cmd, sender_id, receiver_id);

// 再调用串口发送接口，把 tx_buf 前 len 字节发出
```

这里要特别注意：

- `Referee0x301Prepare(...)` 只负责协议封装
- 真正的串口发出动作还需要由串口类或 HAL 发送接口完成

## 12. 为什么当前这套实现适合做“常态化维护”

这一套架构最大的优点，不是“能收一帧”，而是能稳定维护两份最新状态：

1. 主协议状态

```cpp
globals->referee->data()
```

2. 用户子协议状态

```cpp
globals->subReferee->data()
```

这意味着应用层可以像访问普通状态变量一样读取最新通信结果，而不是每次都重新解析原始字节。

对当前项目来说，这非常适合：

1. 持续接收常规链路
2. 持续接收 `0x301` 用户协议
3. 在主循环中周期性做决策
4. 在需要时重新打包发送

## 13. 当前边界与注意事项

### 13.1 串口回调层尽量只做协议层工作

当前接收链路已经足够清楚：

- `RefereeUART`
  - 喂字节
- `Referee`
  - 分帧 + CRC + 主结构体更新
- `RefereeUser`
  - `0x301` 子结构体更新

建议不要把大量业务判断塞到这层，否则后续会难维护、难测试、难做文件回放。

### 13.2 新增字段时要同时改结构体和映射

主协议层：

1. `RefereeProtocol<kNewV120>`
2. `RefereeProtocolMemoryMap<kNewV120>`

子协议层：

1. `RefereeSubProtocol`
2. `RefereeSubProtocolMemoryMap::map`
3. `RefereeSubProtocolMemoryMap::mapSize`

少改一处都会导致：

- 收到了帧
- CRC 也对
- 但结构体没正确更新

### 13.3 `0x0A05` 当前是按现有样本长度兼容的

当前仓库里的 `radar4` 虽然来源参考了 `V200`，但已经按当前 `v120` 实测样本做了兼容调整。

所以后续如果拿到新的官方协议样本，应该优先重新核对：

1. `0x0A05` 的 payload 实际长度
2. `radar4` 字段是否还需要扩展
3. 日志输出是否仍然对齐

## 14. 一句话总结

当前项目的核心实现方法就是：

先把串口原始字节流稳定地维护成两份“最新状态结构体”：

- `RefereeProtocol<kNewV120>`
- `RefereeSubProtocol`

然后业务层只负责读这两份状态、做决策、生成待发送结构，再用现成打包器重新封回协议帧发出去。
