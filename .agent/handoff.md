# Handoff

本文件记录当前仓库中与雷达通信最相关的 API 地图，供后续 Agent 和开发者快速接手。

## 当前项目主线

本项目当前核心方向是：

- 接收信息波处理端发送来的数据
- 接收裁判系统/服务器端发送来的数据
- 对两侧数据进行解析、整理、决策与重组
- 将处理结果发送至裁判系统/服务器端

后续阅读和开发都应围绕这条主线理解这些 API，而不是把项目理解成纯视觉识别或纯雷达识别工程。
反无人机 / 空中机器人反制功能不在本项目实现，属于独立项目边界。

## 当前结论

`librm` 已更新。

当前雷达通信应优先复用：

- `include/radar/libs/librm/src/librm/device/referee`
- `include/radar/app/subReferee`
- `include/radar/libs/librm/src/librm/modules/crc.hpp`

后续不要重新发明串口解包、CRC 校验和裁判系统帧结构，默认先从这套 API 开始接。
默认处理流向是：

1. 从信息波处理端或裁判系统侧接收数据
2. 用现有协议和 CRC API 完成解包与校验
3. 将结果落到现有主协议结构或用户子协议结构
4. 按裁判系统/服务器端要求重新打包并发送

## 总体分层地图

从下到上可以这样理解：

1. `rm::modules::Crc8/Crc16`
2. `rm::device::Referee<revision>`
3. `rm::device::RefereeProtocol<revision>`
4. `rm::device::RefereeUser<revision>`
5. `rm::device::RefereeSubProtocol`

职责边界如下：

- `crc.hpp`：统一提供 CRC8 / CRC16 / CRC32 / CCITT 的计算函数。
- `Referee<revision>`：按字节接收裁判系统串口流，完成 SOF、长度、seq、CRC8、CRC16 校验和整包反序列化。
- `RefereeProtocol<revision>`：描述主协议结构体。
- `RefereeUser<revision>`：从主协议 `0x301` 中继续拆出用户子协议。
- `RefereeSubProtocol`：描述本项目自己定义的信息整合结果、雷达数据和 UI 子协议结构。

## CRC API

文件：

- `include/radar/libs/librm/src/librm/modules/crc.hpp`

关键常量：

- `rm::modules::CRC8_INIT = 0xff`
- `rm::modules::CRC16_INIT = 0xffff`

关键函数：

- `rm::modules::Crc8(const u8* input, usize len, u8 init)`
- `rm::modules::Crc16(const u8* input, usize len, u16 init)`

当前仓库里：

- 主裁判系统解包使用 `Crc8` 校验帧头。
- 主裁判系统解包使用 `Crc16` 校验整帧。
- `Referee0x301Prepare()` 与 `RefereePrepare()` 也使用这两套 CRC。

## 主协议入口

文件：

- `include/radar/libs/librm/src/librm/device/referee/referee.hpp`
- `include/radar/libs/librm/src/librm/device/referee/protocol.hpp`

### `Referee<revision>`

这是主串口裁判系统解包入口，也是当前项目接收裁判系统/服务器端相关数据的基础入口。

关键用法：

- 每收到一个字节，就调用一次 `referee << byte`。
- 收到完整且 CRC 正确的一帧后，会自动把 Payload 拷贝到 `RefereeProtocol<revision>` 对应字段。
- 通过 `AttachCallback()` 挂回调，在每次成功收包后拿到 `cmd_id` 和 `seq`。
- 通过 `data()` 获取当前已解出的主协议结构体。

关键协议常量：

- `kRefProtocolHeaderSof = 0xA5`
- `kRefProtocolHeaderLen = 5`
- `kRefProtocolCmdIdLen = 2`
- `kRefProtocolCrc16Len = 2`
- `kRefProtocolAllMetadataLen = 9`
- `kRefProtocolFrameMaxLen = 128`

## 主协议版本结论

文件：

- `include/radar/libs/librm/src/librm/device/referee/protocol.hpp`

当前版本枚举：

- `kV164`
- `kV170`
- `kNewV110`
- `kNewV120`

当前项目里，后续优先关注 `kNewV120`。

原因：

- 工程代码已实际使用 `kNewV120`
- `kNewV120` 已补齐 `0x301`
- `kNewV120` 相比之前已经新增信息波相关命令号 `0x0A01~0x0A06`

## `kNewV120` 主协议地图

文件：

- `include/radar/libs/librm/src/librm/device/referee/protocol_new_v120.hpp`

### 与雷达最相关的主命令

- `0x20b kGroundRobotPosition`
- `0x20c kRadarMarkData`
- `0x20d kSentryInfo`
- `0x20e kRadarInfo`
- `0x301 kRobotInteractionData`
- `0x302 kCustomRobotData`
- `0x303 kMapCommand`
- `0x305 kMapRobotData`
- `0x306 kCustomClientData`
- `0x307 kMapData`
- `0x308 kCustomInfo`
- `0x309 kRobotCustomData`
- `0x310 kRobotCustomData2`
- `0x311 kRobotCustomData3`

### 相比之前新增的信息波命令号

这些命令头已经进入 `RefereeCmdId<RefereeRevision::kNewV120>`：

- `0x0A01 kRadar0`：雷达标记对方位置
- `0x0A02 kRadar1`：雷达标记对方血量
- `0x0A03 kRadar2`：雷达标记对方发弹
- `0x0A04 kRadar3`：雷达标记对方宏观状态
- `0x0A05 kRadar4`：雷达标记对方增益
- `0x0A06 kRadar5`：雷达标记对方干扰波密钥

注意：

- 当前 `protocol_new_v120.hpp` 中已经加入这些命令号常量。
- 但当前 `RefereeProtocol<kNewV120>` 结构体本体还没有看到对应 `0x0A01~0x0A06` 的具体结构字段。
- `RefereeProtocolMemoryMap<kNewV120>` 当前也没有把 `0x0A01~0x0A06` 映射进去。

这意味着：

- 协议命令号层面已经更新到能表达这些信息波帧编号。
- 真正要通过 `Referee<kNewV120>` 自动落到结构体字段，还需要补这些字段和内存映射。
- 在此之前，更适合把信息波处理端的结果先落到现有 `subReferee` 自定义结构体中，再按需要转发到裁判系统/服务器端。

## `kNewV120` 已有主结构体

当前已经能直接通过 `referee.data()` 访问的关键字段：

- `ground_robot_position`
- `radar_mark_data`
- `sentry_info`
- `radar_info`
- `robot_interaction_data`
- `map_command`
- `map_robot_data`
- `map_data`
- `custom_info`
- `custom_robot_data`
- `robot_custom_data`
- `robot_custom_data_2`
- `robot_custom_data_3`
- `custom_client_data`

其中最关键的是：

- `robot_interaction_data`
  - `data_cmd_id`
  - `sender_id`
  - `receiver_id`
  - `user_data[112]`

这就是 `0x301` 用户子协议的主入口。

## 用户子协议入口

文件：

- `include/radar/app/subReferee/referee_user.hpp`
- `include/radar/app/subReferee/protocol_user.hpp`

### `RefereeUser<revision>`

职责：

- 从主协议 `0x301` 中提取 `data_cmd_id`
- 按 `RefereeSubProtocolMemoryMap` 将 `user_data` 拷贝到子协议结构体

关键逻辑：

- 只有 `cmd_id == 0x301` 才继续解析
- 从 `referee_.data().robot_interaction_data.data_cmd_id` 取子命令号
- 从 `referee_.data().robot_interaction_data.user_data` 取实际子协议内容

### 当前子命令地图

文件：

- `include/radar/app/subReferee/protocol_user.hpp`

当前子命令：

- `0x0200 AllyRobotPosition`
- `0x0201 EnemyRobotPosition`
- `0x0202 EnemyRobotHP`
- `0x0203 EnemyRobotProjectileAllowance`
- `0x0204 EnemyGoldCoinRFID`
- `0x0205 EnemyRobotBuff`
- `0x0206 AllRadarInfo`
- `0x0207 Hero2Drone`
- `0x0208 Drone2Hero`
- `0x0100 UILayer`
- `0x0101 UIFigure1`
- `0x0102 UIFigure2`
- `0x0103 UIFigure5`
- `0x0104 UIFigure7`
- `0x0110 UICharacter`
- `0x0305 MapRobotPosition`

注意：

- `0x0201~0x0205` 这组就是当前把信息波处理端结果落地到项目内部结构中的核心结构。
- 注释里已经把它们和雷达 `0x0A01~0x0A05` 对应起来了。

## 子协议打包 API

文件：

- `include/radar/app/subReferee/referee_user.hpp`

### `Referee0x301Prepare()`

用途：

- 把任意一个子协议结构体打成主协议 `0x301` 帧，供发送到裁判系统/服务器端

自动完成：

- 帧头 `0xA5`
- `data_len`
- `seq`
- 头部 CRC8
- 主命令 `0x301`
- 子命令 `getCmd(info)`
- `sender`
- `receiver`
- 整帧 CRC16

适合发送：

- `EnemyRobotPosition`
- `EnemyRobotHP`
- `EnemyRobotProjectileAllowance`
- `EnemyGoldCoinRFID`
- `EnemyRobotBuff`
- `AllRadarInfo`
- UI 相关结构

### `RefereePrepare()`

用途：

- 打包 `MapRobotPosition`，供发送到裁判系统/服务器端

特点：

- 直接使用 `getCmd(structInfo)`，当前就是 `0x0305`
- 不走 `0x301` 子协议头

## 内存映射记忆点

主协议的自动落地依赖：

- `RefereeProtocolMemoryMap<revision>::map`

子协议的自动落地依赖：

- `RefereeSubProtocolMemoryMap::map`
- `RefereeSubProtocolMemoryMap::mapSize`

特别注意：

- `AllRadarInfo` 当前在 `RefereeSubProtocolMemoryMap::map` 中映射到 `enemy_robot_HP` 的起始偏移。
- 它的 `mapSize` 也是按“从 HP 开始到末尾的一整段”计算。
- 这不是普通单结构直接拷贝，而是依赖当前 `RefereeSubProtocol` 的布局。

后续如果改 `RefereeSubProtocol` 字段顺序，必须同步检查 `AllRadarInfo` 的语义是否仍然成立。

## 现有工程接线方式

文件：

- `include/radar/app/main.cc`

现有接法是：

1. 创建 `Referee<RefereeRevision::kNewV120>`
2. 创建 `RefereeUser<RefereeRevision::kNewV120>`
3. 用 `referee->AttachCallback(...)` 把 `RefereeUser::AttachCallback` 挂进去
4. 串口驱动把字节流送给 `Referee`

后续若雷达程序需要接入串口裁判系统，优先继续沿用这套接线方式。
若信息波处理端通过 TCP 或其他链路把处理结果送到雷达程序，也应尽量把结果转换到这些现有结构体后再进入发送链路。

## 对后续雷达通信的默认策略

### 项目主流程

默认按以下主流程理解系统：

1. 接收信息波处理端数据
2. 接收裁判系统/服务器端数据
3. 统一落到现有协议结构体
4. 进行状态处理、信息整合和决策
5. 打包后发送至裁判系统/服务器端

### 串口裁判系统

默认使用：

- `Referee<rm::device::RefereeRevision::kNewV120>`

### 0x301 用户子协议

默认使用：

- `RefereeUser<rm::device::RefereeRevision::kNewV120>`
- `Referee0x301Prepare()`

### 小地图位置数据

默认使用：

- `MapRobotPosition`
- `RefereePrepare()`

### CRC

默认使用：

- `rm::modules::Crc8(...)`
- `rm::modules::Crc16(...)`

### 信息波

当前理解分两层：

1. 官方主协议层
   - `kNewV120` 已引入 `0x0A01~0x0A06` 命令头常量
2. 项目落地层
   - 目前主要通过 `subReferee/protocol_user.hpp` 中的
     `EnemyRobotPosition / HP / ProjectileAllowance / GoldCoinRFID / Buff / AllRadarInfo`
     来承载信息波结果

因此，后续如无新的要求，默认优先复用现有 `subReferee` 结构体作为信息波处理端结果进入雷达程序后的对内数据结构，再通过现有打包 API 发往裁判系统/服务器端。

## 当前未完成点

- `referee.hpp` 现在已经显式包含 `protocol_new_v120.hpp`，因此 `Referee<kNewV120>` 的模板入口已经具备。
- `protocol_new_v120.hpp` 虽然加入了 `0x0A01~0x0A06` 命令头，但主结构体与内存映射还没补这些命令的数据落点。
- 如果后续需要让 `Referee<kNewV120>` 直接自动接住信息波帧，就要继续补主协议结构体字段与 `MemoryMap`。

## 一句话记忆

后续雷达通信默认策略：

- 串口主帧交给 `Referee<kNewV120>`
- `0x301` 子协议交给 `RefereeUser`
- 打包走 `Referee0x301Prepare` / `RefereePrepare`
- CRC 一律走 `rm::modules::Crc8/Crc16`
- 信息波处理端结果优先落到 `protocol_user.hpp` 已有结构体
- 最终结果按协议要求发送至裁判系统/服务器端
