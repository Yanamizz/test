#ifndef RADAR_INCLUDE_CONFIG_CONFIG_HPP
#define RADAR_INCLUDE_CONFIG_CONFIG_HPP

/**
 * @file  include/config/config.hpp
 * @brief 雷达项目集中配置
 */

#include <array>
#include <cstddef>

#include "librm/device/referee/protocol.hpp"

namespace radar::config {

/**
 * @brief 日志运行模式
 */
enum class RadarLogMode {
  kDebug,  ///< 调试模式：保留更多结构化日志和快照
  kMatch,  ///< 比赛模式：只保留原始字节流和通信状态日志
};

/**
 * @brief 主程序输入源模式
 */
enum class RefereeInputSourceMode {
  kReal,  ///< 使用真实串口或 TCP 字节流
  kFile,  ///< 使用预制二进制文件回放字节流
};

/// 文件回放是否到达末尾后从第一帧循环播放。
constexpr bool kReplayLoop = true;

/// 裁判系统协议版本唯一配置入口。
constexpr auto kRefereeRevision = rm::device::RefereeRevision::kNewV200;
/// 项目默认日志模式。
constexpr auto kRadarLogMode = RadarLogMode::kMatch;
/// 调试模式下允许缺失输入接口，缺失链路对应状态按 0 值处理。
constexpr bool kDebugAllowMissingInterfaces = kRadarLogMode == RadarLogMode::kDebug;
/// 是否保留逐帧主协议结构体快照日志；比赛模式下关闭以减轻主线程格式化压力。
constexpr bool kEnableMainProtocolStructLog = kRadarLogMode == RadarLogMode::kDebug;
/// 裁判系统默认串口波特率。
constexpr int kDefaultRefereeBaud = 115200;
/// 常规链路优先尝试的串口设备。
constexpr const char *kDefaultRefereeDevice = "/dev/ttyUSB0";
/// 串口设备名大小写差异时使用的回退设备。
constexpr const char *kFallbackRefereeDevice = "/dev/ttyusb0";
/// 串口主协议输入模式。
constexpr auto kSerialRefereeInputMode = RefereeInputSourceMode::kReal;
/// 串口主协议回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kSerialRefereeReplayFile =
    "/home/hanni/Radar/test/logs/20260731_191242_641/raw/serial_referee_rx.bin";
/// 串口主协议回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kSerialRefereeReplayRateHz = 10;
/// TCP 本机绑定地址；留空表示由系统按路由自动选择本机出口地址。
constexpr const char *kTcpLocalBindAddress = "";
/// TCP 对端服务端地址；`8001/8002/8003` 均主动连接到该地址。
constexpr const char *kTcpServerAddress = "192.168.12.99";
/// 是否启用额外的 TCP server 通道，供其他设备主动接入本程序。
constexpr bool kExternalTcpServerEnabled = true;
/// 额外 TCP server 的监听地址；`0.0.0.0` 表示监听所有网卡。
constexpr const char *kExternalTcpServerBindAddress = "0.0.0.0";
/// 额外 TCP server 的监听端口。
constexpr int kExternalTcpServerPort = 9001;
/// 信息波输入模式，仅覆盖 `8001`。
constexpr auto kInfoWaveInputMode = RefereeInputSourceMode::kReal;
/// 信息波 `8001` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kInfoWaveReplayFile =
    "/home/hanni/Radar/test/logs/20260731_191242_641/raw/tcp_8001_info_wave_rx.bin";
/// 信息波 `8001` 回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kInfoWaveReplayRateHz = 50;
/// 信息波位置数据对端服务端端口。
constexpr int kInfoWaveTcpServerPort = 8001;
/// 敌方一级密钥输入模式，仅覆盖 `8002`。
constexpr auto kEnemyLevel1KeyInputMode = RefereeInputSourceMode::kReal;
/// 敌方一级密钥 `8002` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kEnemyLevel1KeyReplayFile =
    "/home/hanni/Radar/test/logs/20260731_191242_641/raw/tcp_8002_enemy_level1_key_rx.bin";
/// 敌方一级密钥 `8002` 回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kEnemyLevel1KeyReplayRateHz = 10;
/// 敌方一级密钥对端服务端端口。
constexpr int kEnemyLevel1KeyTcpServerPort = 8002;
/// 敌方二级密钥输入模式，仅覆盖 `8003`。
constexpr auto kEnemyLevel2KeyInputMode = RefereeInputSourceMode::kReal;
/// 敌方二级密钥 `8003` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kEnemyLevel2KeyReplayFile = "/home/hanni/Radar/test/info/5.30WMJvsIROBOT/raw_tcp_8003.bin";
/// 敌方二级密钥 `8003` 回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kEnemyLevel2KeyReplayRateHz = 10;
/// 敌方二级密钥对端服务端端口。
constexpr int kEnemyLevel2KeyTcpServerPort = 8003;
/// 敌方密钥 TCP 模拟器监听地址；`0.0.0.0` 表示监听所有网卡。
constexpr const char *kEnemyKeySimulatorBindAddress = "0.0.0.0";
/// `0x0305` 最小发送间隔。
constexpr int kMapRobotMinSendIntervalMs = 200;
/// 临时联调：固定 `0x0305` 全部己方机器人坐标，验证选手端地图显示链路。
constexpr bool kMapRobotUseFixedAllyRobotPositions = false;
/// 临时固定的己方机器人坐标，顺序为英雄、工程、3 号步兵、4 号步兵、空中、哨兵，单位 cm。
constexpr std::array<std::array<rm::u16, 2>, 6> kMapRobotFixedAllyRobotPositionsCm{{
    {{300, 300}},
    {{500, 500}},
    {{700, 700}},
    {{900, 900}},
    {{1100, 1100}},
    {{1300, 1300}},
}};
/// 临时联调：仅固定 `0x0305` 己方哨兵坐标，兼容单字段实验。
constexpr bool kMapRobotUseFixedAllySentryPosition = false;
/// 临时固定的己方哨兵 X 坐标，单位 cm。
constexpr rm::u16 kMapRobotFixedAllySentryPositionX = 1000;
/// 临时固定的己方哨兵 Y 坐标，单位 cm。
constexpr rm::u16 kMapRobotFixedAllySentryPositionY = 1000;
/// 双倍易伤保底机制总开关。
constexpr bool kDoubleDebuffFallbackEnabled = true;
/// 保底第一阶段阈值：比赛中剩余时间降到该值仍未触发双倍易伤时，请求第一次（`radar_cmd` 目标 1）。
constexpr int kDoubleDebuffFallbackStage1RemainSec = 270;
/// 保底第二阶段阈值：剩余时间降到该值时请求第二次（`radar_cmd` 目标 2）。
constexpr int kDoubleDebuffFallbackStage2RemainSec = 180;
/// 保底机制重发间隔，用于对抗单帧丢失；实际仍受调度器 `0x0301` 34ms 间隔约束。
constexpr int kDoubleDebuffFallbackResendIntervalMs = 1000;
/// 两次 `radar_cmd` 自增之间的最小间隔，确保裁判端能分别识别两次请求。
constexpr int kDoubleDebuffFallbackMinIncrementIntervalMs = 1000;
/// 观测到敌方进入双倍易伤（`0x020E` bit2 上升沿）后是否停止本阶段重发。
constexpr bool kDoubleDebuffFallbackStopOnConfirm = true;
/// `0x0001` 状态超过该时限视为过期，保底机制不再依据其时间判断。
constexpr int kGameStatusStaleTimeoutMs = 3000;
/// 是否只在 `0x0001` 阶段为“比赛中”时才提交敌方密钥（`password_cmd=2`）。
/// @note 赛前提交不会被裁判系统采纳，却会消耗掉本地密钥候选，因此默认开启。
///       文件回放或缺失 `0x0001` 的联调场景可临时关掉。
constexpr bool kOpponentKeyRequireMatchRunning = true;
/// 提交敌方密钥后，未观测到干扰波等级上升时是否按冷却周期重投。
/// @note 关掉后退回“发一次就丢弃”的旧行为。
constexpr bool kOpponentKeyRetryUntilLevelUp = true;
/// 可修改己方密钥时按顺序使用的预置密钥。
constexpr std::array<std::array<rm::u8, 6>, 3> kRadarPresetAllyKeys{{
    {{'A', '5', '0', '6', '0', '0'}},
    {{'0', '6', '0', 'A', '0', '6'}},
    {{'Q', '0', 'D', '0', 'G', '6'}},
}};
}  // namespace radar::config

#endif  // RADAR_INCLUDE_CONFIG_CONFIG_HPP
