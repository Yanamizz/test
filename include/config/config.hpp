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

/// 裁判系统协议版本唯一配置入口。
constexpr auto kRefereeRevision = rm::device::RefereeRevision::kNewV200;
/// 项目默认日志模式。
constexpr auto kRadarLogMode = RadarLogMode::kDebug;
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
constexpr const char *kSerialRefereeReplayFile = "/home/hanni/Radar/test/info/5.30WMJvsIROBOT/raw_serial.bin";
/// 串口主协议回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kSerialRefereeReplayRateHz = 10;
/// TCP 本机绑定地址；留空表示由系统按路由自动选择本机出口地址。
constexpr const char *kTcpLocalBindAddress = "";
/// TCP 对端服务端地址；`8001/8002/8003` 均主动连接到该地址。
constexpr const char *kTcpServerAddress = "192.168.50.112";
/// 是否启用额外的 TCP server 通道，供其他设备主动接入本程序。
constexpr bool kExternalTcpServerEnabled = true;
/// 额外 TCP server 的监听地址；`0.0.0.0` 表示监听所有网卡。
constexpr const char *kExternalTcpServerBindAddress = "192.168.50.75";
/// 额外 TCP server 的监听端口。
constexpr int kExternalTcpServerPort = 9001;
/// 信息波输入模式，仅覆盖 `8001`。
constexpr auto kInfoWaveInputMode = RefereeInputSourceMode::kReal;
/// 信息波 `8001` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kInfoWaveReplayFile = "/home/hanni/Radar/test/info/outputInfo.bin";
/// 信息波 `8001` 回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kInfoWaveReplayRateHz = 50;
/// 信息波位置数据对端服务端端口。
constexpr int kInfoWaveTcpServerPort = 8001;
/// 敌方一级密钥输入模式，仅覆盖 `8002`。
constexpr auto kEnemyLevel1KeyInputMode = RefereeInputSourceMode::kReal;
/// 敌方一级密钥 `8002` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kEnemyLevel1KeyReplayFile = "/home/hanni/Radar/test/info/5.30WMJvsIROBOT/raw_tcp_8002.bin";
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
/// `0x0305` 最小发送间隔。
constexpr int kMapRobotMinSendIntervalMs = 200;
/// 可修改己方密钥时按顺序使用的预置密钥。
constexpr std::array<std::array<rm::u8, 6>, 3> kRadarPresetAllyKeys{{
    {{'A', '5', '0', '6', '0', '0'}},
    {{'0', '6', '0', 'A', '0', '6'}},
    {{'Q', '0', 'D', '0', 'G', '6'}},
}};
}  // namespace radar::config

#endif  // RADAR_INCLUDE_CONFIG_CONFIG_HPP
