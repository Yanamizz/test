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
  kMatch,  ///< 比赛模式：优先保留原始二进制证据链
};

/**
 * @brief 主程序输入源模式
 */
enum class RefereeInputSourceMode {
  kReal,  ///< 使用真实串口或 TCP 字节流
  kFile,  ///< 使用预制二进制文件回放字节流
};

/// 裁判系统协议版本唯一配置入口。
constexpr auto kRefereeRevision = rm::device::RefereeRevision::kNewV120;
/// 项目默认日志模式。
constexpr auto kRadarLogMode = RadarLogMode::kMatch;
/// 调试模式下允许缺失输入接口，缺失链路对应状态按 0 值处理。
constexpr bool kDebugAllowMissingInterfaces = kRadarLogMode == RadarLogMode::kDebug;
/// 裁判系统默认串口波特率。
constexpr int kDefaultRefereeBaud = 115200;
/// 常规链路优先尝试的串口设备。
constexpr const char *kDefaultRefereeDevice = "/dev/ttyUSB0";
/// 串口设备名大小写差异时使用的回退设备。
constexpr const char *kFallbackRefereeDevice = "/dev/ttyusb0";
/// 串口主协议输入模式。
constexpr auto kSerialRefereeInputMode = RefereeInputSourceMode::kReal;
/// 串口主协议回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kSerialRefereeReplayFile = "";
/// 串口主协议回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kSerialRefereeReplayRateHz = 100;
/// TCP 本机绑定地址；留空表示由系统按路由自动选择本机出口地址。
constexpr const char *kTcpLocalBindAddress = "";
/// TCP 对端服务端地址；`8001/8002/8003` 均主动连接到该地址。
constexpr const char *kTcpServerAddress = "192.168.50.112";
/// 信息波输入模式，仅覆盖 `8001`。
constexpr auto kInfoWaveInputMode = RefereeInputSourceMode::kReal;
/// 信息波 `8001` 回放文件路径，模式为 `kFile` 时必须非空。
constexpr const char *kInfoWaveReplayFile = "";
/// 信息波 `8001` 回放频率，单位 Hz；文件回放按完整协议帧节拍推进。
constexpr int kInfoWaveReplayRateHz = 100;
/// 信息波位置数据对端服务端端口。
constexpr int kInfoWaveTcpServerPort = 8001;
/// 敌方一级密钥对端服务端端口。
constexpr int kEnemyLevel1KeyTcpServerPort = 8002;
/// 敌方二级密钥对端服务端端口。
constexpr int kEnemyLevel2KeyTcpServerPort = 8003;
/// 串口掉线后的自动重连周期。
constexpr int kSerialReconnectIntervalMs = 500;
/// TCP 连接失败后的重试周期。
constexpr int kTcpReconnectIntervalMs = 1000;
/// 单次 TCP 连接尝试的超时时间。
constexpr int kTcpConnectTimeoutMs = 3000;
/// 信息波状态过期时间。
constexpr int kInfoWaveStateTimeoutMs = 500;
/// 常规链路状态过期时间。
constexpr int kRefereeStateTimeoutMs = 3000;
/// 信息波 TCP 客户端空闲断开时间；`<=0` 表示保持连接，不因空闲主动断开。
constexpr int kInfoWaveTcpIdleTimeoutMs = 0;
/// 敌方密钥 TCP 客户端空闲断开时间。
constexpr int kEnemyKeyTcpIdleTimeoutMs = 3000;
/// `0x0305` 最小发送间隔。
constexpr int kMapRobotMinSendIntervalMs = 200;
/// `password_cmd=2` 冷却时间。
constexpr int kRadarPasswordVerifyCooldownMs = 10000;
/// 运行指标刷新周期。
constexpr int kRuntimeMetricsFlushIntervalMs = 1000;
/// 主循环告警阈值。
constexpr int kRuntimeLoopWarnMs = 5;
/// 主循环长尾阈值。
constexpr int kRuntimeLoopTailMs = 20;
/// 主循环严重阻塞阈值。
constexpr int kRuntimeLoopCriticalMs = 100;
/// 日志队列平均占用的告警水位。
constexpr std::size_t kRuntimeLogQueueOccupancyWarnPercent = 50;
/// Best-effort 日志队列上限。
constexpr std::size_t kBestEffortLogQueueCapacity = 4096;
/// Critical 日志软上限，超过后优先挤出低优先级日志。
constexpr std::size_t kCriticalLogSoftQueueCapacity = 8192;
constexpr rm::u16 kDefaultRadarSenderId = 9;  // 9：红方雷达  109：蓝方雷达
/// 裁判系统服务器接收 ID。
constexpr rm::u16 kRefereeServerReceiverId = 0x8080;
/// 可修改己方密钥时按顺序使用的预置密钥。
constexpr std::array<std::array<rm::u8, 6>, 3> kRadarPresetAllyKeys{{
    {{'A', '5', '0', '6', '0', '0'}},
    {{'0', '6', '0', 'A', '0', '6'}},
    {{'Q', '0', 'D', '0', 'G', '6'}},
}};

}  // namespace radar::config

#endif  // RADAR_INCLUDE_CONFIG_CONFIG_HPP
