/**
 * @file  src/main.cc
 * @brief 雷达主程序入口
 */

#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/double_debuff_fallback.hpp"
#include "include/referee/external_server_sender.hpp"
#include "include/referee/map_robot_relay.hpp"
#include "include/referee/replay_input_source.hpp"
#include "include/referee/radar_decision_tree.hpp"
#include "include/referee/referee_input_loop.hpp"
#include "include/referee/referee_tx_scheduler.hpp"
#include "include/referee/serial_connection_log.hpp"
#include "include/referee/serial_port.hpp"
#include "include/referee/tcp_client.hpp"
#include "include/referee/tcp_connection_log.hpp"
#include "include/referee/tcp_server.hpp"
#include "include/referee/ui_user1_sender.hpp"
#include "librm/device/referee/referee.hpp"

namespace {

/// 主程序统一使用配置中的裁判系统协议版本。
constexpr auto kRevision = radar::config::kRefereeRevision;

using Referee = rm::device::Referee<kRevision>;

/// 全局运行标志，由信号处理函数控制退出。
std::atomic<bool> g_running{true};

/**
 * @brief 信号处理函数
 * @param 当前收到的信号值，逻辑中不需要区分
 */
void OnSignal(int) { g_running.store(false); }

}  // namespace

int main() {
  try {
    // 注册退出信号，保证主循环可以平滑结束。
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    // 初始化本轮日志目录，并尽量在启动阶段完成基础资源创建。
    const auto run_log_root = radar::log::CreateRunLogRoot();
    radar::referee::TcpConnectionLog tcp_log(run_log_root);

    radar::referee::SerialPort serial;
    std::string serial_open_error;
    if (!serial.TryOpenDefault(radar::config::kDefaultRefereeBaud, &serial_open_error)) {
      radar::referee::SerialConnectionLog(run_log_root).LogState(
          "open_failed", radar::referee::SelectRefereeDevice(), radar::config::kDefaultRefereeBaud,
          serial_open_error, false);
      radar::log::GetRuntimeMetrics(run_log_root).RecordSerialOpenFailure();
    }
    radar::referee::TcpClient info_wave_tcp;
    if (radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kReal) {
      radar::referee::TryOpenConfiguredTcpClient(&info_wave_tcp, "info_wave_tcp",
                                                 radar::config::kInfoWaveTcpServerPort, true, tcp_log, &std::cerr);
    }
    radar::referee::TcpClient enemy_level1_key_tcp;
    if (radar::config::kEnemyLevel1KeyInputMode == radar::config::RefereeInputSourceMode::kReal) {
      radar::referee::TryOpenConfiguredTcpClient(&enemy_level1_key_tcp, "enemy_level1_key_tcp",
                                                 radar::config::kEnemyLevel1KeyTcpServerPort, true, tcp_log,
                                                 &std::cerr);
    }
    radar::referee::TcpClient enemy_level2_key_tcp;
    if (radar::config::kEnemyLevel2KeyInputMode == radar::config::RefereeInputSourceMode::kReal) {
      radar::referee::TryOpenConfiguredTcpClient(&enemy_level2_key_tcp, "enemy_level2_key_tcp",
                                                 radar::config::kEnemyLevel2KeyTcpServerPort, true, tcp_log,
                                                 &std::cerr);
    }
    std::optional<radar::referee::TcpServer> external_tcp_server;
    if (radar::config::kExternalTcpServerEnabled) {
      external_tcp_server.emplace();
      radar::referee::TryOpenConfiguredTcpServer(&*external_tcp_server, "external_tcp_server",
                                                 radar::config::kExternalTcpServerPort, tcp_log, &std::cerr);
    }

    // 创建常规链路、信息波链路与发送链路各自的状态维护对象。
    Referee serial_referee;
    Referee info_wave_referee;
    radar::log::BinaryLogStore raw_log_store(run_log_root);
    radar::referee::RefereeTxScheduler tx_scheduler(serial, run_log_root);
    radar::referee::RadarCommandSender radar_command_sender(tx_scheduler, run_log_root);
    radar::referee::UiUser1Sender<kRevision> ui_user1_sender(radar_command_sender, tx_scheduler, run_log_root);
    radar::referee::RadarDecisionTree<kRevision> radar_decision_tree(radar_command_sender);
    radar::referee::DoubleDebuffFallback<kRevision> double_debuff_fallback(radar_command_sender, run_log_root);
    radar::referee::MapRobotRelay<kRevision> relay(tx_scheduler, run_log_root);
    radar::referee::ExternalServerSender external_server_sender(external_tcp_server ? &*external_tcp_server : nullptr,
                                                                run_log_root);
    radar::referee::EnemyKeyReceiver<kRevision> enemy_level1_key_receiver(
        "enemy_level1_key", radar::config::kEnemyLevel1KeyTcpServerPort, radar_command_sender, run_log_root);
    radar::referee::EnemyKeyReceiver<kRevision> enemy_level2_key_receiver(
        "enemy_level2_key", radar::config::kEnemyLevel2KeyTcpServerPort, radar_command_sender, run_log_root);

    std::optional<radar::referee::ReplayInputSource> serial_replay_source;
    std::optional<radar::referee::ReplayInputSource> info_wave_replay_source;
    std::optional<radar::referee::ReplayInputSource> enemy_level1_key_replay_source;
    std::optional<radar::referee::ReplayInputSource> enemy_level2_key_replay_source;
    if (radar::config::kSerialRefereeInputMode == radar::config::RefereeInputSourceMode::kFile) {
      serial_replay_source.emplace(radar::config::kSerialRefereeReplayFile, radar::config::kSerialRefereeReplayRateHz,
                                   "serial_referee_replay");
    }
    if (radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kFile) {
      info_wave_replay_source.emplace(radar::config::kInfoWaveReplayFile, radar::config::kInfoWaveReplayRateHz,
                                      "info_wave_replay");
    }
    if (radar::config::kEnemyLevel1KeyInputMode == radar::config::RefereeInputSourceMode::kFile) {
      enemy_level1_key_replay_source.emplace(radar::config::kEnemyLevel1KeyReplayFile,
                                             radar::config::kEnemyLevel1KeyReplayRateHz,
                                             "enemy_level1_key_replay");
    }
    if (radar::config::kEnemyLevel2KeyInputMode == radar::config::RefereeInputSourceMode::kFile) {
      enemy_level2_key_replay_source.emplace(radar::config::kEnemyLevel2KeyReplayFile,
                                             radar::config::kEnemyLevel2KeyReplayRateHz,
                                             "enemy_level2_key_replay");
    }

    radar::log::FileLogStore input_mode_log(run_log_root);
    {
      std::ostringstream oss;
      oss << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"serial_input_mode\":\""
          << (radar::config::kSerialRefereeInputMode == radar::config::RefereeInputSourceMode::kReal ? "real"
                                                                                                    : "file")
          << "\","
          << "\"serial_replay_file\":\"" << radar::config::kSerialRefereeReplayFile << "\","
          << "\"serial_replay_rate_hz\":" << radar::config::kSerialRefereeReplayRateHz << ','
          << "\"replay_loop\":" << (radar::config::kReplayLoop ? "true" : "false") << ','
          << "\"info_wave_input_mode\":\""
          << (radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kReal ? "real" : "file")
          << "\","
          << "\"enemy_level1_key_input_mode\":\""
          << (radar::config::kEnemyLevel1KeyInputMode == radar::config::RefereeInputSourceMode::kReal ? "real"
                                                                                                        : "file")
          << "\","
          << "\"enemy_level1_key_replay_file\":\"" << radar::config::kEnemyLevel1KeyReplayFile << "\","
          << "\"enemy_level1_key_replay_rate_hz\":" << radar::config::kEnemyLevel1KeyReplayRateHz << ','
          << "\"enemy_level2_key_input_mode\":\""
          << (radar::config::kEnemyLevel2KeyInputMode == radar::config::RefereeInputSourceMode::kReal ? "real"
                                                                                                        : "file")
          << "\","
          << "\"enemy_level2_key_replay_file\":\"" << radar::config::kEnemyLevel2KeyReplayFile << "\","
          << "\"enemy_level2_key_replay_rate_hz\":" << radar::config::kEnemyLevel2KeyReplayRateHz << ','
          << "\"tcp_role\":\""
          << (radar::config::kExternalTcpServerEnabled ? "mixed" : "client")
          << "\","
          << "\"tcp_server_address\":\"" << radar::config::kTcpServerAddress << "\","
          << "\"tcp_local_bind_address\":\"" << radar::config::kTcpLocalBindAddress << "\","
          << "\"external_tcp_server_enabled\":" << (radar::config::kExternalTcpServerEnabled ? "true" : "false")
          << ','
          << "\"external_tcp_server_bind_address\":\"" << radar::config::kExternalTcpServerBindAddress << "\","
          << "\"external_tcp_server_port\":" << radar::config::kExternalTcpServerPort << ','
          << "\"info_wave_replay_file\":\"" << radar::config::kInfoWaveReplayFile << "\","
          << "\"info_wave_replay_rate_hz\":" << radar::config::kInfoWaveReplayRateHz << "}";
      input_mode_log.Append("main/input_mode.log", oss.str(), radar::log::LogPriority::kCriticalDecision);
    }

    // 串口主协议回调：维护常规链路结构体，并驱动自主决策树。
    serial_referee.AttachCallback([&](rm::u16 cmd_id, rm::u8 seq) {
      radar_command_sender.UpdateAllySideFromRobotId(serial_referee.data().robot_status.robot_id);
      ui_user1_sender.ProcessSerial(cmd_id, serial_referee.data());
      relay.ProcessSerial(cmd_id, seq, serial_referee);
      radar_decision_tree.ProcessSerial(cmd_id, seq, serial_referee);
      double_debuff_fallback.ProcessSerial(cmd_id, serial_referee.data());
      external_server_sender.ProcessSerial(cmd_id, seq, serial_referee.data());
    });
    // 信息波回调：维护 `0x0A01~0x0A06` 状态，并结合串口 `0x0301/0x0200` 驱动 `0x0305` 组包链路。
    info_wave_referee.AttachCallback([&](rm::u16 cmd_id, rm::u8 seq) {
      ui_user1_sender.ProcessInfoWave(cmd_id, info_wave_referee.data());
      relay.ProcessInfoWaveTcp(cmd_id, seq, info_wave_referee);
    });

    // 主循环统一负责接收、处理、发送与自动重连。
    radar::referee::RunSerialInfoWaveAndKeyTcpLoop(serial, info_wave_tcp, enemy_level1_key_tcp,
                                                   enemy_level2_key_tcp, serial_referee, info_wave_referee,
                                                   serial_replay_source ? &*serial_replay_source : nullptr,
                                                   info_wave_replay_source ? &*info_wave_replay_source : nullptr,
                                                   enemy_level1_key_replay_source ? &*enemy_level1_key_replay_source
                                                                                  : nullptr,
                                                   enemy_level2_key_replay_source ? &*enemy_level2_key_replay_source
                                                                                  : nullptr,
                                                   enemy_level1_key_receiver, enemy_level2_key_receiver,
                                                   radar_command_sender, ui_user1_sender, relay, external_server_sender,
                                                   double_debuff_fallback, tx_scheduler,
                                                   raw_log_store, external_tcp_server ? &*external_tcp_server : nullptr,
                                                   g_running);

    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
