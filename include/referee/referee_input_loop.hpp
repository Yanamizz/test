#ifndef RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP
#define RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP

/**
 * @file  include/referee/referee_input_loop.hpp
 * @brief 主程序收发事件循环
 */

#include <array>
#include <atomic>
#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <poll.h>

#include "include/log/referee_main_log.hpp"
#include "include/referee/enemy_key_receiver.hpp"
#include "include/referee/replay_input_source.hpp"
#include "include/referee/serial_connection_log.hpp"
#include "include/referee/referee_tx_scheduler.hpp"
#include "include/referee/serial_port.hpp"
#include "include/referee/tcp_client.hpp"
#include "include/referee/tcp_connection_log.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

/**
 * @brief 将一段字节流逐字节喂给裁判协议解包器
 * @tparam Referee 解包器类型
 * @param referee 目标解包器
 * @param data 字节流首地址
 * @param size 字节数
 */
template <typename Referee>
void FeedBytes(Referee &referee, const rm::u8 *data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    referee << data[i];
  }
}

/**
 * @brief 完整主循环：串口接收 + 信息波 TCP 客户端 + 敌方密钥 TCP 客户端 + 统一发送调度
 * @tparam SerialReferee 常规链路解包器类型
 * @tparam InfoWaveReferee 信息波解包器类型
 * @tparam EnemyKeyReceiverT 敌方密钥接收器类型
 * @tparam RadarCommandSenderT 雷达指令发送器类型
 * @tparam MapRobotRelayT `0x0305` relay 类型
 */
template <typename SerialReferee, typename InfoWaveReferee, typename EnemyKeyReceiverT, typename RadarCommandSenderT,
          typename MapRobotRelayT>
void RunSerialInfoWaveAndKeyTcpLoop(SerialPort &serial, TcpClient &info_wave_tcp, TcpClient &enemy_level1_key_tcp,
                                    TcpClient &enemy_level2_key_tcp, SerialReferee &serial_referee,
                                    InfoWaveReferee &info_wave_referee,
                                    ReplayInputSource *serial_replay,
                                    ReplayInputSource *info_wave_replay,
                                    EnemyKeyReceiverT &enemy_level1_key_receiver,
                                    EnemyKeyReceiverT &enemy_level2_key_receiver,
                                    RadarCommandSenderT &radar_command_sender,
                                    MapRobotRelayT &map_robot_relay,
                                    RefereeTxScheduler &tx_scheduler,
                                    radar::log::BinaryLogStore &raw_log_store,
                                    const std::atomic<bool> &running) {
  if ((serial_replay == nullptr &&
       radar::config::kSerialRefereeInputMode == radar::config::RefereeInputSourceMode::kFile) ||
      (info_wave_replay == nullptr &&
       radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kFile)) {
    throw std::runtime_error("replay input source is required but missing");
  }

  const bool serial_input_is_file =
      radar::config::kSerialRefereeInputMode == radar::config::RefereeInputSourceMode::kFile;
  const bool info_wave_input_is_file =
      radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kFile;
  const auto drain_duration = std::chrono::milliseconds(
      std::max(radar::config::kRefereeStateTimeoutMs, radar::config::kInfoWaveStateTimeoutMs) + 1000);

  std::array<rm::u8, 512> read_buffer{};
  auto &metrics = radar::log::GetRuntimeMetrics(raw_log_store.root());
  SerialConnectionLog serial_log(raw_log_store.root());
  TcpConnectionLog tcp_log(raw_log_store.root());
  using LoopClock = std::chrono::steady_clock;
  std::optional<LoopClock::time_point> last_serial_open_attempt_time;
  bool serial_has_connected_once = serial.is_open();
  bool serial_retry_state_logged = false;
  std::optional<LoopClock::time_point> last_info_wave_connect_attempt_time;
  std::optional<LoopClock::time_point> last_level1_connect_attempt_time;
  std::optional<LoopClock::time_point> last_level2_connect_attempt_time;
  bool info_wave_connect_retry_state_logged = false;
  bool level1_connect_retry_state_logged = false;
  bool level2_connect_retry_state_logged = false;

  if (serial.is_open()) {
    serial_log.LogState("connected", serial.device(), serial.baud(), "serial port ready", true);
  }

  /**
   * @brief 返回当前串口波特率
   * @return 已打开串口时返回当前波特率，否则返回默认配置值
   */
  const auto serial_baud = [&]() -> int {
    return serial.baud() != 0 ? serial.baud() : radar::config::kDefaultRefereeBaud;
  };

  /**
   * @brief 返回当前串口设备名
   * @return 已打开串口时返回当前设备名，否则返回默认选择结果
   */
  const auto serial_device = [&]() -> std::string {
    if (!serial.device().empty()) {
      return serial.device();
    }
    return SelectRefereeDevice();
  };

  /**
   * @brief 将串口标记为断开状态并写日志
   * @param detail 断开原因
   */
  const auto mark_serial_disconnected = [&](const std::string &detail) {
    const bool was_open = serial.is_open();
    if (was_open) {
      serial.Close();
    }
    metrics.RecordSerialDisconnect();
    serial_log.LogState("disconnected", serial_device(), serial_baud(), detail, false);
    serial_retry_state_logged = false;
  };

  /**
   * @brief 在需要时尝试自动重连串口
   */
  const auto try_reconnect_serial = [&]() {
    if (serial.is_open()) {
      return;
    }

    const auto now = LoopClock::now();
    if (serial_has_connected_once && !serial_retry_state_logged) {
      serial_log.LogState("reconnecting", serial_device(), serial_baud(), "waiting for next retry", false);
      serial_retry_state_logged = true;
    }
    if (last_serial_open_attempt_time.has_value() &&
        now - *last_serial_open_attempt_time <
            std::chrono::milliseconds(radar::config::kSerialReconnectIntervalMs)) {
      return;
    }

    last_serial_open_attempt_time = now;
    std::string error;
    if (serial.TryOpenDefault(serial_baud(), &error)) {
      if (serial_has_connected_once) {
        metrics.RecordSerialReconnect();
        serial_log.LogState("reconnected", serial.device(), serial.baud(), "serial port reopened", true);
      } else {
        serial_log.LogState("connected", serial.device(), serial.baud(), "serial port opened", true);
      }
      serial_has_connected_once = true;
      serial_retry_state_logged = false;
      return;
    }

    metrics.RecordSerialOpenFailure();
    serial_log.LogState("open_failed", serial_device(), serial_baud(), error, false);
  };

  /**
   * @brief 在需要时尝试重新发起 TCP 客户端连接
   */
  const auto try_reconnect_tcp_client = [&](TcpClient &client, const std::string &name, int port,
                                            bool enable, bool allow_debug_disable,
                                            std::optional<LoopClock::time_point> *last_attempt_time,
                                            bool *retry_state_logged) {
    if (!enable || client.is_open()) {
      return;
    }

    const auto now = LoopClock::now();
    if (!*retry_state_logged) {
      tcp_log.LogListenerState(name, radar::config::kTcpLocalBindAddress, port, "reconnecting",
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
      const std::string detail = std::string("remote=") + radar::config::kTcpServerAddress;
      tcp_log.LogListenerState(name, radar::config::kTcpLocalBindAddress, port,
                               client.is_connected() ? "connected" : "connecting", detail);
      tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port,
                             client.is_connected() ? client.peer_ip() : radar::config::kTcpServerAddress,
                             client.is_connected() ? "connected" : "connecting", detail);
      *retry_state_logged = false;
      return;
    }

    tcp_log.LogListenerState(name, radar::config::kTcpLocalBindAddress, port, "connect_failed", error);
    if (!allow_debug_disable || !radar::config::kDebugAllowMissingInterfaces) {
      return;
    }
    tcp_log.LogListenerState(name, radar::config::kTcpLocalBindAddress, port, "disabled", "debug_allow_missing");
  };

  /**
   * @brief 推进周期性任务
   * @param loop_start 本轮事件循环开始时间
   * @note  负责 idle timeout、`0x0305` 周期处理、`0x0121` 待发队列与运行指标刷新。
   */
  const auto service_periodic_tasks = [&](const LoopClock::time_point &loop_start) {
    const auto log_connect_timeout = [&](TcpClient &client, const std::string &name) {
      const auto port = client.port();
      if (client.CloseIfConnectTimedOut(radar::config::kTcpConnectTimeoutMs)) {
        tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port, radar::config::kTcpServerAddress,
                               "disconnected", "connect_attempt_timed_out");
      }
    };
    if (!info_wave_input_is_file) {
      log_connect_timeout(info_wave_tcp, "info_wave_tcp");
    }
    if (!enemy_level1_key_receiver.completed()) {
      log_connect_timeout(enemy_level1_key_tcp, "enemy_level1_key_tcp");
    }
    if (!enemy_level2_key_receiver.completed()) {
      log_connect_timeout(enemy_level2_key_tcp, "enemy_level2_key_tcp");
    }
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

  /**
   * @brief 判断文件模式输入是否均已完成
   * @return 所有文件输入均完成时返回 true
   */
  const auto all_file_inputs_completed = [&]() -> bool {
    const bool serial_done = !serial_input_is_file || (serial_replay != nullptr && serial_replay->completed());
    const bool info_wave_done =
        !info_wave_input_is_file || (info_wave_replay != nullptr && info_wave_replay->completed());
    return serial_done && info_wave_done;
  };

  /**
   * @brief 计算本轮 poll 的超时时间
   * @param now 当前时间
   * @return 超时毫秒数
   */
  const auto compute_poll_timeout_ms = [&](const LoopClock::time_point &now) -> int {
    int timeout_ms = 100;
    const auto apply_replay_deadline = [&](const ReplayInputSource *source) {
      if (source == nullptr || source->completed()) {
        return;
      }
      const auto wait_ms = source->TimeUntilNextTickMs(now);
      if (wait_ms.has_value()) {
        timeout_ms = std::min(timeout_ms, *wait_ms);
      }
    };

    apply_replay_deadline(serial_replay);
    apply_replay_deadline(info_wave_replay);
    return std::max(0, timeout_ms);
  };

  std::optional<LoopClock::time_point> drain_deadline;

  while (running.load()) {
    const auto loop_start = std::chrono::steady_clock::now();
    try_reconnect_serial();
    try_reconnect_tcp_client(info_wave_tcp, "info_wave_tcp", radar::config::kInfoWaveTcpListenPort,
                             !info_wave_input_is_file, true, &last_info_wave_connect_attempt_time,
                             &info_wave_connect_retry_state_logged);
    try_reconnect_tcp_client(enemy_level1_key_tcp, "enemy_level1_key_tcp",
                             radar::config::kEnemyLevel1KeyTcpListenPort, !enemy_level1_key_receiver.completed(), true,
                             &last_level1_connect_attempt_time, &level1_connect_retry_state_logged);
    try_reconnect_tcp_client(enemy_level2_key_tcp, "enemy_level2_key_tcp",
                             radar::config::kEnemyLevel2KeyTcpListenPort, !enemy_level2_key_receiver.completed(), true,
                             &last_level2_connect_attempt_time, &level2_connect_retry_state_logged);
    const auto loop_now = LoopClock::now();

    const auto process_serial_replay = [&](const LoopClock::time_point &now) {
      if (serial_input_is_file && serial_replay != nullptr) {
        return serial_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
          raw_log_store.Append("raw/file_serial_referee_rx.bin", bytes, size);
          FeedBytes(serial_referee, bytes, size);
        });
      }
      return false;
    };

    const auto process_info_wave_replay = [&](const LoopClock::time_point &now) {
      if (info_wave_input_is_file && info_wave_replay != nullptr) {
        return info_wave_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
          raw_log_store.Append("raw/file_tcp_8001_info_wave_rx.bin", bytes, size);
          FeedBytes(info_wave_referee, bytes, size);
        });
      }
      return false;
    };

    (void)process_serial_replay(loop_now);
    (void)process_info_wave_replay(loop_now);

    if (serial_input_is_file && info_wave_input_is_file && all_file_inputs_completed() &&
        !drain_deadline.has_value()) {
      drain_deadline = loop_now + drain_duration;
    }
    if (drain_deadline.has_value() && loop_now >= *drain_deadline) {
      service_periodic_tasks(loop_start);
      break;
    }

    std::array<pollfd, 8> fds{};
    nfds_t nfds = 0;

    std::optional<nfds_t> serial_index;
    if (!serial_input_is_file && serial.is_open()) {
      serial_index = nfds;
      fds[nfds++] = pollfd{serial.fd(), static_cast<short>(POLLIN | POLLHUP | POLLERR), 0};
    }

    std::optional<nfds_t> info_wave_index;
    if (!info_wave_input_is_file && info_wave_tcp.is_open()) {
      info_wave_index = nfds;
      short events = static_cast<short>(POLLHUP | POLLERR);
      if (info_wave_tcp.is_connecting()) {
        events = static_cast<short>(events | POLLOUT);
      }
      if (info_wave_tcp.is_connected()) {
        events = static_cast<short>(events | POLLIN);
      }
      fds[nfds++] = pollfd{info_wave_tcp.fd(), events, 0};
    }
    std::optional<nfds_t> level1_index;
    const bool level1_active = !enemy_level1_key_receiver.completed() && enemy_level1_key_tcp.is_open();
    if (level1_active) {
      level1_index = nfds;
      short events = static_cast<short>(POLLHUP | POLLERR);
      if (enemy_level1_key_tcp.is_connecting()) {
        events = static_cast<short>(events | POLLOUT);
      }
      if (enemy_level1_key_tcp.is_connected()) {
        events = static_cast<short>(events | POLLIN);
      }
      fds[nfds++] = pollfd{enemy_level1_key_tcp.fd(), events, 0};
    }

    std::optional<nfds_t> level2_index;
    const bool level2_active = !enemy_level2_key_receiver.completed() && enemy_level2_key_tcp.is_open();
    if (level2_active) {
      level2_index = nfds;
      short events = static_cast<short>(POLLHUP | POLLERR);
      if (enemy_level2_key_tcp.is_connecting()) {
        events = static_cast<short>(events | POLLOUT);
      }
      if (enemy_level2_key_tcp.is_connected()) {
        events = static_cast<short>(events | POLLIN);
      }
      fds[nfds++] = pollfd{enemy_level2_key_tcp.fd(), events, 0};
    }

    const int result = ::poll(fds.data(), nfds, compute_poll_timeout_ms(loop_now));
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
    }
    if (result == 0) {
      service_periodic_tasks(loop_start);
      if (drain_deadline.has_value() && loop_now >= *drain_deadline && all_file_inputs_completed()) {
        break;
      }
      continue;
    }

    const auto after_poll_now = LoopClock::now();

    if (serial_index.has_value()) {
      const auto revents = fds[*serial_index].revents;
      if ((revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        mark_serial_disconnected("serial poll reported hangup or error");
      } else if ((revents & POLLIN) != 0) {
        std::size_t bytes_read = 0;
        std::string error;
        if (!serial.TryRead(read_buffer.data(), read_buffer.size(), &bytes_read, &error)) {
          mark_serial_disconnected(error);
        } else {
          metrics.RecordSerialRead(bytes_read);
          raw_log_store.Append("raw/serial_referee_rx.bin", read_buffer.data(), bytes_read);
          FeedBytes(serial_referee, read_buffer.data(), bytes_read);
        }
      }
    }

    (void)process_serial_replay(after_poll_now);
    (void)process_info_wave_replay(after_poll_now);

    if (info_wave_index.has_value()) {
      const auto revents = fds[*info_wave_index].revents;
      if ((revents & POLLOUT) != 0 && info_wave_tcp.is_connecting()) {
        std::string error;
        if (info_wave_tcp.FinishConnect(&error)) {
          tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress, info_wave_tcp.port(),
                                 info_wave_tcp.peer_ip(), "connected", "connect_completed");
        } else {
          tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress,
                                 radar::config::kInfoWaveTcpListenPort, radar::config::kTcpServerAddress,
                                 "disconnected", error);
        }
      }
      if ((revents & POLLIN) != 0 && info_wave_tcp.is_connected()) {
        const auto bytes_read = info_wave_tcp.Read(read_buffer.data(), read_buffer.size());
        if (bytes_read != 0) {
          raw_log_store.Append("raw/tcp_8001_info_wave_rx.bin", read_buffer.data(), bytes_read);
          FeedBytes(info_wave_referee, read_buffer.data(), bytes_read);
        }
      }
      if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        const auto port = info_wave_tcp.port();
        const std::string peer = info_wave_tcp.peer_ip().empty() ? radar::config::kTcpServerAddress
                                                                  : info_wave_tcp.peer_ip();
        tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress, port, peer, "disconnected",
                               "poll reported hangup or error");
        info_wave_tcp.Close();
      }
    }

    if (level1_index.has_value()) {
      const auto revents = fds[*level1_index].revents;
      if ((revents & POLLOUT) != 0 && enemy_level1_key_tcp.is_connecting()) {
        std::string error;
        if (enemy_level1_key_tcp.FinishConnect(&error)) {
          tcp_log.LogClientState("enemy_level1_key_tcp", radar::config::kTcpLocalBindAddress,
                                 enemy_level1_key_tcp.port(), enemy_level1_key_tcp.peer_ip(), "connected",
                                 "connect_completed");
        } else {
          tcp_log.LogClientState("enemy_level1_key_tcp", radar::config::kTcpLocalBindAddress,
                                 radar::config::kEnemyLevel1KeyTcpListenPort, radar::config::kTcpServerAddress,
                                 "disconnected", error);
        }
      }
      if ((revents & POLLIN) != 0 && enemy_level1_key_tcp.is_connected()) {
        const auto bytes_read = enemy_level1_key_tcp.Read(read_buffer.data(), read_buffer.size());
        if (bytes_read != 0) {
          raw_log_store.Append("raw/tcp_8002_enemy_level1_key_rx.bin", read_buffer.data(), bytes_read);
        }
        if (enemy_level1_key_receiver.ProcessBytes(read_buffer.data(), bytes_read, serial_referee.data())) {
          enemy_level1_key_tcp.Stop();
        }
      }
      if (!enemy_level1_key_receiver.completed() && (revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        const auto port = enemy_level1_key_tcp.port();
        const std::string peer = enemy_level1_key_tcp.peer_ip().empty() ? radar::config::kTcpServerAddress
                                                                         : enemy_level1_key_tcp.peer_ip();
        tcp_log.LogClientState("enemy_level1_key_tcp", radar::config::kTcpLocalBindAddress, port, peer,
                               "disconnected", "poll reported hangup or error");
        enemy_level1_key_tcp.Close();
      }
    }

    if (level2_index.has_value()) {
      const auto revents = fds[*level2_index].revents;
      if ((revents & POLLOUT) != 0 && enemy_level2_key_tcp.is_connecting()) {
        std::string error;
        if (enemy_level2_key_tcp.FinishConnect(&error)) {
          tcp_log.LogClientState("enemy_level2_key_tcp", radar::config::kTcpLocalBindAddress,
                                 enemy_level2_key_tcp.port(), enemy_level2_key_tcp.peer_ip(), "connected",
                                 "connect_completed");
        } else {
          tcp_log.LogClientState("enemy_level2_key_tcp", radar::config::kTcpLocalBindAddress,
                                 radar::config::kEnemyLevel2KeyTcpListenPort, radar::config::kTcpServerAddress,
                                 "disconnected", error);
        }
      }
      if ((revents & POLLIN) != 0 && enemy_level2_key_tcp.is_connected()) {
        const auto bytes_read = enemy_level2_key_tcp.Read(read_buffer.data(), read_buffer.size());
        if (bytes_read != 0) {
          raw_log_store.Append("raw/tcp_8003_enemy_level2_key_rx.bin", read_buffer.data(), bytes_read);
        }
        if (enemy_level2_key_receiver.ProcessBytes(read_buffer.data(), bytes_read, serial_referee.data())) {
          enemy_level2_key_tcp.Stop();
        }
      }
      if (!enemy_level2_key_receiver.completed() && (revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        const auto port = enemy_level2_key_tcp.port();
        const std::string peer = enemy_level2_key_tcp.peer_ip().empty() ? radar::config::kTcpServerAddress
                                                                         : enemy_level2_key_tcp.peer_ip();
        tcp_log.LogClientState("enemy_level2_key_tcp", radar::config::kTcpLocalBindAddress, port, peer,
                               "disconnected", "poll reported hangup or error");
        enemy_level2_key_tcp.Close();
      }
    }

    service_periodic_tasks(loop_start);

    if (drain_deadline.has_value() && after_poll_now >= *drain_deadline && all_file_inputs_completed()) {
      break;
    }
  }
}

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP
