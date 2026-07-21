#ifndef RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP
#define RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP

/**
 * @file  include/referee/referee_input_loop.hpp
 * @brief 主程序收发事件循环
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include <poll.h>

#include "include/log/referee_main_log.hpp"
#include "include/referee/enemy_key_receiver.hpp"
#include "include/referee/replay_input_source.hpp"
#include "include/referee/referee_tx_scheduler.hpp"
#include "include/referee/serial_connection_log.hpp"
#include "include/referee/serial_port.hpp"
#include "include/referee/tcp_client.hpp"
#include "include/referee/tcp_connection_log.hpp"
#include "include/referee/tcp_server.hpp"
#include "include/referee/ui_user1_sender.hpp"
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

namespace detail {

inline constexpr int kSerialReconnectIntervalMs = 500;
inline constexpr int kTcpReconnectIntervalMs = 1000;
inline constexpr int kTcpConnectTimeoutMs = 3000;
inline constexpr int kInfoWaveTcpIdleTimeoutMs = 0;
inline constexpr int kEnemyKeyTcpIdleTimeoutMs = 0;
inline constexpr int kExternalTcpServerIdleTimeoutMs = 0;

struct RefereeInputLoopRuntime {
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t kReadBufferSize = 2048;
  static constexpr std::size_t kMaxPollFds = 10;

  RefereeInputLoopRuntime(radar::log::BinaryLogStore &raw_log_store, bool serial_input_is_file_value,
                          bool info_wave_input_is_file_value, bool enemy_level1_input_is_file_value,
                          bool enemy_level2_input_is_file_value)
      : metrics(radar::log::GetRuntimeMetrics(raw_log_store.root())),
        serial_log(raw_log_store.root()),
        tcp_log(raw_log_store.root()),
        serial_input_is_file(serial_input_is_file_value),
        info_wave_input_is_file(info_wave_input_is_file_value),
        enemy_level1_input_is_file(enemy_level1_input_is_file_value),
        enemy_level2_input_is_file(enemy_level2_input_is_file_value),
        drain_duration(std::chrono::milliseconds(
            std::max(radar::referee::kRefereeStateTimeoutMs, radar::referee::kInfoWaveStateTimeoutMs) + 1000)) {}

  std::array<rm::u8, kReadBufferSize> read_buffer{};
  radar::log::RuntimeMetrics &metrics;
  SerialConnectionLog serial_log;
  TcpConnectionLog tcp_log;
  bool serial_input_is_file = false;
  bool info_wave_input_is_file = false;
  bool enemy_level1_input_is_file = false;
  bool enemy_level2_input_is_file = false;
  Clock::duration drain_duration{};
};

template <typename SerialReferee, typename InfoWaveReferee, typename EnemyKeyReceiverT, typename RadarCommandSenderT,
          typename UiUserSenderT, typename MapRobotRelayT, typename ExternalServerSenderT>
struct RefereeInputLoopServices {
  SerialPort &serial;
  TcpClient &info_wave_tcp;
  TcpClient &enemy_level1_key_tcp;
  TcpClient &enemy_level2_key_tcp;
  SerialReferee &serial_referee;
  InfoWaveReferee &info_wave_referee;
  ReplayInputSource *serial_replay = nullptr;
  ReplayInputSource *info_wave_replay = nullptr;
  ReplayInputSource *enemy_level1_key_replay = nullptr;
  ReplayInputSource *enemy_level2_key_replay = nullptr;
  EnemyKeyReceiverT &enemy_level1_key_receiver;
  EnemyKeyReceiverT &enemy_level2_key_receiver;
  RadarCommandSenderT &radar_command_sender;
  UiUserSenderT &ui_user_sender;
  MapRobotRelayT &map_robot_relay;
  ExternalServerSenderT &external_server_sender;
  RefereeTxScheduler &tx_scheduler;
  radar::log::BinaryLogStore &raw_log_store;
  TcpServer *external_tcp_server = nullptr;
  const std::atomic<bool> &running;
};

struct RefereeInputLoopState {
  using Clock = RefereeInputLoopRuntime::Clock;

  explicit RefereeInputLoopState(bool serial_connected) : serial_has_connected_once(serial_connected) {}

  std::optional<Clock::time_point> last_serial_open_attempt_time;
  bool serial_has_connected_once = false;
  bool serial_retry_state_logged = false;
  std::optional<Clock::time_point> last_info_wave_connect_attempt_time;
  std::optional<Clock::time_point> last_level1_connect_attempt_time;
  std::optional<Clock::time_point> last_level2_connect_attempt_time;
  std::optional<Clock::time_point> last_external_server_open_attempt_time;
  bool info_wave_connect_retry_state_logged = false;
  bool level1_connect_retry_state_logged = false;
  bool level2_connect_retry_state_logged = false;
  bool external_server_retry_state_logged = false;
  bool map_robot_early_send_attempted_this_loop = false;
  std::optional<std::size_t> level1_key_completion_upgrade_generation;
  std::optional<Clock::time_point> drain_deadline;
};

struct RefereeInputPollPlan {
  std::array<pollfd, RefereeInputLoopRuntime::kMaxPollFds> fds{};
  nfds_t nfds = 0;
  std::optional<nfds_t> serial_index;
  std::optional<nfds_t> info_wave_index;
  std::optional<nfds_t> level1_index;
  std::optional<nfds_t> level2_index;
  std::optional<nfds_t> external_listener_index;
  std::optional<nfds_t> external_client_index;
  int timeout_ms = 100;
};

template <typename SerialReferee, typename InfoWaveReferee, typename EnemyKeyReceiverT, typename RadarCommandSenderT,
          typename UiUserSenderT, typename MapRobotRelayT, typename ExternalServerSenderT>
class RefereeInputLoopRunner {
 public:
  using Services =
      RefereeInputLoopServices<SerialReferee, InfoWaveReferee, EnemyKeyReceiverT, RadarCommandSenderT, UiUserSenderT,
                               MapRobotRelayT, ExternalServerSenderT>;
  using Clock = RefereeInputLoopRuntime::Clock;

  RefereeInputLoopRunner(RefereeInputLoopRuntime &runtime, Services &services, RefereeInputLoopState &state)
      : runtime_(runtime), services_(services), state_(state) {}

  void Run() {
    ValidateReplaySources();
    if (services_.serial.is_open()) {
      runtime_.serial_log.LogState("connected", services_.serial.device(), services_.serial.baud(), "serial port ready",
                                   true);
    }

    while (services_.running.load()) {
      const auto loop_start = Clock::now();
      state_.map_robot_early_send_attempted_this_loop = false;

      TryReconnectSerial();
      TryReconnectTcpClient(services_.info_wave_tcp, "info_wave_tcp", radar::config::kInfoWaveTcpServerPort,
                            !runtime_.info_wave_input_is_file, true, state_.last_info_wave_connect_attempt_time,
                            state_.info_wave_connect_retry_state_logged);
      TryReconnectTcpClient(services_.enemy_level1_key_tcp, "enemy_level1_key_tcp",
                            radar::config::kEnemyLevel1KeyTcpServerPort,
                            !runtime_.enemy_level1_input_is_file && !services_.enemy_level1_key_receiver.completed(),
                            true,
                            state_.last_level1_connect_attempt_time, state_.level1_connect_retry_state_logged);
      TryReconnectTcpClient(services_.enemy_level2_key_tcp, "enemy_level2_key_tcp",
                            radar::config::kEnemyLevel2KeyTcpServerPort,
                            !runtime_.enemy_level2_input_is_file && !services_.enemy_level2_key_receiver.completed(),
                            true,
                            state_.last_level2_connect_attempt_time, state_.level2_connect_retry_state_logged);
      TryReopenExternalServer();

      const auto loop_now = Clock::now();
      PumpReplaySources(loop_now);
      if (AllConfiguredInputsAreFile() && AllFileInputsCompleted() &&
          !state_.drain_deadline.has_value()) {
        state_.drain_deadline = loop_now + runtime_.drain_duration;
      }
      if (state_.drain_deadline.has_value() && loop_now >= *state_.drain_deadline) {
        ServicePeriodicTasks(loop_start);
        break;
      }

      auto poll_plan = BuildPollPlan(loop_now);
      const int result = ::poll(poll_plan.fds.data(), poll_plan.nfds, poll_plan.timeout_ms);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
      }
      if (result == 0) {
        runtime_.metrics.RecordPollTimeoutWakeup();
        ServicePeriodicTasks(loop_start);
        if (state_.drain_deadline.has_value() && loop_now >= *state_.drain_deadline && AllFileInputsCompleted()) {
          break;
        }
        continue;
      }

      const auto after_poll_now = Clock::now();
      HandleSerialEvents(poll_plan);
      PumpReplaySources(after_poll_now);
      HandleInfoWaveEvents(poll_plan);
      TryFastPathMapRobotSend();
      HandleEnemyKeyEvents(poll_plan, services_.enemy_level1_key_tcp, services_.enemy_level1_key_receiver,
                           poll_plan.level1_index, radar::config::kEnemyLevel1KeyTcpServerPort,
                           "enemy_level1_key_tcp", "raw/tcp_8002_enemy_level1_key_rx.bin", false);
      HandleEnemyKeyEvents(poll_plan, services_.enemy_level2_key_tcp, services_.enemy_level2_key_receiver,
                           poll_plan.level2_index, radar::config::kEnemyLevel2KeyTcpServerPort,
                           "enemy_level2_key_tcp", "raw/tcp_8003_enemy_level2_key_rx.bin", true);
      HandleExternalServerEvents(poll_plan);

      ServicePeriodicTasks(loop_start);
      if (state_.drain_deadline.has_value() && after_poll_now >= *state_.drain_deadline &&
          AllFileInputsCompleted()) {
        break;
      }
    }
  }

 private:
  void ValidateReplaySources() const {
    if ((services_.serial_replay == nullptr && runtime_.serial_input_is_file) ||
        (services_.info_wave_replay == nullptr && runtime_.info_wave_input_is_file) ||
        (services_.enemy_level1_key_replay == nullptr && runtime_.enemy_level1_input_is_file) ||
        (services_.enemy_level2_key_replay == nullptr && runtime_.enemy_level2_input_is_file)) {
      throw std::runtime_error("replay input source is required but missing");
    }
  }

  void TryReconnectSerial() {
    if (services_.serial.is_open()) {
      return;
    }

    const auto now = Clock::now();
    if (state_.serial_has_connected_once && !state_.serial_retry_state_logged) {
      runtime_.serial_log.LogState("reconnecting", SerialDevice(), SerialBaud(), "waiting for next retry", false);
      state_.serial_retry_state_logged = true;
    }
    if (state_.last_serial_open_attempt_time.has_value() &&
        now - *state_.last_serial_open_attempt_time <
            std::chrono::milliseconds(kSerialReconnectIntervalMs)) {
      return;
    }

    state_.last_serial_open_attempt_time = now;
    std::string error;
    if (services_.serial.TryOpenDefault(SerialBaud(), &error)) {
      if (state_.serial_has_connected_once) {
        runtime_.metrics.RecordSerialReconnect();
        runtime_.serial_log.LogState("reconnected", services_.serial.device(), services_.serial.baud(),
                                     "serial port reopened", true);
      } else {
        runtime_.serial_log.LogState("connected", services_.serial.device(), services_.serial.baud(),
                                     "serial port opened", true);
      }
      state_.serial_has_connected_once = true;
      state_.serial_retry_state_logged = false;
      return;
    }

    runtime_.metrics.RecordSerialOpenFailure();
    runtime_.serial_log.LogState("open_failed", SerialDevice(), SerialBaud(), error, false);
  }

  void TryReconnectTcpClient(TcpClient &client, const std::string &name, int port, bool enable,
                             bool allow_debug_disable, std::optional<Clock::time_point> &last_attempt_time,
                             bool &retry_state_logged) {
    if (!enable || client.is_open()) {
      return;
    }

    const auto now = Clock::now();
    if (!retry_state_logged) {
      runtime_.tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "reconnecting",
                                       std::string("remote=") + radar::config::kTcpServerAddress);
      retry_state_logged = true;
    }
    if (last_attempt_time.has_value() &&
        now - *last_attempt_time < std::chrono::milliseconds(kTcpReconnectIntervalMs)) {
      return;
    }

    last_attempt_time = now;
    std::string error;
    if (client.TryOpen(radar::config::kTcpServerAddress, port, radar::config::kTcpLocalBindAddress, &error)) {
      const std::string detail = std::string("remote=") + radar::config::kTcpServerAddress;
      runtime_.tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port,
                                       client.is_connected() ? "connected" : "connecting", detail);
      runtime_.tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port,
                                      client.is_connected() ? client.peer_ip() : radar::config::kTcpServerAddress,
                                      client.is_connected() ? "connected" : "connecting", detail);
      retry_state_logged = false;
      return;
    }

    runtime_.tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "connect_failed", error);
    if (!allow_debug_disable || !radar::config::kDebugAllowMissingInterfaces) {
      return;
    }
    runtime_.tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "disabled",
                                     "debug_allow_missing");
  }

  void TryReopenExternalServer() {
    TcpServer *server = services_.external_tcp_server;
    if (!radar::config::kExternalTcpServerEnabled || server == nullptr || server->is_open()) {
      return;
    }

    const auto now = Clock::now();
    if (!state_.external_server_retry_state_logged) {
      runtime_.tcp_log.LogChannelState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                       radar::config::kExternalTcpServerPort, "reopening_listener",
                                       "waiting_for_next_retry");
      state_.external_server_retry_state_logged = true;
    }
    if (state_.last_external_server_open_attempt_time.has_value() &&
        now - *state_.last_external_server_open_attempt_time <
            std::chrono::milliseconds(kTcpReconnectIntervalMs)) {
      return;
    }

    state_.last_external_server_open_attempt_time = now;
    std::string error;
    if (server->TryOpen(radar::config::kExternalTcpServerBindAddress, radar::config::kExternalTcpServerPort,
                        &error)) {
      runtime_.tcp_log.LogChannelState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                       radar::config::kExternalTcpServerPort, "listening", "listener_reopened");
      state_.external_server_retry_state_logged = false;
      return;
    }

    runtime_.tcp_log.LogChannelState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                     radar::config::kExternalTcpServerPort, "listen_failed", error);
  }

  void PumpReplaySources(const Clock::time_point &now) {
    if (runtime_.serial_input_is_file && services_.serial_replay != nullptr) {
      services_.serial_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
        services_.raw_log_store.Append("raw/file_serial_referee_rx.bin", bytes, size);
        FeedBytes(services_.serial_referee, bytes, size);
      });
    }
    if (runtime_.info_wave_input_is_file && services_.info_wave_replay != nullptr) {
      services_.info_wave_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
        services_.raw_log_store.Append("raw/file_tcp_8001_info_wave_rx.bin", bytes, size);
        FeedBytes(services_.info_wave_referee, bytes, size);
      });
    }
    if (runtime_.enemy_level1_input_is_file && services_.enemy_level1_key_replay != nullptr) {
      services_.enemy_level1_key_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
        services_.raw_log_store.Append("raw/file_tcp_8002_enemy_level1_key_rx.bin", bytes, size);
        services_.enemy_level1_key_receiver.ProcessBytes(bytes, size, services_.serial_referee.data());
      });
    }
    if (runtime_.enemy_level2_input_is_file && services_.enemy_level2_key_replay != nullptr) {
      services_.enemy_level2_key_replay->Process(now, [&](const rm::u8 *bytes, std::size_t size) {
        services_.raw_log_store.Append("raw/file_tcp_8003_enemy_level2_key_rx.bin", bytes, size);
        services_.enemy_level2_key_receiver.ProcessBytes(bytes, size, services_.serial_referee.data());
      });
    }
  }

  RefereeInputPollPlan BuildPollPlan(const Clock::time_point &now) const {
    RefereeInputPollPlan plan;

    if (!runtime_.serial_input_is_file && services_.serial.is_open()) {
      plan.serial_index = plan.nfds;
      plan.fds[plan.nfds++] = pollfd{services_.serial.fd(), static_cast<short>(POLLIN | POLLHUP | POLLERR), 0};
    }

    AddTcpClientPollFd(services_.info_wave_tcp, !runtime_.info_wave_input_is_file, plan.info_wave_index, plan);
    AddTcpClientPollFd(services_.enemy_level1_key_tcp,
                       !runtime_.enemy_level1_input_is_file && !services_.enemy_level1_key_receiver.completed(),
                       plan.level1_index, plan);
    AddTcpClientPollFd(services_.enemy_level2_key_tcp,
                       !runtime_.enemy_level2_input_is_file && !services_.enemy_level2_key_receiver.completed(),
                       plan.level2_index, plan);

    if (services_.external_tcp_server != nullptr && services_.external_tcp_server->is_open()) {
      plan.external_listener_index = plan.nfds;
      short listener_events = static_cast<short>(POLLHUP | POLLERR);
      if (!services_.external_tcp_server->has_client()) {
        listener_events = static_cast<short>(listener_events | POLLIN);
      }
      plan.fds[plan.nfds++] = pollfd{services_.external_tcp_server->fd(), listener_events, 0};
      if (services_.external_tcp_server->has_client()) {
        plan.external_client_index = plan.nfds;
        plan.fds[plan.nfds++] = pollfd{services_.external_tcp_server->client_fd(),
                                       static_cast<short>(POLLIN | POLLHUP | POLLERR), 0};
      }
    }

    plan.timeout_ms = ComputePollTimeoutMs(now);
    return plan;
  }

  void HandleSerialEvents(const RefereeInputPollPlan &poll_plan) {
    if (!poll_plan.serial_index.has_value()) {
      return;
    }

    const auto revents = poll_plan.fds[*poll_plan.serial_index].revents;
    if ((revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      MarkSerialDisconnected("serial poll reported hangup or error");
      return;
    }
    if ((revents & POLLIN) == 0) {
      return;
    }

    std::size_t successful_reads = 0;
    while (services_.serial.is_open()) {
      std::size_t bytes_read = 0;
      std::string error;
      if (!services_.serial.TryRead(runtime_.read_buffer.data(), runtime_.read_buffer.size(), &bytes_read, &error)) {
        MarkSerialDisconnected(error);
        return;
      }
      runtime_.metrics.RecordSerialRead(bytes_read);
      if (bytes_read == 0) {
        break;
      }

      ++successful_reads;
      services_.raw_log_store.Append("raw/serial_referee_rx.bin", runtime_.read_buffer.data(), bytes_read);
      FeedBytes(services_.serial_referee, runtime_.read_buffer.data(), bytes_read);
    }
    if (successful_reads > 1) {
      runtime_.metrics.RecordMultiReadDrain("serial");
    }
  }

  void HandleInfoWaveEvents(const RefereeInputPollPlan &poll_plan) {
    if (!poll_plan.info_wave_index.has_value()) {
      return;
    }

    const auto revents = poll_plan.fds[*poll_plan.info_wave_index].revents;
    if ((revents & POLLOUT) != 0 && services_.info_wave_tcp.is_connecting()) {
      std::string error;
      if (services_.info_wave_tcp.FinishConnect(&error)) {
        runtime_.tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress,
                                        services_.info_wave_tcp.port(), services_.info_wave_tcp.peer_ip(),
                                        "connected", "connect_completed");
      } else {
        runtime_.tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress,
                                        radar::config::kInfoWaveTcpServerPort, radar::config::kTcpServerAddress,
                                        "disconnected", error);
      }
    }
    if ((revents & POLLIN) != 0 && services_.info_wave_tcp.is_connected()) {
      std::size_t successful_reads = 0;
      while (services_.info_wave_tcp.is_connected()) {
        const auto bytes_read = services_.info_wave_tcp.Read(runtime_.read_buffer.data(), runtime_.read_buffer.size());
        if (bytes_read == 0) {
          break;
        }

        ++successful_reads;
        services_.raw_log_store.Append("raw/tcp_8001_info_wave_rx.bin", runtime_.read_buffer.data(), bytes_read);
        FeedBytes(services_.info_wave_referee, runtime_.read_buffer.data(), bytes_read);
      }
      if (successful_reads > 1) {
        runtime_.metrics.RecordMultiReadDrain("8001");
      }
    }
    if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      const auto port = services_.info_wave_tcp.port();
      const std::string peer = services_.info_wave_tcp.peer_ip().empty() ? radar::config::kTcpServerAddress
                                                                         : services_.info_wave_tcp.peer_ip();
      runtime_.tcp_log.LogClientState("info_wave_tcp", radar::config::kTcpLocalBindAddress, port, peer,
                                      "disconnected", "poll reported hangup or error");
      services_.info_wave_tcp.Close();
    }
  }

  void HandleEnemyKeyEvents(const RefereeInputPollPlan &poll_plan, TcpClient &client, EnemyKeyReceiverT &receiver,
                            const std::optional<nfds_t> &index, int connect_port, const char *name,
                            const char *raw_log_path, bool close_on_completion) {
    if (!index.has_value()) {
      return;
    }

    const auto revents = poll_plan.fds[*index].revents;
    if ((revents & POLLOUT) != 0 && client.is_connecting()) {
      std::string error;
      if (client.FinishConnect(&error)) {
        runtime_.tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, client.port(), client.peer_ip(),
                                        "connected", "connect_completed");
      } else {
        runtime_.tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, connect_port,
                                        radar::config::kTcpServerAddress, "disconnected", error);
      }
    }
    if ((revents & POLLIN) != 0 && client.is_connected()) {
      std::size_t successful_reads = 0;
      while (client.is_connected()) {
        const auto bytes_read = client.Read(runtime_.read_buffer.data(), runtime_.read_buffer.size());
        if (bytes_read == 0) {
          break;
        }

        ++successful_reads;
        services_.raw_log_store.Append(raw_log_path, runtime_.read_buffer.data(), bytes_read);
        if (receiver.ProcessBytes(runtime_.read_buffer.data(), bytes_read, services_.serial_referee.data())) {
          if (close_on_completion) {
            client.Stop();
          } else {
            state_.level1_key_completion_upgrade_generation =
                services_.radar_command_sender.ally_encryption_level_upgrade_generation();
          }
          break;
        }
      }
      if (successful_reads > 1) {
        runtime_.metrics.RecordMultiReadDrain(connect_port == radar::config::kEnemyLevel1KeyTcpServerPort ? "8002"
                                                                                                           : "8003");
      }
    }
    if (!receiver.completed() && (revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      const auto port = client.port();
      const std::string peer = client.peer_ip().empty() ? radar::config::kTcpServerAddress : client.peer_ip();
      runtime_.tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port, peer, "disconnected",
                                      "poll reported hangup or error");
      client.Close();
    }
  }

  void HandleExternalServerEvents(const RefereeInputPollPlan &poll_plan) {
    TcpServer *server = services_.external_tcp_server;
    if (server == nullptr) {
      return;
    }

    if (poll_plan.external_listener_index.has_value()) {
      const auto revents = poll_plan.fds[*poll_plan.external_listener_index].revents;
      if ((revents & POLLIN) != 0 && !server->has_client()) {
        std::string error;
        if (server->AcceptPending(&error)) {
          runtime_.tcp_log.LogClientState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                          radar::config::kExternalTcpServerPort, server->peer_ip(), "connected",
                                          "accepted_client");
        } else if (!error.empty()) {
          runtime_.tcp_log.LogClientState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                          radar::config::kExternalTcpServerPort, "", "disconnected", error);
        }
      }
      if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        runtime_.tcp_log.LogChannelState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                         radar::config::kExternalTcpServerPort, "listener_closed",
                                         "poll reported hangup or error");
        server->Close();
      }
    }

    if (poll_plan.external_client_index.has_value() && server->has_client()) {
      const auto revents = poll_plan.fds[*poll_plan.external_client_index].revents;
      const std::string peer = server->peer_ip();
      if ((revents & POLLIN) != 0) {
        std::size_t successful_reads = 0;
        while (server->has_client()) {
          const auto bytes_read = server->Read(runtime_.read_buffer.data(), runtime_.read_buffer.size());
          if (bytes_read == 0) {
            if (!server->has_client()) {
              runtime_.tcp_log.LogClientState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                              radar::config::kExternalTcpServerPort, peer, "disconnected",
                                              "peer_closed");
            }
            break;
          }

          ++successful_reads;
          services_.raw_log_store.Append("raw/tcp_external_device_rx.bin", runtime_.read_buffer.data(), bytes_read);
        }
        if (successful_reads > 1) {
          runtime_.metrics.RecordMultiReadDrain("external_tcp_server");
        }
      }
      if ((revents & (POLLHUP | POLLERR | POLLNVAL)) != 0 && server->has_client()) {
        runtime_.tcp_log.LogClientState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                        radar::config::kExternalTcpServerPort, peer, "disconnected",
                                        "poll reported hangup or error");
        server->CloseClient();
      }
    }
  }

  void TryFastPathMapRobotSend() {
    if (!services_.map_robot_relay.ConsumeFreshRadar0Refresh()) {
      return;
    }

    services_.tx_scheduler.Process();
    state_.map_robot_early_send_attempted_this_loop = true;
  }

  void ServicePeriodicTasks(const Clock::time_point &loop_start) {
    const auto log_connect_timeout = [&](TcpClient &client, const std::string &name) {
      const auto port = client.port();
      if (client.CloseIfConnectTimedOut(kTcpConnectTimeoutMs)) {
        runtime_.tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port,
                                        radar::config::kTcpServerAddress, "disconnected",
                                        "connect_attempt_timed_out");
      }
    };

    if (!runtime_.info_wave_input_is_file) {
      log_connect_timeout(services_.info_wave_tcp, "info_wave_tcp");
    }
    if (!runtime_.enemy_level1_input_is_file && !services_.enemy_level1_key_receiver.completed()) {
      log_connect_timeout(services_.enemy_level1_key_tcp, "enemy_level1_key_tcp");
    }
    if (!runtime_.enemy_level2_input_is_file && !services_.enemy_level2_key_receiver.completed()) {
      log_connect_timeout(services_.enemy_level2_key_tcp, "enemy_level2_key_tcp");
    }

    if (services_.info_wave_tcp.CloseIdleClientIfTimedOut(kInfoWaveTcpIdleTimeoutMs)) {
      runtime_.metrics.RecordTcpIdleDisconnect(services_.info_wave_tcp.port());
    }
    if (!runtime_.enemy_level1_input_is_file && !services_.enemy_level1_key_receiver.completed() &&
        services_.enemy_level1_key_tcp.CloseIdleClientIfTimedOut(kEnemyKeyTcpIdleTimeoutMs)) {
      runtime_.metrics.RecordTcpIdleDisconnect(services_.enemy_level1_key_tcp.port());
    }
    if (!runtime_.enemy_level2_input_is_file && !services_.enemy_level2_key_receiver.completed() &&
        services_.enemy_level2_key_tcp.CloseIdleClientIfTimedOut(kEnemyKeyTcpIdleTimeoutMs)) {
      runtime_.metrics.RecordTcpIdleDisconnect(services_.enemy_level2_key_tcp.port());
    }

    CloseEnemyLevel1KeyTcpAfterAllyInterferenceWaveUpgrade();

    const std::string external_peer =
        services_.external_tcp_server != nullptr ? services_.external_tcp_server->peer_ip() : std::string();
    if (services_.external_tcp_server != nullptr &&
        services_.external_tcp_server->CloseIdleClientIfTimedOut(kExternalTcpServerIdleTimeoutMs)) {
      runtime_.metrics.RecordTcpIdleDisconnect(services_.external_tcp_server->port());
      runtime_.tcp_log.LogClientState("external_tcp_server", radar::config::kExternalTcpServerBindAddress,
                                      radar::config::kExternalTcpServerPort, external_peer, "disconnected",
                                      "idle_timeout");
    }

    if (!state_.map_robot_early_send_attempted_this_loop) {
      services_.map_robot_relay.ProcessPeriodic();
    }
    services_.ui_user_sender.ProcessPeriodic();
    services_.enemy_level1_key_receiver.ProcessDeferredQueueing();
    services_.enemy_level2_key_receiver.ProcessDeferredQueueing();
    services_.radar_command_sender.ProcessPending(services_.serial_referee.data());
    services_.external_server_sender.ProcessPeriodic();
    services_.tx_scheduler.Process();

    const auto loop_end = Clock::now();
    const auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
    runtime_.metrics.RecordLoopIteration(loop_duration, loop_duration);
    runtime_.metrics.MaybeFlush();
  }

  /**
   * @brief 在一级敌方密钥接收完成后，等待己方干扰波等级实际升级再关闭 `8002`
   * @note 等级升级由 `0x020E` bit3-4 从较低值变为较高值触发；收到密钥本身不会关闭连接。
   */
  void CloseEnemyLevel1KeyTcpAfterAllyInterferenceWaveUpgrade() {
    if (!state_.level1_key_completion_upgrade_generation.has_value() ||
        !services_.enemy_level1_key_tcp.is_open()) {
      return;
    }

    const auto current_upgrade_generation =
        services_.radar_command_sender.ally_encryption_level_upgrade_generation();
    if (current_upgrade_generation <= *state_.level1_key_completion_upgrade_generation) {
      return;
    }

    const auto port = services_.enemy_level1_key_tcp.port();
    const std::string peer = services_.enemy_level1_key_tcp.peer_ip().empty()
                                 ? radar::config::kTcpServerAddress
                                 : services_.enemy_level1_key_tcp.peer_ip();
    services_.enemy_level1_key_tcp.Stop();
    runtime_.tcp_log.LogClientState("enemy_level1_key_tcp", radar::config::kTcpLocalBindAddress, port, peer,
                                    "disconnected", "ally_interference_wave_upgraded");
  }

  int SerialBaud() const {
    return services_.serial.baud() != 0 ? services_.serial.baud() : radar::config::kDefaultRefereeBaud;
  }

  std::string SerialDevice() const {
    if (!services_.serial.device().empty()) {
      return services_.serial.device();
    }
    return SelectRefereeDevice();
  }

  void MarkSerialDisconnected(const std::string &detail) {
    if (services_.serial.is_open()) {
      services_.serial.Close();
    }
    runtime_.metrics.RecordSerialDisconnect();
    runtime_.serial_log.LogState("disconnected", SerialDevice(), SerialBaud(), detail, false);
    state_.serial_retry_state_logged = false;
  }

  bool AllFileInputsCompleted() const {
    const bool serial_done = !runtime_.serial_input_is_file ||
                             (services_.serial_replay != nullptr && services_.serial_replay->completed());
    const bool info_wave_done = !runtime_.info_wave_input_is_file ||
                                (services_.info_wave_replay != nullptr && services_.info_wave_replay->completed());
    const bool enemy_level1_done =
        !runtime_.enemy_level1_input_is_file ||
        (services_.enemy_level1_key_replay != nullptr && services_.enemy_level1_key_replay->completed());
    const bool enemy_level2_done =
        !runtime_.enemy_level2_input_is_file ||
        (services_.enemy_level2_key_replay != nullptr && services_.enemy_level2_key_replay->completed());
    return serial_done && info_wave_done && enemy_level1_done && enemy_level2_done;
  }

  bool AllConfiguredInputsAreFile() const {
    return runtime_.serial_input_is_file && runtime_.info_wave_input_is_file && runtime_.enemy_level1_input_is_file &&
           runtime_.enemy_level2_input_is_file;
  }

  int ComputePollTimeoutMs(const Clock::time_point &now) const {
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
    const auto apply_due_time = [&](const std::optional<Clock::time_point> &due_time) {
      if (!due_time.has_value()) {
        return;
      }
      if (*due_time <= now) {
        timeout_ms = 0;
        return;
      }
      const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(*due_time - now).count();
      timeout_ms = std::min(timeout_ms, std::max(0, static_cast<int>(wait_ms)));
    };

    apply_replay_deadline(services_.serial_replay);
    apply_replay_deadline(services_.info_wave_replay);
    apply_replay_deadline(services_.enemy_level1_key_replay);
    apply_replay_deadline(services_.enemy_level2_key_replay);
    if (services_.serial.is_open()) {
      apply_due_time(services_.tx_scheduler.NextDueTime(now));
    }
    return std::max(0, timeout_ms);
  }

  static void AddTcpClientPollFd(TcpClient &client, bool enable, std::optional<nfds_t> &index,
                                 RefereeInputPollPlan &plan) {
    if (!enable || !client.is_open()) {
      return;
    }

    index = plan.nfds;
    short events = static_cast<short>(POLLHUP | POLLERR);
    if (client.is_connecting()) {
      events = static_cast<short>(events | POLLOUT);
    }
    if (client.is_connected()) {
      events = static_cast<short>(events | POLLIN);
    }
    plan.fds[plan.nfds++] = pollfd{client.fd(), events, 0};
  }

  RefereeInputLoopRuntime &runtime_;
  Services &services_;
  RefereeInputLoopState &state_;
};

}  // namespace detail

/**
 * @brief 完整主循环：串口接收 + 多路 TCP + 统一发送调度
 * @tparam SerialReferee 常规链路解包器类型
 * @tparam InfoWaveReferee 信息波解包器类型
 * @tparam EnemyKeyReceiverT 敌方密钥接收器类型
 * @tparam RadarCommandSenderT 雷达指令发送器类型
 * @tparam MapRobotRelayT `0x0305` relay 类型
 * @tparam ExternalServerSenderT 外部 TCP server 原始发送器类型
 */
template <typename SerialReferee, typename InfoWaveReferee, typename EnemyKeyReceiverT, typename RadarCommandSenderT,
          typename UiUserSenderT, typename MapRobotRelayT, typename ExternalServerSenderT>
void RunSerialInfoWaveAndKeyTcpLoop(SerialPort &serial, TcpClient &info_wave_tcp, TcpClient &enemy_level1_key_tcp,
                                    TcpClient &enemy_level2_key_tcp, SerialReferee &serial_referee,
                                    InfoWaveReferee &info_wave_referee, ReplayInputSource *serial_replay,
                                    ReplayInputSource *info_wave_replay, ReplayInputSource *enemy_level1_key_replay,
                                    ReplayInputSource *enemy_level2_key_replay,
                                    EnemyKeyReceiverT &enemy_level1_key_receiver,
                                    EnemyKeyReceiverT &enemy_level2_key_receiver,
                                    RadarCommandSenderT &radar_command_sender, UiUserSenderT &ui_user_sender,
                                    MapRobotRelayT &map_robot_relay,
                                    ExternalServerSenderT &external_server_sender, RefereeTxScheduler &tx_scheduler,
                                    radar::log::BinaryLogStore &raw_log_store, TcpServer *external_tcp_server,
                                    const std::atomic<bool> &running) {
  const bool serial_input_is_file =
      radar::config::kSerialRefereeInputMode == radar::config::RefereeInputSourceMode::kFile;
  const bool info_wave_input_is_file =
      radar::config::kInfoWaveInputMode == radar::config::RefereeInputSourceMode::kFile;
  const bool enemy_level1_input_is_file =
      radar::config::kEnemyLevel1KeyInputMode == radar::config::RefereeInputSourceMode::kFile;
  const bool enemy_level2_input_is_file =
      radar::config::kEnemyLevel2KeyInputMode == radar::config::RefereeInputSourceMode::kFile;

  detail::RefereeInputLoopRuntime runtime(raw_log_store, serial_input_is_file, info_wave_input_is_file,
                                          enemy_level1_input_is_file, enemy_level2_input_is_file);
  auto services =
      detail::RefereeInputLoopServices<SerialReferee, InfoWaveReferee, EnemyKeyReceiverT, RadarCommandSenderT,
                                       UiUserSenderT, MapRobotRelayT, ExternalServerSenderT>{serial,
                                                                              info_wave_tcp,
                                                                              enemy_level1_key_tcp,
                                                                              enemy_level2_key_tcp,
                                                                              serial_referee,
                                                                              info_wave_referee,
                                                                              serial_replay,
                                                                              info_wave_replay,
                                                                              enemy_level1_key_replay,
                                                                              enemy_level2_key_replay,
                                                                              enemy_level1_key_receiver,
                                                                              enemy_level2_key_receiver,
                                                                              radar_command_sender,
                                                                              ui_user_sender,
                                                                              map_robot_relay,
                                                                              external_server_sender,
                                                                              tx_scheduler,
                                                                              raw_log_store,
                                                                              external_tcp_server,
                                                                              running};
  detail::RefereeInputLoopState state(serial.is_open());
  detail::RefereeInputLoopRunner<SerialReferee, InfoWaveReferee, EnemyKeyReceiverT, RadarCommandSenderT, UiUserSenderT,
                                 MapRobotRelayT, ExternalServerSenderT>
      runner(runtime, services, state);
  runner.Run();
}

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_REFEREE_INPUT_LOOP_HPP
