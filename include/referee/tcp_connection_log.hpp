#ifndef RADAR_INCLUDE_REFEREE_TCP_CONNECTION_LOG_HPP
#define RADAR_INCLUDE_REFEREE_TCP_CONNECTION_LOG_HPP

/**
 * @file  include/referee/tcp_connection_log.hpp
 * @brief TCP 连接状态日志封装
 */

#include <filesystem>
#include <ostream>
#include <sstream>
#include <string>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/tcp_client.hpp"

namespace radar::referee {

/**
 * @brief 维护 TCP 启动、连接通道与客户端连接状态日志
 */
class TcpConnectionLog {
 public:
  explicit TcpConnectionLog(std::filesystem::path root) : log_store_(std::move(root)) {}

  /**
   * @brief 记录一次 TCP 启动结果
   */
  void LogStartup(const std::string &name, const std::string &bind_address, int port, const std::string &result,
                  const std::string &detail) {
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
        << "\"name\":\"" << name << "\","
        << "\"bind_address\":\"" << bind_address << "\","
        << "\"port\":" << port << ','
        << "\"result\":\"" << result << "\","
        << "\"detail\":\"" << detail << "\"}";
    log_store_.Append("main/tcp_startup.log", oss.str(), radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一次 TCP 连接通道状态变化
   */
  void LogChannelState(const std::string &name, const std::string &bind_address, int port, const std::string &state,
                       const std::string &detail) {
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
        << "\"name\":\"" << name << "\","
        << "\"bind_address\":\"" << bind_address << "\","
        << "\"port\":" << port << ','
        << "\"state\":\"" << state << "\","
        << "\"detail\":\"" << detail << "\"}";
    log_store_.Append("main/tcp_channel_state.log", oss.str(), radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一次 TCP 客户端连接状态变化
   */
  void LogClientState(const std::string &name, const std::string &bind_address, int port, const std::string &peer_ip,
                      const std::string &state, const std::string &detail) {
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
        << "\"name\":\"" << name << "\","
        << "\"bind_address\":\"" << bind_address << "\","
        << "\"port\":" << port << ','
        << "\"peer_ip\":\"" << peer_ip << "\","
        << "\"state\":\"" << state << "\","
        << "\"detail\":\"" << detail << "\"}";
    log_store_.Append("main/tcp_client_state.log", oss.str(), radar::log::LogPriority::kCriticalDecision);
  }

 private:
  radar::log::FileLogStore log_store_;
};

/**
 * @brief 启动阶段按配置尝试发起 TCP 客户端连接
 */
inline void TryOpenConfiguredTcpClient(TcpClient *client, const std::string &name, int port, bool optional_in_debug,
                                       TcpConnectionLog &tcp_log, std::ostream *error_stream = nullptr) {
  std::string error;
  if (client->TryOpen(radar::config::kTcpServerAddress, port, radar::config::kTcpLocalBindAddress, &error)) {
    const std::string detail = std::string("remote=") + radar::config::kTcpServerAddress;
    tcp_log.LogStartup(name, radar::config::kTcpLocalBindAddress, port,
                       client->is_connected() ? "connected" : "connecting", detail);
    tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port,
                            client->is_connected() ? "connected" : "connecting", detail);
    tcp_log.LogClientState(name, radar::config::kTcpLocalBindAddress, port,
                           client->is_connected() ? client->peer_ip() : radar::config::kTcpServerAddress,
                           client->is_connected() ? "connected" : "connecting", detail);
    return;
  }

  tcp_log.LogStartup(name, radar::config::kTcpLocalBindAddress, port, "connect_failed", error);
  tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "connect_failed", error);
  if (error_stream != nullptr) {
    *error_stream << error << '\n';
  }

  if (optional_in_debug && radar::config::kDebugAllowMissingInterfaces) {
    tcp_log.LogStartup(name, radar::config::kTcpLocalBindAddress, port, "disabled", "debug_allow_missing");
    tcp_log.LogChannelState(name, radar::config::kTcpLocalBindAddress, port, "disabled", "debug_allow_missing");
  }
}

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_TCP_CONNECTION_LOG_HPP
