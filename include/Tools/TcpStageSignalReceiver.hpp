/**
 * @file    include/Tools/TcpStageSignalReceiver.hpp
 * @brief   收发 TCP 阶段控制命令并按协议帧输出。
 *
 * 当前协议约定：
 * - `0x91 + 1Byte + 2Byte`：第 1 个 payload byte 的低 4 bit 为 `game_progress`，
 *   后 2 个 byte 按网络序（大端）组成 `stage_remain_time`
 * - `0x92 + 1Byte`：payload byte 的低 1 bit 为“敌方无人机是否被反制”状态
 *
 * 主程序 client 模式主动连接外部 server 并持续接收；测试工具仍可复用
 * sender 或 listener。该模块只负责 TCP 连接生命周期、流式收包和协议帧切分，
 * 不直接修改阶段状态机。
 */

#pragma once

#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "Tools/TcpStageProtocol.hpp"
#include "Tools/TcpStageReceiveLogger.hpp"

namespace Tools {

struct TcpStageSignalConfig {
  std::string bind_ip = "0.0.0.0";
  std::uint16_t bind_port = 19001;
  int backlog = 1;
  std::string receive_log_path = "tcp_stage_receive.log";
};

struct TcpStageSendConfig {
  std::string host = "127.0.0.1";
  std::uint16_t port = 19001;
};

struct TcpStageClientReceiveConfig {
  std::string server_ip = "192.168.50.75";
  std::uint16_t server_port = 9001;
  int connect_timeout_ms = 500;
  int reconnect_interval_ms = 1000;
  std::string receive_log_path = "tcp_stage_receive.log";
};

class TcpStageSignalReceiver {
public:
  explicit TcpStageSignalReceiver(TcpStageSignalConfig config)
      : config_(std::move(config)), receive_logger_(config_.receive_log_path) {}

  ~TcpStageSignalReceiver() { Close(); }

  bool PollNextCommand(TcpStageCommand *command) {
    if (command == nullptr) {
      return false;
    }

    if (listen_fd_ < 0 && !EnsureListening_()) {
      return false;
    }

    if (client_fd_ < 0 && !TryAcceptClient_()) {
      return false;
    }

    if (TryParseNextCommand_(command)) {
      return true;
    }

    std::array<std::uint8_t, 64> recv_chunk{};
    const ssize_t recv_count =
        ::recv(client_fd_, recv_chunk.data(), recv_chunk.size(), 0);
    if (recv_count > 0) {
      recv_buffer_.insert(recv_buffer_.end(), recv_chunk.begin(),
                          recv_chunk.begin() + recv_count);
      return TryParseNextCommand_(command);
    }

    if (recv_count == 0) {
      std::cout << "[TCP阶段] 客户端断开连接" << std::endl;
      CloseClient_();
      return false;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return false;
    }

    std::cerr << "[TCP阶段] recv 失败: " << std::strerror(errno) << std::endl;
    CloseClient_();
    return false;
  }

  static bool SendGameState(const std::string &host, std::uint16_t port,
                            std::uint8_t game_progress,
                            std::uint16_t stage_remain_time) {
    const auto payload =
        EncodeGameStateCommand(game_progress, stage_remain_time);
    return SendSingleCommand_(host, port, payload.data(), payload.size());
  }

  static bool SendCounteredState(const std::string &host, std::uint16_t port,
                                 bool countered) {
    const auto payload = EncodeCounteredStateCommand(countered);
    return SendSingleCommand_(host, port, payload.data(), payload.size());
  }

  void Close() {
    CloseClient_();
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
  }

private:
  static bool SendSingleCommand_(const std::string &host, std::uint16_t port,
                                 const std::uint8_t *payload,
                                 std::size_t payload_size) {
    if (payload == nullptr || payload_size == 0) {
      return false;
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      std::cerr << "[TCP阶段] 创建 socket 失败: " << std::strerror(errno)
                << std::endl;
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      std::cerr << "[TCP阶段] 非法目标 IP: " << host << std::endl;
      ::close(fd);
      return false;
    }

    if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) <
        0) {
      std::cerr << "[TCP阶段] connect 失败: " << std::strerror(errno)
                << std::endl;
      ::close(fd);
      return false;
    }

    if (!SendAll_(fd, payload, payload_size)) {
      ::close(fd);
      return false;
    }

    ::close(fd);
    return true;
  }

  bool TryParseNextCommand_(TcpStageCommand *command) {
    while (!recv_buffer_.empty()) {
      std::size_t consumed_size = 0;
      const auto decode_status = TryDecodeTcpStageCommand(
          recv_buffer_.data(), recv_buffer_.size(), command, &consumed_size);
      if (decode_status == TcpStageDecodeStatus::Decoded) {
        receive_logger_.LogDecodedCommand(recv_buffer_.data(), consumed_size,
                                           *command);
        recv_buffer_.erase(recv_buffer_.begin(),
                           recv_buffer_.begin() + consumed_size);
        return true;
      }
      if (decode_status == TcpStageDecodeStatus::NeedMoreData) {
        return false;
      }

      receive_logger_.LogInvalidBytes(recv_buffer_.data(), 1);
      std::cout << "[TCP阶段] 忽略未知命令码 0x" << std::hex
                << static_cast<int>(recv_buffer_.front()) << std::dec
                << std::endl;
      recv_buffer_.erase(recv_buffer_.begin());
    }
    return false;
  }

  static bool SendAll_(int fd, const std::uint8_t *payload,
                       std::size_t payload_size) {
    std::size_t offset = 0;
    while (offset < payload_size) {
      const ssize_t sent =
          ::send(fd, payload + offset, payload_size - offset, MSG_NOSIGNAL);
      if (sent > 0) {
        offset += static_cast<std::size_t>(sent);
        continue;
      }

      if (sent < 0 && errno == EINTR) {
        continue;
      }

      std::cerr << "[TCP阶段] send 失败: " << std::strerror(errno)
                << std::endl;
      return false;
    }
    return true;
  }

  bool EnsureListening_() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      std::cerr << "[TCP阶段] 创建监听 socket 失败: " << std::strerror(errno)
                << std::endl;
      return false;
    }

    int enable = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable,
                 sizeof(enable));
    if (!SetNonBlocking_(listen_fd_)) {
      Close();
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.bind_port);
    if (config_.bind_ip.empty() || config_.bind_ip == "0.0.0.0") {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, config_.bind_ip.c_str(), &addr.sin_addr) !=
               1) {
      std::cerr << "[TCP阶段] 非法监听 IP: " << config_.bind_ip << std::endl;
      Close();
      return false;
    }

    if (::bind(listen_fd_, reinterpret_cast<const sockaddr *>(&addr),
               sizeof(addr)) < 0) {
      std::cerr << "[TCP阶段] bind 失败: " << std::strerror(errno)
                << " ip=" << config_.bind_ip
                << " port=" << config_.bind_port << std::endl;
      Close();
      return false;
    }

    if (::listen(listen_fd_, config_.backlog) < 0) {
      std::cerr << "[TCP阶段] listen 失败: " << std::strerror(errno)
                << std::endl;
      Close();
      return false;
    }

    std::cout << "[TCP阶段] 监听 " << config_.bind_ip << ":"
              << config_.bind_port << std::endl;
    return true;
  }

  bool TryAcceptClient_() {
    sockaddr_in client_addr{};
    socklen_t client_addr_len = sizeof(client_addr);
    const int accepted_fd =
        ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_addr),
                 &client_addr_len);
    if (accepted_fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return false;
      }

      std::cerr << "[TCP阶段] accept 失败: " << std::strerror(errno)
                << std::endl;
      return false;
    }

    if (!SetNonBlocking_(accepted_fd)) {
      ::close(accepted_fd);
      return false;
    }

    client_fd_ = accepted_fd;
    char ip_buffer[INET_ADDRSTRLEN] = {0};
    const char *client_ip = ::inet_ntop(AF_INET, &client_addr.sin_addr,
                                        ip_buffer, sizeof(ip_buffer));
    std::cout << "[TCP阶段] 客户端接入 "
              << (client_ip != nullptr ? client_ip : "unknown") << ":"
              << ntohs(client_addr.sin_port) << std::endl;
    return true;
  }

  static bool SetNonBlocking_(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
      std::cerr << "[TCP阶段] fcntl(F_GETFL) 失败: "
                << std::strerror(errno) << std::endl;
      return false;
    }
    if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      std::cerr << "[TCP阶段] fcntl(F_SETFL) 失败: "
                << std::strerror(errno) << std::endl;
      return false;
    }
    return true;
  }

  void CloseClient_() {
    if (client_fd_ >= 0) {
      ::close(client_fd_);
      client_fd_ = -1;
    }
    recv_buffer_.clear();
  }

  TcpStageSignalConfig config_{};
  TcpStageReceiveLogger receive_logger_{};
  int listen_fd_ = -1;
  int client_fd_ = -1;
  std::vector<std::uint8_t> recv_buffer_;
};

class TcpStageSignalClientReceiver {
public:
  explicit TcpStageSignalClientReceiver(TcpStageClientReceiveConfig config)
      : config_(std::move(config)), receive_logger_(config_.receive_log_path) {}

  ~TcpStageSignalClientReceiver() { Close(); }

  bool PollNextCommand(TcpStageCommand *command) {
    if (command == nullptr || !EnsureConnected_()) {
      return false;
    }

    if (TryParseNextCommand_(command)) {
      return true;
    }

    std::array<std::uint8_t, 64> recv_chunk{};
    const ssize_t recv_count =
        ::recv(server_fd_, recv_chunk.data(), recv_chunk.size(), 0);
    if (recv_count > 0) {
      recv_buffer_.insert(recv_buffer_.end(), recv_chunk.begin(),
                          recv_chunk.begin() + recv_count);
      return TryParseNextCommand_(command);
    }

    if (recv_count == 0) {
      std::cout << "[TCP阶段] server 断开连接，等待重连" << std::endl;
      CloseConnection_();
      return false;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return false;
    }

    std::cerr << "[TCP阶段] client recv 失败: " << std::strerror(errno)
              << "，等待重连" << std::endl;
    CloseConnection_();
    return false;
  }

  void Close() { CloseConnection_(); }

private:
  bool EnsureConnected_() {
    if (server_fd_ >= 0) {
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_connect_time_) {
      return false;
    }

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      ScheduleReconnect_();
      return false;
    }
    if (!SetNonBlocking_(server_fd_)) {
      CloseConnection_();
      return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(config_.server_port);
    if (::inet_pton(AF_INET, config_.server_ip.c_str(),
                    &server_addr.sin_addr) != 1) {
      std::cerr << "[TCP阶段] 非法 server IP: " << config_.server_ip
                << std::endl;
      CloseConnection_();
      return false;
    }

    const int connect_result =
        ::connect(server_fd_, reinterpret_cast<const sockaddr *>(&server_addr),
                  sizeof(server_addr));
    if (connect_result < 0 && errno != EINPROGRESS) {
      std::cerr << "[TCP阶段] 连接 server 失败: " << std::strerror(errno)
                << "，等待重连" << std::endl;
      CloseConnection_();
      return false;
    }

    if (connect_result < 0) {
      fd_set write_set;
      FD_ZERO(&write_set);
      FD_SET(server_fd_, &write_set);
      timeval timeout{};
      timeout.tv_sec = std::max(0, config_.connect_timeout_ms) / 1000;
      timeout.tv_usec =
          (std::max(0, config_.connect_timeout_ms) % 1000) * 1000;
      const int ready = ::select(server_fd_ + 1, nullptr, &write_set,
                                 nullptr, &timeout);
      if (ready <= 0) {
        std::cerr << "[TCP阶段] 连接 server 超时，等待重连" << std::endl;
        CloseConnection_();
        return false;
      }

      int socket_error = 0;
      socklen_t socket_error_size = sizeof(socket_error);
      if (::getsockopt(server_fd_, SOL_SOCKET, SO_ERROR, &socket_error,
                       &socket_error_size) < 0 ||
          socket_error != 0) {
        std::cerr << "[TCP阶段] 连接 server 失败: "
                  << std::strerror(socket_error != 0 ? socket_error : errno)
                  << "，等待重连" << std::endl;
        CloseConnection_();
        return false;
      }
    }

    next_connect_time_ = std::chrono::steady_clock::time_point{};
    std::cout << "[TCP阶段] client 已连接 server " << config_.server_ip
              << ":" << config_.server_port << std::endl;
    return true;
  }

  bool TryParseNextCommand_(TcpStageCommand *command) {
    while (!recv_buffer_.empty()) {
      std::size_t consumed_size = 0;
      const auto decode_status = TryDecodeTcpStageCommand(
          recv_buffer_.data(), recv_buffer_.size(), command, &consumed_size);
      if (decode_status == TcpStageDecodeStatus::Decoded) {
        receive_logger_.LogDecodedCommand(recv_buffer_.data(), consumed_size,
                                           *command);
        recv_buffer_.erase(recv_buffer_.begin(),
                           recv_buffer_.begin() + consumed_size);
        return true;
      }
      if (decode_status == TcpStageDecodeStatus::NeedMoreData) {
        return false;
      }

      receive_logger_.LogInvalidBytes(recv_buffer_.data(), 1);
      std::cout << "[TCP阶段] 忽略未知命令码 0x" << std::hex
                << static_cast<int>(recv_buffer_.front()) << std::dec
                << std::endl;
      recv_buffer_.erase(recv_buffer_.begin());
    }
    return false;
  }

  static bool SetNonBlocking_(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      std::cerr << "[TCP阶段] 设置 client socket 非阻塞失败: "
                << std::strerror(errno) << std::endl;
      return false;
    }
    return true;
  }

  void ScheduleReconnect_() {
    next_connect_time_ =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(1, config_.reconnect_interval_ms));
  }

  void CloseConnection_() {
    if (server_fd_ >= 0) {
      ::close(server_fd_);
      server_fd_ = -1;
    }
    recv_buffer_.clear();
    ScheduleReconnect_();
  }

  TcpStageClientReceiveConfig config_{};
  TcpStageReceiveLogger receive_logger_{};
  int server_fd_ = -1;
  std::vector<std::uint8_t> recv_buffer_;
  std::chrono::steady_clock::time_point next_connect_time_{};
};

class TcpStageSignalSender {
public:
  explicit TcpStageSignalSender(TcpStageSendConfig config)
      : config_(std::move(config)) {}

  ~TcpStageSignalSender() { Close(); }

  bool SendGameState(std::uint8_t game_progress,
                     std::uint16_t stage_remain_time) {
    if (!EnsureConnected_()) {
      return false;
    }

    const auto payload = EncodeGameStateCommand(game_progress, stage_remain_time);
    return SendPayload_(payload.data(), payload.size());
  }

  bool SendCounteredState(bool countered) {
    if (!EnsureConnected_()) {
      return false;
    }

    const auto payload = EncodeCounteredStateCommand(countered);
    return SendPayload_(payload.data(), payload.size());
  }

  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  bool SendPayload_(const std::uint8_t *payload, std::size_t payload_size) {
    if (payload == nullptr || payload_size == 0) {
      return false;
    }

    std::size_t offset = 0;
    while (offset < payload_size) {
      const ssize_t sent =
          ::send(fd_, payload + offset, payload_size - offset, MSG_NOSIGNAL);
      if (sent > 0) {
        offset += static_cast<std::size_t>(sent);
        continue;
      }

      if (sent < 0 && errno == EINTR) {
        continue;
      }

      std::cerr << "[TCP阶段] send 失败: " << std::strerror(errno)
                << std::endl;
      Close();
      return false;
    }
    return true;
  }

  bool EnsureConnected_() {
    if (fd_ >= 0) {
      return true;
    }

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) {
      std::cerr << "[TCP阶段] 创建 socket 失败: " << std::strerror(errno)
                << std::endl;
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.port);
    if (::inet_pton(AF_INET, config_.host.c_str(), &addr.sin_addr) != 1) {
      std::cerr << "[TCP阶段] 非法目标 IP: " << config_.host << std::endl;
      Close();
      return false;
    }

    if (::connect(fd_, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) <
        0) {
      std::cerr << "[TCP阶段] connect 失败: " << std::strerror(errno)
                << std::endl;
      Close();
      return false;
    }

    std::cout << "[TCP阶段] 已连接 " << config_.host << ":" << config_.port
              << std::endl;
    return true;
  }

  TcpStageSendConfig config_{};
  int fd_ = -1;
};

} // namespace Tools
