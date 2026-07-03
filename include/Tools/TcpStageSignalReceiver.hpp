/**
 * @file    include/Tools/TcpStageSignalReceiver.hpp
 * @brief   监听 TCP 阶段信号并按字节输出 0x00/0x01。
 *
 * 该模块提供一个轻量监听器：主程序作为 TCP server 监听外部阶段脉冲输入，
 * 每次收到单字节 `0x00` 或 `0x01` 时交给上层做边沿判定。它只负责网络连接
 * 生命周期与字节接收，不直接修改阶段状态机。
 */

#pragma once

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace Tools {

struct TcpStageSignalConfig {
  std::string bind_ip = "0.0.0.0";
  std::uint16_t bind_port = 19001;
  int backlog = 1;
};

struct TcpStageSendConfig {
  std::string host = "127.0.0.1";
  std::uint16_t port = 19001;
};

class TcpStageSignalReceiver {
public:
  explicit TcpStageSignalReceiver(TcpStageSignalConfig config)
      : config_(std::move(config)) {}

  ~TcpStageSignalReceiver() { Close(); }

  bool PollNextByte(std::uint8_t *value) {
    if (value == nullptr) {
      return false;
    }

    if (listen_fd_ < 0 && !EnsureListening_()) {
      return false;
    }

    if (client_fd_ < 0 && !TryAcceptClient_()) {
      return false;
    }

    std::uint8_t byte = 0;
    const ssize_t recv_count = ::recv(client_fd_, &byte, sizeof(byte), 0);
    if (recv_count == 1) {
      if (byte == 0x00 || byte == 0x01) {
        *value = byte;
        return true;
      }
      std::cout << "[TCP阶段] 忽略非法字节 0x" << std::hex
                << static_cast<int>(byte) << std::dec << std::endl;
      return false;
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

  static bool SendSingleByte(const std::string &host, std::uint16_t port,
                             std::uint8_t value) {
    if (value != 0x00 && value != 0x01) {
      std::cerr << "[TCP阶段] 只允许发送 0x00 或 0x01" << std::endl;
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

    const ssize_t sent = ::send(fd, &value, sizeof(value), 0);
    if (sent != static_cast<ssize_t>(sizeof(value))) {
      std::cerr << "[TCP阶段] send 失败: " << std::strerror(errno)
                << std::endl;
      ::close(fd);
      return false;
    }

    ::close(fd);
    return true;
  }

  void Close() {
    CloseClient_();
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
  }

private:
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
  }

  TcpStageSignalConfig config_{};
  int listen_fd_ = -1;
  int client_fd_ = -1;
};

class TcpStageSignalSender {
public:
  explicit TcpStageSignalSender(TcpStageSendConfig config)
      : config_(std::move(config)) {}

  ~TcpStageSignalSender() { Close(); }

  bool SendByte(std::uint8_t value) {
    if (value != 0x00 && value != 0x01) {
      std::cerr << "[TCP阶段] 只允许发送 0x00 或 0x01" << std::endl;
      return false;
    }

    if (!EnsureConnected_()) {
      return false;
    }

    const ssize_t sent = ::send(fd_, &value, sizeof(value), 0);
    if (sent == static_cast<ssize_t>(sizeof(value))) {
      return true;
    }

    std::cerr << "[TCP阶段] send 失败: " << std::strerror(errno) << std::endl;
    Close();
    return false;
  }

  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
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
