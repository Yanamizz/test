#ifndef RADAR_INCLUDE_REFEREE_TCP_CLIENT_HPP
#define RADAR_INCLUDE_REFEREE_TCP_CLIENT_HPP

/**
 * @file  include/referee/tcp_client.hpp
 * @brief 非阻塞 TCP 客户端连接与单会话读流封装
 */

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "include/referee/socket_fd_util.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

inline sockaddr_in MakeTcpClientSockAddr(const std::string &ip, int port, bool allow_any) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (allow_any && (ip.empty() || ip == "0.0.0.0")) {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    return addr;
  }
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    throw std::runtime_error("invalid IPv4 address: " + ip);
  }
  return addr;
}

/**
 * @brief 维护一个非阻塞 TCP 客户端连接
 */
class TcpClient {
 public:
  using Clock = std::chrono::steady_clock;

  TcpClient() = default;

  TcpClient(const TcpClient &) = delete;
  TcpClient &operator=(const TcpClient &) = delete;

  TcpClient(TcpClient &&other) noexcept { MoveFrom(std::move(other)); }

  TcpClient &operator=(TcpClient &&other) noexcept {
    if (this != &other) {
      Close();
      MoveFrom(std::move(other));
    }
    return *this;
  }

  ~TcpClient() { Close(); }

  /**
   * @brief 发起一次非阻塞 TCP 连接
   * @param server_address 对端服务端 IPv4 地址
   * @param port 对端端口
   * @param local_bind_address 本机绑定地址，留空表示由系统自动选择
   */
  void Open(std::string server_address, int port, std::string local_bind_address = {}) {
    Close();
    if (port <= 0 || port > 65535) {
      throw std::runtime_error("invalid TCP remote port: " + std::to_string(port));
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      throw std::runtime_error("failed to create TCP socket: " + std::string(std::strerror(errno)));
    }

    try {
      SetFdNonBlocking(fd, "TCP client socket");
      if (!local_bind_address.empty()) {
        const auto local_addr = MakeTcpClientSockAddr(local_bind_address, 0, false);
        if (::bind(fd, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
          throw std::runtime_error("failed to bind TCP client " + local_bind_address + ": " +
                                   std::string(std::strerror(errno)));
        }
      }

      const auto remote_addr = MakeTcpClientSockAddr(server_address, port, false);
      const int result = ::connect(fd, reinterpret_cast<const sockaddr *>(&remote_addr), sizeof(remote_addr));
      if (result != 0 && errno != EINPROGRESS) {
        throw std::runtime_error("failed to connect TCP " + server_address + ":" + std::to_string(port) + ": " +
                                 std::string(std::strerror(errno)));
      }

      fd_ = fd;
      server_address_ = std::move(server_address);
      local_bind_address_ = std::move(local_bind_address);
      port_ = port;
      connect_start_time_ = Clock::now();
      if (result == 0) {
        connected_ = true;
        peer_ip_ = server_address_;
        last_read_time_ = connect_start_time_;
      }
    } catch (...) {
      ::close(fd);
      throw;
    }
  }

  /**
   * @brief 尝试发起一次非阻塞 TCP 连接
   */
  bool TryOpen(std::string server_address, int port, std::string local_bind_address, std::string *error = nullptr) {
    try {
      Open(std::move(server_address), port, std::move(local_bind_address));
      return true;
    } catch (const std::exception &ex) {
      if (error != nullptr) {
        *error = ex.what();
      }
      return false;
    }
  }

  /**
   * @brief 在 poll 可写后确认连接是否真正建立
   * @return 连接已建立时返回 true
   */
  bool FinishConnect(std::string *error = nullptr) {
    if (!is_open()) {
      if (error != nullptr) {
        *error = "TCP client socket is closed";
      }
      return false;
    }
    if (connected_) {
      return true;
    }

    int socket_error = 0;
    socklen_t socket_error_len = sizeof(socket_error);
    if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) != 0) {
      if (error != nullptr) {
        *error = "getsockopt(SO_ERROR) failed: " + std::string(std::strerror(errno));
      }
      Close();
      return false;
    }
    if (socket_error != 0) {
      if (error != nullptr) {
        *error = "failed to connect TCP " + server_address_ + ":" + std::to_string(port_) + ": " +
                 std::string(std::strerror(socket_error));
      }
      Close();
      return false;
    }

    connected_ = true;
    peer_ip_ = server_address_;
    last_read_time_ = Clock::now();
    return true;
  }

  /**
   * @brief 检查当前连接尝试是否超时，若超时则关闭 socket
   */
  bool CloseIfConnectTimedOut(int timeout_ms) {
    if (!is_connecting() || timeout_ms <= 0 || !connect_start_time_.has_value()) {
      return false;
    }
    if (Clock::now() - *connect_start_time_ < std::chrono::milliseconds(timeout_ms)) {
      return false;
    }
    Close();
    return true;
  }

  /**
   * @brief 从已连接对端读取字节流
   */
  std::size_t Read(rm::u8 *data, std::size_t size) {
    if (!connected_ || fd_ < 0) {
      return 0;
    }

    while (true) {
      const ssize_t result = ::recv(fd_, data, size, 0);
      if (result > 0) {
        last_read_time_ = Clock::now();
        return static_cast<std::size_t>(result);
      }
      if (result == 0) {
        Close();
        return 0;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
      }
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("TCP read failed: " + std::string(std::strerror(errno)));
    }
  }

  /**
   * @brief 在连接空闲超时后关闭 socket
   */
  bool CloseIdleClientIfTimedOut(int timeout_ms) {
    if (!connected_ || !last_read_time_.has_value() || timeout_ms <= 0) {
      return false;
    }
    if (Clock::now() - *last_read_time_ < std::chrono::milliseconds(timeout_ms)) {
      return false;
    }
    Close();
    return true;
  }

  void Stop() { Close(); }

  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    connected_ = false;
    peer_ip_.clear();
    connect_start_time_.reset();
    last_read_time_.reset();
  }

  int fd() const { return fd_; }
  int port() const { return port_; }
  bool is_open() const { return fd_ >= 0; }
  bool is_connected() const { return connected_; }
  bool is_connecting() const { return fd_ >= 0 && !connected_; }
  const std::string &server_address() const { return server_address_; }
  const std::string &local_bind_address() const { return local_bind_address_; }
  const std::string &peer_ip() const { return peer_ip_; }

 private:
  void MoveFrom(TcpClient &&other) {
    fd_ = other.fd_;
    connected_ = other.connected_;
    port_ = other.port_;
    server_address_ = std::move(other.server_address_);
    local_bind_address_ = std::move(other.local_bind_address_);
    peer_ip_ = std::move(other.peer_ip_);
    connect_start_time_ = other.connect_start_time_;
    last_read_time_ = other.last_read_time_;

    other.fd_ = -1;
    other.connected_ = false;
    other.port_ = 0;
    other.connect_start_time_.reset();
    other.last_read_time_.reset();
  }

  int fd_ = -1;
  bool connected_ = false;
  int port_ = 0;
  std::string server_address_;
  std::string local_bind_address_;
  std::string peer_ip_;
  std::optional<Clock::time_point> connect_start_time_;
  std::optional<Clock::time_point> last_read_time_;
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_TCP_CLIENT_HPP
