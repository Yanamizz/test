#ifndef RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP
#define RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP

/**
 * @file  include/referee/tcp_server.hpp
 * @brief 单会话非阻塞 TCP server 封装
 */

#include <array>
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
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#include "include/referee/socket_fd_util.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

inline sockaddr_in MakeTcpServerSockAddr(const std::string &ip, int port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (ip.empty() || ip == "0.0.0.0") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    return addr;
  }
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    throw std::runtime_error("invalid IPv4 address: " + ip);
  }
  return addr;
}

inline std::string PeerIpFromSockAddr(const sockaddr_in &addr) {
  std::array<char, INET_ADDRSTRLEN> buffer{};
  const char *result = ::inet_ntop(AF_INET, &addr.sin_addr, buffer.data(), buffer.size());
  return result == nullptr ? std::string() : std::string(result);
}

/**
 * @brief 维护一个监听 socket 与最多一个已接入客户端
 */
class TcpServer {
 public:
  using Clock = std::chrono::steady_clock;

  TcpServer() = default;

  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

  TcpServer(TcpServer &&other) noexcept { MoveFrom(std::move(other)); }

  TcpServer &operator=(TcpServer &&other) noexcept {
    if (this != &other) {
      Close();
      MoveFrom(std::move(other));
    }
    return *this;
  }

  ~TcpServer() { Close(); }

  /**
   * @brief 启动一个非阻塞 TCP server 监听
   * @param bind_address 本机绑定地址；留空或 `0.0.0.0` 表示监听所有网卡
   * @param port 监听端口
   */
  void Open(std::string bind_address, int port) {
    Close();
    if (port <= 0 || port > 65535) {
      throw std::runtime_error("invalid TCP server port: " + std::to_string(port));
    }

    const int listener_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener_fd < 0) {
      throw std::runtime_error("failed to create TCP server socket: " + std::string(std::strerror(errno)));
    }

    try {
      int enable_reuseaddr = 1;
      if (::setsockopt(listener_fd, SOL_SOCKET, SO_REUSEADDR, &enable_reuseaddr, sizeof(enable_reuseaddr)) != 0) {
        throw std::runtime_error("failed to set SO_REUSEADDR on TCP server socket: " +
                                 std::string(std::strerror(errno)));
      }
      SetFdNonBlocking(listener_fd, "TCP server listener");
      const auto bind_addr = MakeTcpServerSockAddr(bind_address, port);
      if (::bind(listener_fd, reinterpret_cast<const sockaddr *>(&bind_addr), sizeof(bind_addr)) != 0) {
        throw std::runtime_error("failed to bind TCP " + EffectiveBindAddress(bind_address) + ":" +
                                 std::to_string(port) + ": " + std::string(std::strerror(errno)));
      }
      if (::listen(listener_fd, 1) != 0) {
        throw std::runtime_error("failed to listen TCP " + EffectiveBindAddress(bind_address) + ":" +
                                 std::to_string(port) + ": " + std::string(std::strerror(errno)));
      }

      listener_fd_ = listener_fd;
      bind_address_ = EffectiveBindAddress(bind_address);
      port_ = port;
    } catch (...) {
      ::close(listener_fd);
      throw;
    }
  }

  /**
   * @brief 尝试启动一个非阻塞 TCP server 监听
   */
  bool TryOpen(std::string bind_address, int port, std::string *error = nullptr) {
    try {
      Open(std::move(bind_address), port);
      return true;
    } catch (const std::exception &ex) {
      if (error != nullptr) {
        *error = ex.what();
      }
      return false;
    }
  }

  /**
   * @brief 接受一个待接入的客户端
   * @param error 发生真实错误时返回错误信息；无待接入连接时保持为空
   * @return 是否成功接受到客户端
   */
  bool AcceptPending(std::string *error = nullptr) {
    if (!is_open()) {
      if (error != nullptr) {
        *error = "TCP server listener is closed";
      }
      return false;
    }
    if (has_client()) {
      if (error != nullptr) {
        *error = "TCP server already has an active client";
      }
      return false;
    }

    while (true) {
      sockaddr_in peer_addr{};
      socklen_t peer_addr_len = sizeof(peer_addr);
      const int client_fd = ::accept(listener_fd_, reinterpret_cast<sockaddr *>(&peer_addr), &peer_addr_len);
      if (client_fd >= 0) {
        try {
          SetFdNonBlocking(client_fd, "TCP server client");
          client_fd_ = client_fd;
          peer_ip_ = PeerIpFromSockAddr(peer_addr);
          last_read_time_ = Clock::now();
          return true;
        } catch (...) {
          ::close(client_fd);
          throw;
        }
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to accept TCP client on " + bind_address_ + ":" + std::to_string(port_) + ": " +
                 std::string(std::strerror(errno));
      }
      return false;
    }
  }

  /**
   * @brief 读取当前已接入客户端发来的字节流
   */
  std::size_t Read(rm::u8 *data, std::size_t size) {
    if (!has_client() || client_fd_ < 0) {
      return 0;
    }

    while (true) {
      const ssize_t result = ::recv(client_fd_, data, size, 0);
      if (result > 0) {
        last_read_time_ = Clock::now();
        return static_cast<std::size_t>(result);
      }
      if (result == 0) {
        CloseClient();
        return 0;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
      }
      if (errno == EINTR) {
        continue;
      }
      // ECONNRESET 等非致命连接错误：关闭客户端，等待主循环重新接受，不抛出异常
      CloseClient();
      return 0;
    }
  }

  /**
   * @brief 向当前已接入客户端完整写出一帧数据
   */
  bool TryWriteAll(const rm::u8 *data, std::size_t size, std::string *error = nullptr) const {
    if (!has_client()) {
      if (error != nullptr) {
        *error = "TCP server write attempted while no client is connected";
      }
      return false;
    }
    std::size_t written = 0;
    while (written < size) {
      const ssize_t result = ::send(client_fd_, data + written, size - written, MSG_NOSIGNAL);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::string wait_error;
          if (!WaitWritable(&wait_error)) {
            if (error != nullptr) {
              *error = wait_error;
            }
            return false;
          }
          continue;
        }
        if (error != nullptr) {
          *error = "TCP server write failed: " + std::string(std::strerror(errno));
        }
        return false;
      }
      if (result == 0) {
        if (error != nullptr) {
          *error = "TCP server write returned 0";
        }
        return false;
      }
      written += static_cast<std::size_t>(result);
    }
    return true;
  }

  /**
   * @brief 在连接空闲超时后关闭当前已接入客户端
   */
  bool CloseIdleClientIfTimedOut(int timeout_ms) {
    if (!has_client() || !last_read_time_.has_value() || timeout_ms <= 0) {
      return false;
    }
    if (Clock::now() - *last_read_time_ < std::chrono::milliseconds(timeout_ms)) {
      return false;
    }
    CloseClient();
    return true;
  }

  void CloseClient() {
    if (client_fd_ >= 0) {
      ::close(client_fd_);
      client_fd_ = -1;
    }
    peer_ip_.clear();
    last_read_time_.reset();
  }

  void Close() {
    CloseClient();
    if (listener_fd_ >= 0) {
      ::close(listener_fd_);
      listener_fd_ = -1;
    }
  }

  int fd() const { return listener_fd_; }
  int client_fd() const { return client_fd_; }
  int port() const { return port_; }
  bool is_open() const { return listener_fd_ >= 0; }
  bool has_client() const { return client_fd_ >= 0; }
  const std::string &bind_address() const { return bind_address_; }
  const std::string &peer_ip() const { return peer_ip_; }

 private:
  static std::string EffectiveBindAddress(const std::string &bind_address) {
    return bind_address.empty() ? "0.0.0.0" : bind_address;
  }

  bool WaitWritable(std::string *error = nullptr) const {
    pollfd fd{client_fd_, POLLOUT, 0};
    while (true) {
      const int result = ::poll(&fd, 1, kWritePollTimeoutMs);
      if (result > 0) {
        if ((fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          if (error != nullptr) {
            *error = "TCP server client became unwritable";
          }
          return false;
        }
        return true;
      }
      if (result == 0) {
        if (error != nullptr) {
          *error = "poll timeout while waiting TCP server client writable";
        }
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = "poll failed while waiting TCP server client writable: " + std::string(std::strerror(errno));
      }
      return false;
    }
  }

  void MoveFrom(TcpServer &&other) {
    listener_fd_ = other.listener_fd_;
    client_fd_ = other.client_fd_;
    port_ = other.port_;
    bind_address_ = std::move(other.bind_address_);
    peer_ip_ = std::move(other.peer_ip_);
    last_read_time_ = other.last_read_time_;

    other.listener_fd_ = -1;
    other.client_fd_ = -1;
    other.port_ = 0;
    other.last_read_time_.reset();
  }

  static constexpr int kWritePollTimeoutMs = 1000;

  int listener_fd_ = -1;
  int client_fd_ = -1;
  int port_ = 0;
  std::string bind_address_;
  std::string peer_ip_;
  std::optional<Clock::time_point> last_read_time_;
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP
