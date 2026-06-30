#ifndef RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP
#define RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP

/**
 * @file  include/referee/tcp_server.hpp
 * @brief 非阻塞 TCP 监听与单客户端会话封装
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
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "include/config/config.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

/**
 * @brief 将文件描述符设置为非阻塞模式
 * @param fd 目标文件描述符
 * @param name 出错时用于日志提示的名字
 */
inline void SetFdNonBlocking(int fd, const std::string &name) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    throw std::runtime_error("fcntl(F_GETFL) failed for " + name + ": " + std::strerror(errno));
  }
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
    throw std::runtime_error("fcntl(F_SETFL) failed for " + name + ": " + std::strerror(errno));
  }
}

/**
 * @brief 维护一个单监听端口、单活动客户端的 TCP 服务端
 * @note  当前主程序使用该类接入信息波与敌方密钥两个 TCP 入口。
 */
class TcpServer {
 public:
  using Clock = std::chrono::steady_clock;

  TcpServer() = default;

  TcpServer(std::string bind_address, int port) { Open(std::move(bind_address), port); }

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
   * @brief 创建并绑定 TCP 服务端
   * @param bind_address 监听地址
   * @param port 监听端口
   */
  void Open(std::string bind_address, int port) {
    Close();
    if (port <= 0 || port > 65535) {
      throw std::runtime_error("invalid TCP listen port: " + std::to_string(port));
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      throw std::runtime_error("failed to create TCP socket: " + std::string(std::strerror(errno)));
    }

    const int reuse = 1;
    if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
      const auto message = std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(message);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind_address.empty() || bind_address == "0.0.0.0") {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
      const auto message = "invalid TCP bind address: " + bind_address;
      ::close(fd);
      throw std::runtime_error(message);
    }

    if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
      const auto message = "failed to bind TCP " + bind_address + ":" + std::to_string(port) + ": " +
                           std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(message);
    }

    if (::listen(fd, 1) != 0) {
      const auto message = std::string("failed to listen on TCP socket: ") + std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(message);
    }

    SetFdNonBlocking(fd, "TCP listen socket");
    listen_fd_ = fd;
    bind_address_ = std::move(bind_address);
    port_ = port;
  }

  /**
   * @brief 尝试创建并绑定 TCP 服务端
   * @param bind_address 监听地址
   * @param port 监听端口
   * @param error 失败时输出错误信息
   * @return 绑定成功返回 true，失败返回 false
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
   * @brief 接收所有待处理连接
   * @note  如果当前已有活动客户端，新连接会被直接拒绝。
   */
  void AcceptPending() {
    while (true) {
      sockaddr_in peer_addr{};
      socklen_t peer_len = sizeof(peer_addr);
      const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&peer_addr), &peer_len);
      if (fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("TCP accept failed: " + std::string(std::strerror(errno)));
      }

      char peer_ip[INET_ADDRSTRLEN] = {};
      const char *peer_text = ::inet_ntop(AF_INET, &peer_addr.sin_addr, peer_ip, sizeof(peer_ip));
      const std::string peer = peer_text != nullptr ? peer_text : "";
      if (!IsAllowedPeer(peer)) {
        ::close(fd);
        continue;
      }

      if (has_client()) {
        ::close(fd);
        continue;
      }

      SetFdNonBlocking(fd, "TCP client socket");
      client_fd_ = fd;
      client_peer_ip_ = peer;
      last_accept_time_ = Clock::now();
      last_read_time_ = last_accept_time_;
    }
  }

  /**
   * @brief 从当前活动客户端读取字节流
   * @param data 读缓冲区
   * @param size 缓冲区大小
   * @return 实际读取字节数；若客户端关闭则返回 0
   */
  std::size_t Read(rm::u8 *data, std::size_t size) {
    if (client_fd_ < 0) {
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
      throw std::runtime_error("TCP read failed: " + std::string(std::strerror(errno)));
    }
  }

  /**
   * @brief 主动关闭当前客户端连接
   */
  void CloseClient() {
    if (client_fd_ >= 0) {
      ::close(client_fd_);
      client_fd_ = -1;
    }
    client_peer_ip_.clear();
    last_read_time_.reset();
  }

  /**
   * @brief 停止整个 TCP 服务端
   */
  void Stop() { Close(); }

  /**
   * @brief 在客户端长时间无数据时将其断开
   * @param timeout_ms 空闲超时阈值
   * @return 若本次确实关闭了客户端则返回 true
   */
  bool CloseIdleClientIfTimedOut(int timeout_ms) {
    if (!has_client() || !last_read_time_.has_value() || timeout_ms <= 0) {
      return false;
    }
    const auto now = Clock::now();
    if (now - *last_read_time_ < std::chrono::milliseconds(timeout_ms)) {
      return false;
    }
    CloseClient();
    return true;
  }

  int listen_fd() const { return listen_fd_; }
  int client_fd() const { return client_fd_; }
  int port() const { return port_; }
  bool is_open() const { return listen_fd_ >= 0; }
  bool has_client() const { return client_fd_ >= 0; }
  const std::string &client_peer_ip() const { return client_peer_ip_; }

 private:
  /**
   * @brief 判断指定 IP 是否允许接入
   * @param peer_ip 对端 IP
   * @return 满足白名单约束时返回 true
   */
  bool IsAllowedPeer(const std::string &peer_ip) const {
    const std::string allowed = radar::config::kAllowedTcpPeerIp;
    return allowed.empty() || peer_ip == allowed;
  }

  void Close() {
    CloseClient();
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
  }

  void MoveFrom(TcpServer &&other) {
    listen_fd_ = other.listen_fd_;
    client_fd_ = other.client_fd_;
    bind_address_ = std::move(other.bind_address_);
    client_peer_ip_ = std::move(other.client_peer_ip_);
    port_ = other.port_;
    last_accept_time_ = other.last_accept_time_;
    last_read_time_ = other.last_read_time_;
    other.listen_fd_ = -1;
    other.client_fd_ = -1;
  }

  int listen_fd_ = -1;                             ///< 监听 socket
  int client_fd_ = -1;                             ///< 当前活动客户端 socket
  std::string bind_address_;                       ///< 当前绑定地址
  std::string client_peer_ip_;                     ///< 当前活动客户端 IP
  int port_ = 0;                                   ///< 当前监听端口
  std::optional<Clock::time_point> last_accept_time_;  ///< 最近一次 accept 时间
  std::optional<Clock::time_point> last_read_time_;    ///< 最近一次成功读到数据的时间
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_TCP_SERVER_HPP
