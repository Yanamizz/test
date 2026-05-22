/**
 * @file    include/NetworkTask/SocketCommon.hpp
 * @brief   封装跨平台套接字初始化、收发、关闭与等待等公共操作。
 */

#pragma once

#include <cstdlib>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#endif

namespace NetworkTask {

inline constexpr int kDefaultTcpPort = 5000;
inline constexpr const char *kDefaultPeerIp = "192.168.12.2";

#ifdef _WIN32
using socket_t = SOCKET;
constexpr socket_t kInvalidSocketFd = INVALID_SOCKET;
#else
using socket_t = int;
constexpr socket_t kInvalidSocketFd = -1;
#endif

inline bool PrepareSocketRuntime() {
#ifdef _WIN32
  static const bool initialized = []() {
    WSADATA wsa_data{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
      return false;
    }
    std::atexit([]() { WSACleanup(); });
    return true;
  }();
  return initialized;
#else
  static const bool initialized = []() {
    signal(SIGPIPE, SIG_IGN);
    return true;
  }();
  return initialized;
#endif
}

inline void CloseSocket(socket_t fd) {
  if (fd == kInvalidSocketFd) {
    return;
  }
#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif
}

inline void ShutdownSocket(socket_t fd) {
  if (fd == kInvalidSocketFd) {
    return;
  }
#ifdef _WIN32
  shutdown(fd, SD_BOTH);
#else
  shutdown(fd, SHUT_RDWR);
#endif
}

inline bool SetReuseAddress(socket_t fd) {
  int opt = 1;
#ifdef _WIN32
  return setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&opt), sizeof(opt)) == 0;
#else
  return setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == 0;
#endif
}

inline bool WaitForReadable(socket_t fd, int timeout_ms) {
  if (fd == kInvalidSocketFd) {
    return false;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd, &read_fds);

  timeval timeout{};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

#ifdef _WIN32
  const int ready = select(0, &read_fds, nullptr, nullptr, &timeout);
#else
  const int ready = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
#endif
  return ready > 0 && FD_ISSET(fd, &read_fds);
}

inline bool SendAll(socket_t fd, const std::string &data) {
  size_t total = 0;

  while (total < data.size()) {
#ifdef _WIN32
    int n = send(fd, data.data() + total, static_cast<int>(data.size() - total), 0);
#else
    ssize_t n = send(fd, data.data() + total, data.size() - total, 0);
#endif
    if (n <= 0) {
      return false;
    }
    total += static_cast<size_t>(n);
  }

  return true;
}

inline bool SendText(socket_t fd, const std::string &text) { return SendAll(fd, text); }

inline bool ReceiveText(socket_t fd, std::string &out_text) {
  char buffer[1024];

#ifdef _WIN32
  int n = recv(fd, buffer, sizeof(buffer) - 1, 0);
#else
  ssize_t n = recv(fd, buffer, sizeof(buffer) - 1, 0);
#endif
  if (n <= 0) {
    return false;
  }

  out_text.assign(buffer, buffer + n);
  return true;
}

}  // namespace NetworkTask
