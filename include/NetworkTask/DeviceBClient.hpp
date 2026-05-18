/**
 * @file    include/NetworkTask/DeviceBClient.hpp
 * @brief   提供 TCP 客户端连接服务端的基础能力。
 */

#pragma once

#include <cstdint>
#include <string>

#include "NetworkTask/SocketCommon.hpp"

namespace NetworkTask {

inline bool ConnectToServer(socket_t &fd,
                            const char *server_ip = "192.168.10.2",
                            int port = 5000) {
  if (!PrepareSocketRuntime()) {
    return false;
  }

  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == kInvalidSocketFd) {
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(static_cast<uint16_t>(port));

  if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
    CloseSocket(fd);
    fd = kInvalidSocketFd;
    return false;
  }

  if (connect(fd, reinterpret_cast<sockaddr *>(&server_addr),
              sizeof(server_addr)) < 0) {
    CloseSocket(fd);
    fd = kInvalidSocketFd;
    return false;
  }

  return true;
}

} // namespace NetworkTask
