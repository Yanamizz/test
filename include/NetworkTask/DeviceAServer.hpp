/**
 * @file    include/NetworkTask/DeviceAServer.hpp
 * @brief   提供 TCP 监听套接字创建与客户端接入能力。
 */

#pragma once

#include <cstdint>
#include <string>

#include "NetworkTask/SocketCommon.hpp"

namespace NetworkTask {

inline bool CreateListeningSocket(socket_t &listen_fd,
                                  int port = kDefaultTcpPort) {
  if (!PrepareSocketRuntime()) {
    return false;
  }

  listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd == kInvalidSocketFd) {
    return false;
  }

  if (!SetReuseAddress(listen_fd)) {
    CloseSocket(listen_fd);
    listen_fd = kInvalidSocketFd;
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(static_cast<uint16_t>(port));

  if (bind(listen_fd, reinterpret_cast<sockaddr *>(&server_addr),
           sizeof(server_addr)) < 0) {
    CloseSocket(listen_fd);
    listen_fd = kInvalidSocketFd;
    return false;
  }

  if (listen(listen_fd, 1) < 0) {
    CloseSocket(listen_fd);
    listen_fd = kInvalidSocketFd;
    return false;
  }

  return true;
}

inline bool AcceptClient(socket_t listen_fd, socket_t &client_fd,
                         std::string *client_ip = nullptr) {
  sockaddr_in client_addr{};

#ifdef _WIN32
  int client_len = sizeof(client_addr);
#else
  socklen_t client_len = sizeof(client_addr);
#endif

  client_fd = accept(listen_fd, reinterpret_cast<sockaddr *>(&client_addr),
                     &client_len);
  if (client_fd == kInvalidSocketFd) {
    return false;
  }

  if (client_ip != nullptr) {
    char buffer[64] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, buffer, sizeof(buffer));
    *client_ip = buffer;
  }

  return true;
}

} // namespace NetworkTask
