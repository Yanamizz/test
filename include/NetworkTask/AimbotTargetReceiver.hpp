#pragma once

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <string>

#include "NetworkTask/DeviceAServer.hpp"

namespace NetworkTask {

inline bool ParseAimbotTargetMessage(const std::string &message,
                                     uint8_t &target) {
  std::string text;
  text.reserve(message.size());
  for (unsigned char c : message) {
    if (std::isprint(c) && !std::isspace(c)) {
      text.push_back(static_cast<char>(std::toupper(c)));
    }
  }

  if (text == "0" || text == "00" || text == "0X00") {
    target = 0x00;
    return true;
  }
  if (text == "1" || text == "01" || text == "0X01") {
    target = 0x01;
    return true;
  }
  if (!text.empty() && std::all_of(text.begin(), text.end(), [](char c) {
        return c == '0' || c == '1' || c == 'X';
      })) {
    for (auto it = text.rbegin(); it != text.rend(); ++it) {
      if (*it == '0' || *it == '1') {
        target = (*it == '1') ? 0x01 : 0x00;
        return true;
      }
    }
  }

  bool found_binary_target = false;
  uint8_t binary_target = 0x00;
  for (unsigned char c : message) {
    if (c == 0x00 || c == 0x01) {
      binary_target = c;
      found_binary_target = true;
    }
  }
  if (!found_binary_target) {
    return false;
  }

  target = binary_target;
  return true;
}

template <typename RunningPredicate>
inline void RunAimbotTargetReceiver(std::atomic<uint8_t> &aimbot_target,
                                    RunningPredicate is_running,
                                    int port = 5000) {
  socket_t listen_fd = kInvalidSocketFd;
  if (!CreateListeningSocket(listen_fd, port)) {
    std::cerr << "[Network] AimbotTarget 监听端口 " << port << " 失败"
              << std::endl;
    return;
  }

  std::cout << "[Network] AimbotTarget 接收端已启动，监听端口：" << port
            << std::endl;

  while (is_running()) {
    socket_t client_fd = kInvalidSocketFd;
    std::string client_ip;
    while (is_running()) {
      if (!WaitForReadable(listen_fd, 100)) {
        continue;
      }
      if (AcceptClient(listen_fd, client_fd, &client_ip)) {
        break;
      }
    }

    if (!is_running() || client_fd == kInvalidSocketFd) {
      break;
    }

    std::cout << "[Network] AimbotTarget 发送端已连接，IP：" << client_ip
              << std::endl;

    while (is_running()) {
      if (!WaitForReadable(client_fd, 100)) {
        continue;
      }

      std::string received_content;
      if (!ReceiveText(client_fd, received_content)) {
        std::cout << "[Network] AimbotTarget 发送端已断开" << std::endl;
        break;
      }

      uint8_t target = 0x00;
      if (!ParseAimbotTargetMessage(received_content, target)) {
        std::cerr << "[Network] 忽略非法 AimbotTarget 数据" << std::endl;
        continue;
      }

      aimbot_target.store(target, std::memory_order_release);
      std::cout << "[Network] AimbotTarget=" << static_cast<int>(target)
                << std::endl;
    }

    CloseSocket(client_fd);
  }

  CloseSocket(listen_fd);
}

} // namespace NetworkTask
