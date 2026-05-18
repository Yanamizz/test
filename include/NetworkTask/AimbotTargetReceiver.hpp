/**
 * @file    include/NetworkTask/AimbotTargetReceiver.hpp
 * @brief   提供瞄准目标标志位的网络接收与解析逻辑。
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <string>

#include "NetworkTask/DeviceAServer.hpp"
#include "SerialTask/Common.hpp"

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
    target = SerialTask::kAimbotTargetMin;
    return true;
  }
  if (text == "1" || text == "01" || text == "0X01") {
    target = SerialTask::kAimbotTargetActiveThreshold;
    return true;
  }
  if (!text.empty() && std::all_of(text.begin(), text.end(), [](char c) {
        return c == '0' || c == '1' || c == 'X';
      })) {
    for (auto it = text.rbegin(); it != text.rend(); ++it) {
      if (*it == '0' || *it == '1') {
        target = (*it == '1') ? SerialTask::kAimbotTargetActiveThreshold
                              : SerialTask::kAimbotTargetMin;
        return true;
      }
    }
  }

  bool found_binary_target = false;
  uint8_t binary_target = SerialTask::kAimbotTargetMin;
  for (unsigned char c : message) {
    if (c == SerialTask::kAimbotTargetMin ||
        c == SerialTask::kAimbotTargetActiveThreshold) {
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
                                    int port = kDefaultTcpPort) {
  const auto saturating_increment = [&aimbot_target]() {
    uint8_t old_value = aimbot_target.load(std::memory_order_acquire);
    while (old_value < SerialTask::kAimbotTargetMax) {
      if (aimbot_target.compare_exchange_weak(
              old_value, static_cast<uint8_t>(old_value + 1),
              std::memory_order_acq_rel, std::memory_order_acquire)) {
        return;
      }
    }
  };

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

      uint8_t target = SerialTask::kAimbotTargetMin;
      if (!ParseAimbotTargetMessage(received_content, target)) {
        std::cerr << "[Network] 忽略非法 AimbotTarget 数据" << std::endl;
        continue;
      }

      if (target == SerialTask::kAimbotTargetActiveThreshold) {
        saturating_increment();
      }

      const uint8_t current_target =
          aimbot_target.load(std::memory_order_acquire);
      std::cout << "[Network] AimbotTarget(raw)="
                << static_cast<int>(current_target)
                << " (rx=" << static_cast<int>(target) << ")" << std::endl;
    }

    CloseSocket(client_fd);
  }

  CloseSocket(listen_fd);
}

} // namespace NetworkTask
