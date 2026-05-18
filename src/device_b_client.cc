#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>

#include "NetworkTask/NetworkTask.hpp"
#include "SerialTask/Common.hpp"

namespace {

std::atomic<bool> g_running{true};

void HandleSignal(int) { g_running.store(false); }

bool ParseAimbotTarget(const std::string &content, uint8_t &target) {
  bool has_target = false;

  for (unsigned char data : content) {
    if (data == SerialTask::kAimbotTargetMin ||
        data == SerialTask::kAimbotTargetActiveThreshold) {
      target = data;
      has_target = true;
    } else if (data == '0' || data == '1') {
      target = static_cast<uint8_t>(data - '0');
      has_target = true;
    }
  }

  return has_target;
}

} // namespace

int main() {
  std::signal(SIGINT, HandleSignal);

  NetworkTask::socket_t listen_fd = NetworkTask::kInvalidSocketFd;
  if (!NetworkTask::CreateListeningSocket(listen_fd,
                                          NetworkTask::kDefaultTcpPort)) {
    std::cerr << "设备 B 创建监听 socket 失败\n";
    return 1;
  }

  std::cout << "设备 B 接收端已启动，监听端口："
            << NetworkTask::kDefaultTcpPort << "\n";
  std::cout << "等待设备 A 连接，按 Ctrl+C 退出。\n";

  uint8_t aimbot_target = SerialTask::kAimbotTargetMin;
  while (g_running.load()) {
    NetworkTask::socket_t client_fd = NetworkTask::kInvalidSocketFd;
    std::string client_ip;

    while (g_running.load()) {
      if (!NetworkTask::WaitForReadable(listen_fd, 100)) {
        continue;
      }
      if (NetworkTask::AcceptClient(listen_fd, client_fd, &client_ip)) {
        break;
      }
    }

    if (!g_running.load() || client_fd == NetworkTask::kInvalidSocketFd) {
      break;
    }

    std::cout << "设备 A 已连接，IP：" << client_ip << "\n";

    while (g_running.load()) {
      if (!NetworkTask::WaitForReadable(client_fd, 100)) {
        continue;
      }

      std::string received_content;
      if (!NetworkTask::ReceiveText(client_fd, received_content)) {
        std::cout << "设备 A 已断开，重新等待连接。\n";
        break;
      }

      if (ParseAimbotTarget(received_content, aimbot_target)) {
        std::cout << "AimbotTarget=" << static_cast<int>(aimbot_target)
                  << "\n";
      } else {
        std::cout << "忽略非法数据\n";
      }
    }

    NetworkTask::CloseSocket(client_fd);
  }

  NetworkTask::CloseSocket(listen_fd);
  std::cout << "设备 B 程序退出\n";
  return 0;
}
