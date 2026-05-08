#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "NetworkTask/DeviceBClient.hpp"

namespace {

std::atomic<bool> g_running{true};

void HandleSignal(int) { g_running.store(false); }

bool ConnectWithRetry(NetworkTask::socket_t &fd, const char *server_ip,
                      int port) {
  while (g_running.load()) {
    if (NetworkTask::ConnectToServer(fd, server_ip, port)) {
      return true;
    }

    std::cout << "连接设备 B 失败，500ms 后重试...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return false;
}

bool ParseInputTarget(const std::string &input, uint8_t &target) {
  if (input == "0" || input == "0x00" || input == "0X00") {
    target = 0x00;
    return true;
  }
  if (input == "1" || input == "0x01" || input == "0X01") {
    target = 0x01;
    return true;
  }
  return false;
}

bool SendAimbotTarget(NetworkTask::socket_t fd, uint8_t target) {
  const std::string payload(1, static_cast<char>(target));
  return NetworkTask::SendText(fd, payload);
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, HandleSignal);

  const char *receiver_ip = argc > 1 ? argv[1] : "192.168.10.2";
  constexpr int kReceiverPort = 5000;

  NetworkTask::socket_t fd = NetworkTask::kInvalidSocketFd;
  if (!ConnectWithRetry(fd, receiver_ip, kReceiverPort)) {
    std::cout << "设备 A 程序退出\n";
    return 0;
  }

  std::cout << "设备 A 已连接设备 B：" << receiver_ip << ":" << kReceiverPort
            << "\n";
  std::cout << "输入 0 发送 0x00，输入 1 发送 0x01，输入 quit 退出。\n";

  std::string input;
  while (g_running.load() && std::getline(std::cin, input)) {
    if (input == "quit") {
      break;
    }

    uint8_t target = 0x00;
    if (!ParseInputTarget(input, target)) {
      std::cout << "无效输入，只能输入 0、1 或 quit。\n";
      continue;
    }

    if (!SendAimbotTarget(fd, target)) {
      std::cout << "发送失败，重新连接设备 B...\n";
      NetworkTask::CloseSocket(fd);
      fd = NetworkTask::kInvalidSocketFd;
      if (!ConnectWithRetry(fd, receiver_ip, kReceiverPort)) {
        break;
      }
      if (!SendAimbotTarget(fd, target)) {
        std::cout << "重连后仍发送失败\n";
        break;
      }
    }

    std::cout << "已发送 AimbotTarget=" << static_cast<int>(target) << "\n";
  }

  NetworkTask::ShutdownSocket(fd);
  NetworkTask::CloseSocket(fd);
  std::cout << "设备 A 程序退出\n";
  return 0;
}
