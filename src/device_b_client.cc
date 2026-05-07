#include <atomic>
#include <iostream>
#include <string>
#include <thread>

#include "NetworkTask/DeviceBClient.hpp"

namespace {

void recv_loop(NetworkTask::socket_t fd, std::atomic<bool> &running,
               std::string &received_content) {
  while (running) {
    if (NetworkTask::ReceiveText(fd, received_content)) {
      std::cout << "\n[收到] " << received_content << std::flush;
    } else {
      std::cout << "\n[提示] 对端已关闭连接\n";
      running = false;
      break;
    }
  }
}

} // namespace

int main() {
  NetworkTask::socket_t fd = NetworkTask::kInvalidSocketFd;
  if (!NetworkTask::ConnectToServer(fd)) {
    std::cerr << "连接失败，请检查 IP、网线、防火墙和服务端程序是否已启动\n";
    return 1;
  }

  std::cout << "已连接设备 A：192.168.10.1:5000" << std::endl;
  std::cout << "输入消息后回车发送；输入 quit 退出。\n";

  std::atomic<bool> running(true);
  std::string received_content;
  std::thread recv_thread(recv_loop, fd, std::ref(running),
                          std::ref(received_content));

  std::string input_content;
  while (running && std::getline(std::cin, input_content)) {
    if (input_content == "quit") {
      running = false;
      break;
    }

    input_content += "\n";
    if (!NetworkTask::SendText(fd, input_content)) {
      std::cout << "[错误] 发送失败\n";
      running = false;
      break;
    }
  }

  running = false;
  NetworkTask::ShutdownSocket(fd);
  NetworkTask::CloseSocket(fd);

  if (recv_thread.joinable()) {
    recv_thread.join();
  }

  std::cout << "设备 B 程序退出\n";
  return 0;
}