#include <atomic>
#include <iostream>
#include <string>
#include <thread>

#include "NetworkTask/DeviceAServer.hpp"

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
  NetworkTask::socket_t listen_fd = NetworkTask::kInvalidSocketFd;
  if (!NetworkTask::CreateListeningSocket(listen_fd)) {
    std::cerr << "创建监听 socket 失败\n";
    return 1;
  }

  std::cout << "设备 A 服务端已启动，监听端口：5000" << std::endl;
  std::cout << "等待设备 B 连接..." << std::endl;

  NetworkTask::socket_t client_fd = NetworkTask::kInvalidSocketFd;
  std::string client_ip;
  if (!NetworkTask::AcceptClient(listen_fd, client_fd, &client_ip)) {
    std::cerr << "accept 失败\n";
    NetworkTask::CloseSocket(listen_fd);
    return 1;
  }

  std::cout << "设备 B 已连接，IP：" << client_ip << std::endl;
  std::cout << "输入消息后回车发送；输入 quit 退出。\n";

  std::atomic<bool> running(true);
  std::string received_content;
  std::thread recv_thread(recv_loop, client_fd, std::ref(running),
                          std::ref(received_content));

  std::string input_content;
  while (running && std::getline(std::cin, input_content)) {
    if (input_content == "quit") {
      running = false;
      break;
    }

    input_content += "\n";
    if (!NetworkTask::SendText(client_fd, input_content)) {
      std::cout << "[错误] 发送失败\n";
      running = false;
      break;
    }
  }

  running = false;
  NetworkTask::ShutdownSocket(client_fd);
  NetworkTask::CloseSocket(client_fd);
  NetworkTask::CloseSocket(listen_fd);

  if (recv_thread.joinable()) {
    recv_thread.join();
  }

  std::cout << "设备 A 程序退出\n";
  return 0;
}