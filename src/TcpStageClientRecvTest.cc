#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "Tools/TcpStageSignalReceiver.hpp"

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "用法: " << argv[0] << " <server_ip> <port> [log_path]"
              << std::endl;
    return 1;
  }

  const int port = std::atoi(argv[2]);
  if (port <= 0 || port > 65535) {
    std::cerr << "非法端口: " << port << std::endl;
    return 2;
  }

  const std::string log_path =
      argc >= 4 ? argv[3] : "tcp_stage_client_receive_test.log";
  Tools::TcpStageSignalClientReceiver receiver(
      Tools::TcpStageClientReceiveConfig{
          argv[1], static_cast<std::uint16_t>(port), 500, 1000, log_path});

  std::cout << "TCP 阶段客户端接收测试启动，server=" << argv[1] << ":"
            << port << std::endl;
  while (true) {
    Tools::TcpStageCommand command{};
    if (receiver.PollNextCommand(&command)) {
      if (command.type == Tools::TcpStageCommandType::GameState91) {
        std::cout << "recv 0x91 game_progress="
                  << static_cast<int>(command.game_progress)
                  << " stage_remain_time=" << command.stage_remain_time
                  << std::endl;
      } else {
        std::cout << "recv 0x92 countered="
                  << (command.countered ? 1 : 0) << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
