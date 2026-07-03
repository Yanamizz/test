#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "Tools/TcpStageSignalReceiver.hpp"

int main(int argc, char **argv) {
  const std::string bind_ip = argc >= 2 ? argv[1] : "0.0.0.0";
  const int port = argc >= 3 ? std::atoi(argv[2]) : 8080;
  if (port <= 0 || port > 65535) {
    std::cerr << "非法端口: " << port << std::endl;
    return 1;
  }

  Tools::TcpStageSignalReceiver receiver(Tools::TcpStageSignalConfig{bind_ip, static_cast<std::uint16_t>(port), 1});
  std::cout << "TCP 阶段接收测试启动，监听 " << bind_ip << ":" << port << std::endl;

  while (true) {
    Tools::TcpStageCommand command{};
    if (receiver.PollNextCommand(&command)) {
      if (command.type == Tools::TcpStageCommandType::GameState91) {
        std::cout << "recv 0x91 game_progress="
                  << static_cast<int>(command.game_progress)
                  << " stage_remain_time=" << command.stage_remain_time
                  << std::endl;
      } else {
        std::cout << "recv 0x92 countered=" << (command.countered ? 1 : 0)
                  << std::endl;
      }
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
