#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "Tools/TcpStageSignalReceiver.hpp"

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "用法: " << argv[0] << " <host> <port>" << std::endl;
    return 1;
  }

  const std::string host = argv[1];
  const int port = std::atoi(argv[2]);
  if (port <= 0 || port > 65535) {
    std::cerr << "非法端口: " << port << std::endl;
    return 2;
  }

  Tools::TcpStageSignalSender sender(
      Tools::TcpStageSendConfig{host, static_cast<std::uint16_t>(port)});
  std::cout << "TCP 阶段发送测试启动，目标 " << host << ":" << port
            << std::endl;
  std::cout << "输入 `91 <game_progress 0-15> <stage_remain_time 0-65535>` 或 `92 <0|1>`，输入 q 退出。"
            << std::endl;

  std::string line;
  while (std::cout << "> " && std::getline(std::cin, line)) {
    if (line == "q" || line == "quit" || line == "exit") {
      break;
    }

    std::istringstream iss(line);
    int cmd = -1;
    iss >> cmd;
    if (!iss) {
      std::cout << "请输入 `91 ...`、`92 ...` 或 q" << std::endl;
      continue;
    }

    if (cmd == 91) {
      int game_progress = -1;
      int stage_remain_time = -1;
      iss >> game_progress >> stage_remain_time;
      if (!iss || game_progress < 0 || game_progress > 15 ||
          stage_remain_time < 0 || stage_remain_time > 65535) {
        std::cout << "格式：91 <game_progress 0-15> <stage_remain_time 0-65535>"
                  << std::endl;
        continue;
      }

      if (!sender.SendGameState(static_cast<std::uint8_t>(game_progress),
                                static_cast<std::uint16_t>(stage_remain_time))) {
        std::cout << "发送失败，下次输入时会尝试重新连接" << std::endl;
        continue;
      }
      std::cout << "send 0x91 game_progress=" << game_progress
                << " stage_remain_time=" << stage_remain_time << " -> "
                << host << ":" << port << std::endl;
      continue;
    }

    if (cmd == 92) {
      int countered = -1;
      iss >> countered;
      if (!iss || (countered != 0 && countered != 1)) {
        std::cout << "格式：92 <0|1>" << std::endl;
        continue;
      }

      if (!sender.SendCounteredState(countered == 1)) {
        std::cout << "发送失败，下次输入时会尝试重新连接" << std::endl;
        continue;
      }
      std::cout << "send 0x92 countered=" << countered << " -> " << host
                << ":" << port << std::endl;
      continue;
    }

    std::cout << "请输入 `91 ...`、`92 ...` 或 q" << std::endl;
  }

  return 0;
}
