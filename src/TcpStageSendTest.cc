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
  std::cout << "输入 0 或 1 发送单字节，输入 q 退出。" << std::endl;

  std::string line;
  while (std::cout << "> " && std::getline(std::cin, line)) {
    if (line == "q" || line == "quit" || line == "exit") {
      break;
    }

    std::istringstream iss(line);
    int signal = -1;
    iss >> signal;
    if (!iss || (signal != 0 && signal != 1)) {
      std::cout << "请输入 0、1 或 q" << std::endl;
      continue;
    }

    const std::uint8_t value = signal == 0 ? 0x00 : 0x01;
    if (!sender.SendByte(value)) {
      std::cout << "发送失败，下次输入时会尝试重新连接" << std::endl;
      continue;
    }
    std::cout << "send 0x" << std::hex << static_cast<int>(value) << std::dec
              << " -> " << host << ":" << port << std::endl;
  }

  return 0;
}
