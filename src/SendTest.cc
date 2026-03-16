#include <iostream>
#include <string>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <algorithm>

#include <serial/serial.h>

#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialSend.hpp"

namespace {
std::atomic<bool> g_running{true};

void HandleSignal(int) { g_running.store(false); }
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr << "用法: " << argv[0] << " <pitch_deg> <yaw_deg> [hz]" << std::endl;
      return 1;
    }

    const float pitch_deg = std::stof(argv[1]);
    const float yaw_deg = std::stof(argv[2]);
    const float hz = (argc >= 4) ? std::stof(argv[3]) : 50.0f;
    if (hz <= 0.0f) {
      std::cerr << "发送频率必须大于 0" << std::endl;
      return 1;
    }

    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(hz));

    serial::Serial port;
    SerialTask::DefaultConfig(port);

    port.open();
    if (!port.isOpen()) {
      std::cerr << "串口打开失败" << std::endl;
      return 2;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << std::fixed << std::setprecision(3) << "开始持续发送: pitch=" << pitch_deg << " deg, yaw=" << yaw_deg
              << " deg, freq=" << hz << " Hz (Ctrl+C 结束)" << std::endl;

    const uint64_t log_every = std::max<uint64_t>(1, static_cast<uint64_t>(hz));

    uint64_t send_count = 0;
    while (g_running.load()) {
      SerialTask::SerialSend(port, pitch_deg, yaw_deg, 0x00);
      ++send_count;

      if ((send_count % log_every) == 0) {
        std::cout << "已发送 " << send_count << " 次" << std::endl;
      }

      std::this_thread::sleep_for(period);
    }

    std::cout << "停止发送，总计 " << send_count << " 次" << std::endl;

    port.close();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "SendTest 异常: " << e.what() << std::endl;
    return 3;
  }
}
