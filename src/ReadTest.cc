#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include <serial/serial.h>

#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/Common.hpp"

namespace {
std::atomic<bool> g_running{true};
void HandleSignal(int) { g_running.store(false); }
}  // namespace

int main() {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  serial::Serial port;
  SerialTask::DefaultConfig(port);

  try {
    port.open();
  } catch (const std::exception &e) {
    std::cerr << "串口打开失败: " << e.what() << std::endl;
    return 1;
  }
  if (!port.isOpen()) {
    std::cerr << "串口打开失败（未知原因）" << std::endl;
    return 1;
  }

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "开始读取 IMU 数据，Ctrl+C 退出..." << std::endl;

  uint64_t recv_count = 0;
  while (g_running.load()) {
    SerialTask::EulerAngles angles{};

    // 直接使用 SerialRead.hpp 中的统一解析与姿态解算逻辑
    if (SerialTask::ReadIMUData(port, angles)) {
      ++recv_count;
      std::cout << "[" << recv_count << "]" << "  Roll=" << angles.roll << "  Pitch=" << angles.pitch
                << "  Yaw=" << angles.yaw << std::endl;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  port.close();
  std::cout << "已退出，共收到 " << recv_count << " 帧。" << std::endl;
  return 0;
}
