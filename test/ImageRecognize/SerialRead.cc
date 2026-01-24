#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialSend.hpp"
#include <iostream>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include <cmath>

int main() {
  serial::Serial serial_port;
  try {
    SerialTask::DefaultConfig(serial_port);
    serial_port.open();
    if (!serial_port.isOpen()) {
      std::cerr << "无法打开串口！" << std::endl;
      return -1;
    }
  } catch (const std::exception& e) {
    std::cerr << "串口配置或打开时出错: " << e.what() << std::endl;
    return -1;
  }

  SerialTask::SerialSend(serial_port, 0.0f, 10.0f);  // 发送示例数据

  serial_port.close();
  return 0;
}