#include <iostream>
#include <string>
#include <iomanip>

#include <serial/serial.h>

#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialSend.hpp"

int main(int argc, char** argv) {
  try {
    const float pitch_deg = std::stof(argv[1]);
    const float yaw_deg = std::stof(argv[2]);

    serial::Serial port;
    SerialTask::DefaultConfig(port);

    port.open();
    if (!port.isOpen()) {
      std::cerr << "串口打开失败" << std::endl;
      return 2;
    }

    SerialTask::SerialSend(port, pitch_deg, yaw_deg);

    std::cout << std::fixed << std::setprecision(3) << "单次发送完成: pitch=" << pitch_deg << " deg, yaw=" << yaw_deg
              << " deg" << std::endl;

    port.close();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "SendTest 异常: " << e.what() << std::endl;
    return 3;
  }
}
