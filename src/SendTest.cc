#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <serial/serial.h>

#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"

int main(int argc, char **argv) {
  try {
    if (argc < 3) {
      std::cerr << "用法: " << argv[0]
                << " <pitch_relative_deg> <yaw_relative_deg>" << std::endl;
      return 1;
    }

    const float pitch_relative_deg = std::stof(argv[1]);
    const float yaw_relative_deg = std::stof(argv[2]);

    serial::Serial port;
    SerialTask::DefaultConfig(port);

    port.open();
    if (!port.isOpen()) {
      std::cerr << "串口打开失败" << std::endl;
      return 2;
    }

    // 丢弃串口历史缓存，避免读取到较旧 IMU 帧。
    port.flushInput();

    SerialTask::EulerAngles imu_angles{};
    bool got_imu = false;
    const auto read_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
    int valid_frames = 0;

    // 在短时间窗口内连续取帧，始终保留“最后一帧”作为当前姿态。
    while (std::chrono::steady_clock::now() < read_deadline) {
      SerialTask::EulerAngles sample{};
      if (SerialTask::ReadIMUData(port, sample)) {
        imu_angles = sample;
        got_imu = true;
        ++valid_frames;

        // 至少拿到几帧后即可认为是新鲜数据，减少等待时间。
        if (valid_frames >= 3) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!got_imu) {
      std::cerr << "读取 IMU 超时，未发送数据" << std::endl;
      port.close();
      return 4;
    }

    const float absolute_pitch_deg = imu_angles.pitch + pitch_relative_deg;
    const float absolute_yaw_deg = imu_angles.yaw + yaw_relative_deg;

    SerialTask::SerialSend(port, absolute_pitch_deg, absolute_yaw_deg,
                           pitch_relative_deg, yaw_relative_deg, 0, 0, 0x01);

    std::cout << std::fixed << std::setprecision(3)
              << "IMU当前角度: pitch=" << imu_angles.pitch
              << " deg, yaw=" << imu_angles.yaw << " deg" << std::endl;
    std::cout << "输入相对角度: pitch=" << pitch_relative_deg
              << " deg, yaw=" << yaw_relative_deg << " deg" << std::endl;
    std::cout << "发送绝对角度: pitch=" << absolute_pitch_deg
              << " deg, yaw=" << absolute_yaw_deg << " deg (已发送 1 次)"
              << std::endl;

    port.close();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "SendTest 异常: " << e.what() << std::endl;
    return 3;
  }
}
