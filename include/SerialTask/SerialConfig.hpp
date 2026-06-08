/**
 * @file    include/SerialTask/SerialConfig.hpp
 * @brief   提供串口默认端口、波特率与超时等基础配置。
 *
 * 该文件集中保存串口设备名、baudrate、timeout 等默认连接参数，供读线程
 * 和发送线程复用。它只表达默认配置，不打开设备，也不包含协议帧结构。
 */

#pragma once

#include <serial/serial.h>
#include <iostream>  // 添加头文件以支持输出

namespace SerialTask {
// ===== 手动配置区（统一放在文件末尾）=====
inline constexpr const char *DEFAULT_PORT = "/dev/ttyACM0";  // 串口默认设备路径
inline constexpr int DEFAULT_BAUD_RATE = 115200;             // 串口默认波特率
inline constexpr int DEFAULT_TIMEOUT_MS = 8;                 // 串口默认读取超时（毫秒）

/**
 * @brief 默认构造函数，初始化默认串口配置参数
 */
inline void DefaultConfig(serial::Serial &serial_port) {
  std::cerr << "正在配置串口默认设置..." << std::endl;
  serial_port.setPort(DEFAULT_PORT);
  std::cerr << "串口端口设置为: " << DEFAULT_PORT << std::endl;
  serial_port.setBaudrate(DEFAULT_BAUD_RATE);
  std::cerr << "波特率设置为: " << DEFAULT_BAUD_RATE << std::endl;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT_MS);
  serial_port.setTimeout(timeout);
  std::cerr << "超时时间设置为: " << (DEFAULT_TIMEOUT_MS / 1000.0f) << " 秒" << std::endl;
}

}  // namespace SerialTask
