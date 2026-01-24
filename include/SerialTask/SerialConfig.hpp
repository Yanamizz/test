/**
 * @file   include/SerialTask/SerialConfig.hpp
 *
 * @brief  初始化串口配置参数

 *
 */

#pragma once

#include <serial/serial.h>
#include <string>
#include <iostream>  // 添加头文件以支持输出

const std::string DEFAULT_PORT = "/dev/ttyACM0";  // 串口设备路径
const int DEFAULT_BAUD_RATE = 115200;             // 波特率
const float DEFAULT_TIMEOUT = 0.1;                // 读取超时秒数

namespace SerialTask {

/**
 * @brief 默认构造函数，初始化默认串口配置参数
 */
inline void DefaultConfig(serial::Serial& serial_port) {
  std::cerr << "正在配置串口默认设置..." << std::endl;
  serial_port.setPort(DEFAULT_PORT);
  std::cerr << "串口端口设置为: " << DEFAULT_PORT << std::endl;
  serial_port.setBaudrate(DEFAULT_BAUD_RATE);
  std::cerr << "波特率设置为: " << DEFAULT_BAUD_RATE << std::endl;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT * 1000);
  serial_port.setTimeout(timeout);
  std::cerr << "超时时间设置为: " << DEFAULT_TIMEOUT << " 秒" << std::endl;
}

};  // namespace SerialTask