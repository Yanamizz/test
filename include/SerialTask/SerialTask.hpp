/**
 * @file    include/SerialTask/SerialTask.hpp
 * @brief   SerialTask 模块统一入口，汇总串口协议、配置、收发与 IMU 缓冲相关头文件。
 *
 * 该头文件作为串口通信子系统的聚合 include，方便主程序一次性引入协议常量、
 * 串口默认配置、IMU 读取、控制发送和 IMU 缓冲能力。它本身不新增状态或线程。
 */

#pragma once

#include "SerialTask/Common.hpp"
#include "SerialTask/ImuBuffer.hpp"
#include "SerialTask/SerialConfig.hpp"
#include "SerialTask/SerialRead.hpp"
#include "SerialTask/SerialSend.hpp"
