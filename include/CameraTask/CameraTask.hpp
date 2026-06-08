/**
 * @file    include/CameraTask/CameraTask.hpp
 * @brief   CameraTask 模块统一入口，汇总相机采图与曝光控制相关头文件。
 *
 * 该头文件作为相机任务层的聚合入口，供主程序一次性引入 Galaxy 相机
 * 采集封装、曝光热键控制和运行时曝光参数管理能力。它本身不定义新逻辑，
 * 主要用于稳定 include 边界，减少调用侧对相机子模块文件布局的依赖。
 */

#pragma once

#include "CameraTask/ExposureHotkeyController.hpp"
#include "CameraTask/GetImage.hpp"
