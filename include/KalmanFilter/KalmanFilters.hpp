/**
 * @file    include/KalmanFilter/KalmanFilters.hpp
 * @brief   KalmanFilter 模块统一入口，汇总角度滤波相关模型与滤波器实现。
 *
 * 该头文件聚合角度状态模型、线性 KF、EKF、UKF、CKF 和 One Euro Filter 等实现，
 * 供 AngleCalculate 一次性引入。它不定义新的滤波算法，只维护滤波子系统的 include
 * 边界。
 */

#pragma once

#include "KalmanFilter/AngleKalmanModels.hpp"
#include "KalmanFilter/CubatureKalmanFilter.hpp"
#include "KalmanFilter/ExtendedKalmanFilter.hpp"
#include "KalmanFilter/KalmanFilter.hpp"
#include "KalmanFilter/OneEuroFilter.hpp"
#include "KalmanFilter/UnscentedKalmanFilter.hpp"
