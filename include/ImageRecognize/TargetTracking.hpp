/**
 * @file    include/ImageRecognize/TargetTracking.hpp
 * @brief   ImageRecognize 目标筛选、关联、预测与阶段判定模块统一入口。
 *
 * 该头文件聚合目标跟踪相关能力，方便主程序和测试代码一次性引入目标类别
 * 筛选、跨帧关联、跟踪 pipeline 和阶段锁定判定。它本身不新增行为，只维护
 * ImageRecognize 跟踪子系统的 include 边界。
 */

#pragma once

#include "ImageRecognize/AerialRobotLaserLockJudge.hpp"
#include "ImageRecognize/OutputDataProcess.hpp"
#include "ImageRecognize/TargetAssociation.hpp"
#include "ImageRecognize/TargetClassFilter.hpp"
#include "ImageRecognize/TargetTrackPipeline.hpp"
