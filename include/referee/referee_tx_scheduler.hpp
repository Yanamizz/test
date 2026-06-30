#ifndef RADAR_INCLUDE_REFEREE_REFEREE_TX_SCHEDULER_HPP
#define RADAR_INCLUDE_REFEREE_REFEREE_TX_SCHEDULER_HPP

/**
 * @file  include/referee/referee_tx_scheduler.hpp
 * @brief 常规链路统一发送调度器
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/serial_connection_log.hpp"
#include "include/referee/serial_port.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::referee {

/**
 * @brief 统一管理 `0x0301` 与 `0x0305` 发送节奏
 * @note  `0x0301` 采用 FIFO，高优先级；`0x0305` 采用 latest-only。
 */
class RefereeTxScheduler {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using SentCallback = std::function<void()>;

  /**
   * @brief 创建发送调度器
   * @param serial 实际发送串口
   * @param log_root 本轮运行日志根目录
   */
  explicit RefereeTxScheduler(SerialPort &serial, std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : serial_(serial), log_store_(std::move(log_root)), serial_log_(log_store_.root()) {}

  /**
   * @brief 向 `0x0301` 发送队列压入一帧待发数据
   * @param frame 已完成组包的字节流
   * @param name 日志名称
   * @param source_port 来源端口
   * @param decision 决策标签
   * @param on_sent 真正写出后的回调
   * @return 是否成功进入队列
   */
  bool EnqueueRadarCommand(std::vector<rm::u8> frame, std::string name, int source_port, std::string decision,
                           SentCallback on_sent) {
    if (radar_command_queue_.size() >= kRadarCommandQueueCapacity) {
      LogRadarCommandQueueDrop(name, source_port, decision, frame);
      return false;
    }

    RadarCommandTask task;
    task.frame = std::move(frame);
    task.name = std::move(name);
    task.source_port = source_port;
    task.decision = std::move(decision);
    task.on_sent = std::move(on_sent);
    radar_command_queue_.push_back(std::move(task));
    return true;
  }

  /**
   * @brief 覆盖当前最新的 `0x0305` 待发送帧
   * @param frame 已完成组包的字节流
   * @param on_sent 真正写出后的回调
   */
  void UpdateLatestMapRobotFrame(std::vector<rm::u8> frame, SentCallback on_sent) {
    latest_map_robot_frame_.emplace();
    latest_map_robot_frame_->frame = std::move(frame);
    latest_map_robot_frame_->on_sent = std::move(on_sent);
  }

  /**
   * @brief 清除当前暂存的 `0x0305` 帧
   */
  void ClearLatestMapRobotFrame() { latest_map_robot_frame_.reset(); }

  /**
   * @brief 判断是否仍有待发送内容
   * @return 队列或 latest-only 帧非空时返回 true
   */
  bool HasPending() const { return !radar_command_queue_.empty() || latest_map_robot_frame_.has_value(); }

  /**
   * @brief 判断当前是否存在待发送的 `0x0305`
   * @return latest-only 帧存在时返回 true
   */
  bool HasLatestMapRobotFrame() const { return latest_map_robot_frame_.has_value(); }

  /**
   * @brief 推进一次发送调度
   * @note  若串口断开则本轮仅保留队列状态，不执行实际写串口。
   */
  void Process() {
    const auto now = Clock::now();
    if (!serial_.is_open()) {
      return;
    }

    if (!radar_command_queue_.empty() && IsRadarCommandSendDue(now)) {
      auto &task = radar_command_queue_.front();
      std::string error;
      if (!serial_.TryWriteAll(task.frame.data(), task.frame.size(), &error)) {
        HandleSerialWriteFailure(error);
        return;
      }
      auto sent_task = std::move(task);
      radar_command_queue_.pop_front();
      last_radar_command_send_time_ = now;
      if (sent_task.on_sent) {
        sent_task.on_sent();
      }
    }

    if (!serial_.is_open()) {
      return;
    }
    if (latest_map_robot_frame_.has_value() && IsMapRobotSendDue(now)) {
      std::string error;
      if (!serial_.TryWriteAll(latest_map_robot_frame_->frame.data(), latest_map_robot_frame_->frame.size(), &error)) {
        HandleSerialWriteFailure(error);
        return;
      }
      last_map_robot_send_time_ = now;
      if (latest_map_robot_frame_->on_sent) {
        latest_map_robot_frame_->on_sent();
      }
    }
  }

  /**
   * @brief 判断当前是否允许发送 `0x0305`
   * @param now 当前时间点
   * @return 是否已满足最小发送间隔
   */
  bool IsMapRobotSendDue(TimePoint now = Clock::now()) const {
    return !last_map_robot_send_time_.has_value() ||
           now - *last_map_robot_send_time_ >= std::chrono::milliseconds(config::kMapRobotMinSendIntervalMs);
  }

 private:
  /// `0x0301` 队列中的单个待发送任务。
  struct RadarCommandTask {
    std::vector<rm::u8> frame;
    std::string name;
    int source_port = 0;
    std::string decision;
    SentCallback on_sent;
  };

  /// latest-only 模式下缓存的 `0x0305` 任务。
  struct MapRobotTask {
    std::vector<rm::u8> frame;
    SentCallback on_sent;
  };

  /// `0x0301` 队列最大容量。
  static constexpr std::size_t kRadarCommandQueueCapacity = 32;

  /**
   * @brief 判断当前是否允许发送下一帧 `0x0301`
   * @param now 当前时间点
   * @return 是否已满足发送窗口
   */
  bool IsRadarCommandSendDue(TimePoint now) const {
    return !last_radar_command_send_time_.has_value() ||
           now - *last_radar_command_send_time_ >= std::chrono::milliseconds(34);
  }

  /**
   * @brief 记录一次 `0x0301` 队列满导致的丢弃
   * @param name 日志名称
   * @param source_port 来源端口
   * @param decision 决策标签
   * @param frame 被丢弃的完整帧
   */
  void LogRadarCommandQueueDrop(const std::string &name, int source_port, const std::string &decision,
                                const std::vector<rm::u8> &frame) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"type\":\"0x0301\","
          << "\"name\":\"" << name << "\","
          << "\"source_port\":" << source_port << ','
          << "\"decision\":\"" << decision << "\","
          << "\"result\":\"dropped_queue_full\","
          << "\"queue_capacity\":" << kRadarCommandQueueCapacity << ','
          << "\"frame_hex\":\"" << radar::log::HexBytes(frame.data(), frame.size()) << "\"}";
    log_store_.Append("main/tx_scheduler.log", entry.str(), radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 处理串口写失败
   * @param detail 失败详情
   * @note  一旦发送失败，串口会被打回断开状态，等待主循环自动重连。
   */
  void HandleSerialWriteFailure(const std::string &detail) {
    const std::string device = serial_.device().empty() ? SelectRefereeDevice() : serial_.device();
    const int baud = serial_.baud() == 0 ? config::kDefaultRefereeBaud : serial_.baud();
    serial_log_.LogState("disconnected", device, baud, detail, false);
    radar::log::GetRuntimeMetrics(log_store_.root()).RecordSerialDisconnect();
    serial_.Close();
  }

  SerialPort &serial_;                                 ///< 实际发送串口
  radar::log::FileLogStore log_store_;                ///< 调度器自身日志
  SerialConnectionLog serial_log_;                    ///< 串口掉线日志
  std::deque<RadarCommandTask> radar_command_queue_;  ///< `0x0301` FIFO 队列
  std::optional<MapRobotTask> latest_map_robot_frame_;  ///< 当前最新 `0x0305`
  std::optional<TimePoint> last_radar_command_send_time_;  ///< 最近一次 `0x0301` 发送时间
  std::optional<TimePoint> last_map_robot_send_time_;      ///< 最近一次 `0x0305` 发送时间
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_REFEREE_TX_SCHEDULER_HPP
