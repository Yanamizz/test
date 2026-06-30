#ifndef RADAR_INCLUDE_LOG_LOG_BACKEND_HPP
#define RADAR_INCLUDE_LOG_LOG_BACKEND_HPP

/**
 * @file  include/log/log_backend.hpp
 * @brief 异步日志后端与运行指标统计
 */

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "include/config/config.hpp"
#include "librm/core/typedefs.hpp"

namespace radar::log {

/**
 * @brief 日志优先级
 */
enum class LogPriority {
  kBestEffort,       ///< 队列满时允许丢弃
  kCriticalDecision, ///< 关键决策日志，尽量保留
  kCriticalRaw,      ///< 原始二进制日志，优先级最高
};

namespace detail {

/**
 * @brief 后端专用时间戳格式化函数
 * @return `YYYY-MM-DD HH:MM:SS.mmm` 格式时间
 */
inline std::string BackendTimestampNow() {
  using namespace std::chrono;

  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  const auto time = system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream oss;
  oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

/**
 * @brief 将微秒转换为毫秒浮点值
 */
inline double DurationToMilliseconds(std::chrono::microseconds duration) {
  return static_cast<double>(duration.count()) / 1000.0;
}

/**
 * @brief 计算样本分位数
 * @param values 样本集合
 * @param quantile 分位数，例如 `0.95`
 * @return 对应分位数值
 */
inline double Percentile(std::vector<double> values, double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = quantile * static_cast<double>(values.size() - 1);
  const auto index = static_cast<std::size_t>(position);
  const double fraction = position - static_cast<double>(index);
  if (index + 1 >= values.size()) {
    return values.back();
  }
  return values[index] + (values[index + 1] - values[index]) * fraction;
}

/**
 * @brief 将数字格式化为 JSON 可写字符串
 */
inline std::string JsonNumber(double value, int precision = 3) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

/**
 * @brief 确保目标路径的父目录存在
 */
inline void EnsureParentDirectory(const std::filesystem::path &path) {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
}

/**
 * @brief 同步向文本日志追加一行
 */
inline void AppendLineSync(const std::filesystem::path &path, const std::string &line) {
  EnsureParentDirectory(path);
  std::ofstream stream(path, std::ios::app);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open log file: " + path.string());
  }
  stream << line << '\n';
}

}  // namespace detail

/**
 * @brief 运行时健康指标统计器
 * @note  按 1Hz 聚合主循环负载、日志压力与通信稳定性指标。
 */
class RuntimeMetrics {
 public:
  using Clock = std::chrono::steady_clock;

  /**
   * @brief 创建运行指标统计器
   * @param root 本轮运行日志根目录
   */
  explicit RuntimeMetrics(std::filesystem::path root) : root_(std::move(root)), last_flush_time_(Clock::now()) {
    std::filesystem::create_directories(root_ / "main");
  }

  /**
   * @brief 记录一次事件循环耗时
   */
  void RecordLoopIteration(std::chrono::microseconds total, std::chrono::microseconds process) {
    const double total_ms = detail::DurationToMilliseconds(total);
    const double process_ms = detail::DurationToMilliseconds(process);
    std::lock_guard<std::mutex> lock(mutex_);
    loop_total_ms_samples_.push_back(total_ms);
    loop_process_ms_samples_.push_back(process_ms);
    if (total_ms > max_total_loop_ms_) {
      max_total_loop_ms_ = total_ms;
    }
    if (process_ms > max_process_loop_ms_) {
      max_process_loop_ms_ = process_ms;
    }
    ++loop_iteration_count_;
    if (total_ms > static_cast<double>(config::kRuntimeLoopWarnMs)) {
      ++loop_over_warn_count_;
    }
    if (total_ms > static_cast<double>(config::kRuntimeLoopTailMs)) {
      ++loop_over_tail_count_;
    }
    if (total_ms > static_cast<double>(config::kRuntimeLoopCriticalMs)) {
      ++loop_over_critical_count_;
    }
  }

  /**
   * @brief 记录一次串口空读
   */
  void RecordSerialRead(std::size_t bytes_read) {
    if (bytes_read != 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ++serial_idle_read_count_;
  }

  /// 记录一次串口打开失败。
  void RecordSerialOpenFailure() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++serial_open_failure_count_;
  }

  /// 记录一次串口断开。
  void RecordSerialDisconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++serial_disconnect_count_;
  }

  /// 记录一次串口重连成功。
  void RecordSerialReconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++serial_reconnect_count_;
  }

  /**
   * @brief 记录一次 TCP 空闲断开
   * @param port 发生断开的端口
   */
  void RecordTcpIdleDisconnect(int port) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++tcp_idle_disconnect_count_;
    ++tcp_idle_disconnects_by_port_[port];
  }

  /**
   * @brief 记录一次 `0x0305` 跳过发送
   * @param reason 跳过原因
   */
  void RecordMapRobotSkip(const std::string &reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reason == "rate_limited") {
      ++map_robot_rate_limited_skip_count_;
    } else if (reason == "opponent_stale") {
      ++map_robot_opponent_stale_skip_count_;
    }
  }

  /// 记录一次 `0x0121` 因冷却等待。
  void RecordRadarCommandWaitingCooldown() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++radar_command_waiting_cooldown_count_;
  }

  /**
   * @brief 记录当前链路丢包率
   * @param source 链路名称
   * @param loss_rate 当前丢包率
   */
  void RecordLossRate(const std::string &source, float loss_rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    loss_rate_by_source_[source] = loss_rate;
  }

  /**
   * @brief 观察当前日志队列长度
   * @param queue_size 当前队列长度
   */
  void ObserveLogQueue(std::size_t queue_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_size > log_queue_peak_) {
      log_queue_peak_ = queue_size;
    }
    log_queue_current_ = queue_size;
    log_queue_size_sum_ += queue_size;
    ++log_queue_size_sample_count_;
  }

  /**
   * @brief 记录一次日志入队
   * @param queue_size 入队后的队列长度
   */
  void RecordLogEnqueue(LogPriority, std::size_t queue_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++log_event_count_;
    if (queue_size > log_queue_peak_) {
      log_queue_peak_ = queue_size;
    }
    log_queue_current_ = queue_size;
    log_queue_size_sum_ += queue_size;
    ++log_queue_size_sample_count_;
  }

  /**
   * @brief 记录一次日志被丢弃
   * @param priority 被丢弃日志的优先级
   */
  void RecordLogDrop(LogPriority priority) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (priority == LogPriority::kBestEffort) {
      ++best_effort_log_drop_count_;
      return;
    }
    ++critical_log_drop_count_;
  }

  /**
   * @brief 记录一次实际磁盘写入
   */
  void RecordLogWrite(std::size_t bytes, bool binary) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (binary) {
      raw_binary_log_bytes_ += bytes;
    } else {
      text_log_bytes_ += bytes;
    }
  }

  /**
   * @brief 记录一次后台批量 flush
   */
  void RecordLogFlushBatch(std::size_t task_count, std::chrono::microseconds elapsed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_count == 0) {
      return;
    }
    ++log_flush_batch_count_;
    log_flush_total_ms_ += detail::DurationToMilliseconds(elapsed);
    const double elapsed_ms = detail::DurationToMilliseconds(elapsed);
    if (elapsed_ms > log_flush_max_ms_) {
      log_flush_max_ms_ = elapsed_ms;
    }
  }

  /**
   * @brief 到达周期时将聚合指标落盘
   */
  void MaybeFlush() {
    const auto now = Clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (now - last_flush_time_ < std::chrono::milliseconds(config::kRuntimeMetricsFlushIntervalMs)) {
        return;
      }
      last_flush_time_ = now;
    }

    Snapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot.timestamp = detail::BackendTimestampNow();
      snapshot.loop_total_ms_samples = std::move(loop_total_ms_samples_);
      snapshot.loop_process_ms_samples = std::move(loop_process_ms_samples_);
      snapshot.loop_iteration_count = loop_iteration_count_;
      snapshot.loop_over_warn_count = loop_over_warn_count_;
      snapshot.loop_over_tail_count = loop_over_tail_count_;
      snapshot.loop_over_critical_count = loop_over_critical_count_;
      snapshot.max_total_loop_ms = max_total_loop_ms_;
      snapshot.max_process_loop_ms = max_process_loop_ms_;
      snapshot.log_event_count = log_event_count_;
      snapshot.raw_binary_log_bytes = raw_binary_log_bytes_;
      snapshot.text_log_bytes = text_log_bytes_;
      snapshot.log_queue_peak = log_queue_peak_;
      snapshot.log_queue_current = log_queue_current_;
      snapshot.log_queue_size_sum = log_queue_size_sum_;
      snapshot.log_queue_size_sample_count = log_queue_size_sample_count_;
      snapshot.best_effort_log_drop_count = best_effort_log_drop_count_;
      snapshot.critical_log_drop_count = critical_log_drop_count_;
      snapshot.log_flush_batch_count = log_flush_batch_count_;
      snapshot.log_flush_total_ms = log_flush_total_ms_;
      snapshot.log_flush_max_ms = log_flush_max_ms_;
      snapshot.serial_idle_read_count = serial_idle_read_count_;
      snapshot.serial_open_failure_count = serial_open_failure_count_;
      snapshot.serial_disconnect_count = serial_disconnect_count_;
      snapshot.serial_reconnect_count = serial_reconnect_count_;
      snapshot.tcp_idle_disconnect_count = tcp_idle_disconnect_count_;
      snapshot.tcp_idle_disconnects_by_port = tcp_idle_disconnects_by_port_;
      snapshot.map_robot_rate_limited_skip_count = map_robot_rate_limited_skip_count_;
      snapshot.map_robot_opponent_stale_skip_count = map_robot_opponent_stale_skip_count_;
      snapshot.radar_command_waiting_cooldown_count = radar_command_waiting_cooldown_count_;
      snapshot.loss_rate_by_source = loss_rate_by_source_;

      loop_total_ms_samples_.clear();
      loop_process_ms_samples_.clear();
      loop_iteration_count_ = 0;
      loop_over_warn_count_ = 0;
      loop_over_tail_count_ = 0;
      loop_over_critical_count_ = 0;
      max_total_loop_ms_ = 0.0;
      max_process_loop_ms_ = 0.0;
      log_event_count_ = 0;
      raw_binary_log_bytes_ = 0;
      text_log_bytes_ = 0;
      log_queue_peak_ = log_queue_current_;
      log_queue_size_sum_ = 0;
      log_queue_size_sample_count_ = 0;
      best_effort_log_drop_count_ = 0;
      critical_log_drop_count_ = 0;
      log_flush_batch_count_ = 0;
      log_flush_total_ms_ = 0.0;
      log_flush_max_ms_ = 0.0;
      serial_idle_read_count_ = 0;
      serial_open_failure_count_ = 0;
      serial_disconnect_count_ = 0;
      serial_reconnect_count_ = 0;
      tcp_idle_disconnect_count_ = 0;
      tcp_idle_disconnects_by_port_.clear();
      map_robot_rate_limited_skip_count_ = 0;
      map_robot_opponent_stale_skip_count_ = 0;
      radar_command_waiting_cooldown_count_ = 0;
    }

    const auto line = BuildJson(snapshot);
    detail::AppendLineSync(root_ / "main/runtime_metrics.log", line);
  }

 private:
  /**
   * @brief 一次 1Hz 刷新周期内的指标快照
   */
  struct Snapshot {
    std::string timestamp;
    std::vector<double> loop_total_ms_samples;
    std::vector<double> loop_process_ms_samples;
    std::size_t loop_iteration_count = 0;
    std::size_t loop_over_warn_count = 0;
    std::size_t loop_over_tail_count = 0;
    std::size_t loop_over_critical_count = 0;
    double max_total_loop_ms = 0.0;
    double max_process_loop_ms = 0.0;
    std::size_t log_event_count = 0;
    std::size_t raw_binary_log_bytes = 0;
    std::size_t text_log_bytes = 0;
    std::size_t log_queue_peak = 0;
    std::size_t log_queue_current = 0;
    std::size_t log_queue_size_sum = 0;
    std::size_t log_queue_size_sample_count = 0;
    std::size_t best_effort_log_drop_count = 0;
    std::size_t critical_log_drop_count = 0;
    std::size_t log_flush_batch_count = 0;
    double log_flush_total_ms = 0.0;
    double log_flush_max_ms = 0.0;
    std::size_t serial_idle_read_count = 0;
    std::size_t serial_open_failure_count = 0;
    std::size_t serial_disconnect_count = 0;
    std::size_t serial_reconnect_count = 0;
    std::size_t tcp_idle_disconnect_count = 0;
    std::map<int, std::size_t> tcp_idle_disconnects_by_port;
    std::size_t map_robot_rate_limited_skip_count = 0;
    std::size_t map_robot_opponent_stale_skip_count = 0;
    std::size_t radar_command_waiting_cooldown_count = 0;
    std::map<std::string, float> loss_rate_by_source;
  };

  /**
   * @brief 将指标快照序列化为 JSON
   */
  std::string BuildJson(const Snapshot &snapshot) const {
    const auto avg_total_ms = Average(snapshot.loop_total_ms_samples);
    const auto avg_process_ms = Average(snapshot.loop_process_ms_samples);
    const auto p95_total_ms = detail::Percentile(snapshot.loop_total_ms_samples, 0.95);
    const auto p99_total_ms = detail::Percentile(snapshot.loop_total_ms_samples, 0.99);
    const auto p95_process_ms = detail::Percentile(snapshot.loop_process_ms_samples, 0.95);
    const auto p99_process_ms = detail::Percentile(snapshot.loop_process_ms_samples, 0.99);
    const auto avg_flush_ms = snapshot.log_flush_batch_count == 0
                                  ? 0.0
                                  : snapshot.log_flush_total_ms / static_cast<double>(snapshot.log_flush_batch_count);
    const double queue_occupancy_percent =
        snapshot.log_queue_size_sample_count == 0
            ? 0.0
            : (static_cast<double>(snapshot.log_queue_size_sum) /
               static_cast<double>(snapshot.log_queue_size_sample_count)) /
                  static_cast<double>(config::kCriticalLogSoftQueueCapacity) *
                  100.0;

    std::ostringstream oss;
    oss << "{"
        << "\"timestamp\":\"" << snapshot.timestamp << "\","
        << "\"loop\":{"
        << "\"samples\":" << snapshot.loop_iteration_count << ','
        << "\"total_ms_avg\":" << detail::JsonNumber(avg_total_ms) << ','
        << "\"total_ms_p95\":" << detail::JsonNumber(p95_total_ms) << ','
        << "\"total_ms_p99\":" << detail::JsonNumber(p99_total_ms) << ','
        << "\"total_ms_max\":" << detail::JsonNumber(snapshot.max_total_loop_ms) << ','
        << "\"process_ms_avg\":" << detail::JsonNumber(avg_process_ms) << ','
        << "\"process_ms_p95\":" << detail::JsonNumber(p95_process_ms) << ','
        << "\"process_ms_p99\":" << detail::JsonNumber(p99_process_ms) << ','
        << "\"process_ms_max\":" << detail::JsonNumber(snapshot.max_process_loop_ms) << ','
        << "\"over_5ms\":" << snapshot.loop_over_warn_count << ','
        << "\"over_20ms\":" << snapshot.loop_over_tail_count << ','
        << "\"over_100ms\":" << snapshot.loop_over_critical_count << "},"
        << "\"log\":{"
        << "\"events_per_sec\":" << snapshot.log_event_count << ','
        << "\"raw_binary_bytes_per_sec\":" << snapshot.raw_binary_log_bytes << ','
        << "\"text_log_bytes_per_sec\":" << snapshot.text_log_bytes << ','
        << "\"queue_current\":" << snapshot.log_queue_current << ','
        << "\"queue_peak\":" << snapshot.log_queue_peak << ','
        << "\"queue_avg_occupancy_percent\":" << detail::JsonNumber(queue_occupancy_percent) << ','
        << "\"best_effort_dropped\":" << snapshot.best_effort_log_drop_count << ','
        << "\"critical_dropped\":" << snapshot.critical_log_drop_count << ','
        << "\"flush_batches\":" << snapshot.log_flush_batch_count << ','
        << "\"flush_ms_avg\":" << detail::JsonNumber(avg_flush_ms) << ','
        << "\"flush_ms_max\":" << detail::JsonNumber(snapshot.log_flush_max_ms) << "},"
        << "\"communication\":{"
        << "\"serial_idle_reads\":" << snapshot.serial_idle_read_count << ','
        << "\"serial_open_failures\":" << snapshot.serial_open_failure_count << ','
        << "\"serial_disconnects\":" << snapshot.serial_disconnect_count << ','
        << "\"serial_reconnects\":" << snapshot.serial_reconnect_count << ','
        << "\"tcp_idle_disconnects_total\":" << snapshot.tcp_idle_disconnect_count << ','
        << "\"map_0305_rate_limited_skips\":" << snapshot.map_robot_rate_limited_skip_count << ','
        << "\"map_0305_opponent_stale_skips\":" << snapshot.map_robot_opponent_stale_skip_count << ','
        << "\"radar_0121_waiting_cooldown\":" << snapshot.radar_command_waiting_cooldown_count << ','
        << "\"tcp_idle_disconnects_by_port\":{";
    bool first = true;
    for (const auto &[port, count] : snapshot.tcp_idle_disconnects_by_port) {
      if (!first) {
        oss << ',';
      }
      first = false;
      oss << '"' << port << "\":" << count;
    }
    oss << "},\"loss_rate_by_source\":{";
    first = true;
    for (const auto &[source, loss_rate] : snapshot.loss_rate_by_source) {
      if (!first) {
        oss << ',';
      }
      first = false;
      oss << '"' << source << "\":" << detail::JsonNumber(loss_rate, 4);
    }
    oss << "}}}";
    return oss.str();
  }

  /**
   * @brief 计算平均值
   */
  static double Average(const std::vector<double> &values) {
    if (values.empty()) {
      return 0.0;
    }
    double total = 0.0;
    for (const auto value : values) {
      total += value;
    }
    return total / static_cast<double>(values.size());
  }

  std::filesystem::path root_;
  std::mutex mutex_;
  Clock::time_point last_flush_time_;
  std::vector<double> loop_total_ms_samples_;
  std::vector<double> loop_process_ms_samples_;
  std::size_t loop_iteration_count_ = 0;
  std::size_t loop_over_warn_count_ = 0;
  std::size_t loop_over_tail_count_ = 0;
  std::size_t loop_over_critical_count_ = 0;
  double max_total_loop_ms_ = 0.0;
  double max_process_loop_ms_ = 0.0;
  std::size_t log_event_count_ = 0;
  std::size_t raw_binary_log_bytes_ = 0;
  std::size_t text_log_bytes_ = 0;
  std::size_t log_queue_peak_ = 0;
  std::size_t log_queue_current_ = 0;
  std::size_t log_queue_size_sum_ = 0;
  std::size_t log_queue_size_sample_count_ = 0;
  std::size_t best_effort_log_drop_count_ = 0;
  std::size_t critical_log_drop_count_ = 0;
  std::size_t log_flush_batch_count_ = 0;
  double log_flush_total_ms_ = 0.0;
  double log_flush_max_ms_ = 0.0;
  std::size_t serial_idle_read_count_ = 0;
  std::size_t serial_open_failure_count_ = 0;
  std::size_t serial_disconnect_count_ = 0;
  std::size_t serial_reconnect_count_ = 0;
  std::size_t tcp_idle_disconnect_count_ = 0;
  std::map<int, std::size_t> tcp_idle_disconnects_by_port_;
  std::size_t map_robot_rate_limited_skip_count_ = 0;
  std::size_t map_robot_opponent_stale_skip_count_ = 0;
  std::size_t radar_command_waiting_cooldown_count_ = 0;
  std::map<std::string, float> loss_rate_by_source_;
};

/**
 * @brief 获取全局运行指标实例
 * @param root 首次调用时用于初始化的日志根目录
 * @return 单例运行指标对象
 */
inline RuntimeMetrics &GetRuntimeMetrics(const std::filesystem::path &root = {}) {
  static std::mutex mutex;
  static std::unique_ptr<RuntimeMetrics> instance;
  std::lock_guard<std::mutex> lock(mutex);
  if (!instance) {
    if (root.empty()) {
      throw std::runtime_error("runtime metrics root not initialized");
    }
    instance = std::make_unique<RuntimeMetrics>(root);
  }
  return *instance;
}

/**
 * @brief 异步日志管理器
 * @note  主线程仅负责入队，后台线程负责真正的磁盘写入。
 */
class AsyncLogManager {
 public:
  /**
   * @brief 创建异步日志管理器
   * @param root 本轮运行日志根目录
   */
  explicit AsyncLogManager(std::filesystem::path root) : root_(std::move(root)) {
    std::filesystem::create_directories(root_);
    std::filesystem::create_directories(root_ / "main");
    std::filesystem::create_directories(root_ / "raw");
    worker_ = std::thread([this]() { Run(); });
  }

  AsyncLogManager(const AsyncLogManager &) = delete;
  AsyncLogManager &operator=(const AsyncLogManager &) = delete;

  ~AsyncLogManager() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  /**
   * @brief 追加文本日志
   */
  void AppendText(const std::filesystem::path &relative_path, const std::string &line, LogPriority priority) {
    EnqueueTask(LogTask::MakeText(relative_path, line, priority));
  }

  /**
   * @brief 追加二进制日志
   */
  void AppendBinary(const std::filesystem::path &relative_path, const rm::u8 *data, std::size_t size,
                    LogPriority priority) {
    if (size == 0) {
      return;
    }
    EnqueueTask(LogTask::MakeBinary(relative_path, data, size, priority));
  }

 private:
  /**
   * @brief 单条异步日志任务
   */
  struct LogTask {
    /**
     * @brief 日志任务类型
     */
    enum class Type {
      kAppendText,
      kAppendBinary,
    };

    Type type = Type::kAppendText;
    std::filesystem::path relative_path;
    std::string payload;
    LogPriority priority = LogPriority::kBestEffort;

    /**
     * @brief 构造文本日志任务
     */
    static LogTask MakeText(const std::filesystem::path &relative_path, const std::string &line,
                            LogPriority priority) {
      LogTask task;
      task.type = Type::kAppendText;
      task.relative_path = relative_path;
      task.payload = line;
      task.priority = priority;
      return task;
    }

    /**
     * @brief 构造二进制日志任务
     */
    static LogTask MakeBinary(const std::filesystem::path &relative_path, const rm::u8 *data, std::size_t size,
                              LogPriority priority) {
      LogTask task;
      task.type = Type::kAppendBinary;
      task.relative_path = relative_path;
      task.payload.assign(reinterpret_cast<const char *>(data), size);
      task.priority = priority;
      return task;
    }
  };

  /**
   * @brief 将任务压入异步队列
   */
  void EnqueueTask(LogTask task) {
    auto &metrics = GetRuntimeMetrics(root_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (task.priority == LogPriority::kBestEffort &&
        best_effort_queue_size_ >= config::kBestEffortLogQueueCapacity) {
      metrics.RecordLogDrop(task.priority);
      return;
    }

    if (task.priority != LogPriority::kBestEffort &&
        queue_.size() >= config::kCriticalLogSoftQueueCapacity) {
      DropOldestBestEffortLocked();
    }

    if (task.priority == LogPriority::kBestEffort &&
        best_effort_queue_size_ >= config::kBestEffortLogQueueCapacity) {
      metrics.RecordLogDrop(task.priority);
      return;
    }

    if (task.priority == LogPriority::kBestEffort) {
      ++best_effort_queue_size_;
    }
    queue_.push_back(std::move(task));
    metrics.RecordLogEnqueue(queue_.back().priority, queue_.size());
    cv_.notify_one();
  }

  /**
   * @brief 在 critical 队列压力过高时剔除最旧的 best-effort 日志
   */
  void DropOldestBestEffortLocked() {
    auto &metrics = GetRuntimeMetrics(root_);
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
      if (it->priority == LogPriority::kBestEffort) {
        queue_.erase(it);
        if (best_effort_queue_size_ != 0) {
          --best_effort_queue_size_;
        }
        metrics.RecordLogDrop(LogPriority::kBestEffort);
        metrics.ObserveLogQueue(queue_.size());
        return;
      }
    }
  }

  /**
   * @brief 后台写盘线程主循环
   */
  void Run() {
    while (true) {
      std::deque<LogTask> batch;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
        if (stop_ && queue_.empty()) {
          break;
        }
        batch.swap(queue_);
        best_effort_queue_size_ = 0;
      }

      auto &metrics = GetRuntimeMetrics(root_);
      metrics.ObserveLogQueue(0);

      const auto flush_start = RuntimeMetrics::Clock::now();
      for (const auto &task : batch) {
        WriteTask(task);
      }
      const auto flush_end = RuntimeMetrics::Clock::now();
      metrics.RecordLogFlushBatch(batch.size(),
                                  std::chrono::duration_cast<std::chrono::microseconds>(flush_end - flush_start));
    }
  }

  /**
   * @brief 执行单条日志任务
   */
  void WriteTask(const LogTask &task) {
    switch (task.type) {
      case LogTask::Type::kAppendText: {
        const auto path = root_ / task.relative_path;
        detail::EnsureParentDirectory(path);
        auto &stream = append_streams_[path.string()];
        if (!stream.is_open()) {
          stream.open(path, std::ios::app);
          if (!stream.is_open()) {
            throw std::runtime_error("failed to open log file: " + path.string());
          }
        }
        stream << task.payload << '\n';
        GetRuntimeMetrics(root_).RecordLogWrite(task.payload.size() + 1, false);
        return;
      }
      case LogTask::Type::kAppendBinary: {
        const auto path = root_ / task.relative_path;
        detail::EnsureParentDirectory(path);
        auto &stream = binary_streams_[path.string()];
        if (!stream.is_open()) {
          stream.open(path, std::ios::binary | std::ios::app);
          if (!stream.is_open()) {
            throw std::runtime_error("failed to open binary log file: " + path.string());
          }
        }
        stream.write(task.payload.data(), static_cast<std::streamsize>(task.payload.size()));
        GetRuntimeMetrics(root_).RecordLogWrite(task.payload.size(), true);
        return;
      }
    }
  }

  std::filesystem::path root_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<LogTask> queue_;
  std::size_t best_effort_queue_size_ = 0;
  bool stop_ = false;
  std::thread worker_;
  std::unordered_map<std::string, std::ofstream> append_streams_;
  std::unordered_map<std::string, std::ofstream> binary_streams_;
};

/**
 * @brief 获取全局异步日志管理器实例
 * @param root 首次调用时用于初始化的日志根目录
 * @return 单例异步日志管理器
 */
inline AsyncLogManager &GetAsyncLogManager(const std::filesystem::path &root = {}) {
  static std::mutex mutex;
  static std::unique_ptr<AsyncLogManager> instance;
  std::lock_guard<std::mutex> lock(mutex);
  if (!instance) {
    if (root.empty()) {
      throw std::runtime_error("async log root not initialized");
    }
    GetRuntimeMetrics(root);
    instance = std::make_unique<AsyncLogManager>(root);
  }
  return *instance;
}

/**
 * @brief 面向文本日志的轻量封装
 */
class FileLogStore {
 public:
  /**
   * @brief 创建文本日志存储器
   * @param root 本轮运行日志根目录
   */
  explicit FileLogStore(std::filesystem::path root) : root_(std::move(root)) { GetAsyncLogManager(root_); }

  /**
   * @brief 追加一条文本日志
   */
  void Append(const std::filesystem::path &relative_path, const std::string &line,
              LogPriority priority = LogPriority::kBestEffort) {
    GetAsyncLogManager(root_).AppendText(relative_path, line, priority);
  }

  const std::filesystem::path &root() const { return root_; }

 private:
  std::filesystem::path root_;  ///< 本轮运行日志根目录
};

/**
 * @brief 面向原始二进制日志的轻量封装
 */
class BinaryLogStore {
 public:
  /**
   * @brief 创建二进制日志存储器
   * @param root 本轮运行日志根目录
   */
  explicit BinaryLogStore(std::filesystem::path root) : root_(std::move(root)) { GetAsyncLogManager(root_); }

  /**
   * @brief 追加一段原始二进制数据
   */
  void Append(const std::filesystem::path &relative_path, const rm::u8 *data, std::size_t size,
              LogPriority priority = LogPriority::kCriticalRaw) {
    GetAsyncLogManager(root_).AppendBinary(relative_path, data, size, priority);
  }

  const std::filesystem::path &root() const { return root_; }

 private:
  std::filesystem::path root_;  ///< 本轮运行日志根目录
};

}  // namespace radar::log

#endif  // RADAR_INCLUDE_LOG_LOG_BACKEND_HPP
