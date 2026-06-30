#ifndef RADAR_INCLUDE_REFEREE_SERIAL_CONNECTION_LOG_HPP
#define RADAR_INCLUDE_REFEREE_SERIAL_CONNECTION_LOG_HPP

/**
 * @file  include/referee/serial_connection_log.hpp
 * @brief 串口连接状态日志封装
 */

#include <filesystem>
#include <sstream>
#include <string>

#include "include/log/referee_main_log.hpp"

namespace radar::referee {

/**
 * @brief 维护串口连接状态日志
 * @note   用于记录打开失败、掉线、重连等关键状态切换。
 */
class SerialConnectionLog {
 public:
  explicit SerialConnectionLog(std::filesystem::path root) : log_store_(std::move(root)) {}

  /**
   * @brief 记录一次串口状态变化
   * @param state 当前状态名，例如 `connected` 或 `reconnected`
   * @param device 对应的设备节点
   * @param baud 当前串口波特率
   * @param detail 补充说明
   * @param connected 串口是否处于可用状态
   */
  void LogState(const std::string &state, const std::string &device, int baud, const std::string &detail,
                bool connected) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"state\":\"" << state << "\","
          << "\"device\":\"" << device << "\","
          << "\"baud\":" << baud << ','
          << "\"connected\":" << (connected ? "true" : "false") << ','
          << "\"detail\":\"" << EscapeJson(detail) << "\"}";
    log_store_.Append("main/serial_state.log", entry.str(), radar::log::LogPriority::kCriticalDecision);
  }

 private:
  /**
   * @brief 转义 JSON 中的特殊字符
   * @param text 待写入日志的原始文本
   * @return 可安全写入 JSON 字符串的文本
   */
  static std::string EscapeJson(const std::string &text) {
    std::ostringstream escaped;
    for (const char ch : text) {
      switch (ch) {
        case '\\':
          escaped << "\\\\";
          break;
        case '"':
          escaped << "\\\"";
          break;
        case '\n':
          escaped << "\\n";
          break;
        case '\r':
          escaped << "\\r";
          break;
        case '\t':
          escaped << "\\t";
          break;
        default:
          escaped << ch;
          break;
      }
    }
    return escaped.str();
  }

  radar::log::FileLogStore log_store_;  ///< 串口状态日志输出器
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_SERIAL_CONNECTION_LOG_HPP
