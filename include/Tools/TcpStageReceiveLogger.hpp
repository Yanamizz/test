/**
 * @file    include/Tools/TcpStageReceiveLogger.hpp
 * @brief   记录 TCP 阶段接收的原始字节和解析结果。
 *
 * 该日志器只负责追加写入接收日志，不参与 TCP 协议解析和业务状态更新。
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include "Tools/TcpStageProtocol.hpp"

namespace Tools {

class TcpStageReceiveLogger {
public:
  explicit TcpStageReceiveLogger(
      std::string file_path = "tcp_stage_receive.log")
      : file_path_(std::move(file_path)),
        stream_(file_path_, std::ios::out | std::ios::app) {}

  bool IsOpen() const { return stream_.is_open(); }

  void LogDecodedCommand(const std::uint8_t *raw_data, std::size_t raw_size,
                         const TcpStageCommand &command) {
    std::ostringstream line;
    line << "RX raw=" << HexBytes(raw_data, raw_size);
    if (command.type == TcpStageCommandType::GameState91) {
      line << " type=0x91 game_progress="
           << static_cast<int>(command.game_progress)
           << " stage_remain_time=" << command.stage_remain_time;
    } else {
      line << " type=0x92 countered=" << (command.countered ? 1 : 0);
    }
    WriteLine(line.str());
  }

  void LogInvalidBytes(const std::uint8_t *raw_data, std::size_t raw_size) {
    WriteLine("RX invalid raw=" + HexBytes(raw_data, raw_size));
  }

private:
  static std::string Timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) %
        1000;

    std::tm local_time{};
    localtime_r(&seconds, &local_time);

    std::ostringstream out;
    out << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setfill('0') << std::setw(3) << milliseconds.count();
    return out.str();
  }

  static std::string HexBytes(const std::uint8_t *data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return "<empty>";
    }

    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < size; ++i) {
      if (i != 0) {
        out << ' ';
      }
      out << std::setw(2) << static_cast<int>(data[i]);
    }
    return out.str();
  }

  void WriteLine(const std::string &message) {
    if (!stream_.is_open()) {
      return;
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    stream_ << Timestamp() << ' ' << message << '\n';
    stream_.flush();
  }

  std::string file_path_;
  std::ofstream stream_;
  inline static std::mutex write_mutex_;
};

} // namespace Tools
