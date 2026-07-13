#ifndef RADAR_INCLUDE_REFEREE_EXTERNAL_SERVER_SENDER_HPP
#define RADAR_INCLUDE_REFEREE_EXTERNAL_SERVER_SENDER_HPP

/**
 * @file  include/referee/external_server_sender.hpp
 * @brief 向外部 TCP server 客户端发送原始自定义状态字节流
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "include/log/referee_main_log.hpp"
#include "include/referee/tcp_server.hpp"
#include "librm/device/referee/protocol.hpp"

#ifndef RADAR_DEFAULT_LOG_DIR
#define RADAR_DEFAULT_LOG_DIR "test/logs"
#endif

namespace radar::referee {

/// 外部自定义状态：比赛进程与剩余时间。
constexpr rm::u8 kExternalServerGameStatusCmd = 0x91;
/// 外部自定义状态：对方空中机器人当前是否处于被反制状态。
constexpr rm::u8 kExternalServerRadarMarkBitCmd = 0x92;
/// `0x020c radar_mark_data.mark_progress` 的 bit13 掩码。
constexpr rm::u16 kOpponentAerialRobotCounteredMask = static_cast<rm::u16>(1u << 13);
/// 外部 TCP server 自定义状态发送频率。
constexpr int kExternalServerSendIntervalMs = 1000;

/**
 * @brief 将串口主协议中的指定状态透传给额外 TCP server 客户端
 * @note  此处不走裁判系统封包，只发送“命令头 + 信息”原始字节。
 */
class ExternalServerSender {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit ExternalServerSender(TcpServer *server, std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : server_(server), log_store_(std::move(log_root)) {}

  /**
   * @brief 根据最新串口协议状态更新外部自定义消息缓存
   * @tparam revision 当前裁判系统协议版本
   * @param cmd_id 本次完成解包的主命令码
   * @param seq 协议序号；当前仅用于保持回调接口一致
   * @param protocol 当前串口维护的完整协议状态
   */
  template <rm::device::RefereeRevision revision>
  void ProcessSerial(rm::u16 cmd_id, rm::u8 seq, const rm::device::RefereeProtocol<revision> &protocol) {
    (void)seq;
    using Cmd = rm::device::RefereeCmdId<revision>;
    if (cmd_id == Cmd::kGameStatus) {
      latest_game_status_ =
          GameStatusSnapshot{protocol.game_status.game_progress, protocol.game_status.stage_remain_time};
      return;
    }
    if (cmd_id == Cmd::kRadarMarkData) {
      latest_opponent_aerial_robot_countered_ = static_cast<rm::u8>(
          (protocol.radar_mark_data.mark_progress & kOpponentAerialRobotCounteredMask) != 0);
    }
  }

  /**
   * @brief 以固定 1Hz 频率向外部客户端发送最近一次缓存状态
   */
  void ProcessPeriodic() {
    const auto now = Clock::now();
    if (latest_game_status_.has_value() && IsDue(last_game_status_dispatch_time_, now)) {
      last_game_status_dispatch_time_ = now;
      SendGameStatus(latest_game_status_->game_progress, latest_game_status_->stage_remain_time);
    }
    if (latest_opponent_aerial_robot_countered_.has_value() && IsDue(last_radar_mark_dispatch_time_, now)) {
      last_radar_mark_dispatch_time_ = now;
      SendOpponentAerialRobotCountered(*latest_opponent_aerial_robot_countered_);
    }
  }

 private:
  struct GameStatusSnapshot {
    rm::u8 game_progress = 0;
    rm::u16 stage_remain_time = 0;
  };

  static bool IsDue(const std::optional<TimePoint> &last_dispatch_time, const TimePoint &now) {
    return !last_dispatch_time.has_value() ||
           now - *last_dispatch_time >= std::chrono::milliseconds(kExternalServerSendIntervalMs);
  }

  bool SendGameStatus(rm::u8 game_progress, rm::u16 stage_remain_time) {
    const std::array<rm::u8, 4> payload = {
        kExternalServerGameStatusCmd,
        game_progress,
        static_cast<rm::u8>(stage_remain_time & 0xff),
        static_cast<rm::u8>((stage_remain_time >> 8) & 0xff),
    };
    return SendRaw(payload.data(), payload.size(), kExternalServerGameStatusCmd, 0x0001);
  }

  bool SendOpponentAerialRobotCountered(rm::u8 countered) {
    const std::array<rm::u8, 2> payload = {
        kExternalServerRadarMarkBitCmd,
        static_cast<rm::u8>(countered != 0 ? 1 : 0),
    };
    return SendRaw(payload.data(), payload.size(), kExternalServerRadarMarkBitCmd, 0x020c);
  }

  bool SendRaw(const rm::u8 *payload, std::size_t payload_size, rm::u8 custom_cmd, rm::u16 source_cmd_id) {
    if (server_ == nullptr || !server_->is_open() || !server_->has_client()) {
      return false;
    }

    std::string error;
    if (!server_->TryWriteAll(payload, payload_size, &error)) {
      LogWriteFailure(custom_cmd, source_cmd_id, payload, payload_size, error);
      return false;
    }

    LogSent(custom_cmd, source_cmd_id, payload, payload_size);
    return true;
  }

  void LogSent(rm::u8 custom_cmd, rm::u16 source_cmd_id, const rm::u8 *payload, std::size_t payload_size) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"result\":\"sent\","
          << "\"peer\":\"" << server_->peer_ip() << "\","
          << "\"custom_cmd\":\"" << radar::log::HexU16(custom_cmd) << "\","
          << "\"source_cmd\":\"" << radar::log::HexU16(source_cmd_id) << "\","
          << "\"payload_len\":" << payload_size << ','
          << "\"payload_hex\":\"" << radar::log::HexBytes(payload, payload_size) << "\"}";
    log_store_.Append("main/external_tcp_server_tx.log", entry.str(), radar::log::LogPriority::kCriticalDecision);
  }

  void LogWriteFailure(rm::u8 custom_cmd, rm::u16 source_cmd_id, const rm::u8 *payload, std::size_t payload_size,
                       const std::string &reason) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"result\":\"write_failed\","
          << "\"reason\":\"" << reason << "\","
          << "\"peer\":\"" << server_->peer_ip() << "\","
          << "\"custom_cmd\":\"" << radar::log::HexU16(custom_cmd) << "\","
          << "\"source_cmd\":\"" << radar::log::HexU16(source_cmd_id) << "\","
          << "\"payload_len\":" << payload_size << ','
          << "\"payload_hex\":\"" << radar::log::HexBytes(payload, payload_size) << "\"}";
    log_store_.Append("main/external_tcp_server_tx.log", entry.str(), radar::log::LogPriority::kCriticalDecision);
  }

  TcpServer *server_ = nullptr;
  radar::log::FileLogStore log_store_;
  std::optional<GameStatusSnapshot> latest_game_status_;
  std::optional<rm::u8> latest_opponent_aerial_robot_countered_;
  std::optional<TimePoint> last_game_status_dispatch_time_;
  std::optional<TimePoint> last_radar_mark_dispatch_time_;
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_EXTERNAL_SERVER_SENDER_HPP
