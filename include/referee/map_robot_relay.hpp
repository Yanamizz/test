#ifndef RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP
#define RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP

/**
 * @file  include/referee/map_robot_relay.hpp
 * @brief `0x0A01 + 0x0301/0x0200 -> 0x0305` 状态维护、组包与日志
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/radar/app/subReferee/referee_user.hpp"
#include "include/referee/referee_tx_scheduler.hpp"
#include "librm/device/referee/referee.hpp"

#ifndef RADAR_DEFAULT_LOG_DIR
#define RADAR_DEFAULT_LOG_DIR "test/logs"
#endif

namespace radar::referee {

/// 信息波状态过期时间。
inline constexpr int kInfoWaveStateTimeoutMs = 500;
/// 常规链路状态过期时间。
inline constexpr int kRefereeStateTimeoutMs = 3000;

/**
 * @brief 当前 `MapRobotRelay` 支持的协议版本
 */
template <rm::device::RefereeRevision revision>
constexpr bool kSupportedMapRobotRelayRevision =
    revision == rm::device::RefereeRevision::kNewV120 || revision == rm::device::RefereeRevision::kNewV200;

/**
 * @brief 将单位为米的浮点坐标转换为裁判协议中的厘米整数
 * @param value 米制坐标
 * @return 裁剪到 `u16` 范围内的厘米值
 */
inline rm::u16 FloatMetersToCentimeters(float value) {
  if (!std::isfinite(value) || value <= 0.0f) {
    return 0;
  }

  const double scaled = std::round(static_cast<double>(value) * 100.0);
  const double clamped = std::max(0.0, std::min(scaled, static_cast<double>(std::numeric_limits<rm::u16>::max())));
  return static_cast<rm::u16>(clamped);
}

/**
 * @brief 将 `MapRobotPosition` 格式化为 JSON 片段
 * @param map 当前维护的小地图位置结构体
 * @return JSON 字符串
 */
inline std::string FormatMapRobotData(const rm::device::MapRobotPosition &map) {
  std::ostringstream oss;
  oss << "{"
      << "\"opponent_hero_position_x\":" << radar::log::JsonScalar(map.opponent_hero_position_x) << ','
      << "\"opponent_hero_position_y\":" << radar::log::JsonScalar(map.opponent_hero_position_y) << ','
      << "\"opponent_engineer_position_x\":" << radar::log::JsonScalar(map.opponent_engineer_position_x) << ','
      << "\"opponent_engineer_position_y\":" << radar::log::JsonScalar(map.opponent_engineer_position_y) << ','
      << "\"opponent_infantry_3_position_x\":" << radar::log::JsonScalar(map.opponent_infantry_3_position_x) << ','
      << "\"opponent_infantry_3_position_y\":" << radar::log::JsonScalar(map.opponent_infantry_3_position_y) << ','
      << "\"opponent_infantry_4_position_x\":" << radar::log::JsonScalar(map.opponent_infantry_4_position_x) << ','
      << "\"opponent_infantry_4_position_y\":" << radar::log::JsonScalar(map.opponent_infantry_4_position_y) << ','
      << "\"opponent_aerial_position_x\":" << radar::log::JsonScalar(map.opponent_aerial_position_x) << ','
      << "\"opponent_aerial_position_y\":" << radar::log::JsonScalar(map.opponent_aerial_position_y) << ','
      << "\"opponent_sentry_position_x\":" << radar::log::JsonScalar(map.opponent_sentry_position_x) << ','
      << "\"opponent_sentry_position_y\":" << radar::log::JsonScalar(map.opponent_sentry_position_y) << ','
      << "\"ally_hero_position_x\":" << radar::log::JsonScalar(map.ally_hero_position_x) << ','
      << "\"ally_hero_position_y\":" << radar::log::JsonScalar(map.ally_hero_position_y) << ','
      << "\"ally_engineer_position_x\":" << radar::log::JsonScalar(map.ally_engineer_position_x) << ','
      << "\"ally_engineer_position_y\":" << radar::log::JsonScalar(map.ally_engineer_position_y) << ','
      << "\"ally_infantry_3_position_x\":" << radar::log::JsonScalar(map.ally_infantry_3_position_x) << ','
      << "\"ally_infantry_3_position_y\":" << radar::log::JsonScalar(map.ally_infantry_3_position_y) << ','
      << "\"ally_infantry_4_position_x\":" << radar::log::JsonScalar(map.ally_infantry_4_position_x) << ','
      << "\"ally_infantry_4_position_y\":" << radar::log::JsonScalar(map.ally_infantry_4_position_y) << ','
      << "\"ally_aerial_position_x\":" << radar::log::JsonScalar(map.ally_aerial_position_x) << ','
      << "\"ally_aerial_position_y\":" << radar::log::JsonScalar(map.ally_aerial_position_y) << ','
      << "\"ally_sentry_position_x\":" << radar::log::JsonScalar(map.ally_sentry_position_x) << ','
      << "\"ally_sentry_position_y\":" << radar::log::JsonScalar(map.ally_sentry_position_y) << "}";
  return oss.str();
}

/**
 * @brief 用敌方协议状态与己方 `0x0301/0x0200` 状态组装 `0x0305`
 * @tparam revision 当前裁判协议版本
 * @param opponent_protocol 敌方状态来源，主要读取 `radar0`
 * @param ally_position 己方状态来源，读取 `0x0301` 下的 `0x0200 AllyRobotPosition`
 * @return 已完成字段映射的 `MapRobotPosition`
 */
template <rm::device::RefereeRevision revision>
rm::device::MapRobotPosition BuildMapRobotPosition(const rm::device::RefereeProtocol<revision> &opponent_protocol,
                                                   const rm::device::AllyRobotPosition &ally_position) {
  static_assert(kSupportedMapRobotRelayRevision<revision>,
                "MapRobotRelay only supports kNewV120 and kNewV200");

  rm::device::MapRobotPosition map{};
  const auto &opponent = opponent_protocol.radar0;
  const auto &ally = ally_position;

  map.opponent_hero_position_x = opponent.opponent_hero_position_x;
  map.opponent_hero_position_y = opponent.opponent_hero_position_y;
  map.opponent_engineer_position_x = opponent.opponent_engineer_position_x;
  map.opponent_engineer_position_y = opponent.opponent_engineer_position_y;
  map.opponent_infantry_3_position_x = opponent.opponent_infantry_3_position_x;
  map.opponent_infantry_3_position_y = opponent.opponent_infantry_3_position_y;
  map.opponent_infantry_4_position_x = opponent.opponent_infantry_4_position_x;
  map.opponent_infantry_4_position_y = opponent.opponent_infantry_4_position_y;
  map.opponent_aerial_position_x = opponent.opponent_aerial_position_x;
  map.opponent_aerial_position_y = opponent.opponent_aerial_position_y;
  map.opponent_sentry_position_x = opponent.opponent_sentry_position_x;
  map.opponent_sentry_position_y = opponent.opponent_sentry_position_y;

  map.ally_hero_position_x = FloatMetersToCentimeters(ally.hero_1_x);
  map.ally_hero_position_y = FloatMetersToCentimeters(ally.hero_1_y);
  map.ally_engineer_position_x = FloatMetersToCentimeters(ally.engineer_2_x);
  map.ally_engineer_position_y = FloatMetersToCentimeters(ally.engineer_2_y);
  map.ally_infantry_3_position_x = FloatMetersToCentimeters(ally.standard_3_x);
  map.ally_infantry_3_position_y = FloatMetersToCentimeters(ally.standard_3_y);
  map.ally_infantry_4_position_x = FloatMetersToCentimeters(ally.standard_4_x);
  map.ally_infantry_4_position_y = FloatMetersToCentimeters(ally.standard_4_y);
  map.ally_aerial_position_x = 0;
  map.ally_aerial_position_y = 0;
  if (radar::config::kMapRobotUseFixedAllyRobotPositions) {
    const auto &fixed = radar::config::kMapRobotFixedAllyRobotPositionsCm;
    map.ally_hero_position_x = fixed[0][0];
    map.ally_hero_position_y = fixed[0][1];
    map.ally_engineer_position_x = fixed[1][0];
    map.ally_engineer_position_y = fixed[1][1];
    map.ally_infantry_3_position_x = fixed[2][0];
    map.ally_infantry_3_position_y = fixed[2][1];
    map.ally_infantry_4_position_x = fixed[3][0];
    map.ally_infantry_4_position_y = fixed[3][1];
    map.ally_aerial_position_x = fixed[4][0];
    map.ally_aerial_position_y = fixed[4][1];
    map.ally_sentry_position_x = fixed[5][0];
    map.ally_sentry_position_y = fixed[5][1];
  } else if (radar::config::kMapRobotUseFixedAllySentryPosition) {
    map.ally_sentry_position_x = radar::config::kMapRobotFixedAllySentryPositionX;
    map.ally_sentry_position_y = radar::config::kMapRobotFixedAllySentryPositionY;
  } else {
    map.ally_sentry_position_x = FloatMetersToCentimeters(ally.reserved_1);
    map.ally_sentry_position_y = FloatMetersToCentimeters(ally.reserved_2);
  }

  return map;
}

/**
 * @brief 生成一次成功发送 `0x0305` 的日志内容
 */
inline std::string BuildMapRobotLogEntry(const rm::device::MapRobotPosition &map, const rm::u8 *frame,
                                         std::size_t frame_len, bool has_ally_robot_position,
                                         std::size_t ally_position_seen_count, bool ally_position_available,
                                         const std::string &ally_position_source,
                                         bool opponent_fresh, bool ally_position_fresh,
                                         std::optional<long long> last_radar0_interval_ms,
                                         std::optional<long long> last_opponent_age_ms,
                                         std::optional<long long> last_ally_position_age_ms,
                                         std::optional<long long> tx_queue_delay_ms,
                                         const std::string &send_skipped_reason) {
  const auto payload_offset = rm::device::kRefProtocolHeaderLen + rm::device::kRefProtocolCmdIdLen;
  std::ostringstream oss;
  oss << "{"
      << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
      << "\"seq\":\"" << radar::log::HexU8(frame[3]) << "\","
      << "\"cmd_id\":\"" << radar::log::HexU16(rm::device::getCmd(map)) << "\","
      << "\"name\":\"map_robot_data\","
      << "\"source_cmd\":\"" << radar::log::HexU16(0x0a01) << "\","
      << "\"ally_source_cmd\":\"0x0301/0x0200\","
      << "\"has_0x0200\":" << (has_ally_robot_position ? "true" : "false") << ','
      << "\"ally_position_seen_count\":" << ally_position_seen_count << ','
      << "\"ally_position_available\":" << (ally_position_available ? "true" : "false") << ','
      << "\"ally_position_source\":\"" << ally_position_source << "\","
      << "\"opponent_fresh\":" << (opponent_fresh ? "true" : "false") << ','
      << "\"ally_position_fresh\":" << (ally_position_fresh ? "true" : "false") << ','
      << "\"last_0x0a01_interval_ms\":";
  if (last_radar0_interval_ms.has_value()) {
    oss << *last_radar0_interval_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0a01_age_ms\":";
  if (last_opponent_age_ms.has_value()) {
    oss << *last_opponent_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0200_age_ms\":";
  if (last_ally_position_age_ms.has_value()) {
    oss << *last_ally_position_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"tx_queue_delay_ms\":";
  if (tx_queue_delay_ms.has_value()) {
    oss << *tx_queue_delay_ms;
  } else {
    oss << "null";
  }
  oss << ",\"send_skipped_reason\":\"" << send_skipped_reason << "\","
      << "\"payload_len\":" << sizeof(map) << ','
      << "\"frame_hex\":\"" << radar::log::HexBytes(frame, frame_len) << "\","
      << "\"payload_hex\":\"" << radar::log::HexBytes(frame + payload_offset, sizeof(map)) << "\","
      << "\"state\":" << FormatMapRobotData(map) << "}";
  return oss.str();
}

/**
 * @brief 生成一次未发送 `0x0305` 的跳过日志内容
 */
inline std::string BuildMapRobotSkipLogEntry(bool has_ally_robot_position, bool opponent_fresh,
                                             std::size_t ally_position_seen_count, bool ally_position_available,
                                             const std::string &ally_position_source,
                                             bool ally_position_fresh,
                                             std::optional<long long> last_radar0_interval_ms,
                                             std::optional<long long> last_opponent_age_ms,
                                             std::optional<long long> last_ally_position_age_ms,
                                             const std::string &send_skipped_reason) {
  std::ostringstream oss;
  oss << "{"
      << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
      << "\"cmd_id\":\"" << radar::log::HexU16(0x0305) << "\","
      << "\"name\":\"map_robot_data\","
      << "\"source_cmd\":\"" << radar::log::HexU16(0x0a01) << "\","
      << "\"ally_source_cmd\":\"0x0301/0x0200\","
      << "\"has_0x0200\":" << (has_ally_robot_position ? "true" : "false") << ','
      << "\"ally_position_seen_count\":" << ally_position_seen_count << ','
      << "\"ally_position_available\":" << (ally_position_available ? "true" : "false") << ','
      << "\"ally_position_source\":\"" << ally_position_source << "\","
      << "\"opponent_fresh\":" << (opponent_fresh ? "true" : "false") << ','
      << "\"ally_position_fresh\":" << (ally_position_fresh ? "true" : "false") << ','
      << "\"last_0x0a01_interval_ms\":";
  if (last_radar0_interval_ms.has_value()) {
    oss << *last_radar0_interval_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0a01_age_ms\":";
  if (last_opponent_age_ms.has_value()) {
    oss << *last_opponent_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0200_age_ms\":";
  if (last_ally_position_age_ms.has_value()) {
    oss << *last_ally_position_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"send_skipped_reason\":\"" << send_skipped_reason << "\"}";
  return oss.str();
}

/**
 * @brief 维护 `0x0A01` / `0x0301(0x0200)` 状态并驱动 `0x0305` 发送
 * @tparam revision 当前裁判协议版本
 */
template <rm::device::RefereeRevision revision>
class MapRobotRelay {
 public:
  using Referee = rm::device::Referee<revision>;
  using Protocol = rm::device::RefereeProtocol<revision>;
  using Cmd = rm::device::RefereeCmdId<revision>;
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  static_assert(kSupportedMapRobotRelayRevision<revision>,
                "MapRobotRelay only supports kNewV120 and kNewV200");

  /**
   * @brief 创建 `0x0305` relay
   * @param tx_scheduler 统一发送调度器
   * @param log_root 本轮运行日志根目录
   */
  explicit MapRobotRelay(RefereeTxScheduler &tx_scheduler, std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : tx_scheduler_(tx_scheduler),
        main_logger_(log_root),
        send_log_store_(std::move(log_root)),
        metrics_(radar::log::GetRuntimeMetrics(main_logger_.root())) {}

  /**
   * @brief 单源回放入口
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param referee 当前维护状态
   */
  void Process(rm::u16 cmd_id, rm::u8 seq, Referee &referee) {
    ProcessSingleSource(cmd_id, seq, referee);
  }

  /**
   * @brief 处理常规链路主协议回调
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param serial_referee 当前常规链路状态
   */
  void ProcessSerial(rm::u16 cmd_id, rm::u8 seq, Referee &serial_referee) {
    main_logger_.LogFrame(cmd_id, seq, serial_referee.data(), serial_referee.loss_rate());
    metrics_.RecordLossRate("serial", serial_referee.loss_rate());

    if (cmd_id == Cmd::kRobotInteractionData) {
      UpdateAllyRobotPositionState(serial_referee, cmd_id);
    }
  }

  /**
   * @brief 处理信息波 TCP 主协议回调
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param info_wave_referee 当前信息波状态
   */
  void ProcessInfoWaveTcp(rm::u16 cmd_id, rm::u8 seq, const Referee &info_wave_referee) {
    main_logger_.LogFrame(cmd_id, seq, info_wave_referee.data(), info_wave_referee.loss_rate());
    metrics_.RecordLossRate("info_wave_tcp", info_wave_referee.loss_rate());

    if (cmd_id == Cmd::kRadar0) {
      const auto now = Clock::now();
      UpdateRadar0State(info_wave_referee.data(), now);
      RefreshLatestMapRobotFrame(now);
    }
  }

  /**
   * @brief 推进一次 `0x0305` 周期处理
   * @note  负责检查状态是否过期、是否限频，以及是否更新 latest-only 待发送帧。
   */
  void ProcessPeriodic() {
    const auto now = Clock::now();
    const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
    const bool ally_position_fresh = IsFreshAt(last_ally_robot_position_time_, now, AllyPositionTimeout());
    if (!opponent_fresh && !AllowMissingInterfaces()) {
      tx_scheduler_.ClearLatestMapRobotFrame();
      if (last_radar0_time_.has_value()) {
        LogMapRobotSkip(now, opponent_fresh, ally_position_fresh, "opponent_stale");
      } else {
        LogMapRobotSkip(now, opponent_fresh, ally_position_fresh, "opponent_missing");
      }
      return;
    }
    if (!tx_scheduler_.IsMapRobotSendDue(now)) {
      LogMapRobotSkip(now, opponent_fresh, ally_position_fresh, "rate_limited");
    }

    QueueMapRobotPosition(latest_opponent_protocol_, latest_ally_robot_position_, now);
  }

  /**
   * @brief 消费一次“本轮已收到 fresh `0x0A01` 并刷新 latest-only `0x0305`”标记
   * @return 若本轮应尝试提前发送则返回 true；返回后自动清零
   */
  bool ConsumeFreshRadar0Refresh() {
    const bool refreshed = fresh_radar0_refresh_pending_;
    fresh_radar0_refresh_pending_ = false;
    return refreshed;
  }

  bool has_ally_robot_position() const { return has_ally_robot_position_; }

 private:
  /**
   * @brief 单源回放模式下的统一处理逻辑
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param referee 当前维护状态
   */
  void ProcessSingleSource(rm::u16 cmd_id, rm::u8 seq, Referee &referee) {
    main_logger_.LogFrame(cmd_id, seq, referee.data(), referee.loss_rate());
    metrics_.RecordLossRate("replay", referee.loss_rate());

    if (cmd_id == Cmd::kRobotInteractionData) {
      UpdateAllyRobotPositionState(referee, cmd_id);
      return;
    }

    if (cmd_id != Cmd::kRadar0) {
      return;
    }

    UpdateRadar0State(referee.data(), Clock::now());
    ProcessPeriodic();
  }

  /**
   * @brief 用当前 `0x0A01` 更新敌方状态与时间戳
   */
  void UpdateRadar0State(const Protocol &opponent_protocol, TimePoint now) {
    if (last_radar0_time_.has_value()) {
      last_radar0_interval_ms_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_radar0_time_).count();
    }
    latest_opponent_protocol_ = opponent_protocol;
    last_radar0_time_ = now;
  }

  /**
   * @brief 基于当前维护状态刷新 latest-only `0x0305`
   * @note  仅更新待发送帧，不在此处执行真正串口发送。
   */
  void RefreshLatestMapRobotFrame(TimePoint now) {
    QueueMapRobotPosition(latest_opponent_protocol_, latest_ally_robot_position_, now);
    fresh_radar0_refresh_pending_ = true;
  }

  /**
   * @brief 使用敌方状态和己方 `0x0200` 状态排队一帧 `0x0305`
   * @param opponent_protocol 敌方状态来源
   * @param ally_position 己方状态来源
   * @param now 当前时间
   */
  void QueueMapRobotPosition(const Protocol &opponent_protocol, const rm::device::AllyRobotPosition &ally_position,
                             TimePoint now) {
    static_assert(rm::device::kRefProtocolAllMetadataLen + sizeof(rm::device::MapRobotPosition) <=
                      rm::device::kRefProtocolFrameMaxLen,
                  "MapRobotPosition frame exceeds referee protocol frame buffer");
    Protocol opponent_zero_filled_protocol = opponent_protocol;
    rm::device::AllyRobotPosition ally_position_zero_filled = ally_position;
    const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
    const bool ally_position_fresh = IsFreshAt(last_ally_robot_position_time_, now, AllyPositionTimeout());
    if (!opponent_fresh) {
      opponent_zero_filled_protocol.radar0 = {};
    }
    if (!ally_position_fresh) {
      ally_position_zero_filled = {};
    }

    const auto map = BuildMapRobotPosition<revision>(opponent_zero_filled_protocol, ally_position_zero_filled);
    // RefereePrepare 会写满前 frame_len 字节，且只有这些字节会被拷入 frame，故无需先整体零初始化。
    std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> tx_buffer;
    const rm::u8 frame_len = rm::device::RefereePrepare(tx_buffer.data(), 0, map);
    std::vector<rm::u8> frame(tx_buffer.begin(), tx_buffer.begin() + frame_len);
    const auto enqueue_time = Clock::now();
    tx_scheduler_.UpdateLatestMapRobotFrame(
        std::move(frame), [this, map, opponent_fresh, ally_position_fresh, enqueue_time](const rm::u8 *frame,
                                                                                          std::size_t frame_len) {
          LogMapRobotData(map, frame, frame_len, Clock::now(), opponent_fresh, ally_position_fresh,
                          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - enqueue_time).count(),
                          "");
        });
  }

  /**
   * @brief 从 `0x0301` 当前帧中提取 `0x0200 AllyRobotPosition`
   */
  void UpdateAllyRobotPositionState(Referee &referee, rm::u16 cmd_id) {
    if (cmd_id != Cmd::kRobotInteractionData ||
        referee.data().robot_interaction_data.data_cmd_id != rm::device::RefereeSubCmdId::kAllyRobotPosition) {
      return;
    }

    rm::device::RefereeUser<revision> sub_referee(referee);
    sub_referee.AttachCallback(cmd_id, 0);
    has_ally_robot_position_ = true;
    ++ally_position_seen_count_;
    latest_ally_robot_position_ = sub_referee.data().ally_robot_position;
    last_ally_robot_position_time_ = Clock::now();
  }

  /**
   * @brief 记录一条成功发送的 `0x0305` 日志
   */
  void LogMapRobotData(const rm::device::MapRobotPosition &map, const rm::u8 *frame, std::size_t frame_len,
                       TimePoint now, bool opponent_fresh, bool ally_position_fresh,
                       std::optional<long long> tx_queue_delay_ms,
                       const std::string &send_skipped_reason) {
    const bool ally_position_available = has_ally_robot_position_ && ally_position_fresh;
    const auto entry =
        BuildMapRobotLogEntry(map, frame, frame_len, has_ally_robot_position_, ally_position_seen_count_,
                              ally_position_available, AllyPositionSource(),
                              opponent_fresh, ally_position_fresh, last_radar0_interval_ms_,
                              AgeMs(last_radar0_time_, now), AgeMs(last_ally_robot_position_time_, now),
                              tx_queue_delay_ms,
                              send_skipped_reason);
    const std::filesystem::path basename = "0x0305_map_robot_data";
    send_log_store_.Append(std::filesystem::path("main") / (basename.string() + ".log"), entry,
                           radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一条未发送 `0x0305` 的原因日志
   */
  void LogMapRobotSkip(TimePoint now, bool opponent_fresh, bool ally_position_fresh,
                       const std::string &send_skipped_reason) {
    if (send_skipped_reason == "opponent_stale" &&
        last_opponent_stale_log_radar0_time_ == last_radar0_time_) {
      return;
    }
    if (last_skip_log_time_.has_value() && last_skip_log_reason_ == send_skipped_reason &&
        now - *last_skip_log_time_ < SkipLogMinInterval()) {
      return;
    }

    metrics_.RecordMapRobotSkip(send_skipped_reason);
    const bool ally_position_available = has_ally_robot_position_ && ally_position_fresh;
    const auto entry = BuildMapRobotSkipLogEntry(has_ally_robot_position_, opponent_fresh,
                                                 ally_position_seen_count_, ally_position_available,
                                                 AllyPositionSource(), ally_position_fresh,
                                                 last_radar0_interval_ms_,
                                                 AgeMs(last_radar0_time_, now),
                                                 AgeMs(last_ally_robot_position_time_, now),
                                                 send_skipped_reason);
    const std::filesystem::path basename = "0x0305_map_robot_data_skipped";
    send_log_store_.Append(std::filesystem::path("main") / (basename.string() + ".log"), entry,
                           radar::log::LogPriority::kBestEffort);
    last_skip_log_time_ = now;
    last_skip_log_reason_ = send_skipped_reason;
    if (send_skipped_reason == "opponent_stale") {
      last_opponent_stale_log_radar0_time_ = last_radar0_time_;
    }
  }

  /**
   * @brief 信息波状态超时时间
   * @return 对应超时阈值
   */
  static std::chrono::milliseconds InfoWaveTimeout() {
    return std::chrono::milliseconds(kInfoWaveStateTimeoutMs);
  }

  /**
   * @brief 常规链路 `0x0301/0x0200` 状态超时时间
   * @return 对应超时阈值
   */
  static std::chrono::milliseconds AllyPositionTimeout() {
    return std::chrono::milliseconds(kRefereeStateTimeoutMs);
  }

  /**
   * @brief `0x0305` 最小发送间隔
   * @return 对应限频阈值
   */
  static std::chrono::milliseconds MapRobotMinSendInterval() {
    return std::chrono::milliseconds(radar::config::kMapRobotMinSendIntervalMs);
  }

  /**
   * @brief 跳过日志最小输出间隔
   * @return 日志节流周期
   */
  static std::chrono::milliseconds SkipLogMinInterval() {
    return std::chrono::milliseconds(1000);
  }

  /**
   * @brief 返回当前是否允许在缺失接口时继续按 0 值运行
   */
  static constexpr bool AllowMissingInterfaces() {
    return radar::config::kDebugAllowMissingInterfaces;
  }

  /**
   * @brief 判断某路状态是否仍处于有效窗口
   */
  static bool IsFreshAt(const std::optional<TimePoint> &last_update, TimePoint now,
                        std::chrono::milliseconds timeout) {
    return last_update.has_value() && now - *last_update <= timeout;
  }

  /**
   * @brief 计算当前距离上次更新时间的毫秒数
   */
  static std::optional<long long> AgeMs(const std::optional<TimePoint> &last_update, TimePoint now) {
    if (!last_update.has_value()) {
      return std::nullopt;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_update).count();
  }

  /**
   * @brief 返回当前己方坐标的来源描述
   */
  std::string AllyPositionSource() const {
    return has_ally_robot_position_ ? "0x0301_0x0200" : "none";
  }

  RefereeTxScheduler &tx_scheduler_;                    ///< `0x0305` 发送调度入口
  radar::log::RefereeMainLogger<revision> main_logger_;  ///< 主协议结构体日志维护器
  radar::log::FileLogStore send_log_store_;            ///< `0x0305` 发送日志输出器
  radar::log::RuntimeMetrics &metrics_;                ///< 缓存的运行指标单例引用，避免热路径反复加全局锁
  Protocol latest_opponent_protocol_{};                ///< 最近一次敌方状态
  rm::device::AllyRobotPosition latest_ally_robot_position_{};  ///< 最近一次收到的 `0x0200`
  std::optional<TimePoint> last_radar0_time_;          ///< 最近一次收到 `0x0A01` 的时间
  std::optional<long long> last_radar0_interval_ms_;   ///< 最近两次收到 `0x0A01` 的间隔
  std::optional<TimePoint> last_ally_robot_position_time_;  ///< 最近一次收到 `0x0301/0x0200` 的时间
  std::optional<TimePoint> last_skip_log_time_;        ///< 上次输出 skip 日志的时间
  std::optional<TimePoint> last_opponent_stale_log_radar0_time_;  ///< 上次记录 stale 时对应的 `0x0A01`
  std::string last_skip_log_reason_;                   ///< 上次 skip 原因
  bool fresh_radar0_refresh_pending_ = false;          ///< 本轮是否已因 fresh `0x0A01` 刷新 latest-only 帧
  bool has_ally_robot_position_ = false;               ///< 是否至少收到过一次 `0x0301/0x0200`
  std::size_t ally_position_seen_count_ = 0;           ///< 收到 `0x0301/0x0200` 的次数
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP
