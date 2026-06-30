#ifndef RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP
#define RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP

/**
 * @file  include/referee/map_robot_relay.hpp
 * @brief `0x0A01 -> 0x0305` 状态维护、组包与日志
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
 * @brief 用敌方与己方协议状态组装 `0x0305`
 * @tparam revision 当前裁判协议版本
 * @param opponent_protocol 敌方状态来源，主要读取 `radar0`
 * @param ally_protocol 己方状态来源，主要读取 `ground_robot_position`
 * @return 已完成字段映射的 `MapRobotPosition`
 */
template <rm::device::RefereeRevision revision>
rm::device::MapRobotPosition BuildMapRobotPosition(const rm::device::RefereeProtocol<revision> &opponent_protocol,
                                                   const rm::device::RefereeProtocol<revision> &ally_protocol) {
  static_assert(kSupportedMapRobotRelayRevision<revision>,
                "MapRobotRelay only supports kNewV120 and kNewV200");

  rm::device::MapRobotPosition map{};
  const auto &opponent = opponent_protocol.radar0;
  const auto &ally = ally_protocol.ground_robot_position;

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

  map.ally_hero_position_x = FloatMetersToCentimeters(ally.hero_x);
  map.ally_hero_position_y = FloatMetersToCentimeters(ally.hero_y);
  map.ally_engineer_position_x = FloatMetersToCentimeters(ally.engineer_x);
  map.ally_engineer_position_y = FloatMetersToCentimeters(ally.engineer_y);
  map.ally_infantry_3_position_x = FloatMetersToCentimeters(ally.standard_3_x);
  map.ally_infantry_3_position_y = FloatMetersToCentimeters(ally.standard_3_y);
  map.ally_infantry_4_position_x = FloatMetersToCentimeters(ally.standard_4_x);
  map.ally_infantry_4_position_y = FloatMetersToCentimeters(ally.standard_4_y);
  map.ally_aerial_position_x = 0;
  map.ally_aerial_position_y = 0;
  map.ally_sentry_position_x = FloatMetersToCentimeters(ally.reserved);
  map.ally_sentry_position_y = FloatMetersToCentimeters(ally.reserved_2);

  return map;
}

/**
 * @brief 使用同一协议状态同时构造敌我位置
 * @tparam revision 当前裁判协议版本
 * @param protocol 单一来源协议状态
 * @return 组装后的 `MapRobotPosition`
 */
template <rm::device::RefereeRevision revision>
rm::device::MapRobotPosition BuildMapRobotPosition(const rm::device::RefereeProtocol<revision> &protocol) {
  return BuildMapRobotPosition<revision>(protocol, protocol);
}

/**
 * @brief 生成一次成功发送 `0x0305` 的日志内容
 */
inline std::string BuildMapRobotLogEntry(const rm::device::MapRobotPosition &map, const rm::u8 *frame,
                                         std::size_t frame_len, bool has_ground_robot_position,
                                         std::size_t ground_seen_count, bool ground_available,
                                         const std::string &ground_source,
                                         bool opponent_fresh, bool ground_fresh,
                                         std::optional<long long> last_radar0_interval_ms,
                                         std::optional<long long> last_radar0_age_ms,
                                         std::optional<long long> last_ground_age_ms,
                                         std::optional<long long> tx_queue_delay_ms,
                                         const std::string &send_skipped_reason) {
  const auto payload_offset = rm::device::kRefProtocolHeaderLen + rm::device::kRefProtocolCmdIdLen;
  std::ostringstream oss;
  oss << "{"
      << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
      << "\"seq\":" << static_cast<unsigned>(frame[3]) << ','
      << "\"cmd_id\":\"" << radar::log::HexU16(rm::device::getCmd(map)) << "\","
      << "\"name\":\"map_robot_data\","
      << "\"source_cmd\":\"" << radar::log::HexU16(0x0a01) << "\","
      << "\"has_0x020b\":" << (has_ground_robot_position ? "true" : "false") << ','
      << "\"ground_seen_count\":" << ground_seen_count << ','
      << "\"ground_available\":" << (ground_available ? "true" : "false") << ','
      << "\"ground_source\":\"" << ground_source << "\","
      << "\"opponent_fresh\":" << (opponent_fresh ? "true" : "false") << ','
      << "\"ground_fresh\":" << (ground_fresh ? "true" : "false") << ','
      << "\"last_0x0a01_interval_ms\":";
  if (last_radar0_interval_ms.has_value()) {
    oss << *last_radar0_interval_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0a01_age_ms\":";
  if (last_radar0_age_ms.has_value()) {
    oss << *last_radar0_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x020b_age_ms\":";
  if (last_ground_age_ms.has_value()) {
    oss << *last_ground_age_ms;
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
inline std::string BuildMapRobotSkipLogEntry(bool has_ground_robot_position, bool opponent_fresh,
                                             std::size_t ground_seen_count, bool ground_available,
                                             const std::string &ground_source,
                                             bool ground_fresh,
                                             std::optional<long long> last_radar0_interval_ms,
                                             std::optional<long long> last_radar0_age_ms,
                                             std::optional<long long> last_ground_age_ms,
                                             const std::string &send_skipped_reason) {
  std::ostringstream oss;
  oss << "{"
      << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
      << "\"cmd_id\":\"" << radar::log::HexU16(0x0305) << "\","
      << "\"name\":\"map_robot_data\","
      << "\"source_cmd\":\"" << radar::log::HexU16(0x0a01) << "\","
      << "\"has_0x020b\":" << (has_ground_robot_position ? "true" : "false") << ','
      << "\"ground_seen_count\":" << ground_seen_count << ','
      << "\"ground_available\":" << (ground_available ? "true" : "false") << ','
      << "\"ground_source\":\"" << ground_source << "\","
      << "\"opponent_fresh\":" << (opponent_fresh ? "true" : "false") << ','
      << "\"ground_fresh\":" << (ground_fresh ? "true" : "false") << ','
      << "\"last_0x0a01_interval_ms\":";
  if (last_radar0_interval_ms.has_value()) {
    oss << *last_radar0_interval_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x0a01_age_ms\":";
  if (last_radar0_age_ms.has_value()) {
    oss << *last_radar0_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"last_0x020b_age_ms\":";
  if (last_ground_age_ms.has_value()) {
    oss << *last_ground_age_ms;
  } else {
    oss << "null";
  }
  oss << ",\"send_skipped_reason\":\"" << send_skipped_reason << "\"}";
  return oss.str();
}

/**
 * @brief 维护 `0x0A01` / `0x020B` 状态并驱动 `0x0305` 发送
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
      : tx_scheduler_(tx_scheduler), main_logger_(log_root), send_log_store_(std::move(log_root)) {}

  /**
   * @brief 单源回放入口
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param referee 当前维护状态
   */
  void Process(rm::u16 cmd_id, rm::u8 seq, const Referee &referee) {
    ProcessSingleSource(cmd_id, seq, referee);
  }

  /**
   * @brief 处理常规链路主协议回调
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param serial_referee 当前常规链路状态
   */
  void ProcessSerial(rm::u16 cmd_id, rm::u8 seq, const Referee &serial_referee) {
    main_logger_.LogFrame(cmd_id, seq, serial_referee.data(), serial_referee.loss_rate());
    radar::log::GetRuntimeMetrics(main_logger_.root()).RecordLossRate("serial", serial_referee.loss_rate());

    if (cmd_id == Cmd::kGroundRobotPosition) {
      has_ground_robot_position_ = true;
      ++ground_seen_count_;
      latest_ally_protocol_ = serial_referee.data();
      last_ground_robot_position_time_ = Clock::now();
    }
  }

  /**
   * @brief 处理信息波 TCP 主协议回调
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param info_wave_referee 当前信息波状态
   * @param serial_protocol 当前常规链路状态
   */
  void ProcessInfoWaveTcp(rm::u16 cmd_id, rm::u8 seq, const Referee &info_wave_referee,
                          const Protocol &serial_protocol) {
    main_logger_.LogFrame(cmd_id, seq, info_wave_referee.data(), info_wave_referee.loss_rate());
    radar::log::GetRuntimeMetrics(main_logger_.root()).RecordLossRate("info_wave_tcp", info_wave_referee.loss_rate());

    if (cmd_id == Cmd::kRadar0) {
      if (last_radar0_time_.has_value()) {
        last_radar0_interval_ms_ =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *last_radar0_time_).count();
      }
      latest_opponent_protocol_ = info_wave_referee.data();
      latest_ally_protocol_ = serial_protocol;
      last_radar0_time_ = Clock::now();
    }
  }

  /**
   * @brief 推进一次 `0x0305` 周期处理
   * @note  负责检查状态是否过期、是否限频，以及是否更新 latest-only 待发送帧。
   */
  void ProcessPeriodic() {
    const auto now = Clock::now();
    const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
    const bool ground_fresh = IsFreshAt(last_ground_robot_position_time_, now, RefereeTimeout());
    if (!opponent_fresh && !AllowMissingInterfaces()) {
      tx_scheduler_.ClearLatestMapRobotFrame();
      if (last_radar0_time_.has_value()) {
        LogMapRobotSkip(now, opponent_fresh, ground_fresh, "opponent_stale");
      } else {
        LogMapRobotSkip(now, opponent_fresh, ground_fresh, "opponent_missing");
      }
      return;
    }
    if (!tx_scheduler_.IsMapRobotSendDue(now)) {
      LogMapRobotSkip(now, opponent_fresh, ground_fresh, "rate_limited");
    }

    QueueMapRobotPosition(latest_opponent_protocol_, latest_ally_protocol_, now);
  }

  bool has_ground_robot_position() const { return has_ground_robot_position_; }

 private:
  /**
   * @brief 单源回放模式下的统一处理逻辑
   * @param cmd_id 本次解出的命令码
   * @param seq 当前帧序号
   * @param referee 当前维护状态
   */
  void ProcessSingleSource(rm::u16 cmd_id, rm::u8 seq, const Referee &referee) {
    main_logger_.LogFrame(cmd_id, seq, referee.data(), referee.loss_rate());
    radar::log::GetRuntimeMetrics(main_logger_.root()).RecordLossRate("replay", referee.loss_rate());

    if (cmd_id == Cmd::kGroundRobotPosition) {
      has_ground_robot_position_ = true;
      ++ground_seen_count_;
      latest_ally_protocol_ = referee.data();
      last_ground_robot_position_time_ = Clock::now();
      return;
    }

    if (cmd_id != Cmd::kRadar0) {
      return;
    }

    if (last_radar0_time_.has_value()) {
      last_radar0_interval_ms_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *last_radar0_time_).count();
    }
    latest_opponent_protocol_ = referee.data();
    latest_ally_protocol_ = referee.data();
    last_radar0_time_ = Clock::now();
    ProcessPeriodic();
  }

  /**
   * @brief 使用同一协议状态排队一帧 `0x0305`
   * @param protocol 当前协议状态
   */
  void QueueMapRobotPosition(const Protocol &protocol) {
    QueueMapRobotPosition(protocol, protocol, Clock::now());
  }

  /**
   * @brief 使用敌我两路状态排队一帧 `0x0305`
   * @param opponent_protocol 敌方状态来源
   * @param ally_protocol 己方状态来源
   * @param now 当前时间
   */
  void QueueMapRobotPosition(const Protocol &opponent_protocol, const Protocol &ally_protocol, TimePoint now) {
    static_assert(rm::device::kRefProtocolAllMetadataLen + sizeof(rm::device::MapRobotPosition) <=
                      rm::device::kRefProtocolFrameMaxLen,
                  "MapRobotPosition frame exceeds referee protocol frame buffer");
    Protocol opponent_zero_filled_protocol = opponent_protocol;
    Protocol ground_protocol = ally_protocol;
    const bool opponent_fresh = IsFreshAt(last_radar0_time_, now, InfoWaveTimeout());
    const bool ground_fresh = IsFreshAt(last_ground_robot_position_time_, now, RefereeTimeout());
    if (!opponent_fresh) {
      opponent_zero_filled_protocol.radar0 = {};
    }
    if (!ground_fresh) {
      ground_protocol.ground_robot_position = {};
    }

    const auto map = BuildMapRobotPosition<revision>(opponent_zero_filled_protocol, ground_protocol);
    std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> tx_buffer{};
    const rm::u8 frame_len = rm::device::RefereePrepare(tx_buffer.data(), 0, map);
    std::vector<rm::u8> frame(tx_buffer.begin(), tx_buffer.begin() + frame_len);
    const auto enqueue_time = Clock::now();
    tx_scheduler_.UpdateLatestMapRobotFrame(
        frame, [this, map, frame, opponent_fresh, ground_fresh, enqueue_time]() {
          LogMapRobotData(map, frame.data(), frame.size(), Clock::now(), opponent_fresh, ground_fresh,
                          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - enqueue_time).count(),
                          "");
        });
  }

  /**
   * @brief 记录一条成功发送的 `0x0305` 日志
   */
  void LogMapRobotData(const rm::device::MapRobotPosition &map, const rm::u8 *frame, std::size_t frame_len,
                       TimePoint now, bool opponent_fresh, bool ground_fresh,
                       std::optional<long long> tx_queue_delay_ms,
                       const std::string &send_skipped_reason) {
    const bool ground_available = has_ground_robot_position_ && ground_fresh;
    const auto entry =
        BuildMapRobotLogEntry(map, frame, frame_len, has_ground_robot_position_, ground_seen_count_,
                              ground_available, GroundSource(),
                              opponent_fresh, ground_fresh, last_radar0_interval_ms_,
                              AgeMs(last_radar0_time_, now), AgeMs(last_ground_robot_position_time_, now),
                              tx_queue_delay_ms,
                              send_skipped_reason);
    const std::filesystem::path basename = "0x0305_map_robot_data";
    send_log_store_.Append(std::filesystem::path("main") / (basename.string() + ".log"), entry,
                           radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一条未发送 `0x0305` 的原因日志
   */
  void LogMapRobotSkip(TimePoint now, bool opponent_fresh, bool ground_fresh,
                       const std::string &send_skipped_reason) {
    if (send_skipped_reason == "opponent_stale" &&
        last_opponent_stale_log_radar0_time_ == last_radar0_time_) {
      return;
    }
    if (last_skip_log_time_.has_value() && last_skip_log_reason_ == send_skipped_reason &&
        now - *last_skip_log_time_ < SkipLogMinInterval()) {
      return;
    }

    radar::log::GetRuntimeMetrics(send_log_store_.root()).RecordMapRobotSkip(send_skipped_reason);
    const bool ground_available = has_ground_robot_position_ && ground_fresh;
    const auto entry = BuildMapRobotSkipLogEntry(has_ground_robot_position_, opponent_fresh,
                                                 ground_seen_count_, ground_available, GroundSource(), ground_fresh,
                                                 last_radar0_interval_ms_,
                                                 AgeMs(last_radar0_time_, now),
                                                 AgeMs(last_ground_robot_position_time_, now),
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
    return std::chrono::milliseconds(radar::config::kInfoWaveStateTimeoutMs);
  }

  /**
   * @brief 常规链路状态超时时间
   * @return 对应超时阈值
   */
  static std::chrono::milliseconds RefereeTimeout() {
    return std::chrono::milliseconds(radar::config::kRefereeStateTimeoutMs);
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
   * @brief 返回当前己方地面机器人位置的来源描述
   */
  std::string GroundSource() const {
    return has_ground_robot_position_ ? "serial_0x020b" : "none";
  }

  RefereeTxScheduler &tx_scheduler_;                    ///< `0x0305` 发送调度入口
  radar::log::RefereeMainLogger<revision> main_logger_;  ///< 主协议结构体日志维护器
  radar::log::FileLogStore send_log_store_;            ///< `0x0305` 发送日志输出器
  Protocol latest_opponent_protocol_{};                ///< 最近一次敌方状态
  Protocol latest_ally_protocol_{};                    ///< 最近一次己方状态
  std::optional<TimePoint> last_radar0_time_;          ///< 最近一次收到 `0x0A01` 的时间
  std::optional<long long> last_radar0_interval_ms_;   ///< 最近两次收到 `0x0A01` 的间隔
  std::optional<TimePoint> last_ground_robot_position_time_;  ///< 最近一次收到 `0x020B` 的时间
  std::optional<TimePoint> last_skip_log_time_;        ///< 上次输出 skip 日志的时间
  std::optional<TimePoint> last_opponent_stale_log_radar0_time_;  ///< 上次记录 stale 时对应的 `0x0A01`
  std::string last_skip_log_reason_;                   ///< 上次 skip 原因
  bool has_ground_robot_position_ = false;             ///< 是否至少收到过一次 `0x020B`
  std::size_t ground_seen_count_ = 0;                  ///< 收到 `0x020B` 的次数
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_MAP_ROBOT_RELAY_HPP
