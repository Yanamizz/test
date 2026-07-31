#ifndef RADAR_INCLUDE_REFEREE_DOUBLE_DEBUFF_FALLBACK_HPP
#define RADAR_INCLUDE_REFEREE_DOUBLE_DEBUFF_FALLBACK_HPP

/**
 * @file  include/referee/double_debuff_fallback.hpp
 * @brief 双倍易伤保底机制：比赛后段无视 `0x020E` 机会位强行请求触发
 *
 * 背景：`0x020E` bit0-1 声明的“剩余触发机会”在实测中可能长期为 0，
 * 但实际赛场仍有可能触发。本模块按 `0x0001` 的比赛阶段与剩余时间，
 * 在比赛后段主动把 `0x0121` 的 `radar_cmd` 自增并间歇重发，作为保底。
 *
 * 协议约束：`radar_cmd` 必须单调递增且每次仅能加 1，否则视为不合法。
 * 因此本模块与 `RadarDecisionTree` 共用 `RadarCommandSender` 内的同一计数器，
 * 只在“当前值低于本阶段目标值”时自增一次，其余时间仅重发当前值。
 */

#include <chrono>
#include <cstddef>
#include <optional>
#include <sstream>
#include <string>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/radar_command_sender.hpp"
#include "include/referee/radar_decision_tree.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/device/referee/referee.hpp"

namespace radar::referee {

/// `0x0001` bit4-7 中代表“比赛中”的阶段值。
constexpr rm::u8 kGameProgressInMatch = 4;
/// 保底机制最多把 `radar_cmd` 推进到的目标值（对应至多 2 次触发机会）。
constexpr rm::u8 kDoubleDebuffFallbackMaxTarget = 2;

/**
 * @brief 双倍易伤保底请求器
 * @tparam revision 当前裁判协议版本
 */
template <rm::device::RefereeRevision revision>
class DoubleDebuffFallback {
 public:
  using Protocol = rm::device::RefereeProtocol<revision>;
  using Cmd = rm::device::RefereeCmdId<revision>;
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  /**
   * @brief 创建保底请求器
   * @param command_sender `0x0121` 组包与发送入口
   * @param log_root 本轮运行日志根目录
   */
  explicit DoubleDebuffFallback(RadarCommandSender &command_sender,
                                std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : command_sender_(command_sender), log_store_(std::move(log_root)) {}

  /**
   * @brief 缓存保底判断所需的串口状态
   * @param cmd_id 本次解出的主命令码
   * @param protocol 当前常规链路状态
   * @note 只关心 `0x0001`（阶段与剩余时间）与 `0x020E`（敌方双倍易伤生效位）。
   */
  void ProcessSerial(rm::u16 cmd_id, const Protocol &protocol) {
    if (cmd_id == Cmd::kGameStatus) {
      game_progress_ = protocol.game_status.game_progress;
      stage_remain_time_ = protocol.game_status.stage_remain_time;
      game_status_receive_time_ = Clock::now();
      return;
    }
    if (cmd_id == Cmd::kRadarInfo) {
      ObserveRadarInfo(DecodeRadarInfo(protocol.radar_info.radar_info));
    }
  }

  /**
   * @brief 按当前比赛剩余时间推进保底请求
   * @param protocol 当前常规链路状态
   */
  void ProcessPeriodic(const Protocol &protocol) {
    if (!radar::config::kDoubleDebuffFallbackEnabled) {
      return;
    }
    const auto remain = EffectiveRemainSeconds();
    if (!remain.has_value()) {
      return;
    }

    const rm::u8 target = TargetRadarCmdForRemain(*remain);
    if (target == 0) {
      return;
    }
    EscalateStage(target, *remain);

    if (radar::config::kDoubleDebuffFallbackStopOnConfirm && confirmed_target_ >= target) {
      return;
    }
    MaybeIncrementTowardTarget(target, protocol, *remain);
    MaybeResendCurrent(target, protocol, *remain);
  }

  /// 返回当前保底阶段目标值（0 表示尚未进入保底窗口）。
  rm::u8 current_target() const { return current_target_; }
  /// 返回已被 `0x020E` bit2 上升沿确认生效的目标值。
  rm::u8 confirmed_target() const { return confirmed_target_; }
  /// 返回保底机制累计自增 `radar_cmd` 的次数。
  std::size_t increment_count() const { return increment_count_; }
  /// 返回保底机制累计重发帧数。
  std::size_t resend_count() const { return resend_count_; }

 private:
  /**
   * @brief 记录一次 `0x020E` 采样，检测敌方双倍易伤的上升沿
   * @param state 解码后的雷达决策状态
   * @note 只有在本模块已请求过某个目标值之后出现的上升沿，才算该目标已生效。
   */
  void ObserveRadarInfo(const RadarInfoDecisionState &state) {
    const bool rising_edge = state.opponent_double_debuff_active && !last_opponent_double_debuff_active_;
    last_opponent_double_debuff_active_ = state.opponent_double_debuff_active;
    if (rising_edge && requested_target_ > confirmed_target_) {
      confirmed_target_ = requested_target_;
      LogEvent("confirmed", requested_target_, std::nullopt, state.raw);
    }
  }

  /**
   * @brief 计算当前有效的比赛剩余秒数
   * @return 处于比赛中且 `0x0001` 未过期时返回插值后的剩余秒数，否则返回空
   * @note `0x0001` 为 1Hz，两帧之间用单调时钟插值，避免阈值判断出现整秒抖动。
   */
  std::optional<int> EffectiveRemainSeconds() {
    if (!game_status_receive_time_.has_value() || game_progress_ != kGameProgressInMatch) {
      return std::nullopt;
    }
    const auto age = Clock::now() - *game_status_receive_time_;
    if (age >= std::chrono::milliseconds(radar::config::kGameStatusStaleTimeoutMs)) {
      LogStaleGameStatusOnce();
      return std::nullopt;
    }
    stale_game_status_logged_ = false;
    const auto age_sec = std::chrono::duration_cast<std::chrono::seconds>(age).count();
    const auto remain = static_cast<long long>(stage_remain_time_) - age_sec;
    return remain <= 0 ? 0 : static_cast<int>(remain);
  }

  /**
   * @brief 根据剩余时间选出本阶段应达到的 `radar_cmd` 目标值
   * @param remain_sec 当前比赛剩余秒数
   * @return 剩余 <=180s 返回 2，<=270s 返回 1，其余返回 0
   */
  static rm::u8 TargetRadarCmdForRemain(int remain_sec) {
    if (remain_sec <= radar::config::kDoubleDebuffFallbackStage2RemainSec) {
      return kDoubleDebuffFallbackMaxTarget;
    }
    if (remain_sec <= radar::config::kDoubleDebuffFallbackStage1RemainSec) {
      return 1;
    }
    return 0;
  }

  /**
   * @brief 在阶段目标提升时记录一次日志
   */
  void EscalateStage(rm::u8 target, int remain_sec) {
    if (target <= current_target_) {
      return;
    }
    current_target_ = target;
    LogEvent(target == 1 ? "stage1_armed" : "stage2_armed", target, remain_sec, std::nullopt);
  }

  /**
   * @brief 当 `radar_cmd` 仍低于目标值时自增一次并立即发送
   * @note 自增严格每次加 1，且两次自增之间保持最小间隔。
   */
  void MaybeIncrementTowardTarget(rm::u8 target, const Protocol &protocol, int remain_sec) {
    if (command_sender_.current_radar_cmd() >= target) {
      return;
    }
    const auto now = Clock::now();
    if (last_increment_time_.has_value() &&
        now - *last_increment_time_ <
            std::chrono::milliseconds(radar::config::kDoubleDebuffFallbackMinIncrementIntervalMs)) {
      return;
    }

    auto cmd = command_sender_.MakeCommand();
    cmd.radar_cmd = command_sender_.IncrementRadarCommand();
    if (!SendFallbackFrame(cmd, protocol, "fallback_request", remain_sec)) {
      return;
    }
    last_increment_time_ = now;
    last_resend_time_ = now;
    requested_target_ = cmd.radar_cmd;
    ++increment_count_;
    LogEvent("incremented", cmd.radar_cmd, remain_sec, std::nullopt);
  }

  /**
   * @brief 间歇重发当前 `radar_cmd`，对抗单帧丢失
   * @note 不自增，只把当前计数值再送一遍；受重发间隔与调度器限频双重约束。
   */
  void MaybeResendCurrent(rm::u8 target, const Protocol &protocol, int remain_sec) {
    const auto current = command_sender_.current_radar_cmd();
    if (current == 0 || current > target) {
      return;
    }
    const auto now = Clock::now();
    if (last_resend_time_.has_value() &&
        now - *last_resend_time_ < std::chrono::milliseconds(radar::config::kDoubleDebuffFallbackResendIntervalMs)) {
      return;
    }

    auto cmd = command_sender_.MakeCommand();
    if (!SendFallbackFrame(cmd, protocol, "fallback_resend", remain_sec)) {
      return;
    }
    last_resend_time_ = now;
    ++resend_count_;
  }

  /**
   * @brief 组装保底上下文并交给发送器
   * @return 是否成功进入发送调度
   */
  bool SendFallbackFrame(const rm::device::RadarCMD &cmd, const Protocol &protocol, const char *decision,
                         int remain_sec) {
    RadarCommandContext context;
    context.name = "double_debuff_fallback";
    context.source = "fallback";
    context.decision = decision;
    context.ally_encryption_level = 0;
    last_send_remain_sec_ = remain_sec;
    return command_sender_.Send(cmd, protocol, context);
  }

  /**
   * @brief 记录一次保底机制状态变化
   */
  void LogEvent(const char *event, rm::u8 target, std::optional<int> remain_sec,
                std::optional<rm::u8> radar_info) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"double_debuff_fallback\","
          << "\"event\":\"" << event << "\","
          << "\"target_radar_cmd\":" << static_cast<unsigned>(target) << ','
          << "\"current_radar_cmd\":" << static_cast<unsigned>(command_sender_.current_radar_cmd()) << ','
          << "\"requested_target\":" << static_cast<unsigned>(requested_target_) << ','
          << "\"confirmed_target\":" << static_cast<unsigned>(confirmed_target_) << ','
          << "\"increment_count\":" << increment_count_ << ','
          << "\"resend_count\":" << resend_count_ << ','
          << "\"game_progress\":" << static_cast<unsigned>(game_progress_) << ','
          << "\"stage_remain_time\":" << stage_remain_time_ << ',';
    if (remain_sec.has_value()) {
      entry << "\"effective_remain_sec\":" << *remain_sec << ',';
    }
    if (radar_info.has_value()) {
      entry << "\"radar_info\":\"" << radar::log::HexU8(*radar_info) << "\",";
    }
    entry << "\"last_send_remain_sec\":" << last_send_remain_sec_ << "}";

    log_store_.Append(std::filesystem::path("main") / "0x0121_double_debuff_fallback_state.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief `0x0001` 过期时只记录一次，避免刷日志
   */
  void LogStaleGameStatusOnce() {
    if (stale_game_status_logged_) {
      return;
    }
    stale_game_status_logged_ = true;
    LogEvent("game_status_stale", current_target_, std::nullopt, std::nullopt);
  }

  RadarCommandSender &command_sender_;            ///< `0x0121` 发送入口
  radar::log::FileLogStore log_store_;            ///< 保底机制状态日志
  rm::u8 game_progress_ = 0;                      ///< 最近一次 `0x0001` 比赛阶段
  rm::u16 stage_remain_time_ = 0;                 ///< 最近一次 `0x0001` 阶段剩余秒数
  std::optional<TimePoint> game_status_receive_time_;  ///< 最近一次 `0x0001` 到达时间
  std::optional<TimePoint> last_increment_time_;   ///< 最近一次 `radar_cmd` 自增时间
  std::optional<TimePoint> last_resend_time_;      ///< 最近一次重发时间
  rm::u8 current_target_ = 0;                     ///< 当前阶段目标 `radar_cmd`
  rm::u8 requested_target_ = 0;                   ///< 已实际请求出去的最大 `radar_cmd`
  rm::u8 confirmed_target_ = 0;                   ///< 已被 bit2 上升沿确认生效的目标值
  bool last_opponent_double_debuff_active_ = false;  ///< 上一次 `0x020E` bit2 采样
  bool stale_game_status_logged_ = false;          ///< `0x0001` 过期是否已记录
  int last_send_remain_sec_ = -1;                 ///< 最近一次发送时的剩余秒数
  std::size_t increment_count_ = 0;                ///< 累计自增次数
  std::size_t resend_count_ = 0;                   ///< 累计重发帧数
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_DOUBLE_DEBUFF_FALLBACK_HPP
