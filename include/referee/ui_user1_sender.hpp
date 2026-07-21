#ifndef RADAR_INCLUDE_REFEREE_UI_USER1_SENDER_HPP
#define RADAR_INCLUDE_REFEREE_UI_USER1_SENDER_HPP

/**
 * @file include/referee/ui_user1_sender.hpp
 * @brief UIuser1 的 Linux 主链发送适配
 *
 * 将嵌入式 UIuser1 中的“对方信息波”图元改为项目侧状态和统一
 * `0x0301` 调度器发送。目标为己方英雄、3/4 号步兵和空中机器人选手端，
 * 刻意排除工程和 5 号步兵选手端。
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "include/config/config.hpp"
#include "include/radar/app/subReferee/protocol_user.hpp"
#include "include/referee/radar_command_sender.hpp"
#include "include/referee/robot_interaction_sender.hpp"

namespace radar::referee {

template <rm::device::RefereeRevision revision>
class UiUser1Sender {
 public:
  using Protocol = rm::device::RefereeProtocol<revision>;
  using Cmd = rm::device::RefereeCmdId<revision>;
  using Operation = rm::device::UIFigure::Operation;
  using Clock = std::chrono::steady_clock;

  static constexpr rm::u8 kPreMatchProgress = 0x03;
  static constexpr rm::u8 kMatchStartedProgress = 0x04;
  static constexpr int kEditIntervalMs = 1000;

  UiUser1Sender(RadarCommandSender &radar_command_sender, RefereeTxScheduler &tx_scheduler,
                std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : radar_command_sender_(radar_command_sender),
        interaction_sender_(tx_scheduler, [this]() { return ResolveSenderId(); }, std::move(log_root)) {}

  /**
   * @brief 处理常规链路帧，仅在 game_progress 由 3 上升到 4 时启动本轮 UI 注册
   */
  void ProcessSerial(rm::u16 cmd_id, const Protocol &protocol) {
    if (cmd_id == Cmd::kGameStatus) {
      const auto current_progress = static_cast<rm::u8>(protocol.game_status.game_progress);
      const bool match_start_edge = last_game_progress_.has_value() &&
                                     *last_game_progress_ == kPreMatchProgress &&
                                     current_progress == kMatchStartedProgress;
      if (match_start_edge) {
        BeginMatchRegistration();
      } else if (current_progress == kPreMatchProgress) {
        match_started_ = false;
      }
      last_game_progress_ = current_progress;
    }
    TryQueueRegistration();
  }

  /**
   * @brief 记录信息波状态，供后续 UI edit 使用
   */
  void ProcessInfoWave(rm::u16 cmd_id, const Protocol &protocol) {
    if (cmd_id != Cmd::kRadar1 && cmd_id != Cmd::kRadar2 && cmd_id != Cmd::kRadar3 && cmd_id != Cmd::kRadar4) {
      return;
    }
    info_protocol_ = &protocol;
    info_wave_seen_ = true;
  }

  /**
   * @brief 周期性刷新已经注册的 UI 数据
   */
  void ProcessPeriodic() {
    TryQueueRegistration();
    if (!match_started_ || !ui_registered_ || !info_wave_seen_ || info_protocol_ == nullptr) {
      return;
    }

    const auto now = Clock::now();
    if (last_edit_time_.has_value() && now - *last_edit_time_ < std::chrono::milliseconds(kEditIntervalMs)) {
      return;
    }

    QueueEditFrames(*info_protocol_);
    last_edit_time_ = now;
  }

  bool match_started() const { return match_started_; }
  bool ui_registered() const { return ui_registered_; }

 private:
  void BeginMatchRegistration() {
    match_started_ = true;
    ui_registration_attempted_ = false;
    ui_registered_ = false;
    info_wave_seen_ = false;
    info_protocol_ = nullptr;
    last_edit_time_.reset();
  }

  static constexpr std::array<rm::u16, 4> TargetClientIds(RadarSide side) {
    // 仅向己方选手端注册：工程（0x0102/0x0166）和 5 号步兵（0x0105/0x0169）不在列表中。
    if (side == RadarSide::kRed) {
      return {0x0101, 0x0103, 0x0104, 0x0106};
    }
    if (side == RadarSide::kBlue) {
      return {0x0165, 0x0167, 0x0168, 0x016a};
    }
    return {};
  }

  rm::u16 ResolveSenderId() const {
    const auto robot_id = radar_command_sender_.current_robot_id();
    if (robot_id == kRedRadarRobotId || robot_id == kBlueRadarRobotId) {
      return robot_id;
    }
    if (radar_command_sender_.ally_side() == RadarSide::kRed) {
      return kRedRadarRobotId;
    }
    if (radar_command_sender_.ally_side() == RadarSide::kBlue) {
      return kBlueRadarRobotId;
    }
    return 0;
  }

  RadarSide CurrentSide() const { return radar_command_sender_.ally_side(); }

  bool IsOpponentRed() const { return CurrentSide() == RadarSide::kBlue; }

  void TryQueueRegistration() {
    if (!match_started_ || ui_registration_attempted_ || CurrentSide() == RadarSide::kUnknown) {
      return;
    }

    ui_registration_attempted_ = true;
    const auto side = CurrentSide();
    const auto targets = TargetClientIds(side);
    const auto header = BuildHeader(Operation::Add);
    const auto hp = BuildHp(Operation::Add, nullptr);
    const auto allowance = BuildAllowance(Operation::Add, nullptr);

    for (const auto target : targets) {
      QueuePayload(header, target, "add_header");
      QueuePayload(hp, target, "add_hp");
      QueuePayload(allowance, target, "add_allowance");
    }
    ui_registered_ = true;
    last_edit_time_ = Clock::now();
  }

  void QueueEditFrames(const Protocol &protocol) {
    const auto targets = TargetClientIds(CurrentSide());
    const auto hp = BuildHp(Operation::Edit, &protocol);
    const auto allowance = BuildAllowance(Operation::Edit, &protocol);
    for (const auto target : targets) {
      QueuePayload(hp, target, "edit_hp");
      QueuePayload(allowance, target, "edit_allowance");
    }
  }

  template <typename Payload>
  void QueuePayload(const Payload &payload, rm::u16 target, const char *decision) {
    RobotInteractionContext context;
    context.name = "ui_user1";
    context.decision = decision;
    interaction_sender_.Send(payload, target, context);
  }

  rm::device::UICharacter BuildHeader(Operation operation) const {
    rm::device::UICharacter header{};
    header.character.fillCharacter("Hed", operation, 0, rm::device::UIFigure::Color::Orange, 6,
                                   IsOpponentRed() ? 55 : 1170, 890, 24, 29);
    if (IsOpponentRed()) {
      std::memcpy(header.data, "SEN7 DRO6 STD4 STD3 ENG2 HRO1", 29);
    } else {
      std::memcpy(header.data, "HRO1 ENG2 STD3 STD4 DRO6 SEN7", 29);
    }
    return header;
  }

  static rm::u8 DefenceBuff(const Protocol &protocol, std::size_t index) {
    if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
      const std::array<rm::u8, 5> values{
          protocol.radar4.enemy_hero_defence_buff, protocol.radar4.enemy_engineer_defence_buff,
          protocol.radar4.enemy_infantry_3_defence_buff, protocol.radar4.enemy_infantry_4_defence_buff,
          protocol.radar4.enemy_sentry_defence_buff};
      return values[index];
    } else {
      const std::array<rm::u8, 5> values{
          protocol.radar4.opponent_hero_defence_buff, protocol.radar4.opponent_engineer_defence_buff,
          protocol.radar4.opponent_infantry_3_defence_buff, protocol.radar4.opponent_infantry_4_defence_buff,
          protocol.radar4.opponent_sentry_defence_buff};
      return values[index];
    }
  }

  rm::device::UIFigure7 BuildHp(Operation operation, const Protocol *protocol) const {
    // UIuser1 的嵌入式版本从 robot_custom_data_3 读取 5 个自定义血量值；
    // Linux 主链没有该自定义字段的语义定义，因此使用信息波 0x0A02/radar1
    // 中同样的对方英雄、工程、3/4 号步兵和哨兵血量。
    // 固定客户端屏幕方向并镜像排列：英雄靠近屏幕中心，向外依次为工程、
    // 3/4 号步兵、空中机器人和哨兵。
    const std::array<rm::u16, 5> x = IsOpponentRed()
                                         ? std::array<rm::u16, 5>{55, 295, 415, 535, 655}
                                         : std::array<rm::u16, 5>{1170, 1290, 1410, 1530, 1770};
    const auto y = static_cast<rm::u16>(850);
    std::array<rm::u16, 5> values{};
    if (operation == Operation::Edit && protocol != nullptr) {
      if (IsOpponentRed()) {
        // 红方位于左侧，图元从左向右对应 SEN7、STD4、STD3、ENG2、HRO1。
        values = {protocol->radar1.opponent_sentry_HP, protocol->radar1.opponent_infantry_4_HP,
                  protocol->radar1.opponent_infantry_3_HP, protocol->radar1.opponent_engineer_HP,
                  protocol->radar1.opponent_hero_HP};
      } else {
        values = {protocol->radar1.opponent_hero_HP, protocol->radar1.opponent_engineer_HP,
                  protocol->radar1.opponent_infantry_3_HP, protocol->radar1.opponent_infantry_4_HP,
                  protocol->radar1.opponent_sentry_HP};
      }
    }

    rm::device::UIFigure7 ui{};
    const auto color = [&](std::size_t index) {
      const auto defence_index = IsOpponentRed() ? 4 - index : index;
      return operation == Operation::Edit && protocol != nullptr && DefenceBuff(*protocol, defence_index) >= 100
                 ? rm::device::UIFigure::Color::Yellow
                 : rm::device::UIFigure::Color::RedBlue;
    };
    ui.figure1.fillIntegrate("HP1", operation, 0, color(0), 4, x[0], y, 20, values[0]);
    ui.figure2.fillIntegrate("HP2", operation, 0, color(1), 4, x[1], y, 20, values[1]);
    ui.figure3.fillIntegrate("HP3", operation, 0, color(2), 4, x[2], y, 20, values[2]);
    ui.figure4.fillIntegrate("HP4", operation, 0, color(3), 4, x[3], y, 20, values[3]);
    ui.figure5.fillIntegrate("HP5", operation, 0, color(4), 4, x[4], y, 20, values[4]);
    ui.figure6.fillIntegrate("sco", operation, 0, rm::device::UIFigure::Color::RedBlue, 4,
                             IsOpponentRed() ? 860 : 988, 900, 18,
                             operation == Operation::Edit && protocol != nullptr ? protocol->radar3.enemy_total_gold_coin
                                                                                   : 0);
    ui.figure7.fillIntegrate("cco", operation, 0, rm::device::UIFigure::Color::White, 4,
                             IsOpponentRed() ? 860 : 988, 865, 24,
                             operation == Operation::Edit && protocol != nullptr ? protocol->radar3.enemy_remaining_gold_coin
                                                                                   : 0);
    return ui;
  }

  static rm::u16 AerialAllowance(const Protocol &protocol) {
    if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
      return protocol.radar2.opponent_aerial_allowance_ammo;
    } else {
      return protocol.radar2.opponent_drone_allowance_ammo;
    }
  }

  rm::device::UIFigure5 BuildAllowance(Operation operation, const Protocol *protocol) const {
    const std::array<rm::u16, 5> x = IsOpponentRed()
                                         ? std::array<rm::u16, 5>{55, 175, 295, 415, 655}
                                         : std::array<rm::u16, 5>{1170, 1410, 1530, 1650, 1770};
    std::array<rm::u16, 5> values{};
    if (operation == Operation::Edit && protocol != nullptr) {
      if (IsOpponentRed()) {
        values = {protocol->radar2.opponent_sentry_allowance_ammo, AerialAllowance(*protocol),
                  protocol->radar2.opponent_infantry_4_allowance_ammo,
                  protocol->radar2.opponent_infantry_3_allowance_ammo,
                  protocol->radar2.opponent_hero_allowance_ammo};
      } else {
        values = {protocol->radar2.opponent_hero_allowance_ammo, protocol->radar2.opponent_infantry_3_allowance_ammo,
                  protocol->radar2.opponent_infantry_4_allowance_ammo, AerialAllowance(*protocol),
                  protocol->radar2.opponent_sentry_allowance_ammo};
      }
    }

    rm::device::UIFigure5 ui{};
    ui.figure1.fillIntegrate("AL1", operation, 0, rm::device::UIFigure::Color::White, 4, x[0], 810, 16, values[0]);
    ui.figure2.fillIntegrate("AL2", operation, 0, rm::device::UIFigure::Color::White, 4, x[1], 810, 16, values[1]);
    ui.figure3.fillIntegrate("AL3", operation, 0, rm::device::UIFigure::Color::White, 4, x[2], 810, 16, values[2]);
    ui.figure4.fillIntegrate("AL4", operation, 0, rm::device::UIFigure::Color::White, 4, x[3], 810, 16, values[3]);
    ui.figure5.fillIntegrate("AL5", operation, 0, rm::device::UIFigure::Color::White, 4, x[4], 810, 16, values[4]);
    return ui;
  }

  RadarCommandSender &radar_command_sender_;
  // UI 与 RadarCommandSender 的 0x0121 共用 RefereeTxScheduler 的 34ms FIFO 发送窗口，
  // 因而所有 0x0301 子命令合计不会超过约 29.4Hz（低于 30Hz）。
  RobotInteractionSender interaction_sender_;
  const Protocol *info_protocol_ = nullptr;
  bool match_started_ = false;
  bool ui_registration_attempted_ = false;
  bool ui_registered_ = false;
  bool info_wave_seen_ = false;
  std::optional<rm::u8> last_game_progress_;
  std::optional<Clock::time_point> last_edit_time_;
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_UI_USER1_SENDER_HPP
