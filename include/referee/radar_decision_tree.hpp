#ifndef RADAR_INCLUDE_REFEREE_RADAR_DECISION_TREE_HPP
#define RADAR_INCLUDE_REFEREE_RADAR_DECISION_TREE_HPP

/**
 * @file  include/referee/radar_decision_tree.hpp
 * @brief 基于 `0x020E` 雷达自主决策信息的指令决策树
 */

#include <array>
#include <cstddef>
#include <string>

#include "include/config/config.hpp"
#include "include/referee/radar_command_sender.hpp"
#include "librm/device/referee/referee.hpp"

namespace radar::referee {

/**
 * @brief `0x020E` 雷达自主决策信息的解码结果
 */
struct RadarInfoDecisionState {
  rm::u8 raw = 0;                          ///< 原始 bitfield
  rm::u8 double_debuff_chances = 0;        ///< bit0-1：剩余双倍易伤机会
  bool opponent_double_debuff_active = false;  ///< bit2：敌方是否正处于双倍易伤
  rm::u8 ally_encryption_level = 0;        ///< bit3-4：己方当前加密等级
  bool can_modify_ally_key = false;        ///< bit5：当前是否允许修改己方密钥
};

/**
 * @brief 从原始 bitfield 中提取雷达决策状态
 * @param radar_info 协议中的 `radar_info`
 * @return 拆解后的业务语义字段
 */
inline RadarInfoDecisionState DecodeRadarInfo(rm::u8 radar_info) {
  RadarInfoDecisionState state{};
  state.raw = radar_info;
  state.double_debuff_chances = radar_info & 0x03;
  state.opponent_double_debuff_active = (radar_info & (1U << 2U)) != 0;
  state.ally_encryption_level = (radar_info >> 3U) & 0x03;
  state.can_modify_ally_key = (radar_info & (1U << 5U)) != 0;
  return state;
}

template <rm::device::RefereeRevision revision>
class RadarDecisionTree {
 public:
  using Referee = rm::device::Referee<revision>;
  using Protocol = rm::device::RefereeProtocol<revision>;
  using Cmd = rm::device::RefereeCmdId<revision>;

  /**
   * @brief 创建雷达自主决策树
   * @param command_sender 负责真正组包并进入发送调度的发送器
   */
  explicit RadarDecisionTree(RadarCommandSender &command_sender) : command_sender_(command_sender) {}

  /**
   * @brief 处理串口主协议回调
   * @param cmd_id 本次解出的主命令码
   * @param serial_referee 当前常规链路维护状态
   */
  void ProcessSerial(rm::u16 cmd_id, rm::u8 /*seq*/, const Referee &serial_referee) {
    if (cmd_id != Cmd::kRadarInfo) {
      return;
    }

    ProcessRadarInfo(serial_referee.data());
  }

 private:
  /**
   * @brief 根据最新 `radar_info` 生成 `0x0121` 决策
   * @param protocol 当前常规链路状态
   */
  void ProcessRadarInfo(const Protocol &protocol) {
    const auto state = DecodeRadarInfo(protocol.radar_info.radar_info);
    auto cmd = command_sender_.MakeCommand();

    const bool request_double_debuff = ShouldRequestDoubleDebuff(state.double_debuff_chances);
    if (request_double_debuff) {
      cmd.radar_cmd = command_sender_.IncrementRadarCommand();
    }

    const bool update_ally_key = ShouldUpdateAllyKey(state.can_modify_ally_key);
    int preset_key_index = -1;
    if (update_ally_key) {
      preset_key_index = static_cast<int>(ally_key_update_count_);
      FillPresetKey(preset_key_index, &cmd);
      ++ally_key_update_count_;
    }

    if (!request_double_debuff && !update_ally_key) {
      return;
    }

    RadarCommandContext context;
    context.name = "radar_decision_tree";
    context.source = "serial";
    context.decision = DecisionName(request_double_debuff, update_ally_key);
    context.has_radar_info = true;
    context.radar_info = state.raw;
    context.double_debuff_chances = state.double_debuff_chances;
    context.opponent_double_debuff_active = state.opponent_double_debuff_active;
    context.ally_encryption_level = state.ally_encryption_level;
    context.can_modify_ally_key = state.can_modify_ally_key;
    context.preset_key_index = preset_key_index >= 0 ? preset_key_index + 1 : -1;
    command_sender_.Send(cmd, protocol, context);
  }

  /**
   * @brief 判断是否应申请触发双倍易伤
   * @param double_debuff_chances 当前剩余机会
   * @return 是否需要自增 `radar_cmd`
   */
  bool ShouldRequestDoubleDebuff(rm::u8 double_debuff_chances) {
    return double_debuff_chances >= 1;
  }

  /**
   * @brief 判断是否应切换到下一组预置己方密钥
   * @param can_modify_ally_key 当前是否允许修改密钥
   * @return 是否触发一次 `password_cmd=1`
   */
  bool ShouldUpdateAllyKey(bool can_modify_ally_key) {
    const bool rising_edge = can_modify_ally_key && !last_can_modify_ally_key_;
    last_can_modify_ally_key_ = can_modify_ally_key;
    return rising_edge && ally_key_update_count_ < radar::config::kRadarPresetAllyKeys.size();
  }

  /**
   * @brief 将指定下标的预置密钥写入指令体
   * @param preset_key_index 预置密钥下标
   * @param cmd 待写入的指令结构体
   */
  void FillPresetKey(int preset_key_index, rm::device::RadarCMD *cmd) {
    const auto &key = radar::config::kRadarPresetAllyKeys.at(static_cast<std::size_t>(preset_key_index));
    cmd->password_cmd = kRadarUpdateAllyKeyCommand;
    cmd->password_1 = key[0];
    cmd->password_2 = key[1];
    cmd->password_3 = key[2];
    cmd->password_4 = key[3];
    cmd->password_5 = key[4];
    cmd->password_6 = key[5];
  }

  /**
   * @brief 生成人类可读的决策名
   * @param request_double_debuff 是否请求双倍易伤
   * @param update_ally_key 是否更新己方密钥
   * @return 日志中使用的决策标签
   */
  std::string DecisionName(bool request_double_debuff, bool update_ally_key) const {
    if (request_double_debuff && update_ally_key) {
      return "request_double_debuff_and_update_ally_key";
    }
    if (request_double_debuff) {
      return "request_double_debuff";
    }
    return "update_ally_key";
  }

  RadarCommandSender &command_sender_;               ///< 指令发送入口
  bool last_can_modify_ally_key_ = false;           ///< 用于检测密钥修改权限上升沿
  std::size_t ally_key_update_count_ = 0;           ///< 已消耗的预置己方密钥次数
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_RADAR_DECISION_TREE_HPP
