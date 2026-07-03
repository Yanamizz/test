#ifndef RADAR_INCLUDE_REFEREE_ENEMY_KEY_RECEIVER_HPP
#define RADAR_INCLUDE_REFEREE_ENEMY_KEY_RECEIVER_HPP

/**
 * @file  include/referee/enemy_key_receiver.hpp
 * @brief 敌方密钥 TCP 接收与 `0x0A06` 解包维护
 */

#include <array>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <utility>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/radar_command_sender.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/device/referee/referee.hpp"

namespace radar::referee {

/**
 * @brief 判断单字节是否为规则允许的 ASCII 字母或数字
 * @param value 待校验字节
 * @return 是否满足 `0x0A06` 密钥字符约束
 */
constexpr bool IsAsciiAlphaNumeric(rm::u8 value) {
  return (value >= '0' && value <= '9') || (value >= 'A' && value <= 'Z') || (value >= 'a' && value <= 'z');
}

/**
 * @brief 判断给定数组是否是合法雷达密钥
 * @tparam size 数组长度
 * @param key 待校验密钥
 * @return 长度为 6 且全部为字母或数字时返回 true
 */
template <std::size_t size>
constexpr bool IsRadarKey(const std::array<rm::u8, size> &key) {
  if constexpr (size != 6) {
    return false;
  }

  for (const auto value : key) {
    if (!IsAsciiAlphaNumeric(value)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 校验预置己方密钥表是否全部合法
 * @tparam key_count 预置密钥数量
 * @param keys 预置密钥数组
 * @return 全部合法时返回 true
 */
template <std::size_t key_count>
constexpr bool AreRadarPresetAllyKeysValid(const std::array<std::array<rm::u8, 6>, key_count> &keys) {
  for (const auto &key : keys) {
    if (!IsRadarKey(key)) {
      return false;
    }
  }
  return true;
}

static_assert(AreRadarPresetAllyKeysValid(radar::config::kRadarPresetAllyKeys),
              "radar preset ally keys must be 6 ASCII letters/digits");

template <rm::device::RefereeRevision revision>
class EnemyKeyReceiver {
 public:
  using Referee = rm::device::Referee<revision>;
  using Cmd = rm::device::RefereeCmdId<revision>;

  static constexpr std::size_t kKeyBytes = 6;

  /**
   * @brief 创建一个敌方密钥接收器
   * @param name 当前端口对应的日志名称
   * @param source_port 当前 TCP 端口
   * @param command_sender 解包成功后用于提交 `0x0121` 的发送器
   * @param log_root 本轮运行日志根目录
   */
  EnemyKeyReceiver(std::string name, int source_port, RadarCommandSender &command_sender,
                   std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : name_(std::move(name)),
        source_port_(source_port),
        command_sender_(command_sender),
        log_store_(std::move(log_root)) {
    referee_.AttachCallback([this](rm::u16 cmd_id, rm::u8 seq) { OnFrame(cmd_id, seq); });
  }

  /**
   * @brief 向 `0x0A06` 解包器喂入 TCP 字节流
   * @tparam SerialProtocol 常规链路协议类型，当前仅用于接口兼容
   * @param data 本次收到的字节流
   * @param size 字节数
   * @return 当端口完成一次合法接收后返回 true
   */
  template <typename SerialProtocol>
  bool ProcessBytes(const rm::u8 *data, std::size_t size, const SerialProtocol &) {
    if (completed_) {
      return true;
    }

    for (std::size_t i = 0; i < size; ++i) {
      referee_ << data[i];
    }
    return completed_;
  }

  bool has_key() const { return has_key_; }
  bool completed() const { return completed_; }
  bool rejected() const { return rejected_; }
  const std::array<rm::u8, kKeyBytes> &latest_key() const { return latest_key_; }
  std::size_t update_count() const { return update_count_; }
  std::size_t valid_frame_count() const { return valid_frame_count_; }

  /**
   * @brief 推进一次“已接收但尚未满足发送门控”的密钥提交
   */
  void ProcessDeferredQueueing() {
    if (!has_key_ || queued_to_sender_ || !RequiresLevel1Gate()) {
      return;
    }
    if (!command_sender_.HasSentOpponentKeyFromPort(radar::config::kEnemyLevel1KeyTcpServerPort)) {
      return;
    }
    QueueAcceptedKey();
  }

 private:
  /**
   * @brief 处理一次完整主协议帧
   * @param cmd_id 主命令码
   * @param seq 本次帧序号
   */
  void OnFrame(rm::u16 cmd_id, rm::u8 seq) {
    ++valid_frame_count_;
    radar::log::GetRuntimeMetrics(log_store_.root()).RecordLossRate(name_, referee_.loss_rate());

    if (cmd_id != Cmd::kRadar5) {
      LogUnexpectedCmd(cmd_id, seq);
      return;
    }

    for (std::size_t i = 0; i < kKeyBytes; ++i) {
      latest_key_[i] = referee_.data().radar5.key[i];
    }
    ++update_count_;

    if (!IsRadarKey(latest_key_)) {
      rejected_ = true;
      command_sender_.LogRejectedKey(name_, source_port_, latest_key_.data(), latest_key_.size(),
                                     "0x0a06_key_must_be_6_ascii_letters_or_digits");
      LogRejectedFrame(seq);
      return;
    }

    has_key_ = true;
    completed_ = true;
    QueueAcceptedKey();
    LogAcceptedFrame(seq);
  }

  bool RequiresLevel1Gate() const { return source_port_ == radar::config::kEnemyLevel2KeyTcpServerPort; }

  void QueueAcceptedKey() {
    if (queued_to_sender_) {
      return;
    }
    if (RequiresLevel1Gate() &&
        !command_sender_.HasSentOpponentKeyFromPort(radar::config::kEnemyLevel1KeyTcpServerPort)) {
      return;
    }
    command_sender_.QueueOpponentKey(name_, source_port_, latest_key_);
    queued_to_sender_ = true;
  }

  /**
   * @brief 记录一次合法 `0x0A06` 接收
   * @param seq 本次帧序号
   */
  void LogAcceptedFrame(rm::u8 seq) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << name_ << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << source_port_ << ','
          << "\"cmd_id\":\"" << radar::log::HexU16(Cmd::kRadar5) << "\","
          << "\"seq\":" << static_cast<unsigned>(seq) << ','
          << "\"decision\":\"accepted\","
          << "\"key_hex\":\"" << radar::log::HexBytes(latest_key_.data(), latest_key_.size()) << "\"}";

    const auto basename = std::string("0x0a06_") + name_;
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一次格式合法但密钥内容非法的接收
   * @param seq 本次帧序号
   */
  void LogRejectedFrame(rm::u8 seq) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << name_ << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << source_port_ << ','
          << "\"cmd_id\":\"" << radar::log::HexU16(Cmd::kRadar5) << "\","
          << "\"seq\":" << static_cast<unsigned>(seq) << ','
          << "\"decision\":\"rejected\","
          << "\"reason\":\"key_must_be_6_ascii_letters_or_digits\","
          << "\"key_hex\":\"" << radar::log::HexBytes(latest_key_.data(), latest_key_.size()) << "\"}";

    const auto basename = std::string("0x0a06_") + name_ + "_rejected";
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一次非 `0x0A06` 的意外命令帧
   * @param cmd_id 主命令码
   * @param seq 本次帧序号
   */
  void LogUnexpectedCmd(rm::u16 cmd_id, rm::u8 seq) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << name_ << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << source_port_ << ','
          << "\"seq\":" << static_cast<unsigned>(seq) << ','
          << "\"decision\":\"unexpected_cmd\","
          << "\"cmd_id\":\"" << radar::log::HexU16(cmd_id) << "\"}";

    const auto basename = std::string("0x0a06_") + name_ + "_unexpected_cmd";
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kBestEffort);
  }

  std::string name_;                        ///< 当前接收器的日志标签
  int source_port_ = 0;                    ///< 当前接收器所属 TCP 端口
  RadarCommandSender &command_sender_;     ///< 密钥验证提交入口
  radar::log::FileLogStore log_store_;     ///< 结构化日志输出器
  Referee referee_;                        ///< 独立的 `0x0A06` 主协议解包器
  std::array<rm::u8, kKeyBytes> latest_key_{};  ///< 最近一次解出的密钥内容
  bool has_key_ = false;                   ///< 是否收到过合法密钥
  bool completed_ = false;                 ///< 当前端口是否已完成一次有效接收
  bool rejected_ = false;                  ///< 是否收到过非法密钥
  bool queued_to_sender_ = false;          ///< 当前密钥是否已进入 `0x0121` 待发送队列
  std::size_t update_count_ = 0;           ///< 已成功解出的 `0x0A06` 次数
  std::size_t valid_frame_count_ = 0;      ///< 所有 CRC 正确主协议帧计数
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_ENEMY_KEY_RECEIVER_HPP
