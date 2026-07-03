#ifndef RADAR_INCLUDE_REFEREE_RADAR_COMMAND_SENDER_HPP
#define RADAR_INCLUDE_REFEREE_RADAR_COMMAND_SENDER_HPP

/**
 * @file  include/referee/radar_command_sender.hpp
 * @brief `0x0121 RadarCMD` 组包、排队与日志维护
 */

#include <array>
#include <chrono>
#include <deque>
#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <functional>
#include <utility>
#include <vector>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/radar/app/subReferee/referee_user.hpp"
#include "include/referee/referee_tx_scheduler.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/device/referee/referee.hpp"

#ifndef RADAR_DEFAULT_LOG_DIR
#define RADAR_DEFAULT_LOG_DIR "test/logs"
#endif

namespace rm::device {

/**
 * @brief 为 `RadarCMD` 指定子命令码映射
 */
template <>
struct TypeToCmd<RadarCMD> {
  static constexpr u16 value = RefereeSubCmdId::kRadarCMD;
};

}  // namespace rm::device

namespace radar::referee {

/// `radar_cmd` 初始值。
constexpr rm::u8 kRadarInitialCommandCounter = 0;
/// 不携带密钥操作时的 `password_cmd`。
constexpr rm::u8 kRadarNoPasswordCommand = 0;
/// 修改己方密钥时的 `password_cmd`。
constexpr rm::u8 kRadarUpdateAllyKeyCommand = 1;
/// 提交敌方密钥时的 `password_cmd`。
constexpr rm::u8 kRadarVerifyOpponentKeyCommand = 2;
/// 默认雷达发送方 ID。
constexpr rm::u16 kDefaultRadarSenderId = 9;  // 9：红方雷达  109：蓝方雷达
/// 裁判系统服务器接收 ID。
constexpr rm::u16 kRefereeServerReceiverId = 0x8080;
/// `password_cmd=2` 冷却时间。
constexpr int kRadarPasswordVerifyCooldownMs = 10000;

/**
 * @brief 一次 `0x0121` 指令的上下文信息
 * @note  用于日志记录本次指令来自哪条业务分支。
 */
struct RadarCommandContext {
  std::string name = "radar_cmd";               ///< 日志名称
  std::string source = "decision";              ///< 业务来源
  int source_port = 0;                          ///< 若来自 TCP，则记录来源端口
  std::string decision = "none";               ///< 决策标签
  rm::u8 radar_info = 0;                        ///< 触发本次决策的原始 radar_info
  bool has_radar_info = false;                  ///< 是否携带 radar_info 上下文
  rm::u8 double_debuff_chances = 0;             ///< 剩余双倍易伤次数
  bool opponent_double_debuff_active = false;   ///< 敌方当前是否正处于双倍易伤
  rm::u8 ally_encryption_level = 0;             ///< 己方当前加密等级
  bool can_modify_ally_key = false;             ///< 当前是否允许修改己方密钥
  int preset_key_index = -1;                    ///< 使用的是第几组预置己方密钥
};

/**
 * @brief 进入 `password_cmd=2` FIFO 的待验证敌方密钥
 */
struct PendingOpponentKey {
  std::string name;               ///< 日志名称
  int source_port = 0;            ///< 来源端口
  std::array<rm::u8, 6> key{};    ///< 敌方密钥内容
};

/**
 * @brief 统一封装 `RadarCMD` 的组包、冷却与日志
 */
class RadarCommandSender {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  /**
   * @brief 创建 `RadarCMD` 发送器
   * @param tx_scheduler 统一发送调度器
   * @param log_root 本轮运行日志根目录
   */
  explicit RadarCommandSender(RefereeTxScheduler &tx_scheduler,
                              std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : tx_scheduler_(tx_scheduler), log_store_(std::move(log_root)) {}

  /**
   * @brief 获取当前 `radar_cmd`
   * @return 当前计数值
   */
  rm::u8 current_radar_cmd() const { return radar_cmd_counter_; }

  /**
   * @brief 将 `radar_cmd` 自增 1
   * @return 自增后的值
   */
  rm::u8 IncrementRadarCommand() {
    if (radar_cmd_counter_ == 0xff) {
      throw std::runtime_error("radar_cmd counter overflow");
    }
    return ++radar_cmd_counter_;
  }

  /**
   * @brief 构造一个默认 `RadarCMD`
   * @return `password_cmd=0` 的基础指令体
   */
  rm::device::RadarCMD MakeCommand() const {
    rm::device::RadarCMD cmd{};
    cmd.radar_cmd = radar_cmd_counter_;
    cmd.password_cmd = kRadarNoPasswordCommand;
    return cmd;
  }

  /**
   * @brief 组包并提交一条 `0x0121` 指令
   * @tparam Protocol 当前串口维护状态类型
   * @param cmd 待发送的 `RadarCMD`
   * @param serial_protocol 当前常规链路状态
   * @param context 本次发送上下文
   * @param on_sent 真正发出后的回调
   * @return 是否成功进入发送调度器
   */
  template <typename Protocol>
  bool Send(rm::device::RadarCMD cmd, const Protocol &serial_protocol, const RadarCommandContext &context,
            std::function<void()> on_sent = {}) {
    static_assert(rm::device::kRefProtocolAllMetadataLen + 6 + sizeof(rm::device::RadarCMD) <=
                      rm::device::kRefProtocolFrameMaxLen,
                  "RadarCMD interaction frame exceeds referee protocol frame buffer");
    std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> tx_buffer{};
    const rm::u16 sender_id = ResolveSenderId(serial_protocol);
    const rm::u16 receiver_id = kRefereeServerReceiverId;
    const rm::u8 frame_len = rm::device::Referee0x301Prepare(tx_buffer.data(), 0, cmd, sender_id, receiver_id);
    std::vector<rm::u8> frame(tx_buffer.begin(), tx_buffer.begin() + frame_len);
    const auto accepted = tx_scheduler_.EnqueueRobotInteraction(
        std::move(frame), context.name, context.source_port, context.decision,
        [this, cmd, sender_id, receiver_id, context, on_sent = std::move(on_sent)](const rm::u8 *frame,
                                                                                    std::size_t frame_len) {
          LogCommand(cmd, frame, frame_len, sender_id, receiver_id, context);
          if (on_sent) {
            on_sent();
          }
        });
    return accepted;
  }

  /**
   * @brief 将一组敌方密钥放入 `password_cmd=2` 等待队列
   * @param name 日志名称
   * @param source_port 来源端口
   * @param key 密钥内容
   */
  void QueueOpponentKey(std::string name, int source_port, const std::array<rm::u8, 6> &key) {
    PendingOpponentKey pending;
    pending.name = std::move(name);
    pending.source_port = source_port;
    pending.key = key;
    pending_opponent_keys_.push_back(pending);
    LogQueuedKey(pending);
  }

  /**
   * @brief 推进敌方密钥待发送队列
   * @tparam Protocol 当前串口维护状态类型
   * @param serial_protocol 当前常规链路状态
   */
  template <typename Protocol>
  void ProcessPending(const Protocol &serial_protocol) {
    if (password_verify_in_flight_ || pending_opponent_keys_.empty()) {
      return;
    }

    const auto now = Clock::now();
    if (password_verify_next_allowed_time_.has_value() && now < *password_verify_next_allowed_time_) {
      LogWaitingCooldown(pending_opponent_keys_.front(), now);
      return;
    }

    const auto pending = pending_opponent_keys_.front();

    auto cmd = MakeCommand();
    cmd.password_cmd = kRadarVerifyOpponentKeyCommand;
    cmd.password_1 = pending.key[0];
    cmd.password_2 = pending.key[1];
    cmd.password_3 = pending.key[2];
    cmd.password_4 = pending.key[3];
    cmd.password_5 = pending.key[4];
    cmd.password_6 = pending.key[5];

    RadarCommandContext context;
    context.name = pending.name;
    context.source = "tcp";
    context.source_port = pending.source_port;
    context.decision = "sent";
    const bool queued = Send(cmd, serial_protocol, context, [this, source_port = pending.source_port]() {
      MarkOpponentKeySent(source_port);
      password_verify_in_flight_ = false;
      password_verify_next_allowed_time_ = Clock::now() + PasswordVerifyCooldown();
    });
    if (!queued) {
      return;
    }
    password_verify_in_flight_ = true;
    pending_opponent_keys_.pop_front();
  }

  /**
   * @brief 判断当前是否仍有待处理敌方密钥
   * @return 队列非空或仍有在途验证时返回 true
   */
  bool HasPending() const { return password_verify_in_flight_ || !pending_opponent_keys_.empty(); }

  /**
   * @brief 判断当前是否有待发送敌方密钥
   * @return 队列非空时返回 true
   */
  bool HasQueuedPending() const { return !pending_opponent_keys_.empty(); }

  /**
   * @brief 判断指定来源端口的敌方密钥是否已经真正发出
   * @param source_port 来源端口
   * @return 已成功写串口发送后返回 true
   */
  bool HasSentOpponentKeyFromPort(int source_port) const {
    if (source_port == radar::config::kEnemyLevel1KeyTcpServerPort) {
      return enemy_level1_key_sent_;
    }
    if (source_port == radar::config::kEnemyLevel2KeyTcpServerPort) {
      return enemy_level2_key_sent_;
    }
    return false;
  }

  /**
   * @brief 记录一条被拒绝的敌方密钥
   * @param name 日志名称
   * @param source_port 来源端口
   * @param key 密钥字节流
   * @param key_len 密钥长度
   * @param reason 拒绝原因
   */
  void LogRejectedKey(const std::string &name, int source_port, const rm::u8 *key, std::size_t key_len,
                      const std::string &reason) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << name << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << source_port << ','
          << "\"decision\":\"rejected\","
          << "\"reason\":\"" << reason << "\","
          << "\"key_len\":" << key_len << ','
          << "\"key_hex\":\"" << radar::log::HexBytes(key, key_len) << "\"}";

    const auto basename = std::string("0x0121_") + name + "_rejected";
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

 private:
  /**
   * @brief 解析当前发送方 ID
   * @tparam Protocol 当前串口维护状态类型
   * @param serial_protocol 当前常规链路状态
   * @return 若 `robot_status.robot_id` 有效则优先使用，否则回退配置项
   */
  template <typename Protocol>
  rm::u16 ResolveSenderId(const Protocol &serial_protocol) const {
    if (serial_protocol.robot_status.robot_id != 0) {
      return serial_protocol.robot_status.robot_id;
    }
    return kDefaultRadarSenderId;
  }

  /**
   * @brief 记录一条真正发出的 `0x0121` 日志
   * @param cmd 指令结构体
   * @param frame 完整主协议帧
   * @param frame_len 帧长度
   * @param sender_id 发送方 ID
   * @param receiver_id 接收方 ID
   * @param context 本次发送上下文
   */
  void LogCommand(const rm::device::RadarCMD &cmd, const rm::u8 *frame, std::size_t frame_len,
                  rm::u16 sender_id, rm::u16 receiver_id, const RadarCommandContext &context) {
    const std::array<rm::u8, 6> password_bytes{
        cmd.password_1, cmd.password_2, cmd.password_3, cmd.password_4, cmd.password_5, cmd.password_6};
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << context.name << "\","
          << "\"source\":\"" << context.source << "\","
          << "\"source_port\":" << context.source_port << ','
          << "\"decision\":\"" << context.decision << "\","
          << "\"cmd_id\":\"" << radar::log::HexU16(0x0301) << "\","
          << "\"sub_cmd_id\":\"" << radar::log::HexU16(rm::device::getCmd(cmd)) << "\","
          << "\"sender_id\":" << sender_id << ','
          << "\"receiver_id\":" << receiver_id << ','
          << "\"frame_hex\":\"" << radar::log::HexBytes(frame, frame_len) << "\","
          << "\"context\":{"
          << "\"has_radar_info\":" << (context.has_radar_info ? "true" : "false") << ','
          << "\"radar_info\":" << radar::log::JsonScalar(context.radar_info) << ','
          << "\"double_debuff_chances\":" << radar::log::JsonScalar(context.double_debuff_chances) << ','
          << "\"opponent_double_debuff_active\":"
          << (context.opponent_double_debuff_active ? "true" : "false") << ','
          << "\"ally_encryption_level\":" << radar::log::JsonScalar(context.ally_encryption_level) << ','
          << "\"can_modify_ally_key\":" << (context.can_modify_ally_key ? "true" : "false") << ','
          << "\"preset_key_index\":" << context.preset_key_index << "},"
          << "\"state\":{"
          << "\"radar_cmd\":" << radar::log::JsonScalar(cmd.radar_cmd) << ','
          << "\"password_cmd\":" << radar::log::JsonScalar(cmd.password_cmd) << ','
          << "\"password_hex\":\"" << radar::log::HexBytes(password_bytes.data(), password_bytes.size()) << "\","
          << "\"password_1\":\"" << radar::log::HexU8(cmd.password_1) << "\","
          << "\"password_2\":\"" << radar::log::HexU8(cmd.password_2) << "\","
          << "\"password_3\":\"" << radar::log::HexU8(cmd.password_3) << "\","
          << "\"password_4\":\"" << radar::log::HexU8(cmd.password_4) << "\","
          << "\"password_5\":\"" << radar::log::HexU8(cmd.password_5) << "\","
          << "\"password_6\":\"" << radar::log::HexU8(cmd.password_6) << "\"}}";

    const auto basename = std::string("0x0121_") + context.name;
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一条进入验证队列的敌方密钥
   * @param pending 待验证敌方密钥
   */
  void LogQueuedKey(const PendingOpponentKey &pending) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << pending.name << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << pending.source_port << ','
          << "\"decision\":\"queued\","
          << "\"queue_size\":" << pending_opponent_keys_.size() << ','
          << "\"key_hex\":\"" << radar::log::HexBytes(pending.key.data(), pending.key.size()) << "\"}";

    const auto basename = std::string("0x0121_") + pending.name + "_queued";
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录当前仍处于 `password_cmd=2` 冷却中的等待状态
   * @param pending 队首待发送密钥
   * @param now 当前时间
   */
  void LogWaitingCooldown(const PendingOpponentKey &pending, TimePoint now) {
    if (last_waiting_cooldown_log_time_.has_value() &&
        now - *last_waiting_cooldown_log_time_ < WaitingCooldownLogMinInterval()) {
      return;
    }

    long long remaining_ms = 0;
    if (password_verify_next_allowed_time_.has_value()) {
      remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         *password_verify_next_allowed_time_ - now)
                         .count();
    }
    if (remaining_ms < 0) {
      remaining_ms = 0;
    }

    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << pending.name << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << pending.source_port << ','
          << "\"decision\":\"waiting_cooldown\","
          << "\"queue_size\":" << pending_opponent_keys_.size() << ','
          << "\"remaining_ms\":" << remaining_ms << ','
          << "\"key_hex\":\"" << radar::log::HexBytes(pending.key.data(), pending.key.size()) << "\"}";

    const auto basename = std::string("0x0121_") + pending.name + "_waiting_cooldown";
    radar::log::GetRuntimeMetrics(log_store_.root()).RecordRadarCommandWaitingCooldown();
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
    last_waiting_cooldown_log_time_ = now;
  }

  /**
   * @brief 返回 `password_cmd=2` 冷却时间
   * @return 冷却时间
   */
  static std::chrono::milliseconds PasswordVerifyCooldown() {
    return std::chrono::milliseconds(kRadarPasswordVerifyCooldownMs);
  }

  /**
   * @brief 限制等待中日志的最小输出间隔
   * @return 日志节流周期
   */
  static std::chrono::milliseconds WaitingCooldownLogMinInterval() {
    return std::chrono::milliseconds(1000);
  }

  void MarkOpponentKeySent(int source_port) {
    if (source_port == radar::config::kEnemyLevel1KeyTcpServerPort) {
      enemy_level1_key_sent_ = true;
      return;
    }
    if (source_port == radar::config::kEnemyLevel2KeyTcpServerPort) {
      enemy_level2_key_sent_ = true;
    }
  }

  RefereeTxScheduler &tx_scheduler_;                   ///< 统一发送调度器
  radar::log::FileLogStore log_store_;                ///< `0x0121` 发送日志输出器
  std::deque<PendingOpponentKey> pending_opponent_keys_;  ///< `password_cmd=2` FIFO 队列
  std::optional<TimePoint> password_verify_next_allowed_time_;  ///< 下一次允许验证密钥的时间
  std::optional<TimePoint> last_waiting_cooldown_log_time_;     ///< 上次输出等待中日志的时间
  bool password_verify_in_flight_ = false;            ///< 当前是否已有一条验证指令等待真正发出
  bool enemy_level1_key_sent_ = false;                ///< `8002` 密钥是否已真正发出
  bool enemy_level2_key_sent_ = false;                ///< `8003` 密钥是否已真正发出
  rm::u8 radar_cmd_counter_ = kRadarInitialCommandCounter;  ///< 当前 `radar_cmd`
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_RADAR_COMMAND_SENDER_HPP
