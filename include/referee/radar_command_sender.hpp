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
/// 红方雷达机器人 ID。
constexpr rm::u16 kRedRadarRobotId = 9;
/// 蓝方雷达机器人 ID。
constexpr rm::u16 kBlueRadarRobotId = 109;
/// 裁判系统服务器接收 ID。
constexpr rm::u16 kRefereeServerReceiverId = 0x8080;
/// `password_cmd=2` 冷却时间。
constexpr int kRadarPasswordVerifyCooldownMs = 10000;
/// `0x020E` bit3-4 的最低有效等级，开局即为 1。
constexpr rm::u8 kRadarMinInterferenceLevel = 1;
/// `0x020E` bit3-4 的最高等级，到达后无需再提交敌方密钥。
constexpr rm::u8 kRadarMaxInterferenceLevel = 3;
/// `0x0001` bit4-7 中代表“比赛中”的阶段值。
constexpr rm::u8 kGameProgressInMatch = 4;
/// 单个端口最多保留的历史密钥数量，超出后丢弃栈底最旧的一条。
constexpr std::size_t kMaxOpponentKeyStackDepth = 16;

/**
 * @brief 按当前干扰波等级选出应当提交密钥的来源端口
 * @param interference_level `0x020E` bit3-4 解出的等级
 * @return 对应 TCP 端口；等级未知或已满级时返回 0
 * @note 一级用 `8002` 收到的密钥，二级用 `8003` 收到的密钥。
 */
constexpr int OpponentKeySourcePortForLevel(rm::u8 interference_level) {
  if (interference_level == 1) {
    return radar::config::kEnemyLevel1KeyTcpServerPort;
  }
  if (interference_level == 2) {
    return radar::config::kEnemyLevel2KeyTcpServerPort;
  }
  return 0;
}

/**
 * @brief 当前己方阵营
 */
enum class RadarSide {
  kUnknown,
  kRed,
  kBlue,
};

/**
 * @brief 根据 `0x0201.robot_id` 识别己方阵营
 */
constexpr RadarSide DetectRadarSide(rm::u16 robot_id) {
  if (robot_id == kRedRadarRobotId) {
    return RadarSide::kRed;
  }
  if (robot_id == kBlueRadarRobotId) {
    return RadarSide::kBlue;
  }
  return RadarSide::kUnknown;
}

/**
 * @brief 返回阵营名称，供日志和状态展示使用
 */
constexpr const char *RadarSideName(RadarSide side) {
  switch (side) {
    case RadarSide::kRed:
      return "red";
    case RadarSide::kBlue:
      return "blue";
    default:
      return "unknown";
  }
}

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
 * @brief 进入 `password_cmd=2` 待验证栈的敌方密钥
 * @note 同一端口解出不同密钥时按后进先出取用，最新密钥优先提交。
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
   * @brief 使用 `0x0201.robot_id` 更新当前己方阵营
   * @param robot_id 当前机器人 ID
   */
  void UpdateAllySideFromRobotId(rm::u16 robot_id) {
    current_robot_id_ = robot_id;
    current_side_ = DetectRadarSide(robot_id);
  }

  /// 返回当前己方阵营。
  RadarSide ally_side() const { return current_side_; }

  /// 返回最近一次观测到的机器人 ID。
  rm::u16 current_robot_id() const { return current_robot_id_; }

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
   * @brief 将一组敌方密钥压入对应端口的 `password_cmd=2` 待验证栈
   * @param name 日志名称
   * @param source_port 来源端口
   * @param key 密钥内容
   * @note  栈内已存在的密钥不重复入栈；超出容量上限时丢弃最旧的一条。
   *        由于重投机制会轮换栈内顺序，这里必须全栈查重而不是只比栈顶。
   */
  void QueueOpponentKey(std::string name, int source_port, const std::array<rm::u8, 6> &key) {
    auto *stack = MutableStackForPort(source_port);
    if (stack == nullptr) {
      return;
    }
    for (const auto &existing : *stack) {
      if (existing.key == key) {
        return;
      }
    }

    PendingOpponentKey pending;
    pending.name = std::move(name);
    pending.source_port = source_port;
    pending.key = key;
    stack->push_back(pending);
    while (stack->size() > kMaxOpponentKeyStackDepth) {
      stack->pop_front();
    }
    LogQueuedKey(pending, stack->size());
  }

  /**
   * @brief 按当前干扰波等级推进敌方密钥验证
   * @tparam Protocol 当前串口维护状态类型
   * @param serial_protocol 当前常规链路状态
   * @note  等级为 1 时只取 `8002` 栈，等级为 2 时只取 `8003` 栈；
   *        同一栈内后进先出，等级未知或已满级时不发送。
   *        密钥发出后不立即出栈：只有 `0x020E` 干扰波等级真正上升才算验证通过
   *        （由 `UpdateAllyEncryptionLevel` 清栈）。等级没升就按冷却周期重投，
   *        栈内有多个候选时轮换发送。
   */
  template <typename Protocol>
  void ProcessPending(const Protocol &serial_protocol) {
    if (password_verify_in_flight_) {
      return;
    }
    if (!IsMatchRunning(serial_protocol)) {
      LogHoldingBeforeMatch(serial_protocol);
      return;
    }

    const int target_port = OpponentKeyPortForCurrentLevel();
    if (target_port == 0) {
      LogNoPortForLevel();
      return;
    }
    auto *stack = MutableStackForPort(target_port);
    if (stack == nullptr || stack->empty()) {
      return;
    }

    const auto now = Clock::now();
    if (password_verify_next_allowed_time_.has_value() && now < *password_verify_next_allowed_time_) {
      LogWaitingCooldown(stack->back(), stack->size(), now);
      return;
    }

    // 冷却已过但等级仍未上升，说明上一发没被采纳：换下一个候选再试。
    if (awaiting_level_up_ && awaiting_level_up_port_ == target_port) {
      if (!radar::config::kOpponentKeyRetryUntilLevelUp) {
        return;
      }
      RotateStack(stack);
    }

    const auto pending = stack->back();
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
    context.decision = awaiting_level_up_ ? "retry" : "sent";
    context.ally_encryption_level = current_ally_encryption_level_;
    const bool queued = Send(cmd, serial_protocol, context, [this, source_port = pending.source_port]() {
      MarkOpponentKeySent(source_port);
      password_verify_in_flight_ = false;
      password_verify_next_allowed_time_ = Clock::now() + PasswordVerifyCooldown();
    });
    if (!queued) {
      return;
    }
    password_verify_in_flight_ = true;
    // 不出栈：等 `0x020E` 等级上升后由 UpdateAllyEncryptionLevel 统一清理。
    awaiting_level_up_ = true;
    awaiting_level_up_port_ = target_port;
    ++opponent_key_attempt_count_;
  }

  /**
   * @brief 判断当前是否仍有待处理敌方密钥
   * @return 任一栈非空或仍有在途验证时返回 true
   */
  bool HasPending() const { return password_verify_in_flight_ || HasQueuedPending(); }

  /**
   * @brief 判断当前是否有待发送敌方密钥
   * @return 任一端口栈非空时返回 true
   */
  bool HasQueuedPending() const {
    return !enemy_level1_key_stack_.empty() || !enemy_level2_key_stack_.empty();
  }

  /**
   * @brief 返回指定来源端口当前栈内待验证密钥数量
   * @param source_port 来源端口
   */
  std::size_t QueuedKeyCount(int source_port) const {
    const auto *stack = StackForPort(source_port);
    return stack == nullptr ? 0 : stack->size();
  }

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
   * @brief 更新最近一次 `0x020E` 解出的干扰波等级
   * @param ally_encryption_level 当前协议中的 bit3-4 值
   * @note 等级上升即视为上一次提交的密钥已被裁判系统采纳。此时按新等级清理
   *       不再有用的密钥栈：升到 2 级说明 `8002` 那一级已过，整栈作废；
   *       升到 3 级说明已满级，两个栈都不再需要。
   */
  void UpdateAllyEncryptionLevel(rm::u8 ally_encryption_level) {
    if (ally_encryption_level <= current_ally_encryption_level_) {
      current_ally_encryption_level_ = ally_encryption_level;
      return;
    }

    const auto previous_level = current_ally_encryption_level_;
    ++ally_encryption_level_upgrade_generation_;
    current_ally_encryption_level_ = ally_encryption_level;

    // 等级已经推进，上一发不需要再重投。
    awaiting_level_up_ = false;
    awaiting_level_up_port_ = 0;

    const std::size_t level1_before = enemy_level1_key_stack_.size();
    const std::size_t level2_before = enemy_level2_key_stack_.size();
    if (ally_encryption_level >= 2) {
      enemy_level1_key_stack_.clear();
    }
    if (ally_encryption_level >= kRadarMaxInterferenceLevel) {
      enemy_level2_key_stack_.clear();
    }
    LogLevelUpgrade(previous_level, ally_encryption_level, level1_before, level2_before);
  }

  /**
   * @brief 返回己方干扰波等级实际升级的累计代次
   * @note 仅当 `0x020E` bit3-4 的等级变大时递增；相同等级的重复帧不会递增。
   */
  std::size_t ally_encryption_level_upgrade_generation() const {
    return ally_encryption_level_upgrade_generation_;
  }

  /**
   * @brief 判断指定来源端口的敌方密钥当前是否允许推进发送
   * @param source_port 来源端口
   * @return 当前干扰波等级正好对应该端口时返回 true
   * @note 等级来自 `0x020E` bit3-4：等级 1 走 `8002`，等级 2 走 `8003`。
   *       等级 0 表示尚未收到 `0x020E`，等级 3 表示已满级，两者都不发送。
   */
  bool CanSendOpponentKeyFromPort(int source_port) const {
    const int target_port = OpponentKeyPortForCurrentLevel();
    return target_port != 0 && target_port == source_port;
  }

  /**
   * @brief 返回当前干扰波等级对应的密钥来源端口
   * @return 等级 1 返回 `8002`，等级 2 返回 `8003`，其余返回 0
   */
  int OpponentKeyPortForCurrentLevel() const {
    return OpponentKeySourcePortForLevel(current_ally_encryption_level_);
  }

  /// 返回最近一次 `0x020E` 解出的干扰波等级。
  rm::u8 current_ally_encryption_level() const { return current_ally_encryption_level_; }

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
          << "\"ally_side\":\"" << RadarSideName(current_side_) << "\","
          << "\"sender_id\":\"" << radar::log::HexU16(sender_id) << "\","
          << "\"receiver_id\":\"" << radar::log::HexU16(receiver_id) << "\","
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
   * @brief 记录一条压入验证栈的敌方密钥
   * @param pending 待验证敌方密钥
   * @param stack_depth 入栈后该端口栈内深度
   */
  void LogQueuedKey(const PendingOpponentKey &pending, std::size_t stack_depth) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"" << pending.name << "\","
          << "\"source\":\"tcp\","
          << "\"source_port\":" << pending.source_port << ','
          << "\"decision\":\"queued\","
          << "\"stack_depth\":" << stack_depth << ','
          << "\"key_hex\":\"" << radar::log::HexBytes(pending.key.data(), pending.key.size()) << "\"}";

    const auto basename = std::string("0x0121_") + pending.name + "_queued";
    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录当前仍处于 `password_cmd=2` 冷却中的等待状态
   * @param pending 栈顶待发送密钥
   * @param stack_depth 当前端口栈内深度
   * @param now 当前时间
   */
  void LogWaitingCooldown(const PendingOpponentKey &pending, std::size_t stack_depth, TimePoint now) {
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
          << "\"stack_depth\":" << stack_depth << ','
          << "\"interference_level\":" << radar::log::JsonScalar(current_ally_encryption_level_) << ','
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

  /**
   * @brief 按来源端口取出可写的待验证密钥栈
   * @param source_port 来源端口
   * @return 对应栈指针；端口不属于 `8002/8003` 时返回 nullptr
   */
  std::deque<PendingOpponentKey> *MutableStackForPort(int source_port) {
    if (source_port == radar::config::kEnemyLevel1KeyTcpServerPort) {
      return &enemy_level1_key_stack_;
    }
    if (source_port == radar::config::kEnemyLevel2KeyTcpServerPort) {
      return &enemy_level2_key_stack_;
    }
    return nullptr;
  }

  /**
   * @brief 按来源端口取出只读的待验证密钥栈
   * @param source_port 来源端口
   * @return 对应栈指针；端口不属于 `8002/8003` 时返回 nullptr
   */
  const std::deque<PendingOpponentKey> *StackForPort(int source_port) const {
    if (source_port == radar::config::kEnemyLevel1KeyTcpServerPort) {
      return &enemy_level1_key_stack_;
    }
    if (source_port == radar::config::kEnemyLevel2KeyTcpServerPort) {
      return &enemy_level2_key_stack_;
    }
    return nullptr;
  }

  /**
   * @brief 记录“当前等级没有对应密钥来源端口”的挂起状态
   * @note 等级 0 表示尚未收到 `0x020E`，等级 3 表示已满级，两者都不需要再提交密钥。
   *       仅在仍有密钥待发送时输出，并按最小间隔节流。
   */
  void LogNoPortForLevel() {
    if (!HasQueuedPending()) {
      return;
    }
    const auto now = Clock::now();
    if (last_no_port_log_time_.has_value() && now - *last_no_port_log_time_ < WaitingCooldownLogMinInterval()) {
      return;
    }

    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"opponent_key\","
          << "\"source\":\"tcp\","
          << "\"decision\":\"holding_no_port_for_level\","
          << "\"interference_level\":" << radar::log::JsonScalar(current_ally_encryption_level_) << ','
          << "\"level1_stack_depth\":" << enemy_level1_key_stack_.size() << ','
          << "\"level2_stack_depth\":" << enemy_level2_key_stack_.size() << "}";

    log_store_.Append(std::filesystem::path("main") / "0x0121_opponent_key_holding.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
    last_no_port_log_time_ = now;
  }

  /**
   * @brief 判断当前是否处于“比赛中”阶段
   * @tparam Protocol 当前串口维护状态类型
   * @param serial_protocol 当前常规链路状态
   * @return 允许提交敌方密钥时返回 true
   * @note 赛前提交的 `password_cmd=2` 不会被裁判系统采纳，却会占掉一次 10s 冷却，
   *       因此默认必须等 `0x0001` 阶段为 4。开关见 `kOpponentKeyRequireMatchRunning`。
   */
  template <typename Protocol>
  static bool IsMatchRunning(const Protocol &serial_protocol) {
    if (!radar::config::kOpponentKeyRequireMatchRunning) {
      return true;
    }
    return serial_protocol.game_status.game_progress == kGameProgressInMatch;
  }

  /**
   * @brief 把栈顶候选轮换到栈底，让下一个候选上来
   * @param stack 当前端口的待验证密钥栈
   * @note 单个候选时等价于原地重投。
   */
  static void RotateStack(std::deque<PendingOpponentKey> *stack) {
    if (stack->size() < 2) {
      return;
    }
    auto rotated = stack->back();
    stack->pop_back();
    stack->push_front(std::move(rotated));
  }

  /**
   * @brief 记录“比赛未开始，密钥暂不提交”的挂起状态
   * @tparam Protocol 当前串口维护状态类型
   * @param serial_protocol 当前常规链路状态
   * @note 仅在仍有密钥待发送时输出，并按最小间隔节流。
   */
  template <typename Protocol>
  void LogHoldingBeforeMatch(const Protocol &serial_protocol) {
    if (!HasQueuedPending()) {
      return;
    }
    const auto now = Clock::now();
    if (last_pre_match_hold_log_time_.has_value() &&
        now - *last_pre_match_hold_log_time_ < WaitingCooldownLogMinInterval()) {
      return;
    }

    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"opponent_key\","
          << "\"source\":\"tcp\","
          << "\"decision\":\"holding_match_not_running\","
          << "\"game_progress\":" << radar::log::JsonScalar(serial_protocol.game_status.game_progress) << ','
          << "\"interference_level\":" << radar::log::JsonScalar(current_ally_encryption_level_) << ','
          << "\"level1_stack_depth\":" << enemy_level1_key_stack_.size() << ','
          << "\"level2_stack_depth\":" << enemy_level2_key_stack_.size() << "}";

    log_store_.Append(std::filesystem::path("main") / "0x0121_opponent_key_holding.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
    last_pre_match_hold_log_time_ = now;
  }

  /**
   * @brief 记录一次干扰波等级上升及对应的密钥栈清理结果
   * @param previous_level 升级前等级
   * @param current_level 升级后等级
   * @param level1_before 清理前 `8002` 栈深度
   * @param level2_before 清理前 `8003` 栈深度
   */
  void LogLevelUpgrade(rm::u8 previous_level, rm::u8 current_level, std::size_t level1_before,
                       std::size_t level2_before) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"name\":\"opponent_key\","
          << "\"source\":\"serial\","
          << "\"decision\":\"level_up\","
          << "\"previous_level\":" << radar::log::JsonScalar(previous_level) << ','
          << "\"current_level\":" << radar::log::JsonScalar(current_level) << ','
          << "\"upgrade_generation\":" << ally_encryption_level_upgrade_generation_ << ','
          << "\"attempt_count\":" << opponent_key_attempt_count_ << ','
          << "\"level1_stack_before\":" << level1_before << ','
          << "\"level1_stack_after\":" << enemy_level1_key_stack_.size() << ','
          << "\"level2_stack_before\":" << level2_before << ','
          << "\"level2_stack_after\":" << enemy_level2_key_stack_.size() << "}";

    log_store_.Append(std::filesystem::path("main") / "0x0121_opponent_key_level_up.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  RefereeTxScheduler &tx_scheduler_;                   ///< 统一发送调度器
  radar::log::FileLogStore log_store_;                ///< `0x0121` 发送日志输出器
  std::deque<PendingOpponentKey> enemy_level1_key_stack_;  ///< `8002` 密钥栈，后进先出
  std::deque<PendingOpponentKey> enemy_level2_key_stack_;  ///< `8003` 密钥栈，后进先出
  std::optional<TimePoint> password_verify_next_allowed_time_;  ///< 下一次允许验证密钥的时间
  std::optional<TimePoint> last_waiting_cooldown_log_time_;     ///< 上次输出等待中日志的时间
  std::optional<TimePoint> last_no_port_log_time_;               ///< 上次输出等级无对应端口日志的时间
  std::optional<TimePoint> last_pre_match_hold_log_time_;        ///< 上次输出赛前挂起日志的时间
  std::size_t opponent_key_attempt_count_ = 0;        ///< 累计提交敌方密钥的次数（含重投）
  int awaiting_level_up_port_ = 0;                    ///< 正在等待等级上升确认的来源端口
  bool awaiting_level_up_ = false;                    ///< 是否已提交但尚未观测到等级上升
  bool password_verify_in_flight_ = false;            ///< 当前是否已有一条验证指令等待真正发出
  bool enemy_level1_key_sent_ = false;                ///< `8002` 密钥是否已真正发出
  bool enemy_level2_key_sent_ = false;                ///< `8003` 密钥是否已真正发出
  rm::u8 current_ally_encryption_level_ = 0;          ///< 最近一次 `0x020E` 解出的 bit3-4 干扰波等级
  std::size_t ally_encryption_level_upgrade_generation_ = 0;  ///< 己方干扰波等级升级累计代次
  rm::u16 current_robot_id_ = 0;                       ///< 最近一次 `0x0201.robot_id`
  RadarSide current_side_ = RadarSide::kUnknown;       ///< 当前己方红蓝方
  rm::u8 radar_cmd_counter_ = kRadarInitialCommandCounter;  ///< 当前 `radar_cmd`
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_RADAR_COMMAND_SENDER_HPP
