#ifndef RADAR_INCLUDE_REFEREE_ROBOT_INTERACTION_SENDER_HPP
#define RADAR_INCLUDE_REFEREE_ROBOT_INTERACTION_SENDER_HPP

/**
 * @file  include/referee/robot_interaction_sender.hpp
 * @brief `0x0301` 子命令通用组包与发送封装
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/radar/app/subReferee/referee_user.hpp"
#include "include/referee/referee_tx_scheduler.hpp"

#ifndef RADAR_DEFAULT_LOG_DIR
#define RADAR_DEFAULT_LOG_DIR "test/logs"
#endif

namespace radar::referee {

/// 默认雷达发送方 ID；若未能从串口状态解析，则回退到此值。
constexpr rm::u16 kDefaultRobotInteractionSenderId = 9;
/// 普通机器人交互主命令码。
constexpr rm::u16 kRobotInteractionCmdId = 0x0301;

/**
 * @brief 返回当前协议版本下 `0x0301` user_data 最大容量
 */
inline constexpr std::size_t RobotInteractionPayloadCapacity() {
  using CurrentProtocol = rm::device::RefereeProtocol<radar::config::kRefereeRevision>;
  return sizeof(((CurrentProtocol *)nullptr)->robot_interaction_data.user_data);
}

/**
 * @brief 一条 `0x0301` 发送任务的上下文
 * @note  最终仍复用 `RefereeTxScheduler` 的 `0x0301` FIFO 队列。
 */
struct RobotInteractionContext {
  std::string name = "robot_interaction";
  int source_port = 0;
  std::string decision = "manual_send";
};

/**
 * @brief 运行时按子命令码组装一帧 `0x0301`
 * @param data 输出缓冲区
 * @param start_index 起始写入位置
 * @param sub_cmd_id `0x0301` 下的子命令码
 * @param payload 负载首地址
 * @param payload_size 负载字节数
 * @param sender 发送方 ID
 * @param receiver 接收方 ID
 * @return 成功时返回帧长；失败时返回 0
 */
[[nodiscard]] inline rm::u8 PrepareRobotInteractionFrame(rm::u8 *data, const rm::u16 start_index,
                                                         const rm::u16 sub_cmd_id, const rm::u8 *payload,
                                                         const std::size_t payload_size, const rm::u16 sender,
                                                         const rm::u16 receiver) {
  if (payload == nullptr || payload_size > RobotInteractionPayloadCapacity()) {
    return 0;
  }

  const std::size_t frame_size = rm::device::kRefProtocolAllMetadataLen + 6 + payload_size;
  if (frame_size > rm::device::kRefProtocolFrameMaxLen) {
    return 0;
  }

  rm::u16 index = start_index;
  data[index++] = rm::device::kRefProtocolHeaderSof;
  const rm::u16 data_len = static_cast<rm::u16>(payload_size + 6);
  data[index++] = data_len & 0xff;
  data[index++] = data_len >> 8;
  data[index++] = rm::device::NextRefereeTxSeq();
  data[index++] =
      rm::modules::Crc8(&data[start_index], rm::device::kRefProtocolHeaderLen - 1, rm::modules::CRC8_INIT);
  data[index++] = kRobotInteractionCmdId & 0xff;
  data[index++] = kRobotInteractionCmdId >> 8;
  data[index++] = sub_cmd_id & 0xff;
  data[index++] = sub_cmd_id >> 8;
  data[index++] = sender & 0xff;
  data[index++] = sender >> 8;
  data[index++] = receiver & 0xff;
  data[index++] = receiver >> 8;
  std::memcpy(&data[index], payload, payload_size);
  index += static_cast<rm::u16>(payload_size);
  const rm::u16 crc16 = rm::modules::Crc16(&data[start_index], index - start_index, rm::modules::CRC16_INIT);
  data[index++] = crc16 & 0xff;
  data[index++] = crc16 >> 8;
  return static_cast<rm::u8>(index - start_index);
}

/**
 * @brief 通用 `0x0301` 发送器
 * @note  若想让“发送接口只传 `sub_cmd_id` 和 `receiver_id`”，先通过 `UpdatePayload(...)`
 *        缓存对应负载，再调用 `SendCached(...)` 即可。
 */
class RobotInteractionSender {
 public:
  using SenderIdProvider = std::function<rm::u16()>;

  /**
   * @brief 创建发送器
   * @param tx_scheduler 主链统一发送调度器
   * @param sender_id_provider 可选的发送方 ID 提供器；返回 0 时会回退默认值
   * @param log_root 本轮运行日志根目录
   */
  explicit RobotInteractionSender(RefereeTxScheduler &tx_scheduler, SenderIdProvider sender_id_provider = {},
                                  std::filesystem::path log_root = RADAR_DEFAULT_LOG_DIR)
      : tx_scheduler_(tx_scheduler), sender_id_provider_(std::move(sender_id_provider)), log_store_(std::move(log_root)) {}

  /**
   * @brief 用类型自动映射的子命令码缓存一份 payload
   * @tparam T 已在 `TypeToCmd` 中注册的子结构体类型
   * @param payload 结构体内容
   */
  template <typename T>
  void UpdatePayload(const T &payload) {
    UpdatePayloadRaw(static_cast<rm::u16>(rm::device::getCmd(payload)),
                     reinterpret_cast<const rm::u8 *>(&payload), sizeof(payload));
  }

  /**
   * @brief 按子命令码缓存一份原始 payload
   * @param sub_cmd_id `0x0301` 下的子命令码
   * @param payload 负载首地址
   * @param payload_size 负载字节数
   */
  void UpdatePayloadRaw(const rm::u16 sub_cmd_id, const rm::u8 *payload, const std::size_t payload_size) {
    if (payload == nullptr) {
      return;
    }
    payload_cache_[sub_cmd_id] = std::vector<rm::u8>(payload, payload + payload_size);
  }

  /**
   * @brief 判断某个子命令码是否已有缓存 payload
   * @param sub_cmd_id `0x0301` 下的子命令码
   * @return 是否存在缓存
   */
  bool HasPayload(const rm::u16 sub_cmd_id) const { return payload_cache_.find(sub_cmd_id) != payload_cache_.end(); }

  /**
   * @brief 发送一条已缓存 payload 的 `0x0301`
   * @param sub_cmd_id `0x0301` 下的子命令码
   * @param receiver_id 接收者机器人 ID
   * @param context 调度器日志上下文
   * @return 是否成功进入发送队列
   */
  bool SendCached(const rm::u16 sub_cmd_id, const rm::u16 receiver_id,
                  const RobotInteractionContext &context = {}) {
    const auto it = payload_cache_.find(sub_cmd_id);
    if (it == payload_cache_.end()) {
      LogValidationFailure(sub_cmd_id, nullptr, 0, ResolveSenderId(), receiver_id, context, "payload_missing");
      return false;
    }
    return SendRaw(sub_cmd_id, it->second.data(), it->second.size(), receiver_id, context);
  }

  /**
   * @brief 直接发送一段原始 payload
   * @param sub_cmd_id `0x0301` 下的子命令码
   * @param payload 负载首地址
   * @param payload_size 负载字节数
   * @param receiver_id 接收者机器人 ID
   * @param context 调度器日志上下文
   * @return 是否成功进入发送队列
   */
  bool SendRaw(const rm::u16 sub_cmd_id, const rm::u8 *payload, const std::size_t payload_size,
               const rm::u16 receiver_id, const RobotInteractionContext &context = {}) {
    const rm::u16 sender_id = ResolveSenderId();
    if (const char *reason = ValidatePayload(payload, payload_size); reason != nullptr) {
      LogValidationFailure(sub_cmd_id, payload, payload_size, sender_id, receiver_id, context, reason);
      return false;
    }

    std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> tx_buffer{};
    const rm::u8 frame_len =
        PrepareRobotInteractionFrame(tx_buffer.data(), 0, sub_cmd_id, payload, payload_size, sender_id, receiver_id);
    if (frame_len == 0) {
      LogValidationFailure(sub_cmd_id, payload, payload_size, sender_id, receiver_id, context, "frame_build_failed");
      return false;
    }

    std::vector<rm::u8> frame(tx_buffer.begin(), tx_buffer.begin() + frame_len);
    return tx_scheduler_.EnqueueRobotInteraction(
        std::move(frame), context.name, context.source_port, context.decision,
        [this, sub_cmd_id, sender_id, receiver_id, payload_size, context](const rm::u8 *frame, std::size_t frame_len) {
          LogSentFrame(sub_cmd_id, frame, frame_len, sender_id, receiver_id, payload_size, context);
        });
  }

  /**
   * @brief 发送一个已注册 `TypeToCmd` 的结构体
   * @tparam T 已注册子命令映射的结构体类型
   * @param payload 结构体内容
   * @param receiver_id 接收者机器人 ID
   * @param context 调度器日志上下文
   * @return 是否成功进入发送队列
   */
  template <typename T>
  bool Send(const T &payload, const rm::u16 receiver_id, const RobotInteractionContext &context = {}) {
    return SendRaw(static_cast<rm::u16>(rm::device::getCmd(payload)),
                   reinterpret_cast<const rm::u8 *>(&payload), sizeof(payload), receiver_id, context);
  }

 private:
  /**
   * @brief 校验待发送 payload 是否满足 `0x0301` 约束
   * @return 成功返回 `nullptr`，失败返回拒绝原因
   */
  static const char *ValidatePayload(const rm::u8 *payload, std::size_t payload_size) {
    if (payload == nullptr) {
      return "null_payload";
    }
    if (payload_size > RobotInteractionPayloadCapacity()) {
      return "payload_too_large";
    }
    if (rm::device::kRefProtocolAllMetadataLen + 6 + payload_size > rm::device::kRefProtocolFrameMaxLen) {
      return "frame_too_large";
    }
    return nullptr;
  }

  /**
   * @brief 解析本端发送方 ID
   * @return 若回调可用且非 0 则使用回调值，否则回退默认值
   */
  rm::u16 ResolveSenderId() const {
    if (sender_id_provider_) {
      const rm::u16 sender_id = sender_id_provider_();
      if (sender_id != 0) {
        return sender_id;
      }
    }
    return kDefaultRobotInteractionSenderId;
  }

  /**
   * @brief 记录一条真正发出的普通 `0x0301`
   */
  void LogSentFrame(rm::u16 sub_cmd_id, const rm::u8 *frame, std::size_t frame_len, rm::u16 sender_id,
                    rm::u16 receiver_id, std::size_t payload_size, const RobotInteractionContext &context) {
    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"result\":\"sent\","
          << "\"name\":\"" << context.name << "\","
          << "\"source_port\":" << context.source_port << ','
          << "\"decision\":\"" << context.decision << "\","
          << "\"cmd_id\":\"" << radar::log::HexU16(kRobotInteractionCmdId) << "\","
          << "\"sub_cmd_id\":\"" << radar::log::HexU16(sub_cmd_id) << "\","
          << "\"sender_id\":" << sender_id << ','
          << "\"receiver_id\":" << receiver_id << ','
          << "\"payload_len\":" << payload_size << ','
          << "\"frame_hex\":\"" << radar::log::HexBytes(frame, frame_len) << "\"}";
    log_store_.Append(std::filesystem::path("main") / "0x0301_robot_interaction.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  /**
   * @brief 记录一次普通 `0x0301` 校验失败
   */
  void LogValidationFailure(rm::u16 sub_cmd_id, const rm::u8 *payload, std::size_t payload_size, rm::u16 sender_id,
                            rm::u16 receiver_id, const RobotInteractionContext &context, const std::string &reason) {
    const rm::u8 empty_payload = 0;
    const rm::u8 *payload_view = payload != nullptr ? payload : &empty_payload;
    const std::size_t payload_view_size = payload != nullptr ? payload_size : 0;

    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << radar::log::TimestampNow() << "\","
          << "\"result\":\"rejected_validation\","
          << "\"reason\":\"" << reason << "\","
          << "\"name\":\"" << context.name << "\","
          << "\"source_port\":" << context.source_port << ','
          << "\"decision\":\"" << context.decision << "\","
          << "\"cmd_id\":\"" << radar::log::HexU16(kRobotInteractionCmdId) << "\","
          << "\"sub_cmd_id\":\"" << radar::log::HexU16(sub_cmd_id) << "\","
          << "\"sender_id\":" << sender_id << ','
          << "\"receiver_id\":" << receiver_id << ','
          << "\"payload_len\":" << payload_size << ','
          << "\"payload_hex\":\"" << radar::log::HexBytes(payload_view, payload_view_size) << "\"}";
    log_store_.Append(std::filesystem::path("main") / "0x0301_robot_interaction_rejected.log", entry.str(),
                      radar::log::LogPriority::kCriticalDecision);
  }

  RefereeTxScheduler &tx_scheduler_;                      ///< 主链统一发送调度器
  SenderIdProvider sender_id_provider_;                   ///< 发送方 ID 解析器
  radar::log::FileLogStore log_store_;                    ///< 普通 `0x0301` 发送日志输出器
  std::map<rm::u16, std::vector<rm::u8>> payload_cache_;  ///< 各子命令的最近一次 payload 缓存
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_ROBOT_INTERACTION_SENDER_HPP
