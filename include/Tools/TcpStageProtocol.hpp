/**
 * @file    include/Tools/TcpStageProtocol.hpp
 * @brief   定义 TCP 阶段控制协议的数据结构与编解码 helper。
 *
 * 该文件只描述协议本身，不负责 socket 生命周期。当前协议包含两类命令：
 * - `0x91 + 1Byte + 2Byte`：低 4 bit 为 `game_progress`，后 2Byte 为
 *   大端 `stage_remain_time`
 * - `0x92 + 1Byte`：低 1 bit 为“敌方无人机是否被反制”
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace Tools {

enum class TcpStageCommandType {
  GameState91,
  Countered92,
};

enum class TcpStageDecodeStatus {
  Decoded,
  NeedMoreData,
  InvalidCommand,
};

struct TcpStageCommand {
  TcpStageCommandType type = TcpStageCommandType::GameState91;
  std::uint8_t game_progress = 0;
  std::uint16_t stage_remain_time = 0;
  bool countered = false;
};

inline std::array<std::uint8_t, 4>
EncodeGameStateCommand(std::uint8_t game_progress,
                       std::uint16_t stage_remain_time) {
  return {
      0x91,
      static_cast<std::uint8_t>(game_progress & 0x0F),
      static_cast<std::uint8_t>((stage_remain_time >> 8) & 0xFF),
      static_cast<std::uint8_t>(stage_remain_time & 0xFF),
  };
}

inline std::array<std::uint8_t, 2>
EncodeCounteredStateCommand(bool countered) {
  return {0x92, static_cast<std::uint8_t>(countered ? 0x01 : 0x00)};
}

inline TcpStageDecodeStatus
TryDecodeTcpStageCommand(const std::uint8_t *data, std::size_t size,
                         TcpStageCommand *command,
                         std::size_t *consumed_size = nullptr) {
  if (command == nullptr || data == nullptr || size == 0) {
    return TcpStageDecodeStatus::NeedMoreData;
  }

  const std::uint8_t command_id = data[0];
  if (command_id == 0x91) {
    constexpr std::size_t kPacketSize = 4;
    if (size < kPacketSize) {
      return TcpStageDecodeStatus::NeedMoreData;
    }

    command->type = TcpStageCommandType::GameState91;
    command->game_progress = static_cast<std::uint8_t>(data[1] & 0x0F);
    command->stage_remain_time = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[2]) << 8) |
        static_cast<std::uint16_t>(data[3]));
    if (consumed_size != nullptr) {
      *consumed_size = kPacketSize;
    }
    return TcpStageDecodeStatus::Decoded;
  }

  if (command_id == 0x92) {
    constexpr std::size_t kPacketSize = 2;
    if (size < kPacketSize) {
      return TcpStageDecodeStatus::NeedMoreData;
    }

    command->type = TcpStageCommandType::Countered92;
    command->countered = (data[1] & 0x01) != 0;
    if (consumed_size != nullptr) {
      *consumed_size = kPacketSize;
    }
    return TcpStageDecodeStatus::Decoded;
  }

  return TcpStageDecodeStatus::InvalidCommand;
}

} // namespace Tools
