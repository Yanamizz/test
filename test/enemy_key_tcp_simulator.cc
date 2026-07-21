/**
 * @file test/enemy_key_tcp_simulator.cc
 * @brief 手动模拟 `8002/8003` 敌方密钥服务端
 *
 * 本工具作为 TCP server 监听配置中的 8002 和 8003，供主程序的
 * `TcpClient` 接入。输入一条合法密钥后，工具生成完整的 `0x0A06` 协议帧
 * 并发送给该端口已接入的雷达客户端。
 */

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <poll.h>
#include <unistd.h>

#include "include/config/config.hpp"
#include "include/referee/enemy_key_receiver.hpp"
#include "include/referee/tcp_server.hpp"
#include "librm/device/referee/protocol.hpp"
#include "librm/modules/crc.hpp"

namespace {

using Key = std::array<rm::u8, radar::referee::EnemyKeyReceiver<radar::config::kRefereeRevision>::kKeyBytes>;
using Frame = std::array<rm::u8, rm::device::kRefProtocolAllMetadataLen + Key{}.size()>;

struct KeyServerChannel {
  int level = 0;
  int port = 0;
  rm::u8 next_seq = 0;
  radar::referee::TcpServer server;
};

void PrintHelp() {
  std::cout << "Commands:\n"
            << "  1 ABC123       send a level-1 key through 8002\n"
            << "  2 ABC123       send a level-2 key through 8003\n"
            << "  status         show listener and client state\n"
            << "  help           show this help\n"
            << "  quit           exit\n";
}

void PrintChannelState(const KeyServerChannel &channel) {
  std::cout << "800" << channel.level + 1 << " (level " << channel.level << "): "
            << (channel.server.has_client() ? "client connected" : "waiting for radar client");
  if (channel.server.has_client()) {
    std::cout << " (" << channel.server.peer_ip() << ')';
  }
  std::cout << '\n';
}

void PrintStatus(const KeyServerChannel &level1, const KeyServerChannel &level2) {
  PrintChannelState(level1);
  PrintChannelState(level2);
}

std::optional<Key> ParseKey(const std::string &input) {
  if (input.size() != Key{}.size()) {
    return std::nullopt;
  }

  Key key{};
  for (std::size_t i = 0; i < key.size(); ++i) {
    key[i] = static_cast<rm::u8>(input[i]);
  }
  if (!radar::referee::IsRadarKey(key)) {
    return std::nullopt;
  }
  return key;
}

Frame BuildRadar5Frame(const Key &key, rm::u8 seq) {
  using Cmd = rm::device::RefereeCmdId<radar::config::kRefereeRevision>;

  Frame frame{};
  frame[0] = rm::device::kRefProtocolHeaderSof;
  frame[1] = static_cast<rm::u8>(key.size());
  frame[2] = 0;
  frame[3] = seq;
  frame[4] = rm::modules::Crc8(frame.data(), rm::device::kRefProtocolHeaderLen - 1, rm::modules::CRC8_INIT);
  frame[5] = static_cast<rm::u8>(Cmd::kRadar5 & 0xff);
  frame[6] = static_cast<rm::u8>(Cmd::kRadar5 >> 8);
  std::memcpy(frame.data() + rm::device::kRefProtocolHeaderLen + rm::device::kRefProtocolCmdIdLen, key.data(),
              key.size());

  const std::size_t crc_offset = frame.size() - rm::device::kRefProtocolCrc16Len;
  const rm::u16 crc16 = rm::modules::Crc16(frame.data(), crc_offset, rm::modules::CRC16_INIT);
  frame[crc_offset] = static_cast<rm::u8>(crc16 & 0xff);
  frame[crc_offset + 1] = static_cast<rm::u8>(crc16 >> 8);
  return frame;
}

void SendKey(KeyServerChannel &channel, const Key &key) {
  if (!channel.server.has_client()) {
    std::cout << "800" << channel.level + 1 << " has no connected radar client; key not sent\n";
    return;
  }

  const Frame frame = BuildRadar5Frame(key, channel.next_seq);
  std::string error;
  if (!channel.server.TryWriteAll(frame.data(), frame.size(), &error)) {
    std::cerr << "send to 800" << channel.level + 1 << " failed: " << error << '\n';
    channel.server.CloseClient();
    return;
  }

  ++channel.next_seq;
  std::cout << "sent 0x0A06 key " << std::string(key.begin(), key.end()) << " through 800"
            << channel.level + 1 << '\n';
}

void AcceptPendingClient(KeyServerChannel &channel) {
  if (channel.server.has_client()) {
    return;
  }

  std::string error;
  if (channel.server.AcceptPending(&error)) {
    std::cout << "radar client connected to 800" << channel.level + 1 << " from " << channel.server.peer_ip()
              << '\n';
  } else if (!error.empty()) {
    std::cerr << error << '\n';
  }
}

void ServiceClient(KeyServerChannel &channel, short revents) {
  if (!channel.server.has_client()) {
    return;
  }

  if ((revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    std::cout << "radar client disconnected from 800" << channel.level + 1 << '\n';
    channel.server.CloseClient();
    return;
  }
  if ((revents & POLLIN) == 0) {
    return;
  }

  std::array<rm::u8, 64> discard{};
  try {
    if (channel.server.Read(discard.data(), discard.size()) == 0 && !channel.server.has_client()) {
      std::cout << "radar client disconnected from 800" << channel.level + 1 << '\n';
    }
  } catch (const std::exception &ex) {
    std::cerr << "read from 800" << channel.level + 1 << " failed: " << ex.what() << '\n';
    channel.server.CloseClient();
  }
}

std::optional<KeyServerChannel *> SelectChannel(const std::string &token, KeyServerChannel &level1,
                                                 KeyServerChannel &level2) {
  if (token == "1" || token == "8002") {
    return &level1;
  }
  if (token == "2" || token == "8003") {
    return &level2;
  }
  return std::nullopt;
}

bool HandleCommand(const std::string &line, KeyServerChannel &level1, KeyServerChannel &level2) {
  std::istringstream input(line);
  std::string command;
  input >> command;
  if (command.empty()) {
    return true;
  }
  if (command == "quit" || command == "exit") {
    return false;
  }
  if (command == "help") {
    PrintHelp();
    return true;
  }
  if (command == "status") {
    PrintStatus(level1, level2);
    return true;
  }

  const auto channel = SelectChannel(command, level1, level2);
  std::string key_text;
  std::string extra;
  if (!channel.has_value() || !(input >> key_text) || (input >> extra)) {
    std::cerr << "invalid command; enter help for usage\n";
    return true;
  }

  const auto key = ParseKey(key_text);
  if (!key.has_value()) {
    std::cerr << "key must contain exactly 6 ASCII letters or digits\n";
    return true;
  }
  SendKey(**channel, *key);
  return true;
}

void AddChannelPollFd(KeyServerChannel &channel, std::array<pollfd, 3> &fds, nfds_t &nfds,
                      std::array<KeyServerChannel *, 3> &owners) {
  fds[nfds] = pollfd{channel.server.has_client() ? channel.server.client_fd() : channel.server.fd(),
                     static_cast<short>(POLLIN | POLLHUP | POLLERR), 0};
  owners[nfds++] = &channel;
}

}  // namespace

int main() {
  try {
    KeyServerChannel level1{1, radar::config::kEnemyLevel1KeyTcpServerPort};
    KeyServerChannel level2{2, radar::config::kEnemyLevel2KeyTcpServerPort};
    level1.server.Open(radar::config::kEnemyKeySimulatorBindAddress, level1.port);
    level2.server.Open(radar::config::kEnemyKeySimulatorBindAddress, level2.port);

    std::cout << "enemy-key simulator listening on " << radar::config::kEnemyKeySimulatorBindAddress << ':'
              << level1.port << " and " << radar::config::kEnemyKeySimulatorBindAddress << ':' << level2.port
              << '\n';
    PrintHelp();

    bool running = true;
    while (running) {
      std::array<pollfd, 3> fds{};
      std::array<KeyServerChannel *, 3> owners{};
      fds[0] = pollfd{STDIN_FILENO, POLLIN, 0};
      nfds_t nfds = 1;
      AddChannelPollFd(level1, fds, nfds, owners);
      AddChannelPollFd(level2, fds, nfds, owners);

      const int result = ::poll(fds.data(), nfds, -1);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
      }

      for (nfds_t i = 1; i < nfds; ++i) {
        if (fds[i].revents == 0) {
          continue;
        }
        if (owners[i]->server.has_client()) {
          ServiceClient(*owners[i], fds[i].revents);
        } else if ((fds[i].revents & POLLIN) != 0) {
          AcceptPendingClient(*owners[i]);
        }
      }

      if ((fds[0].revents & POLLIN) != 0) {
        std::string line;
        if (!std::getline(std::cin, line)) {
          break;
        }
        running = HandleCommand(line, level1, level2);
      }
    }
  } catch (const std::exception &ex) {
    std::cerr << "enemy-key simulator failed: " << ex.what() << '\n';
    return 1;
  }
  return 0;
}
