#ifndef RADAR_INCLUDE_REFEREE_REPLAY_INPUT_SOURCE_HPP
#define RADAR_INCLUDE_REFEREE_REPLAY_INPUT_SOURCE_HPP

/**
 * @file  include/referee/replay_input_source.hpp
 * @brief 预制二进制文件回放输入源
 */

#include <chrono>
#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "include/config/config.hpp"
#include "librm/core/typedefs.hpp"
#include "librm/modules/crc.hpp"

namespace radar::referee {

/**
 * @brief 将频率转换为回放周期
 * @param rate_hz 回放频率
 * @return 对应周期
 */
inline std::chrono::steady_clock::duration ReplayRateToPeriod(int rate_hz) {
  if (rate_hz <= 0) {
    throw std::runtime_error("replay rate must be positive");
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / static_cast<double>(rate_hz)));
}

/**
 * @brief 二进制文件回放输入源
 * @note  按固定频率释放完整协议帧，适合作为主程序的真实输入替代。
 */
class ReplayInputSource {
 public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t kHeaderLen = 5;
  static constexpr std::size_t kCmdIdLen = 2;
  static constexpr std::size_t kCrc16Len = 2;
  static constexpr rm::u8 kFrameSof = 0xA5;

  /**
   * @brief 创建回放输入源
   * @param file_path 预制二进制文件路径
   * @param rate_hz 回放频率
   * @param name 输入源名称
   */
  ReplayInputSource(std::filesystem::path file_path, int rate_hz, std::string name)
      : file_path_(std::move(file_path)),
        rate_hz_(rate_hz),
        period_(ReplayRateToPeriod(rate_hz)),
        name_(std::move(name)) {
    OpenAndLoad();
  }

  /**
   * @brief 判断输入源是否仍有待回放帧
   * @return 仍有帧待释放时返回 true
   */
  bool active() const { return !completed_; }

  /**
   * @brief 判断输入源是否已经回放完成
   * @return 完成时返回 true
   */
  bool completed() const { return completed_; }

  /**
   * @brief 获取当前配置的回放频率
   * @return 频率 Hz
   */
  int rate_hz() const { return rate_hz_; }

  /**
   * @brief 获取回放文件路径
   * @return 文件路径
   */
  const std::filesystem::path &file_path() const { return file_path_; }

  /**
   * @brief 获取已回放字节数
   * @return 已释放的累计字节数
   */
  std::size_t played_bytes() const { return played_bytes_; }

  /**
   * @brief 获取已回放帧数
   * @return 已释放的累计帧数
   */
  std::size_t played_frames() const { return played_frames_; }

  /**
   * @brief 获取回放周期对应的毫秒数
   * @return 周期毫秒数，至少为 1
   */
  int period_ms() const {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(period_).count();
    return std::max(1, static_cast<int>(ms));
  }

  /**
   * @brief 获取最近一次释放帧的时间
   * @return 最近 tick 时间，未开始则为 `nullopt`
   */
  std::optional<Clock::time_point> last_tick_time() const { return last_tick_time_; }

  /**
   * @brief 获取距离下一次释放字节的等待时间
   * @param now 当前时间
   * @return 剩余等待毫秒数；完成后返回 `nullopt`
   */
  std::optional<int> TimeUntilNextTickMs(const Clock::time_point &now) const {
    if (completed_) {
      return std::nullopt;
    }

    const auto due_time = next_tick_time_.has_value() ? *next_tick_time_ : now;
    if (due_time <= now) {
      return 0;
    }

    const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(due_time - now).count();
    return static_cast<int>(std::max<long long>(0, wait_ms));
  }

  /**
   * @brief 推进一次回放
   * @param now 当前时间
   * @param on_bytes 取到到期帧后的回调
   * @return 本次是否实际释放了帧
   */
  template <typename Callback>
  bool Process(const Clock::time_point &now, Callback &&on_bytes) {
    if (completed_) {
      return false;
    }
    auto due_time = next_tick_time_.has_value() ? *next_tick_time_ : now;
    bool progressed = false;

    while (next_frame_index_ < frames_.size() && now >= due_time) {
      const auto &frame = frames_[next_frame_index_++];
      played_bytes_ += frame.size();
      ++played_frames_;
      last_tick_time_ = now;
      on_bytes(frame.data(), frame.size());
      progressed = true;
      due_time += period_;
    }

    next_tick_time_ = due_time;
    if (next_frame_index_ >= frames_.size()) {
      completed_ = true;
    }
    return progressed;
  }

 private:
  using Frame = std::vector<rm::u8>;

  /**
   * @brief 将原始字节流切分为完整协议帧
   * @param bytes 文件中读取的原始字节
   * @return 切分后的完整帧列表
   */
  std::vector<Frame> ParseFrames(const std::vector<rm::u8> &bytes) const {
    std::vector<Frame> frames;
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      if (bytes[offset] != kFrameSof) {
        ++offset;
        continue;
      }
      if (offset + kHeaderLen + kCmdIdLen + kCrc16Len > bytes.size()) {
        break;
      }

      if (rm::modules::Crc8(bytes.data() + offset, kHeaderLen - 1, rm::modules::CRC8_INIT) !=
          bytes[offset + kHeaderLen - 1]) {
        ++offset;
        continue;
      }

      const auto data_len = static_cast<std::size_t>(bytes[offset + 1]) |
                            (static_cast<std::size_t>(bytes[offset + 2]) << 8U);
      const auto frame_len = kHeaderLen + kCmdIdLen + data_len + kCrc16Len;
      if (offset + frame_len > bytes.size()) {
        ++offset;
        continue;
      }

      const auto frame_crc16 =
          static_cast<rm::u16>(bytes[offset + frame_len - 1] << 8U) | bytes[offset + frame_len - 2];
      if (rm::modules::Crc16(bytes.data() + offset, frame_len - kCrc16Len, rm::modules::CRC16_INIT) != frame_crc16) {
        ++offset;
        continue;
      }

      frames.emplace_back(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                          bytes.begin() + static_cast<std::ptrdiff_t>(offset + frame_len));
      offset += frame_len;
    }
    return frames;
  }

  void OpenAndLoad() {
    if (file_path_.empty()) {
      throw std::runtime_error(name_ + ": replay file path is empty");
    }

    std::ifstream stream(file_path_, std::ios::binary);
    if (!stream.is_open()) {
      throw std::runtime_error(name_ + ": failed to open replay file: " + file_path_.string());
    }

    stream.seekg(0, std::ios::end);
    const auto size = stream.tellg();
    if (size <= 0) {
      throw std::runtime_error(name_ + ": replay file is empty: " + file_path_.string());
    }
    stream.seekg(0, std::ios::beg);

    std::vector<rm::u8> bytes(static_cast<std::size_t>(size));
    stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (stream.gcount() != static_cast<std::streamsize>(bytes.size())) {
      throw std::runtime_error(name_ + ": failed to read full replay file: " + file_path_.string());
    }
    frames_ = ParseFrames(bytes);
    if (frames_.empty()) {
      throw std::runtime_error(name_ + ": replay file contains no complete referee frames: " + file_path_.string());
    }
  }

  std::filesystem::path file_path_;
  int rate_hz_ = 0;
  Clock::duration period_{};
  std::string name_;
  std::vector<Frame> frames_;
  std::size_t next_frame_index_ = 0;
  std::size_t played_bytes_ = 0;
  std::size_t played_frames_ = 0;
  bool completed_ = false;
  std::optional<Clock::time_point> next_tick_time_;
  std::optional<Clock::time_point> last_tick_time_;
};

}  // namespace radar::referee

#endif  // RADAR_INCLUDE_REFEREE_REPLAY_INPUT_SOURCE_HPP
