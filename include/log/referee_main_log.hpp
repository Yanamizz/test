#ifndef RADAR_SRC_REFEREE_MAIN_LOG_HPP
#define RADAR_SRC_REFEREE_MAIN_LOG_HPP

/**
 * @file  include/log/referee_main_log.hpp
 * @brief 裁判系统主协议结构体日志格式化工具
 */

#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "include/log/log_backend.hpp"
#include "librm/device/referee/referee.hpp"

#ifndef RADAR_DEFAULT_LOG_DIR
#define RADAR_DEFAULT_LOG_DIR "test/logs_runtime"
#endif

namespace radar::log {

/**
 * @brief 当前主协议日志器支持的协议版本
 */
template <rm::device::RefereeRevision revision>
constexpr bool kSupportedMainLogRevision =
    revision == rm::device::RefereeRevision::kNewV120 || revision == rm::device::RefereeRevision::kNewV200;

/**
 * @brief 生成当前时间戳字符串
 * @return `YYYY-MM-DD HH:MM:SS.mmm` 格式时间
 */
inline std::string TimestampNow() {
  using namespace std::chrono;

  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  const auto time = system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream oss;
  oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

/**
 * @brief 生成本轮日志目录名
 * @return `YYYYMMDD_HHMMSS_mmm` 格式目录名
 */
inline std::string RunDirectoryNameNow() {
  using namespace std::chrono;

  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  const auto time = system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream oss;
  oss << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

/**
 * @brief 创建一次运行对应的独立日志目录
 * @param base_dir 日志根目录
 * @return 本轮运行专属目录路径
 */
inline std::filesystem::path CreateRunLogRoot(const std::filesystem::path &base_dir = RADAR_DEFAULT_LOG_DIR) {
  std::filesystem::create_directories(base_dir);

  const auto base_name = RunDirectoryNameNow();
  for (int suffix = 0; suffix < 1000; ++suffix) {
    std::filesystem::path candidate = base_dir / base_name;
    if (suffix != 0) {
      candidate = base_dir / (base_name + "_" + std::to_string(suffix));
    }
    if (std::filesystem::create_directory(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error("failed to create unique run log directory under: " + base_dir.string());
}

/**
 * @brief 按整数自身位宽格式化为带 `0x` 前缀的十六进制字符串
 * @tparam T 非布尔整数类型
 * @param value 待格式化值
 * @return 十六进制字符串
 */
template <typename T>
inline std::string HexInteger(T value) {
  static_assert(std::is_integral_v<T> && !std::is_same_v<std::remove_cv_t<T>, bool>,
                "HexInteger requires a non-bool integral type");
  static constexpr char kHexDigits[] = "0123456789abcdef";
  using Value = std::remove_cv_t<T>;
  using UnsignedValue = std::make_unsigned_t<Value>;
  constexpr std::size_t kDigitCount = sizeof(Value) * 2;

  const auto unsigned_value = static_cast<UnsignedValue>(value);
  std::string out(kDigitCount + 2, '0');
  out[1] = 'x';
  for (std::size_t index = 0; index < kDigitCount; ++index) {
    const auto shift = static_cast<unsigned>((kDigitCount - index - 1) * 4);
    out[index + 2] = kHexDigits[(unsigned_value >> shift) & 0xF];
  }
  return out;
}

/**
 * @brief 以 `0xNN` 形式格式化 8 位值
 * @param value 待格式化值
 * @return 十六进制字符串
 */
inline std::string HexU8(rm::u8 value) { return HexInteger(value); }

/**
 * @brief 以 `0xNNNN` 形式格式化 16 位值
 * @param value 待格式化值
 * @return 十六进制字符串
 */
inline std::string HexU16(rm::u16 value) { return HexInteger(value); }

/**
 * @brief 将字节数组格式化为连续十六进制字符串
 * @param data 字节数组首地址
 * @param size 字节数
 * @return 十六进制字符串
 */
inline std::string HexBytes(const rm::u8 *data, std::size_t size) {
  // 手写格式化，输出与逐字节 `std::hex/setw(2)/setfill('0')` 完全一致，但避开 ostringstream 开销。
  static constexpr char kHexDigits[] = "0123456789abcdef";
  std::string out(size * 2, '0');
  for (std::size_t i = 0; i < size; ++i) {
    out[2 * i] = kHexDigits[(data[i] >> 4) & 0xF];
    out[2 * i + 1] = kHexDigits[data[i] & 0xF];
  }
  return out;
}

/**
 * @brief 将标量格式化为 JSON 可写字符串
 * @note 协议中的整数按固定字宽输出为带 `0x` 前缀的 JSON 字符串；浮点数和布尔值保持 JSON 标量。
 * @tparam T 标量类型
 * @param value 待格式化值
 * @return JSON 片段
 */
template <typename T>
inline std::string JsonScalar(T value) {
  std::ostringstream oss;
  if constexpr (std::is_floating_point_v<T>) {
    oss << std::fixed << std::setprecision(4) << value;
  } else if constexpr (std::is_same_v<std::remove_cv_t<T>, bool>) {
    oss << (value ? "true" : "false");
  } else if constexpr (std::is_integral_v<T>) {
    oss << '"' << HexInteger(value) << '"';
  } else {
    oss << value;
  }
  return oss.str();
}

/**
 * @brief 将数组格式化为 JSON 数组
 * @tparam T 数组元素类型
 * @param data 数组首地址
 * @param size 元素个数
 * @return JSON 数组字符串
 */
template <typename T>
inline std::string JsonArray(const T *data, std::size_t size) {
  std::ostringstream oss;
  oss << '[';
  for (std::size_t i = 0; i < size; ++i) {
    if (i != 0) {
      oss << ',';
    }
    oss << JsonScalar(data[i]);
  }
  oss << ']';
  return oss.str();
}

/**
 * @brief 根据主命令码返回结构体名称
 * @tparam revision 当前裁判协议版本
 * @param cmd_id 主命令码
 * @return 对应日志名称
 */
template <rm::device::RefereeRevision revision>
inline std::string MainCmdName(rm::u16 cmd_id) {
  static_assert(kSupportedMainLogRevision<revision>, "RefereeMainLogger only supports kNewV120 and kNewV200");
  using Cmd = rm::device::RefereeCmdId<revision>;

  switch (cmd_id) {
    case Cmd::kGameStatus:
      return "game_status";
    case Cmd::kGameResult:
      return "game_result";
    case Cmd::kGameRobotHp:
      return "game_robot_HP";
    case Cmd::kEventData:
      return "event_data";
    case Cmd::kRefereeWarning:
      return "referee_warning";
    case Cmd::kDartInformation:
      return "dart_info";
    case Cmd::kRobotStatus:
      return "robot_status";
    case Cmd::kPowerHeatData:
      return "power_heat_data";
    case Cmd::kRobotPos:
      return "robot_pos";
    case Cmd::kBuff:
      return "buff";
    case Cmd::kHurtData:
      return "hurt_data";
    case Cmd::kShootData:
      return "shoot_data";
    case Cmd::kProjectileAllowance:
      return "projectile_allowance";
    case Cmd::kRfidStatus:
      return "rfid_status";
    case Cmd::kDartClientCmd:
      return "dart_client_cmd";
    case Cmd::kGroundRobotPosition:
      return "ground_robot_position";
    case Cmd::kRadarMarkData:
      return "radar_mark_data";
    case Cmd::kSentryInfo:
      return "sentry_info";
    case Cmd::kRadarInfo:
      return "radar_info";
    case Cmd::kRobotInteractionData:
      return "robot_interaction_data";
    case Cmd::kCustomRobotData:
      return "custom_robot_data";
    case Cmd::kMapCommand:
      return "map_command";
    case Cmd::kMapRobotData:
      return "map_robot_data";
    case Cmd::kCustomClientData:
      return "custom_client_data";
    case Cmd::kMapData:
      return "map_data";
    case Cmd::kCustomInfo:
      return "custom_info";
    case Cmd::kRobotCustomData:
      return "robot_custom_data";
    case Cmd::kRobotCustomData2:
      return "robot_custom_data_2";
    case Cmd::kRobotCustomData3:
      return "robot_custom_data_3";
    case Cmd::kRadar0:
      return "radar0";
    case Cmd::kRadar1:
      return "radar1";
    case Cmd::kRadar2:
      return "radar2";
    case Cmd::kRadar3:
      return "radar3";
    case Cmd::kRadar4:
      return "radar4";
    case Cmd::kRadar5:
      return "radar5";
    default:
      return "unknown_main_cmd";
  }
}

/**
 * @brief 将当前维护的主协议结构体序列化为 JSON
 * @tparam revision 当前裁判协议版本
 * @param cmd_id 主命令码
 * @param protocol 当前维护的主协议状态
 * @return 对应结构体的 JSON 片段
 */
template <rm::device::RefereeRevision revision>
inline std::string FormatMainPayload(rm::u16 cmd_id, const rm::device::RefereeProtocol<revision> &protocol) {
  static_assert(kSupportedMainLogRevision<revision>, "RefereeMainLogger only supports kNewV120 and kNewV200");
  using Cmd = rm::device::RefereeCmdId<revision>;
  std::ostringstream oss;

  switch (cmd_id) {
    case Cmd::kGameStatus: {
      const auto &data = protocol.game_status;
      oss << "{"
          << "\"game_type\":" << JsonScalar(data.game_type) << ','
          << "\"game_progress\":" << JsonScalar(data.game_progress) << ','
          << "\"stage_remain_time\":" << JsonScalar(data.stage_remain_time) << ','
          << "\"SyncTimeStamp\":" << JsonScalar(data.SyncTimeStamp) << "}";
      return oss.str();
    }
    case Cmd::kGameResult: {
      const auto &data = protocol.game_result;
      oss << "{\"winner\":" << JsonScalar(data.winner) << "}";
      return oss.str();
    }
    case Cmd::kGameRobotHp: {
      const auto &data = protocol.game_robot_HP;
      oss << "{"
          << "\"ally_1_robot_HP\":" << JsonScalar(data.ally_1_robot_HP) << ','
          << "\"ally_2_robot_HP\":" << JsonScalar(data.ally_2_robot_HP) << ','
          << "\"ally_3_robot_HP\":" << JsonScalar(data.ally_3_robot_HP) << ','
          << "\"ally_4_robot_HP\":" << JsonScalar(data.ally_4_robot_HP) << ',';
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"damage_difference\":" << JsonScalar(data.damage_difference) << ','
            << "\"ally_7_robot_HP\":" << JsonScalar(data.ally_7_robot_HP) << ','
            << "\"ally_outpost_HP\":" << JsonScalar(data.ally_outpost_HP) << ','
            << "\"ally_base_HP\":" << JsonScalar(data.ally_base_HP) << ','
            << "\"enemy_outpost_HP\":" << JsonScalar(data.enemy_outpost_HP) << ','
            << "\"enemy_base_HP\":" << JsonScalar(data.enemy_base_HP) << "}";
      } else {
        oss << "\"reserved\":" << JsonScalar(data.reserved) << ','
            << "\"ally_7_robot_HP\":" << JsonScalar(data.ally_7_robot_HP) << ','
            << "\"ally_outpost_HP\":" << JsonScalar(data.ally_outpost_HP) << ','
            << "\"ally_base_HP\":" << JsonScalar(data.ally_base_HP) << "}";
      }
      return oss.str();
    }
    case Cmd::kEventData: {
      const auto &data = protocol.event_data;
      oss << "{\"event_data\":" << JsonScalar(data.event_data) << "}";
      return oss.str();
    }
    case Cmd::kRefereeWarning: {
      const auto &data = protocol.referee_warning;
      oss << "{"
          << "\"level\":" << JsonScalar(data.level) << ','
          << "\"offending_robot_id\":" << JsonScalar(data.offending_robot_id) << ','
          << "\"count\":" << JsonScalar(data.count) << "}";
      return oss.str();
    }
    case Cmd::kDartInformation: {
      const auto &data = protocol.dart_info;
      oss << "{"
          << "\"dart_remaining_time\":" << JsonScalar(data.dart_remaining_time) << ','
          << "\"dart_info\":" << JsonScalar(data.dart_info) << "}";
      return oss.str();
    }
    case Cmd::kRobotStatus: {
      const auto &data = protocol.robot_status;
      oss << "{"
          << "\"robot_id\":" << JsonScalar(data.robot_id) << ','
          << "\"robot_level\":" << JsonScalar(data.robot_level) << ','
          << "\"current_HP\":" << JsonScalar(data.current_HP) << ','
          << "\"maximum_HP\":" << JsonScalar(data.maximum_HP) << ','
          << "\"shooter_barrel_cooling_value\":" << JsonScalar(data.shooter_barrel_cooling_value) << ','
          << "\"shooter_barrel_heat_limit\":" << JsonScalar(data.shooter_barrel_heat_limit) << ','
          << "\"chassis_power_limit\":" << JsonScalar(data.chassis_power_limit) << ',';
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"bullet_speed_limit\":" << JsonScalar(data.bullet_speed_limit) << ',';
      }
      oss << "\"power_management_gimbal_output\":" << JsonScalar(data.power_management_gimbal_output) << ','
          << "\"power_management_chassis_output\":" << JsonScalar(data.power_management_chassis_output) << ','
          << "\"power_management_shooter_output\":" << JsonScalar(data.power_management_shooter_output) << "}";
      return oss.str();
    }
    case Cmd::kPowerHeatData: {
      const auto &data = protocol.power_heat_data;
      oss << "{"
          << "\"reserved\":" << JsonScalar(data.reserved) << ','
          << "\"reserved_2\":" << JsonScalar(data.reserved_2) << ','
          << "\"reserved_3\":" << JsonScalar(data.reserved_3) << ','
          << "\"buffer_energy\":" << JsonScalar(data.buffer_energy) << ','
          << "\"shooter_17mm_1_barrel_heat\":" << JsonScalar(data.shooter_17mm_1_barrel_heat) << ','
          << "\"shooter_42mm_barrel_heat\":" << JsonScalar(data.shooter_42mm_barrel_heat) << "}";
      return oss.str();
    }
    case Cmd::kRobotPos: {
      const auto &data = protocol.robot_pos;
      oss << "{"
          << "\"x\":" << JsonScalar(data.x) << ','
          << "\"y\":" << JsonScalar(data.y) << ','
          << "\"angle\":" << JsonScalar(data.angle) << "}";
      return oss.str();
    }
    case Cmd::kBuff: {
      const auto &data = protocol.buff;
      oss << "{"
          << "\"recovery_buff\":" << JsonScalar(data.recovery_buff) << ','
          << "\"cooling_buff\":" << JsonScalar(data.cooling_buff) << ','
          << "\"defence_buff\":" << JsonScalar(data.defence_buff) << ','
          << "\"vulnerability_buff\":" << JsonScalar(data.vulnerability_buff) << ','
          << "\"attack_buff\":" << JsonScalar(data.attack_buff) << ','
          << "\"remaining_energy\":" << JsonScalar(data.remaining_energy) << "}";
      return oss.str();
    }
    case Cmd::kHurtData: {
      const auto &data = protocol.hurt_data;
      oss << "{"
          << "\"armor_id\":" << JsonScalar(data.armor_id) << ','
          << "\"HP_deduction_reason\":" << JsonScalar(data.HP_deduction_reason) << "}";
      return oss.str();
    }
    case Cmd::kShootData: {
      const auto &data = protocol.shoot_data;
      oss << "{"
          << "\"bullet_type\":" << JsonScalar(data.bullet_type) << ','
          << "\"shooter_number\":" << JsonScalar(data.shooter_number) << ','
          << "\"launching_frequency\":" << JsonScalar(data.launching_frequency) << ','
          << "\"initial_speed\":" << JsonScalar(data.initial_speed) << "}";
      return oss.str();
    }
    case Cmd::kProjectileAllowance: {
      const auto &data = protocol.projectile_allowance;
      oss << "{"
          << "\"projectile_allowance_17mm\":" << JsonScalar(data.projectile_allowance_17mm) << ','
          << "\"projectile_allowance_42mm\":" << JsonScalar(data.projectile_allowance_42mm) << ','
          << "\"remaining_gold_coin\":" << JsonScalar(data.remaining_gold_coin) << ','
          << "\"projectile_allowance_fortress\":" << JsonScalar(data.projectile_allowance_fortress) << "}";
      return oss.str();
    }
    case Cmd::kRfidStatus: {
      const auto &data = protocol.rfid_status;
      oss << "{"
          << "\"rfid_status\":" << JsonScalar(data.rfid_status) << ','
          << "\"rfid_status_2\":" << JsonScalar(data.rfid_status_2) << "}";
      return oss.str();
    }
    case Cmd::kDartClientCmd: {
      const auto &data = protocol.dart_client_cmd;
      oss << "{"
          << "\"dart_launch_opening_status\":" << JsonScalar(data.dart_launch_opening_status) << ','
          << "\"reserved\":" << JsonScalar(data.reserved) << ','
          << "\"target_change_time\":" << JsonScalar(data.target_change_time) << ','
          << "\"latest_launch_cmd_time\":" << JsonScalar(data.latest_launch_cmd_time) << "}";
      return oss.str();
    }
    case Cmd::kGroundRobotPosition: {
      const auto &data = protocol.ground_robot_position;
      oss << "{"
          << "\"hero_x\":" << JsonScalar(data.hero_x) << ','
          << "\"hero_y\":" << JsonScalar(data.hero_y) << ','
          << "\"engineer_x\":" << JsonScalar(data.engineer_x) << ','
          << "\"engineer_y\":" << JsonScalar(data.engineer_y) << ','
          << "\"standard_3_x\":" << JsonScalar(data.standard_3_x) << ','
          << "\"standard_3_y\":" << JsonScalar(data.standard_3_y) << ','
          << "\"standard_4_x\":" << JsonScalar(data.standard_4_x) << ','
          << "\"standard_4_y\":" << JsonScalar(data.standard_4_y) << ','
          << "\"reserved\":" << JsonScalar(data.reserved) << ','
          << "\"reserved_2\":" << JsonScalar(data.reserved_2) << "}";
      return oss.str();
    }
    case Cmd::kRadarMarkData: {
      const auto &data = protocol.radar_mark_data;
      constexpr rm::u16 kOpponentAerialRobotTargetedByAllyRadarLaserMask = static_cast<rm::u16>(1u << 12);
      constexpr rm::u16 kOpponentAerialRobotCounteredMask = static_cast<rm::u16>(1u << 13);
      oss << "{"
          << "\"opponent_aerial_robot_targeted_by_ally_radar_laser\":"
          << JsonScalar((data.mark_progress & kOpponentAerialRobotTargetedByAllyRadarLaserMask) != 0) << ','
          << "\"opponent_aerial_robot_countered\":"
          << JsonScalar((data.mark_progress & kOpponentAerialRobotCounteredMask) != 0) << "}";
      return oss.str();
    }
    case Cmd::kSentryInfo: {
      const auto &data = protocol.sentry_info;
      oss << "{"
          << "\"sentry_info\":" << JsonScalar(data.sentry_info) << ','
          << "\"sentry_info_2\":" << JsonScalar(data.sentry_info_2);
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << ",\"sentry_info_3\":" << JsonScalar(data.sentry_info_3);
      }
      oss << "}";
      return oss.str();
    }
    case Cmd::kRadarInfo: {
      const auto &data = protocol.radar_info;
      oss << "{\"radar_info\":" << JsonScalar(data.radar_info) << "}";
      return oss.str();
    }
    case Cmd::kRobotInteractionData: {
      const auto &data = protocol.robot_interaction_data;
      oss << "{"
          << "\"data_cmd_id\":\"" << HexU16(data.data_cmd_id) << "\","
          << "\"sender_id\":" << JsonScalar(data.sender_id) << ','
          << "\"receiver_id\":" << JsonScalar(data.receiver_id) << ','
          << "\"user_data\":\"" << HexBytes(data.user_data, sizeof(data.user_data)) << "\""
          << "}";
      return oss.str();
    }
    case Cmd::kCustomRobotData: {
      const auto &data = protocol.custom_robot_data;
      oss << "{\"data\":\"" << HexBytes(data.data, sizeof(data.data)) << "\"}";
      return oss.str();
    }
    case Cmd::kMapCommand: {
      const auto &data = protocol.map_command;
      oss << "{"
          << "\"target_position_x\":" << JsonScalar(data.target_position_x) << ','
          << "\"target_position_y\":" << JsonScalar(data.target_position_y) << ','
          << "\"cmd_keyboard\":" << JsonScalar(data.cmd_keyboard) << ','
          << "\"target_robot_id\":" << JsonScalar(data.target_robot_id) << ','
          << "\"cmd_source\":" << JsonScalar(data.cmd_source) << "}";
      return oss.str();
    }
    case Cmd::kMapRobotData: {
      const auto &data = protocol.map_robot_data;
      oss << "{";
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"opponent_hero_position_x\":" << JsonScalar(data.opponent_hero_position_x) << ','
            << "\"opponent_hero_position_y\":" << JsonScalar(data.opponent_hero_position_y) << ','
            << "\"opponent_engineer_position_x\":" << JsonScalar(data.opponent_engineer_position_x) << ','
            << "\"opponent_engineer_position_y\":" << JsonScalar(data.opponent_engineer_position_y) << ','
            << "\"opponent_infantry_3_position_x\":" << JsonScalar(data.opponent_infantry_3_position_x) << ','
            << "\"opponent_infantry_3_position_y\":" << JsonScalar(data.opponent_infantry_3_position_y) << ','
            << "\"opponent_infantry_4_position_x\":" << JsonScalar(data.opponent_infantry_4_position_x) << ','
            << "\"opponent_infantry_4_position_y\":" << JsonScalar(data.opponent_infantry_4_position_y) << ','
            << "\"opponent_aerial_position_x\":" << JsonScalar(data.opponent_aerial_position_x) << ','
            << "\"opponent_aerial_position_y\":" << JsonScalar(data.opponent_aerial_position_y) << ','
            << "\"opponent_sentry_position_x\":" << JsonScalar(data.opponent_sentry_position_x) << ','
            << "\"opponent_sentry_position_y\":" << JsonScalar(data.opponent_sentry_position_y) << ','
            << "\"ally_hero_position_x\":" << JsonScalar(data.ally_hero_position_x) << ','
            << "\"ally_hero_position_y\":" << JsonScalar(data.ally_hero_position_y) << ','
            << "\"ally_engineer_position_x\":" << JsonScalar(data.ally_engineer_position_x) << ','
            << "\"ally_engineer_position_y\":" << JsonScalar(data.ally_engineer_position_y) << ','
            << "\"ally_infantry_3_position_x\":" << JsonScalar(data.ally_infantry_3_position_x) << ','
            << "\"ally_infantry_3_position_y\":" << JsonScalar(data.ally_infantry_3_position_y) << ','
            << "\"ally_infantry_4_position_x\":" << JsonScalar(data.ally_infantry_4_position_x) << ','
            << "\"ally_infantry_4_position_y\":" << JsonScalar(data.ally_infantry_4_position_y) << ','
            << "\"ally_aerial_position_x\":" << JsonScalar(data.ally_aerial_position_x) << ','
            << "\"ally_aerial_position_y\":" << JsonScalar(data.ally_aerial_position_y) << ','
            << "\"ally_sentry_position_x\":" << JsonScalar(data.ally_sentry_position_x) << ','
            << "\"ally_sentry_position_y\":" << JsonScalar(data.ally_sentry_position_y) << "}";
      } else {
        oss << "\"hero_position_x\":" << JsonScalar(data.hero_position_x) << ','
            << "\"hero_position_y\":" << JsonScalar(data.hero_position_y) << ','
            << "\"engineer_position_x\":" << JsonScalar(data.engineer_position_x) << ','
            << "\"engineer_position_y\":" << JsonScalar(data.engineer_position_y) << ','
            << "\"infantry_3_position_x\":" << JsonScalar(data.infantry_3_position_x) << ','
            << "\"infantry_3_position_y\":" << JsonScalar(data.infantry_3_position_y) << ','
            << "\"infantry_4_position_x\":" << JsonScalar(data.infantry_4_position_x) << ','
            << "\"infantry_4_position_y\":" << JsonScalar(data.infantry_4_position_y) << ','
            << "\"reserved\":" << JsonScalar(data.reserved) << ','
            << "\"reserved_2\":" << JsonScalar(data.reserved_2) << ','
            << "\"sentry_position_x\":" << JsonScalar(data.sentry_position_x) << ','
            << "\"sentry_position_y\":" << JsonScalar(data.sentry_position_y) << "}";
      }
      return oss.str();
    }
    case Cmd::kCustomClientData: {
      const auto &data = protocol.custom_client_data;
      oss << "{"
          << "\"key_value\":" << JsonScalar(data.key_value) << ','
          << "\"x_position\":" << JsonScalar(data.x_position) << ','
          << "\"mouse_left\":" << JsonScalar(data.mouse_left) << ','
          << "\"y_position\":" << JsonScalar(data.y_position) << ','
          << "\"mouse_right\":" << JsonScalar(data.mouse_right) << ','
          << "\"reserved\":" << JsonScalar(data.reserved) << "}";
      return oss.str();
    }
    case Cmd::kMapData: {
      const auto &data = protocol.map_data;
      oss << "{"
          << "\"intention\":" << JsonScalar(data.intention) << ','
          << "\"start_position_x\":" << JsonScalar(data.start_position_x) << ','
          << "\"start_position_y\":" << JsonScalar(data.start_position_y) << ','
          << "\"delta_x\":" << JsonArray(data.delta_x, 49) << ','
          << "\"delta_y\":" << JsonArray(data.delta_y, 49) << ','
          << "\"sender_id\":" << JsonScalar(data.sender_id) << "}";
      return oss.str();
    }
    case Cmd::kCustomInfo: {
      const auto &data = protocol.custom_info;
      oss << "{"
          << "\"sender_id\":" << JsonScalar(data.sender_id) << ','
          << "\"receiver_id\":" << JsonScalar(data.receiver_id) << ','
          << "\"user_data\":\"" << HexBytes(data.user_data, sizeof(data.user_data)) << "\""
          << "}";
      return oss.str();
    }
    case Cmd::kRobotCustomData: {
      const auto &data = protocol.robot_custom_data;
      oss << "{\"data\":\"" << HexBytes(data.data, sizeof(data.data)) << "\"}";
      return oss.str();
    }
    case Cmd::kRobotCustomData2: {
      const auto &data = protocol.robot_custom_data_2;
      oss << "{\"data\":\"" << HexBytes(data.data, sizeof(data.data)) << "\"}";
      return oss.str();
    }
    case Cmd::kRobotCustomData3: {
      const auto &data = protocol.robot_custom_data_3;
      oss << "{\"data\":\"" << HexBytes(data.data, sizeof(data.data)) << "\"}";
      return oss.str();
    }
    case Cmd::kRadar0: {
      const auto &data = protocol.radar0;
      oss << "{";
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"opponent_hero_position_x\":" << JsonScalar(data.opponent_hero_position_x) << ','
            << "\"opponent_hero_position_y\":" << JsonScalar(data.opponent_hero_position_y) << ','
            << "\"opponent_engineer_position_x\":" << JsonScalar(data.opponent_engineer_position_x) << ','
            << "\"opponent_engineer_position_y\":" << JsonScalar(data.opponent_engineer_position_y) << ','
            << "\"opponent_infantry_3_position_x\":" << JsonScalar(data.opponent_infantry_3_position_x) << ','
            << "\"opponent_infantry_3_position_y\":" << JsonScalar(data.opponent_infantry_3_position_y) << ','
            << "\"opponent_infantry_4_position_x\":" << JsonScalar(data.opponent_infantry_4_position_x) << ','
            << "\"opponent_infantry_4_position_y\":" << JsonScalar(data.opponent_infantry_4_position_y) << ','
            << "\"opponent_aerial_position_x\":" << JsonScalar(data.opponent_aerial_position_x) << ','
            << "\"opponent_aerial_position_y\":" << JsonScalar(data.opponent_aerial_position_y) << ','
            << "\"opponent_sentry_position_x\":" << JsonScalar(data.opponent_sentry_position_x) << ','
            << "\"opponent_sentry_position_y\":" << JsonScalar(data.opponent_sentry_position_y) << "}";
      } else {
        oss << "\"opponent_hero_position_x\":" << JsonScalar(data.opponent_hero_position_x) << ','
            << "\"opponent_hero_position_y\":" << JsonScalar(data.opponent_hero_position_y) << ','
            << "\"opponent_engineer_position_x\":" << JsonScalar(data.opponent_engineer_position_x) << ','
            << "\"opponent_engineer_position_y\":" << JsonScalar(data.opponent_engineer_position_y) << ','
            << "\"opponent_infantry_3_position_x\":" << JsonScalar(data.opponent_infantry_3_position_x) << ','
            << "\"opponent_infantry_3_position_y\":" << JsonScalar(data.opponent_infantry_3_position_y) << ','
            << "\"opponent_infantry_4_position_x\":" << JsonScalar(data.opponent_infantry_4_position_x) << ','
            << "\"opponent_infantry_4_position_y\":" << JsonScalar(data.opponent_infantry_4_position_y) << ','
            << "\"opponent_aerial_position_x\":" << JsonScalar(data.opponent_aerial_position_x) << ','
            << "\"opponent_aerial_position_y\":" << JsonScalar(data.opponent_aerial_position_y) << ','
            << "\"opponent_sentry_position_x\":" << JsonScalar(data.opponent_sentry_position_x) << ','
            << "\"opponent_sentry_position_y\":" << JsonScalar(data.opponent_sentry_position_y) << "}";
      }
      return oss.str();
    }
    case Cmd::kRadar1: {
      const auto &data = protocol.radar1;
      oss << "{";
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"opponent_hero_HP\":" << JsonScalar(data.opponent_hero_HP) << ','
            << "\"opponent_engineer_HP\":" << JsonScalar(data.opponent_engineer_HP) << ','
            << "\"opponent_infantry_3_HP\":" << JsonScalar(data.opponent_infantry_3_HP) << ','
            << "\"opponent_infantry_4_HP\":" << JsonScalar(data.opponent_infantry_4_HP) << ','
            << "\"reserved\":" << JsonScalar(data.reserved) << ','
            << "\"opponent_sentry_HP\":" << JsonScalar(data.opponent_sentry_HP) << "}";
      } else {
        oss << "\"opponent_hero_HP\":" << JsonScalar(data.opponent_hero_HP) << ','
            << "\"opponent_engineer_HP\":" << JsonScalar(data.opponent_engineer_HP) << ','
            << "\"opponent_infantry_3_HP\":" << JsonScalar(data.opponent_infantry_3_HP) << ','
            << "\"opponent_infantry_4_HP\":" << JsonScalar(data.opponent_infantry_4_HP) << ','
            << "\"reserved\":" << JsonScalar(data.reserved) << ','
            << "\"opponent_sentry_HP\":" << JsonScalar(data.opponent_sentry_HP) << "}";
      }
      return oss.str();
    }
    case Cmd::kRadar2: {
      const auto &data = protocol.radar2;
      oss << "{";
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "\"opponent_hero_allowance_ammo\":" << JsonScalar(data.opponent_hero_allowance_ammo) << ','
            << "\"opponent_infantry_3_allowance_ammo\":" << JsonScalar(data.opponent_infantry_3_allowance_ammo) << ','
            << "\"opponent_infantry_4_allowance_ammo\":" << JsonScalar(data.opponent_infantry_4_allowance_ammo) << ','
            << "\"opponent_aerial_allowance_ammo\":" << JsonScalar(data.opponent_aerial_allowance_ammo) << ','
            << "\"opponent_sentry_allowance_ammo\":" << JsonScalar(data.opponent_sentry_allowance_ammo) << "}";
      } else {
        oss << "\"opponent_hero_allowance_ammo\":" << JsonScalar(data.opponent_hero_allowance_ammo) << ','
            << "\"opponent_infantry_3_allowance_ammo\":" << JsonScalar(data.opponent_infantry_3_allowance_ammo) << ','
            << "\"opponent_infantry_4_allowance_ammo\":" << JsonScalar(data.opponent_infantry_4_allowance_ammo) << ','
            << "\"opponent_drone_allowance_ammo\":" << JsonScalar(data.opponent_drone_allowance_ammo) << ','
            << "\"opponent_sentry_allowance_ammo\":" << JsonScalar(data.opponent_sentry_allowance_ammo) << "}";
      }
      return oss.str();
    }
    case Cmd::kRadar3: {
      const auto &data = protocol.radar3;
      oss << "{"
          << "\"enemy_remaining_gold_coin\":" << JsonScalar(data.enemy_remaining_gold_coin) << ','
          << "\"enemy_total_gold_coin\":" << JsonScalar(data.enemy_total_gold_coin) << ','
          << "\"enemy_status\":" << JsonScalar(data.enemy_status) << "}";
      return oss.str();
    }
    case Cmd::kRadar4: {
      const auto &data = protocol.radar4;
      if constexpr (revision == rm::device::RefereeRevision::kNewV200) {
        oss << "{"
            << "\"enemy_hero_hp_buff\":" << JsonScalar(data.enemy_hero_hp_buff) << ','
            << "\"enemy_hero_cooling_buff\":" << JsonScalar(data.enemy_hero_cooling_buff) << ','
            << "\"enemy_hero_defence_buff\":" << JsonScalar(data.enemy_hero_defence_buff) << ','
            << "\"enemy_hero_vulnerability_buff\":" << JsonScalar(data.enemy_hero_vulnerability_buff) << ','
            << "\"enemy_hero_attack_buff\":" << JsonScalar(data.enemy_hero_attack_buff) << ','
            << "\"enemy_engineer_hp_buff\":" << JsonScalar(data.enemy_engineer_hp_buff) << ','
            << "\"enemy_engineer_cooling_buff\":" << JsonScalar(data.enemy_engineer_cooling_buff) << ','
            << "\"enemy_engineer_defence_buff\":" << JsonScalar(data.enemy_engineer_defence_buff) << ','
            << "\"enemy_engineer_vulnerability_buff\":" << JsonScalar(data.enemy_engineer_vulnerability_buff) << ','
            << "\"enemy_engineer_attack_buff\":" << JsonScalar(data.enemy_engineer_attack_buff) << ','
            << "\"enemy_infantry_3_hp_buff\":" << JsonScalar(data.enemy_infantry_3_hp_buff) << ','
            << "\"enemy_infantry_3_cooling_buff\":" << JsonScalar(data.enemy_infantry_3_cooling_buff) << ','
            << "\"enemy_infantry_3_defence_buff\":" << JsonScalar(data.enemy_infantry_3_defence_buff) << ','
            << "\"enemy_infantry_3_vulnerability_buff\":" << JsonScalar(data.enemy_infantry_3_vulnerability_buff) << ','
            << "\"enemy_infantry_3_attack_buff\":" << JsonScalar(data.enemy_infantry_3_attack_buff) << ','
            << "\"enemy_infantry_4_hp_buff\":" << JsonScalar(data.enemy_infantry_4_hp_buff) << ','
            << "\"enemy_infantry_4_cooling_buff\":" << JsonScalar(data.enemy_infantry_4_cooling_buff) << ','
            << "\"enemy_infantry_4_defence_buff\":" << JsonScalar(data.enemy_infantry_4_defence_buff) << ','
            << "\"enemy_infantry_4_vulnerability_buff\":" << JsonScalar(data.enemy_infantry_4_vulnerability_buff) << ','
            << "\"enemy_infantry_4_attack_buff\":" << JsonScalar(data.enemy_infantry_4_attack_buff) << ','
            << "\"enemy_sentry_hp_buff\":" << JsonScalar(data.enemy_sentry_hp_buff) << ','
            << "\"enemy_sentry_cooling_buff\":" << JsonScalar(data.enemy_sentry_cooling_buff) << ','
            << "\"enemy_sentry_defence_buff\":" << JsonScalar(data.enemy_sentry_defence_buff) << ','
            << "\"enemy_sentry_vulnerability_buff\":" << JsonScalar(data.enemy_sentry_vulnerability_buff) << ','
            << "\"enemy_sentry_attack_buff\":" << JsonScalar(data.enemy_sentry_attack_buff) << ','
            << "\"enemy_sentry_posture\":" << JsonScalar(data.enemy_sentry_posture) << ','
            << "\"enemy_hero_status\":" << JsonScalar(data.enemy_hero_status) << ','
            << "\"enemy_engineer_status\":" << JsonScalar(data.enemy_engineer_status) << ','
            << "\"enemy_infantry_3_status\":" << JsonScalar(data.enemy_infantry_3_status) << ','
            << "\"enemy_infantry_4_status\":" << JsonScalar(data.enemy_infantry_4_status) << ','
            << "\"enemy_sentry_status\":" << JsonScalar(data.enemy_sentry_status);
      } else {
        oss << "{"
            << "\"opponent_hero_hp_buff\":" << JsonScalar(data.opponent_hero_hp_buff) << ','
            << "\"opponent_hero_cooling_buff\":" << JsonScalar(data.opponent_hero_cooling_buff) << ','
            << "\"opponent_hero_defence_buff\":" << JsonScalar(data.opponent_hero_defence_buff) << ','
            << "\"opponent_hero_vulnerability_buff\":" << JsonScalar(data.opponent_hero_vulnerability_buff) << ','
            << "\"opponent_hero_attack_buff\":" << JsonScalar(data.opponent_hero_attack_buff) << ','
            << "\"opponent_engineer_hp_buff\":" << JsonScalar(data.opponent_engineer_hp_buff) << ','
            << "\"opponent_engineer_cooling_buff\":" << JsonScalar(data.opponent_engineer_cooling_buff) << ','
            << "\"opponent_engineer_defence_buff\":" << JsonScalar(data.opponent_engineer_defence_buff) << ','
            << "\"opponent_engineer_vulnerability_buff\":" << JsonScalar(data.opponent_engineer_vulnerability_buff) << ','
            << "\"opponent_engineer_attack_buff\":" << JsonScalar(data.opponent_engineer_attack_buff) << ','
            << "\"opponent_infantry_3_hp_buff\":" << JsonScalar(data.opponent_infantry_3_hp_buff) << ','
            << "\"opponent_infantry_3_cooling_buff\":" << JsonScalar(data.opponent_infantry_3_cooling_buff) << ','
            << "\"opponent_infantry_3_defence_buff\":" << JsonScalar(data.opponent_infantry_3_defence_buff) << ','
            << "\"opponent_infantry_3_vulnerability_buff\":" << JsonScalar(data.opponent_infantry_3_vulnerability_buff) << ','
            << "\"opponent_infantry_3_attack_buff\":" << JsonScalar(data.opponent_infantry_3_attack_buff) << ','
            << "\"opponent_infantry_4_hp_buff\":" << JsonScalar(data.opponent_infantry_4_hp_buff) << ','
            << "\"opponent_infantry_4_cooling_buff\":" << JsonScalar(data.opponent_infantry_4_cooling_buff) << ','
            << "\"opponent_infantry_4_defence_buff\":" << JsonScalar(data.opponent_infantry_4_defence_buff) << ','
            << "\"opponent_infantry_4_vulnerability_buff\":" << JsonScalar(data.opponent_infantry_4_vulnerability_buff) << ','
            << "\"opponent_infantry_4_attack_buff\":" << JsonScalar(data.opponent_infantry_4_attack_buff) << ','
            << "\"opponent_sentry_hp_buff\":" << JsonScalar(data.opponent_sentry_hp_buff) << ','
            << "\"opponent_sentry_cooling_buff\":" << JsonScalar(data.opponent_sentry_cooling_buff) << ','
            << "\"opponent_sentry_defence_buff\":" << JsonScalar(data.opponent_sentry_defence_buff) << ','
            << "\"opponent_sentry_vulnerability_buff\":" << JsonScalar(data.opponent_sentry_vulnerability_buff) << ','
            << "\"opponent_sentry_attack_buff\":" << JsonScalar(data.opponent_sentry_attack_buff) << ','
            << "\"opponent_sentry_posture\":" << JsonScalar(data.opponent_sentry_posture);
      }
      oss << "}";
      return oss.str();
    }
    case Cmd::kRadar5: {
      const auto &data = protocol.radar5;
      oss << "{\"key\":\"" << HexBytes(data.key, sizeof(data.key)) << "\"}";
      return oss.str();
    }
    default:
      return "{}";
  }
}

/**
 * @brief 判断主协议帧是否需要生成结构体日志
 * @note 过滤发往普通机器人、飞镖或哨兵的下行状态；雷达标记和雷达信息保留。
 */
template <rm::device::RefereeRevision revision>
constexpr bool ShouldLogMainProtocolFrame(rm::u16 cmd_id) {
  using Cmd = rm::device::RefereeCmdId<revision>;
  switch (cmd_id) {
    case Cmd::kRobotStatus:             // 0x0201
    case Cmd::kPowerHeatData:           // 0x0202
    case Cmd::kRobotPos:                // 0x0203
    case Cmd::kBuff:                    // 0x0204
    case Cmd::kHurtData:                // 0x0206
    case Cmd::kShootData:               // 0x0207
    case Cmd::kProjectileAllowance:     // 0x0208
    case Cmd::kRfidStatus:              // 0x0209
    case Cmd::kDartClientCmd:            // 0x020A
    case Cmd::kGroundRobotPosition:     // 0x020B
    case Cmd::kSentryInfo:              // 0x020D
      return false;
    default:
      return true;
  }
}

/**
 * @brief 主协议结构体日志维护器
 * @tparam revision 当前裁判协议版本
 */
template <rm::device::RefereeRevision revision>
class RefereeMainLogger {
 public:
  using Protocol = rm::device::RefereeProtocol<revision>;
  static_assert(kSupportedMainLogRevision<revision>, "RefereeMainLogger only supports kNewV120 and kNewV200");

  /**
   * @brief 创建主协议日志器
   * @param root 本轮运行日志根目录
   */
  explicit RefereeMainLogger(std::filesystem::path root = RADAR_DEFAULT_LOG_DIR)
      : log_store_(std::move(root)) {}

  /**
   * @brief 记录一次主协议结构体更新
   * @param cmd_id 主命令码
   * @param seq 当前帧序号
   * @param protocol 当前维护状态
   * @param loss_rate 当前链路丢包率
   */
  void LogFrame(rm::u16 cmd_id, rm::u8 seq, const Protocol &protocol, float loss_rate) {
    if (!radar::config::kEnableMainProtocolStructLog || !ShouldLogMainProtocolFrame<revision>(cmd_id)) {
      return;
    }

    const auto payload_json = FormatMainPayload<revision>(cmd_id, protocol);
    const auto cmd_hex = HexU16(cmd_id);
    const auto cmd_name = MainCmdName<revision>(cmd_id);
    const auto basename = cmd_hex + "_" + cmd_name;

    std::ostringstream entry;
    entry << "{"
          << "\"timestamp\":\"" << TimestampNow() << "\","
          << "\"seq\":" << JsonScalar(seq) << ','
          << "\"cmd_id\":\"" << cmd_hex << "\","
          << "\"name\":\"" << cmd_name << "\","
          << "\"loss_rate\":" << JsonScalar(loss_rate) << ','
          << "\"state\":" << payload_json << "}";

    log_store_.Append(std::filesystem::path("main") / (basename + ".log"), entry.str(), LogPriority::kBestEffort);
    ++update_count_[cmd_id];
  }

  /**
   * @brief 查询指定命令码的累计更新次数
   * @param cmd_id 主命令码
   * @return 该命令码累计写日志次数
   */
  std::size_t update_count(rm::u16 cmd_id) const {
    const auto it = update_count_.find(cmd_id);
    if (it == update_count_.end()) {
      return 0;
    }
    return it->second;
  }

  const std::filesystem::path &root() const { return log_store_.root(); }

 private:
  FileLogStore log_store_;                    ///< 主协议日志输出器
  std::map<rm::u16, std::size_t> update_count_;  ///< 每个主命令码的累计更新次数
};

}  // namespace radar::log

#endif  // RADAR_SRC_REFEREE_MAIN_LOG_HPP
