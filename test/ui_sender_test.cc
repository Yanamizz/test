/**
 * @file test/ui_sender_test.cc
 * @brief 独立的 UIDrone `UIFigure5` 发送测试程序
 *
 * 用法：
 *   ui_sender_test [red|blue] [add|edit]
 *
 * 程序通过真实裁判串口发送一帧 `0x0301/0x0103(UIFigure5)`，目标为对应
 * 阵营的友方空中机器人选手端。图元布局和值沿用嵌入式 `UIDroneHero_add`
 * 的第一版测试内容。
 */

#include <array>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "include/config/config.hpp"
#include "include/log/referee_main_log.hpp"
#include "include/referee/robot_interaction_sender.hpp"
#include "include/referee/serial_port.hpp"
#include "include/radar/app/subReferee/protocol_user.hpp"

namespace {

enum class UiTestSide {
  kRed,
  kBlue,
};

struct UiTestTarget {
  const char *side_name;
  rm::u16 sender_robot_id;
  rm::u16 receiver_client_id;
};

constexpr UiTestTarget kRedAerialTarget{"red", 9, 0x0106};
constexpr UiTestTarget kBlueAerialTarget{"blue", 109, 0x016a};

struct UiDroneDemoValues {
  float yaw_degrees = 12.5f;
  float pitch_degrees = -3.0f;
  rm::i32 ammo_adjust = 30;
  float initial_speed = 25.0f;
  float command_yaw_degrees = 15.0f;
};

UiTestSide ParseSide(std::string_view value) {
  if (value == "red") {
    return UiTestSide::kRed;
  }
  if (value == "blue") {
    return UiTestSide::kBlue;
  }
  throw std::runtime_error("side must be red or blue");
}

rm::device::UIFigure::Operation ParseOperation(std::string_view value) {
  if (value == "add") {
    return rm::device::UIFigure::Operation::Add;
  }
  if (value == "edit") {
    return rm::device::UIFigure::Operation::Edit;
  }
  throw std::runtime_error("operation must be add or edit");
}

const UiTestTarget &TargetForSide(UiTestSide side) {
  return side == UiTestSide::kRed ? kRedAerialTarget : kBlueAerialTarget;
}

rm::device::UIFigure5 BuildUidroneFigure5(rm::device::UIFigure::Operation operation,
                                          const UiDroneDemoValues &values) {
  rm::device::UIFigure5 ui{};
  ui.figure1.fillFloat("yaw", operation, 0, rm::device::UIFigure::Color::Yellow, 5, 1460, 470, 20,
                       values.yaw_degrees * 1000.0f);
  ui.figure2.fillFloat("pit", operation, 0, rm::device::UIFigure::Color::Black, 5, 1620, 470, 20,
                       values.pitch_degrees * 1000.0f);
  ui.figure3.fillIntegrate("adj", operation, 0, rm::device::UIFigure::Color::Green, 5, 1650, 580, 22,
                           values.ammo_adjust);
  ui.figure4.fillFloat("isp", operation, 0, rm::device::UIFigure::Color::Cyan, 5, 1460, 580, 22,
                       values.initial_speed * 1000.0f);
  ui.figure5.fillFloat("cya", operation, 0, rm::device::UIFigure::Color::RedBlue, 5, 1460, 520, 24,
                       values.command_yaw_degrees * 1000.0f);
  return ui;
}

std::vector<rm::u8> BuildFrame(const UiTestTarget &target, rm::device::UIFigure::Operation operation,
                               const UiDroneDemoValues &values) {
  const auto ui = BuildUidroneFigure5(operation, values);
  std::array<rm::u8, rm::device::kRefProtocolFrameMaxLen> buffer{};
  const auto frame_len = radar::referee::PrepareRobotInteractionFrame(
      buffer.data(), 0, rm::device::RefereeSubCmdId::kUIFigure5,
      reinterpret_cast<const rm::u8 *>(&ui), sizeof(ui), target.sender_robot_id, target.receiver_client_id);
  if (frame_len == 0) {
    throw std::runtime_error("failed to build UIFigure5 interaction frame");
  }
  return {buffer.begin(), buffer.begin() + frame_len};
}

void PrintUsage(const char *program) {
  std::cout << "Usage: " << program << " [red|blue] [add|edit]\n"
            << "  red  -> sender 9, receiver 0x0106 (red aerial robot player client)\n"
            << "  blue -> sender 109, receiver 0x016A (blue aerial robot player client)\n"
            << "  add  -> add the five UIDrone figures\n"
            << "  edit -> edit the five UIDrone figures\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc > 3) {
      PrintUsage(argv[0]);
      return 2;
    }

    const auto side = ParseSide(argc >= 2 ? std::string_view(argv[1]) : std::string_view("red"));
    const auto operation = ParseOperation(argc >= 3 ? std::string_view(argv[2]) : std::string_view("add"));
    const auto &target = TargetForSide(side);
    const UiDroneDemoValues values{};
    const auto frame = BuildFrame(target, operation, values);

    radar::referee::SerialPort serial;
    std::string error;
    if (!serial.TryOpenDefault(radar::config::kDefaultRefereeBaud, &error)) {
      std::cerr << "failed to open referee serial: " << error << '\n';
      return 1;
    }

    if (!serial.TryWriteAll(frame.data(), frame.size(), &error)) {
      std::cerr << "failed to send UIDrone UIFigure5 frame: " << error << '\n';
      return 1;
    }

    std::cout << "sent UIDrone UIFigure5 " << (operation == rm::device::UIFigure::Operation::Add ? "add" : "edit")
              << " frame: side=" << target.side_name << ", sender=" << target.sender_robot_id << ", receiver="
              << radar::log::HexU16(target.receiver_client_id) << ", frame_len=" << frame.size() << '\n'
              << "frame_hex=" << radar::log::HexBytes(frame.data(), frame.size()) << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
