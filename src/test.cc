#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

#include <serial/serial.h>

#include "SerialTask/SerialTask.hpp"

namespace {
std::atomic<bool> g_running{true};

void HandleSignal(int) { g_running.store(false, std::memory_order_release); }

float ParseArgOrDefault(char **argv, int argc, int index, float fallback) {
  if (index >= argc) {
    return fallback;
  }
  return std::stof(argv[index]);
}

int ParseIntArgOrDefault(char **argv, int argc, int index, int fallback) {
  if (index >= argc) {
    return fallback;
  }
  return std::stoi(argv[index]);
}
}  // namespace

int main(int argc, char **argv) {
  try {
    // Usage:
    // ./test [pitch_offset_max_deg=3.0] [yaw_offset_max_deg=3.0] [interval_ms=50]
    const float pitch_offset_max_deg =
        std::abs(ParseArgOrDefault(argv, argc, 1, 3.0f));
    const float yaw_offset_max_deg =
        std::abs(ParseArgOrDefault(argv, argc, 2, 3.0f));
    const int interval_ms = std::max(1, ParseIntArgOrDefault(argv, argc, 3, 50));

    constexpr float kCenterYawDeg = 72.0f;
    constexpr float kCenterPitchDeg = 82.0f;

    std::signal(SIGINT, HandleSignal);

    serial::Serial port;
    SerialTask::DefaultConfig(port);
    port.open();
    if (!port.isOpen()) {
      std::cerr << "Failed to open serial port." << std::endl;
      return 2;
    }

    // 1) First frame: send center absolute angle, zero offset.
    SerialTask::SerialSend(port, kCenterPitchDeg, kCenterYawDeg, 0.0f, 0.0f,
                           0.0f, 0.0f, 0x01);

    std::cout << std::fixed << std::setprecision(3)
              << "[Init] absolute_yaw=" << kCenterYawDeg
              << " deg, absolute_pitch=" << kCenterPitchDeg
              << " deg, offset_yaw=0 deg, offset_pitch=0 deg" << std::endl;

    // 2) Then keep sending random offsets.
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<float> pitch_dist(-pitch_offset_max_deg,
                                                     pitch_offset_max_deg);
    std::uniform_real_distribution<float> yaw_dist(-yaw_offset_max_deg,
                                                   yaw_offset_max_deg);

    std::cout << "[Loop] random offset range: pitch in ["
              << -pitch_offset_max_deg << ", " << pitch_offset_max_deg
              << "] deg, yaw in [" << -yaw_offset_max_deg << ", "
              << yaw_offset_max_deg << "] deg, interval=" << interval_ms
              << " ms" << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    while (g_running.load(std::memory_order_acquire)) {
      const float pitch_offset_deg = pitch_dist(rng);
      const float yaw_offset_deg = yaw_dist(rng);
      const float absolute_pitch_deg = kCenterPitchDeg + pitch_offset_deg;
      const float absolute_yaw_deg = kCenterYawDeg + yaw_offset_deg;

      SerialTask::SerialSend(port, absolute_pitch_deg, absolute_yaw_deg,
                             pitch_offset_deg, yaw_offset_deg, 0.0f, 0.0f,
                             0x01);

      std::cout << "absolute_yaw=" << absolute_yaw_deg
                << " deg, absolute_pitch=" << absolute_pitch_deg
                << " deg, offset_yaw=" << yaw_offset_deg
                << " deg, offset_pitch=" << pitch_offset_deg << " deg"
                << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    port.close();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "test exception: " << e.what() << std::endl;
    return 3;
  }
}
