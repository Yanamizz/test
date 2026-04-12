#pragma once

#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace Tools {
namespace detail {

inline bool readIntFile(const std::string &path, int &value) {
  std::ifstream file(path);
  if (!file.is_open()) return false;
  file >> value;
  return file.good();
}

inline bool readLongFile(const std::string &path, long &value) {
  std::ifstream file(path);
  if (!file.is_open()) return false;
  file >> value;
  return file.good();
}

inline std::vector<int> detectBigCoresByType(int cpu_count) {
  std::vector<int> big_cores;
  bool any_type = false;
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    int core_type = -1;
    const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/core_type";
    if (readIntFile(path, core_type)) {
      any_type = true;
      if (core_type >= 1) {
        big_cores.push_back(cpu);
      }
    }
  }
  if (!any_type) return {};
  return big_cores;
}

inline std::vector<int> detectBigCoresByFreq(int cpu_count) {
  std::vector<std::pair<long, int>> freqs;
  freqs.reserve(cpu_count);
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    long freq = -1;
    const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/cpuinfo_max_freq";
    if (readLongFile(path, freq)) {
      freqs.emplace_back(freq, cpu);
    }
  }
  if (freqs.empty()) return {};

  std::sort(freqs.begin(), freqs.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
  const size_t pick = std::max<size_t>(1, freqs.size() / 2);
  std::vector<int> big_cores;
  big_cores.reserve(pick);
  for (size_t i = 0; i < pick; ++i) {
    big_cores.push_back(freqs[i].second);
  }
  return big_cores;
}

inline std::vector<int> fallbackAllCores(int cpu_count) {
  std::vector<int> all(cpu_count);
  std::iota(all.begin(), all.end(), 0);
  return all;
}

}  // namespace detail

inline std::vector<int> DetectBigCores() {
  const int cpu_count = static_cast<int>(sysconf(_SC_NPROCESSORS_CONF));
  if (cpu_count <= 0) return {};

  auto by_type = detail::detectBigCoresByType(cpu_count);
  if (!by_type.empty()) return by_type;

  auto by_freq = detail::detectBigCoresByFreq(cpu_count);
  if (!by_freq.empty()) return by_freq;

  return detail::fallbackAllCores(cpu_count);
}

inline bool BindCurrentThreadToCores(const std::vector<int> &cores) {
  if (cores.empty()) return false;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  for (int cpu : cores) {
    if (cpu >= 0) CPU_SET(cpu, &mask);
  }
  return sched_setaffinity(0, sizeof(mask), &mask) == 0;
}

inline bool BindCurrentThreadToBigCores() {
  const auto cores = DetectBigCores();
  const bool ok = BindCurrentThreadToCores(cores);
  if (!ok) {
    std::cerr << "[CpuAffinity] Failed to bind current thread to big cores." << std::endl;
  }
  return ok;
}

inline bool BindCurrentThreadToAllCores() {
  const int cpu_count = static_cast<int>(sysconf(_SC_NPROCESSORS_CONF));
  if (cpu_count <= 0) return false;

  std::vector<int> cores;
  cores.reserve(static_cast<std::size_t>(cpu_count));
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    cores.push_back(cpu);
  }

  const bool ok = BindCurrentThreadToCores(cores);
  if (!ok) {
    std::cerr << "[CpuAffinity] Failed to bind current thread to all cores." << std::endl;
  }
  return ok;
}

}  // namespace Tools
