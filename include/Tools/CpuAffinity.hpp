#pragma once

#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
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

inline std::vector<int> fallbackAllCores(int cpu_count);

inline std::vector<int> detectBigCoresByFreq(int cpu_count) {
  struct CpuTopologyInfo {
    int cpu = -1;
    int package_id = 0;
    int core_id = -1;
    long max_freq = -1;
    bool has_package_id = false;
    bool has_core_id = false;
    bool has_max_freq = false;
  };

  auto readCpuTopologyInfo = [](int cpu, CpuTopologyInfo &info) {
    info = CpuTopologyInfo{};
    info.cpu = cpu;

    bool any_field = false;

    int package_id = -1;
    const std::string package_path =
        "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/physical_package_id";
    if (readIntFile(package_path, package_id)) {
      info.package_id = package_id;
      info.has_package_id = true;
      any_field = true;
    }

    int core_id = -1;
    const std::string core_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/core_id";
    if (readIntFile(core_path, core_id)) {
      info.core_id = core_id;
      info.has_core_id = true;
      any_field = true;
    }

    long freq = -1;
    const std::string freq_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpufreq/cpuinfo_max_freq";
    if (readLongFile(freq_path, freq)) {
      info.max_freq = freq;
      info.has_max_freq = true;
      any_field = true;
    }

    return any_field;
  };

  std::vector<CpuTopologyInfo> cpu_infos;
  cpu_infos.reserve(cpu_count);
  bool has_core_ids = false;
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    CpuTopologyInfo info;
    if (readCpuTopologyInfo(cpu, info)) {
      has_core_ids = has_core_ids || info.has_core_id;
      cpu_infos.push_back(info);
    }
  }
  if (cpu_infos.empty()) return {};

  if (!has_core_ids) {
    std::vector<std::pair<long, int>> freqs;
    freqs.reserve(cpu_infos.size());
    for (const auto &info : cpu_infos) {
      if (info.has_max_freq) {
        freqs.emplace_back(info.max_freq, info.cpu);
      }
    }
    if (freqs.empty()) return {};

    std::sort(freqs.begin(), freqs.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
    const long max_freq = freqs.front().first;
    const long min_freq = freqs.back().first;
    if (max_freq <= 0 || min_freq <= 0 || (max_freq - min_freq) < 200000 || max_freq * 100 <= min_freq * 110) {
      return fallbackAllCores(cpu_count);
    }

    const long big_core_threshold = (max_freq * 90) / 100;
    std::vector<int> big_cores;
    big_cores.reserve(freqs.size());
    for (const auto &[freq, cpu] : freqs) {
      if (freq >= big_core_threshold) {
        big_cores.push_back(cpu);
      }
    }
    if (big_cores.empty()) big_cores.push_back(freqs.front().second);
    return big_cores;
  }

  std::map<std::pair<int, int>, CpuTopologyInfo> grouped_cores;
  for (const auto &info : cpu_infos) {
    if (!info.has_max_freq) continue;
    const int package_key = info.has_package_id ? info.package_id : 0;
    const int core_key = info.has_core_id ? info.core_id : info.cpu;
    const auto key = std::make_pair(package_key, core_key);

    auto it = grouped_cores.find(key);
    if (it == grouped_cores.end() || info.max_freq > it->second.max_freq ||
        (info.max_freq == it->second.max_freq && info.cpu < it->second.cpu)) {
      grouped_cores[key] = info;
    }
  }
  if (grouped_cores.empty()) return {};

  std::vector<CpuTopologyInfo> representative_cores;
  representative_cores.reserve(grouped_cores.size());
  for (const auto &entry : grouped_cores) {
    representative_cores.push_back(entry.second);
  }

  std::sort(representative_cores.begin(), representative_cores.end(), [](const auto &a, const auto &b) {
    if (a.max_freq != b.max_freq) return a.max_freq > b.max_freq;
    return a.cpu < b.cpu;
  });

  const long max_freq = representative_cores.front().max_freq;
  const long min_freq = representative_cores.back().max_freq;
  if (max_freq <= 0 || min_freq <= 0 || (max_freq - min_freq) < 200000 || max_freq * 100 <= min_freq * 110) {
    return fallbackAllCores(cpu_count);
  }

  const long big_core_threshold = (max_freq * 90) / 100;
  std::vector<int> big_cores;
  big_cores.reserve(representative_cores.size());
  for (const auto &core : representative_cores) {
    if (core.max_freq >= big_core_threshold) {
      big_cores.push_back(core.cpu);
    }
  }
  if (big_cores.empty()) big_cores.push_back(representative_cores.front().cpu);
  return big_cores;
}

inline std::vector<int> fallbackAllCores(int cpu_count) {
  std::vector<int> all(cpu_count);
  std::iota(all.begin(), all.end(), 0);
  return all;
}

inline int detectCpuCount() {
  const int cpu_count = static_cast<int>(sysconf(_SC_NPROCESSORS_CONF));
  return cpu_count > 0 ? cpu_count : 0;
}

// 核心拓扑在进程生命周期内通常不会变化，缓存一次即可避免重复扫描 /sys。
inline const std::vector<int> &cachedBigCores() {
  static const std::vector<int> cores = [] {
    const int cpu_count = detectCpuCount();
    if (cpu_count <= 0) {
      return std::vector<int>{};
    }

    auto by_type = detectBigCoresByType(cpu_count);
    if (!by_type.empty()) {
      return by_type;
    }

    auto by_freq = detectBigCoresByFreq(cpu_count);
    if (!by_freq.empty()) {
      return by_freq;
    }

    return fallbackAllCores(cpu_count);
  }();
  return cores;
}

inline const std::vector<int> &cachedAllCores() {
  static const std::vector<int> cores = [] {
    const int cpu_count = detectCpuCount();
    if (cpu_count <= 0) {
      return std::vector<int>{};
    }
    return fallbackAllCores(cpu_count);
  }();
  return cores;
}

inline const std::vector<int> &cachedAuxCores() {
  static const std::vector<int> cores = [] {
    const auto &all_cores = cachedAllCores();
    const auto &big_cores = cachedBigCores();
    if (all_cores.empty()) {
      return std::vector<int>{};
    }
    if (big_cores.empty()) {
      return all_cores;
    }

    std::vector<int> aux_cores;
    aux_cores.reserve(all_cores.size());
    for (int cpu : all_cores) {
      if (std::find(big_cores.begin(), big_cores.end(), cpu) == big_cores.end()) {
        aux_cores.push_back(cpu);
      }
    }

    if (aux_cores.empty()) {
      return all_cores;
    }
    return aux_cores;
  }();
  return cores;
}

}  // namespace detail

inline std::vector<int> DetectBigCores() { return detail::cachedBigCores(); }

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
  const auto &cores = detail::cachedBigCores();
  const bool ok = BindCurrentThreadToCores(cores);
  if (!ok) {
    std::cerr << "[CpuAffinity] Failed to bind current thread to big cores." << std::endl;
  }
  return ok;
}

inline bool BindCurrentThreadToAllCores() {
  const auto &cores = detail::cachedAllCores();
  const bool ok = BindCurrentThreadToCores(cores);
  if (!ok) {
    std::cerr << "[CpuAffinity] Failed to bind current thread to all cores." << std::endl;
  }
  return ok;
}

inline bool BindCurrentThreadToAuxCore(size_t index) {
  const auto &cores = detail::cachedAuxCores();
  if (cores.empty()) return false;

  const size_t selected_index = index < cores.size() ? index : (cores.size() - 1);
  const std::vector<int> selected_core{cores[selected_index]};
  const bool ok = BindCurrentThreadToCores(selected_core);
  if (!ok) {
    std::cerr << "[CpuAffinity] Failed to bind current thread to auxiliary core." << std::endl;
  }
  return ok;
}

}  // namespace Tools
