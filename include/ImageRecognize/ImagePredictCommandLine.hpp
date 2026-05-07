#pragma once

#include <string_view>

namespace ImageRecognize {

struct ImagePredictCommandLineOptions {
  bool enable_display = true;
};

namespace detail {

inline bool StartsWith(std::string_view text, std::string_view prefix) {
  return text.size() >= prefix.size() &&
         text.substr(0, prefix.size()) == prefix;
}

inline bool ParseDisplayValue(std::string_view value, bool fallback) {
  if (value == "0" || value == "false" || value == "off" || value == "no") {
    return false;
  }
  if (value == "1" || value == "true" || value == "on" || value == "yes") {
    return true;
  }
  return fallback;
}

} // namespace detail

inline ImagePredictCommandLineOptions
ParseImagePredictCommandLine(int argc, char **argv) {
  ImagePredictCommandLineOptions options{};
  constexpr std::string_view kEnableDisplayPrefix = "--enable-display=";
  constexpr std::string_view kDisplayPrefix = "--display=";

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i] != nullptr ? argv[i] : "";

    if (arg == "--no-display" || arg == "--disable-display") {
      options.enable_display = false;
      continue;
    }

    if (arg == "--display" || arg == "--enable-display") {
      options.enable_display = true;
      continue;
    }

    if (detail::StartsWith(arg, kEnableDisplayPrefix)) {
      options.enable_display = detail::ParseDisplayValue(
          arg.substr(kEnableDisplayPrefix.size()), options.enable_display);
      continue;
    }

    if (detail::StartsWith(arg, kDisplayPrefix)) {
      options.enable_display = detail::ParseDisplayValue(
          arg.substr(kDisplayPrefix.size()), options.enable_display);
      continue;
    }
  }

  return options;
}

} // namespace ImageRecognize
