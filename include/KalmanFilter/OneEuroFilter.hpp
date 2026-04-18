#pragma once

#include <algorithm>
#include <cmath>

namespace Tools {

class OneEuroFilter {
 public:
  OneEuroFilter(double freq = 120.0, double min_cutoff = 1.0, double beta = 0.0, double d_cutoff = 1.0)
      : freq_(SanitizePositive_(freq, kDefaultFrequencyHz)),
        min_cutoff_(SanitizePositive_(min_cutoff, kDefaultMinCutoffHz)),
        beta_(beta),
        d_cutoff_(SanitizePositive_(d_cutoff, kDefaultDerivativeCutoffHz)) {}

  double filter(double value, double dt = -1.0) {
    if (dt > 0.0) {
      freq_ = 1.0 / dt;
    }

    if (!initialized_) {
      initialized_ = true;
      signal_filter_.Seed(value);
      derivative_filter_.Seed(0.0);
      return value;
    }

    const double previous_filtered = signal_filter_.LastFilteredValue();
    const double derivative = (value - previous_filtered) * freq_;
    const double filtered_derivative = derivative_filter_.FilterWithAlpha(derivative, Alpha_(d_cutoff_));
    const double cutoff = min_cutoff_ + beta_ * std::abs(filtered_derivative);
    return signal_filter_.FilterWithAlpha(value, Alpha_(cutoff));
  }

  void reset() {
    initialized_ = false;
    signal_filter_.Reset();
    derivative_filter_.Reset();
  }

  void setFrequency(double freq) { freq_ = SanitizePositive_(freq, kDefaultFrequencyHz); }
  void setMinCutoff(double min_cutoff) { min_cutoff_ = SanitizePositive_(min_cutoff, kDefaultMinCutoffHz); }
  void setBeta(double beta) { beta_ = beta; }
  void setDerivativeCutoff(double d_cutoff) { d_cutoff_ = SanitizePositive_(d_cutoff, kDefaultDerivativeCutoffHz); }

  double getFrequency() const { return freq_; }
  double getMinCutoff() const { return min_cutoff_; }
  double getBeta() const { return beta_; }
  double getDerivativeCutoff() const { return d_cutoff_; }
  bool hasState() const { return initialized_; }

 private:
  class LowPassFilter {
   public:
    void Reset(double initial_value = 0.0) {
      last_raw_value_ = initial_value;
      last_filtered_value_ = initial_value;
      initialized_ = false;
    }

    void Seed(double value) {
      last_raw_value_ = value;
      last_filtered_value_ = value;
      initialized_ = true;
    }

    double Filter(double value) {
      if (!initialized_) {
        Seed(value);
        return value;
      }

      const double filtered = alpha_ * value + (1.0 - alpha_) * last_filtered_value_;
      last_raw_value_ = value;
      last_filtered_value_ = filtered;
      return filtered;
    }

    double FilterWithAlpha(double value, double alpha) {
      SetAlpha_(alpha);
      return Filter(value);
    }

    double LastRawValue() const { return last_raw_value_; }
    double LastFilteredValue() const { return last_filtered_value_; }

   private:
    void SetAlpha_(double alpha) { alpha_ = std::clamp(alpha, kMinAlpha, 1.0); }

    double last_raw_value_ = 0.0;
    double last_filtered_value_ = 0.0;
    double alpha_ = 1.0;
    bool initialized_ = false;
  };

  static double SanitizePositive_(double value, double fallback) { return value > 0.0 ? value : fallback; }

  double Alpha_(double cutoff) const {
    const double safe_cutoff = std::max(cutoff, kMinPositiveValue);
    const double safe_freq = std::max(freq_, kMinPositiveValue);
    const double te = 1.0 / safe_freq;
    const double tau = 1.0 / (2.0 * kPi * safe_cutoff);
    return 1.0 / (1.0 + tau / te);
  }

  static constexpr double kDefaultFrequencyHz = 120.0;
  static constexpr double kDefaultMinCutoffHz = 1.0;
  static constexpr double kDefaultDerivativeCutoffHz = 1.0;
  static constexpr double kMinPositiveValue = 1e-12;
  static constexpr double kMinAlpha = 1e-6;
  static constexpr double kPi = 3.141592653589793238462643383279502884;

  double freq_ = kDefaultFrequencyHz;
  double min_cutoff_ = kDefaultMinCutoffHz;
  double beta_ = 0.0;
  double d_cutoff_ = kDefaultDerivativeCutoffHz;
  LowPassFilter signal_filter_;
  LowPassFilter derivative_filter_;
  bool initialized_ = false;
};

}  // namespace Tools