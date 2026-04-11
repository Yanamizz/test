/**
 * @file TrackingQualityEvaluator.hpp
 * @brief 跟踪效果评估工具（仅在识别到目标时启用）
 *
 * 算法依据：
 * 1. 过度振荡通常表现为误差频繁正负翻转、控制量变化幅度过大。
 *    因此本算法综合以下证据：
 *    - 误差零交叉率
 *    - 控制信号速度与目标信号速度之比
 *    - 误差信号速度与目标信号速度之比
 *
 * 2. 过度滞后通常表现为控制信号相位落后、持续偏差较大。
 *    因此本算法综合以下证据：
 *    - 互相关最优延迟帧数
 *    - 均方根误差
 *    - 有符号偏置
 *
 * 3. 最终判定通过两个分数（oscillation_score / lag_score）完成：
 *    - 两者都低于阈值：Good
 *    - 仅振荡分高：OverOscillation
 *    - 仅滞后分高：OverLag
 *    - 两者都高：Mixed
 *
 * 效果判断建议：
 * 1. 重点看 issue 和 oscillation_score / lag_score，而不只看单个指标。
 * 2. 若 oscillation_score 高且 lag_score 低：优先降增益、增阻尼或加平滑。
 * 3. 若 lag_score 高且 oscillation_score 低：优先提响应（减平滑、提前馈、提高过程噪声参数或降低测量噪声参数）。
 * 4. 若 Mixed：先抑制高频振荡，再逐步提高响应，避免“又快又抖”。
 *
 * 输入与使用说明：
 * 1. target_signal 与 control_signal 必须时间对齐，且仅包含“检测到目标”时刻的数据。
 * 2. 当 target_detected=false 时函数直接返回 TargetMissing，不进行评估。
 * 3. 窗口样本数建议 >= 18；样本过少时返回 InsufficientData。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <string>
#include <vector>

namespace Tools {

enum class TrackingIssueType {
  Good,
  OverOscillation,
  OverLag,
  Mixed,
  TargetMissing,
  InsufficientData,
};

struct TrackingEvalParams {
  // min_samples：产生可靠判定所需的最少样本数。
  std::size_t min_samples = 20;
  // max_lag_frames：估计相位滞后时搜索的最大帧延迟。
  std::size_t max_lag_frames = 8;

  // error_deadband：忽略零附近的小误差，避免把噪声计入振荡。
  float error_deadband = 0.03f;

  // oscillation_zcr_threshold / oscillation_gain_ratio_threshold /
  // oscillation_error_velocity_ratio_threshold：振荡判定阈值。
  float oscillation_zcr_threshold = 0.28f;
  float oscillation_gain_ratio_threshold = 1.35f;
  float oscillation_error_velocity_ratio_threshold = 1.05f;

  // lag_delay_threshold_frames / lag_rmse_threshold / lag_bias_threshold：滞后判定阈值。
  float lag_delay_threshold_frames = 2.0f;
  float lag_rmse_threshold = 0.65f;
  float lag_bias_threshold = 0.28f;

  // decision_threshold：每个分数的最终判定阈值。
  float decision_threshold = 0.50f;
};

struct TrackingEvalResult {
  bool valid = false;
  TrackingIssueType issue = TrackingIssueType::InsufficientData;

  // oscillation_score / lag_score：归一化分数，范围为 [0, 1]。
  float oscillation_score = 0.0f;
  float lag_score = 0.0f;

  // rmse / best_lag_frames / best_lag_corr / error_zero_crossing_rate / control_to_target_velocity_ratio /
  // error_velocity_ratio / signed_bias：适合用于日志记录和曲线绘制的诊断信息。
  float rmse = 0.0f;
  float best_lag_frames = 0.0f;
  float best_lag_corr = 0.0f;
  float error_zero_crossing_rate = 0.0f;
  float control_to_target_velocity_ratio = 0.0f;
  float error_velocity_ratio = 0.0f;
  float signed_bias = 0.0f;
  std::string note;
};

inline float Clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

inline float MeanAbsDiff(const std::vector<float> &v) {
  if (v.size() < 2) return 0.0f;
  float sum = 0.0f;
  for (std::size_t i = 1; i < v.size(); ++i) {
    sum += std::abs(v[i] - v[i - 1]);
  }
  return sum / static_cast<float>(v.size() - 1);
}

inline float PearsonCorr(const std::vector<float> &a, std::size_t a0, const std::vector<float> &b, std::size_t b0,
                         std::size_t n) {
  if (n < 2) return 0.0f;

  float mean_a = 0.0f;
  float mean_b = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    mean_a += a[a0 + i];
    mean_b += b[b0 + i];
  }
  mean_a /= static_cast<float>(n);
  mean_b /= static_cast<float>(n);

  float num = 0.0f;
  float den_a = 0.0f;
  float den_b = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    const float da = a[a0 + i] - mean_a;
    const float db = b[b0 + i] - mean_b;
    num += da * db;
    den_a += da * da;
    den_b += db * db;
  }

  const float eps = 1e-6f;
  const float den = std::sqrt(std::max(eps, den_a * den_b));
  return num / den;
}

// 仅使用“已检测到目标”的样本评估跟踪质量。
// target_signal：参考轨迹（例如目标角度或中心点）。
// control_signal：跟踪器输出（例如预测角度或控制指令）。
inline TrackingEvalResult EvaluateTrackingQualityWhenDetected(const std::vector<float> &target_signal,
                                                              const std::vector<float> &control_signal,
                                                              bool target_detected,
                                                              const TrackingEvalParams &p = TrackingEvalParams{}) {
  TrackingEvalResult out;

  if (!target_detected) {
    // target_detected=false 时直接返回 TargetMissing。
    out.note = "target missing";
    return out;
  }

  const std::size_t n = std::min(target_signal.size(), control_signal.size());
  if (n < p.min_samples) {
    out.issue = TrackingIssueType::InsufficientData;
    out.note = "insufficient samples";
    return out;
  }

  std::vector<float> target(target_signal.begin(), target_signal.begin() + static_cast<std::ptrdiff_t>(n));
  std::vector<float> control(control_signal.begin(), control_signal.begin() + static_cast<std::ptrdiff_t>(n));

  std::vector<float> err(n, 0.0f);
  float sum_sq = 0.0f;
  float sum_abs = 0.0f;
  float sum_err = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    err[i] = control[i] - target[i];
    sum_sq += err[i] * err[i];
    sum_abs += std::abs(err[i]);
    sum_err += err[i];
  }

  out.rmse = std::sqrt(sum_sq / static_cast<float>(n));
  out.signed_bias = sum_err / (sum_abs + 1e-6f);

  // 振荡证据 1：误差零交叉率较高。
  int crossings = 0;
  int valid_pairs = 0;
  for (std::size_t i = 1; i < n; ++i) {
    const float e0 = err[i - 1];
    const float e1 = err[i];
    if (std::abs(e0) < p.error_deadband || std::abs(e1) < p.error_deadband) continue;
    ++valid_pairs;
    if ((e0 > 0.0f && e1 < 0.0f) || (e0 < 0.0f && e1 > 0.0f)) {
      ++crossings;
    }
  }
  out.error_zero_crossing_rate =
      valid_pairs > 0 ? static_cast<float>(crossings) / static_cast<float>(valid_pairs) : 0.0f;

  // 振荡证据 2：控制量变化幅度明显大于目标变化幅度。
  const float target_vel = MeanAbsDiff(target);
  const float control_vel = MeanAbsDiff(control);
  const float err_vel = MeanAbsDiff(err);
  out.control_to_target_velocity_ratio = control_vel / (target_vel + 1e-6f);
  out.error_velocity_ratio = err_vel / (target_vel + 1e-6f);

  const float osc_zcr = Clamp01((out.error_zero_crossing_rate - p.oscillation_zcr_threshold) /
                                std::max(1e-6f, 0.5f - p.oscillation_zcr_threshold));
  const float osc_gain = Clamp01((out.control_to_target_velocity_ratio - p.oscillation_gain_ratio_threshold) /
                                 std::max(1e-6f, 2.5f - p.oscillation_gain_ratio_threshold));
  const float osc_errv = Clamp01((out.error_velocity_ratio - p.oscillation_error_velocity_ratio_threshold) /
                                 std::max(1e-6f, 2.5f - p.oscillation_error_velocity_ratio_threshold));

  out.oscillation_score = 0.45f * osc_zcr + 0.35f * osc_gain + 0.20f * osc_errv;

  // 滞后证据：互相关峰值出现在正延迟，同时误差持续存在。
  float best_corr = -2.0f;
  std::size_t best_lag = 0;
  const std::size_t lag_max = std::min(p.max_lag_frames, n - 2);
  for (std::size_t lag = 0; lag <= lag_max; ++lag) {
    const std::size_t m = n - lag;
    const float c = PearsonCorr(target, 0, control, lag, m);
    if (c > best_corr) {
      best_corr = c;
      best_lag = lag;
    }
  }
  out.best_lag_frames = static_cast<float>(best_lag);
  out.best_lag_corr = best_corr;

  const float lag_delay = Clamp01((out.best_lag_frames - p.lag_delay_threshold_frames) /
                                  std::max(1e-6f, static_cast<float>(p.max_lag_frames) - p.lag_delay_threshold_frames));
  const float lag_rmse = Clamp01(out.rmse / std::max(1e-6f, p.lag_rmse_threshold));
  const float lag_bias =
      Clamp01((std::abs(out.signed_bias) - p.lag_bias_threshold) / std::max(1e-6f, 1.0f - p.lag_bias_threshold));
  const float lag_corr_gate = Clamp01((best_corr - 0.25f) / 0.75f);

  out.lag_score = (0.45f * lag_delay + 0.35f * lag_rmse + 0.20f * lag_bias) * lag_corr_gate;

  const bool osc_bad = out.oscillation_score >= p.decision_threshold;
  const bool lag_bad = out.lag_score >= p.decision_threshold;

  out.valid = true;
  if (osc_bad && lag_bad) {
    out.issue = TrackingIssueType::Mixed;
    out.note = "both oscillation and lag are strong";
  } else if (osc_bad) {
    out.issue = TrackingIssueType::OverOscillation;
    out.note = "oscillation dominates";
  } else if (lag_bad) {
    out.issue = TrackingIssueType::OverLag;
    out.note = "lag dominates";
  } else {
    out.issue = TrackingIssueType::Good;
    out.note = "tracking quality acceptable";
  }

  return out;
}

}  // namespace Tools
