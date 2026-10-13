#include "local_odom_gateway/core/motion_provider.hpp"

#include <algorithm>
#include <cmath>

namespace local_odom_gateway {

namespace {

constexpr double kEmaAlpha = 0.3;  // 周期 EMA 平滑系数（新样本权重）

}  // namespace

// ============================ 增量源轨迹 ============================

void MotionProvider::pushIncrement(const Pose3d& pose, double stamp) {
  if (!isFinite(pose.position) || !isFinite(pose.orientation) || stamp < 0.0) {
    return;
  }
  if (!increment_history_.empty() && stamp <= increment_history_.back().stamp) {
    return;  // 乱序样本：历史必须单调，否则插值区间失效
  }
  increment_history_.push_back({stamp, pose});
  last_increment_ = increment_history_.back();
  has_increment_ = true;
  while (!increment_history_.empty() &&
         (stamp - increment_history_.front().stamp) > config_.increment_history_sec) {
    increment_history_.pop_front();
  }
}

void MotionProvider::onIncrementSample(const Pose3d& pose, double stamp) {
  pushIncrement(pose, stamp);
}

bool MotionProvider::incrementAt(double stamp, Pose3d* out) const {
  if (out == nullptr || increment_history_.empty() || stamp < 0.0) {
    return false;
  }
  // 找第一个 stamp ≥ target 的样本，与前一个构成插值区间
  std::size_t hi = 0;
  while (hi < increment_history_.size() && increment_history_[hi].stamp < stamp) {
    ++hi;
  }
  if (hi == 0) {
    *out = increment_history_.front().pose;  // 早于历史覆盖：取最早
    return true;
  }
  if (hi >= increment_history_.size()) {
    *out = increment_history_.back().pose;  // 晚于历史覆盖：取最新
    return true;
  }
  const TimedPose& a = increment_history_[hi - 1];
  const TimedPose& b = increment_history_[hi];
  const double dt = b.stamp - a.stamp;
  if (dt <= 1e-9) {
    *out = b.pose;
    return true;
  }
  *out = interpolate(a.pose, b.pose, (stamp - a.stamp) / dt);
  return true;
}

// ============================ 锚 ============================

void MotionProvider::onAnchorSample(const Pose3d& pose, double stamp) {
  if (!isFinite(pose.position) || !isFinite(pose.orientation) || stamp < 0.0) {
    return;
  }
  anchor_.pose = pose;
  anchor_.stamp = stamp;
  anchor_.valid = true;
  // 配对：按【扫描戳】在增量源轨迹插值 D(t_a)。
  // 取到达时刻的增量源状态会差一个主源处理延迟内的运动量 → 增量启停各跳一次。
  anchor_.has_pairing = incrementAt(stamp, &anchor_.increment_at);

  // 周期 EMA（超龄/判死阈值自适应）
  if (last_anchor_stamp_ >= 0.0) {
    const double dt = stamp - last_anchor_stamp_;
    if (dt > 0.0) {
      const double clamped = std::min(config_.max_period_sec,
                                      std::max(config_.min_period_sec, dt));
      period_ema_ = (period_ema_ < 0.0) ? clamped
                                        : (1.0 - kEmaAlpha) * period_ema_ + kEmaAlpha * clamped;
    }
  }
  last_anchor_stamp_ = stamp;
}

double MotionProvider::anchorAge(double now_sec) const {
  return anchor_.valid ? (now_sec - anchor_.stamp) : -1.0;
}

double MotionProvider::overdueLimit() const {
  if (config_.overdue_sec > 0.0) {
    return config_.overdue_sec;  // 显式配置优先
  }
  return (period_ema_ > 0.0) ? config_.overdue_ratio * period_ema_ : config_.min_period_sec;
}

double MotionProvider::deadLimit() const {
  if (config_.dead_sec > 0.0) {
    return config_.dead_sec;
  }
  return (period_ema_ > 0.0) ? config_.dead_ratio * period_ema_ : 2.0 * config_.min_period_sec;
}

Phase MotionProvider::phase(double now_sec) const {
  if (!anchor_.valid) {
    return Phase::kNone;
  }
  const double age = now_sec - anchor_.stamp;
  if (age <= overdueLimit()) {
    return Phase::kRunning;
  }
  return (age <= deadLimit()) ? Phase::kGap : Phase::kOnBackup;
}

// ============================ 输出 ============================

bool MotionProvider::sample(Sample* out, double now_sec) const {
  if (out == nullptr || !anchor_.valid) {
    return false;
  }
  out->anchor = anchor_.pose;  // I4：锚永不含增量

  // I1：kRunning 期增量不应用 —— DR 结构上碰不到运动链
  const bool use_increment = (phase(now_sec) != Phase::kRunning) && anchor_.has_pairing &&
                             has_increment_;
  if (!use_increment) {
    out->motion = anchor_.pose;
    return true;
  }
  // kGap / kOnBackup：A ⊕ (D(t_a)⁻¹ ⊕ D(t))
  out->motion =
      compose(anchor_.pose, compose(inverse(anchor_.increment_at), last_increment_.pose));
  return true;
}

}  // namespace local_odom_gateway
