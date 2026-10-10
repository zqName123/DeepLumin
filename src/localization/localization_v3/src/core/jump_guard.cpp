#include "localization_v3/core/jump_guard.hpp"

#include <cmath>

namespace localization_v3 {

JumpGuardVerdict JumpGuard::evaluate(const Pose3d& current_map_odom,
                                     const Pose3d& candidate_map_odom, const Pose3d& odom_base,
                                     double now_sec) {
  if (!config_.enabled) {
    return JumpGuardVerdict::kAccept;
  }

  // 比较修正前后 fused_pose 变化量（而非 map_odom 绝对值差）：
  // map_odom 是绝对状态，odom_base 漂移累积后的大修正是对漂移的补偿，
  // 不应导致 fused 跳变；直接比较 map_odom 绝对值会误拒这类合理修正。
  const Pose3d old_fused = compose(current_map_odom, odom_base);
  const Pose3d new_fused = compose(candidate_map_odom, odom_base);
  const double jump_trans = translationDiff(old_fused, new_fused);
  const double jump_yaw_deg = std::abs(yawDiffDeg(old_fused, new_fused));

  const bool small_jump = jump_trans <= config_.soft_max_translation &&
                          jump_yaw_deg <= config_.soft_max_yaw_deg;
  const bool huge_jump = jump_trans > config_.hard_max_translation ||
                         jump_yaw_deg > config_.hard_max_yaw_deg;

  // ---- L3 超限：拒绝，清缓冲 ----
  if (huge_jump) {
    candidates_.clear();
    return JumpGuardVerdict::kReject;
  }

  // ---- L1 小跳：直接接受，清缓冲 ----
  if (small_jump) {
    candidates_.clear();
    return JumpGuardVerdict::kAccept;
  }

  // ---- L2 大跳：连续一致确认 ----
  // 确认缓冲保存候选 map->odom（修正量本身）；同一真实修正的连续候选应互相一致，
  // 单帧几何相似误匹配无法连续一致，从而被拦下。
  while (!candidates_.empty() && (now_sec - candidates_.front().stamp) > config_.confirm_window_sec) {
    candidates_.pop_front();
  }
  while (candidates_.size() >= static_cast<std::size_t>(std::max(1, config_.confirm_frames))) {
    candidates_.pop_front();
  }

  if (!candidates_.empty()) {
    const Candidate& last = candidates_.back();
    const double diff_trans = translationDiff(last.map_odom, candidate_map_odom);
    const double diff_yaw = std::abs(yawDiffDeg(last.map_odom, candidate_map_odom));
    if (diff_trans > config_.confirm_consistency_trans ||
        diff_yaw > config_.confirm_consistency_yaw_deg) {
      // 候选互相不一致：说明是不可信跳变序列，清空重新累计
      candidates_.clear();
    }
  }

  candidates_.push_back(Candidate{candidate_map_odom, now_sec});

  if (static_cast<int>(candidates_.size()) >= config_.confirm_frames) {
    candidates_.clear();
    return JumpGuardVerdict::kAccept;
  }
  return JumpGuardVerdict::kConfirmPending;
}

}  // namespace localization_v3
