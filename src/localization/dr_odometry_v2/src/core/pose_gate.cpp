#include "dr_odometry_v2/core/pose_gate.hpp"

namespace dr_odometry_v2 {

GateVerdict PoseGate::evaluate(const Pose3d& current, const Pose3d& candidate, double now_sec) {
  const double dt = (current.position - candidate.position).norm();
  const double dyaw = std::abs(math::logQuat(current.orientation, candidate.orientation).z()) *
                      180.0 / math::kPi;

  const bool small = dt <= config_.soft_trans && dyaw <= config_.soft_yaw_deg;
  const bool huge = dt > config_.hard_trans || dyaw > config_.hard_yaw_deg;

  if (huge) {
    candidates_.clear();
    return GateVerdict::kReject;
  }
  if (small) {
    candidates_.clear();
    return GateVerdict::kAccept;
  }

  // 大跳：连续一致确认（真修正后续稳定在新水平；伪跳变来回跳无法一致）
  while (!candidates_.empty() && (now_sec - candidates_.front().stamp) > 1.0) {
    candidates_.pop_front();
  }
  while (static_cast<int>(candidates_.size()) >= std::max(1, config_.confirm_frames)) {
    candidates_.pop_front();
  }
  if (!candidates_.empty()) {
    const Candidate& last = candidates_.back();
    const double diff_t = (last.pose.position - candidate.position).norm();
    const double diff_yaw =
        std::abs(math::logQuat(last.pose.orientation, candidate.orientation).z()) * 180.0 /
        math::kPi;
    if (diff_t > config_.consistency_trans || diff_yaw > config_.consistency_yaw_deg) {
      candidates_.clear();
    }
  }
  candidates_.push_back(Candidate{candidate, now_sec});

  if (static_cast<int>(candidates_.size()) >= config_.confirm_frames) {
    candidates_.clear();
    return GateVerdict::kAcceptStep;
  }
  return GateVerdict::kConfirmPending;
}

}  // namespace dr_odometry_v2
