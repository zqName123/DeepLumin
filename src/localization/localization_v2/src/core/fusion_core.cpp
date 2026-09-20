#include "localization_v2/core/fusion_core.hpp"

namespace localization_v2 {

void FusionCore::setOdomBase(const Pose3d& odom_base) {
  if (!isFinite(odom_base.position) || !isFinite(odom_base.orientation)) {
    return;
  }
  odom_base_ = odom_base;
  odom_base_.orientation.normalize();
  have_slam_ = true;
}

bool FusionCore::passesJumpGuard(const Pose3d& candidate_map_odom) const {
  if (!jump_guard_.enabled || !have_map_odom_) {
    return true;
  }
  // [FIX] 比较修正前后 fused_pose 的变化量，而非 map_odom 的绝对值差异。
  // 原因：map_odom 是绝对状态，当 odom_base 累积大量漂移时，map_odom 的大修正是
  //       对漂移的补偿，不会导致 fused_pose 跳变。直接比较 map_odom 绝对值会错误
  //       拒绝这些合理修正。
  // 回退：恢复为 translationDiff(map_odom_, candidate_map_odom) 的比较方式。
  if (!have_slam_) {
    // 无 slam 数据时，退化为 map_odom 绝对值比较
    return translationDiff(map_odom_, candidate_map_odom) <= jump_guard_.max_translation &&
           std::abs(yawDiffDeg(map_odom_, candidate_map_odom)) <= jump_guard_.max_yaw_deg;
  }
  const Pose3d old_fused = compose(map_odom_, odom_base_);
  const Pose3d new_fused = compose(candidate_map_odom, odom_base_);
  return translationDiff(old_fused, new_fused) <= jump_guard_.max_translation &&
         std::abs(yawDiffDeg(old_fused, new_fused)) <= jump_guard_.max_yaw_deg;
}

bool FusionCore::updateFromGlobalMatch(const GlobalMatchUpdate& update) {
  if (!update.success || !update.has_pose) {
    return false;
  }

  Pose3d candidate_map_odom = update.pose;
  if (update.pose_is_map_base) {
    // GM 输出 map->base：需要 slam 数据反推 map->odom
    if (!have_slam_) {
      return false;
    }
    candidate_map_odom = compose(update.pose, inverse(odom_base_));
  }

  if (!isFinite(candidate_map_odom.position) || !isFinite(candidate_map_odom.orientation)) {
    return false;
  }

  if (!passesJumpGuard(candidate_map_odom) && !jump_guard_bypass_) {
    return false;
  }

  map_odom_ = candidate_map_odom;
  map_odom_.orientation.normalize();
  have_map_odom_ = true;
  // 人工初值/重定位后的首次 GM 结果作为锚点确认，一次性绕过跳变门限
  jump_guard_bypass_ = false;
  return true;
}

bool FusionCore::setMapOdom(const Pose3d& map_odom) {
  if (!isFinite(map_odom.position) || !isFinite(map_odom.orientation)) {
    return false;
  }
  map_odom_ = map_odom;
  map_odom_.orientation.normalize();
  have_map_odom_ = true;
  return true;
}




bool FusionCore::updateMapOdomFromMapBase(const Pose3d& map_base, bool require_slam) {
  if (!isFinite(map_base.position) || !isFinite(map_base.orientation)) {
    return false;
  }
  if (require_slam && !have_slam_) {
    return false;
  }

  // map_odom = map_base ⊕ (odom_base)⁻¹；无 slam 时 odom_base 视为单位变换
  map_odom_ = have_slam_ ? compose(map_base, inverse(odom_base_)) : map_base;
  map_odom_.orientation.normalize();
  have_map_odom_ = true;
  return true;
}

Pose3d FusionCore::fusedPose() const {
  return compose(map_odom_, odom_base_);
}

}  // namespace localization_v2
