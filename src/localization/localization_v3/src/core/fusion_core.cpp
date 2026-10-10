#include "localization_v3/core/fusion_core.hpp"

namespace localization_v3 {

void FusionCore::setConfig(const Config& config) {
  config_ = config;
  jump_guard_.setConfig(config.jump_guard);
}

void FusionCore::setOdomBase(const Pose3d& odom_base) {
  if (!isFinite(odom_base.position) || !isFinite(odom_base.orientation)) {
    return;
  }
  odom_base_ = odom_base;
  odom_base_.orientation.normalize();
  have_slam_ = true;
}

Pose3d FusionCore::fusedPose() const {
  return compose(map_odom_, odom_base_);
}

bool FusionCore::inManualSettle(double now_sec) const {
  return manual_verify_pending_ && now_sec < manual_settle_until_;
}

void FusionCore::commit(const Pose3d& map_odom, double now_sec) {
  map_odom_ = map_odom;
  map_odom_.orientation.normalize();
  have_map_odom_ = true;
  last_correction_stamp_ = now_sec;
  // 位姿基准更新：清空大跳确认缓冲，后续判定相对新基准进行
  jump_guard_.reset();
}

bool FusionCore::applyManualPose(const Pose3d& map_base, bool require_slam, double now_sec) {
  if (!isFinite(map_base.position) || !isFinite(map_base.orientation)) {
    return false;
  }
  if (require_slam && !have_slam_) {
    return false;
  }
  // map_odom = map_base ⊕ (odom_base)⁻¹；无 slam 时 odom_base 视为单位变换
  const Pose3d map_odom = have_slam_ ? compose(map_base, inverse(odom_base_)) : map_base;

  // 人工位姿：无条件写入（不经 jump guard），并开启 settle + 人工验证流程
  commit(map_odom, now_sec);
  manual_verify_pending_ = true;
  manual_pose_stamp_ = now_sec;
  manual_settle_until_ = now_sec + config_.manual_settle_sec;
  return true;
}

PoseUpdateResult FusionCore::applyGlobalMatch(const Pose3d& candidate_map_odom,
                                              QualityLevel quality, double now_sec) {
  if (!isFinite(candidate_map_odom.position) || !isFinite(candidate_map_odom.orientation)) {
    return PoseUpdateResult::kRejectedQuality;
  }

  // ---- 人工 settle 窗口：丢弃结果（等 GM 以人工锚点算出的新鲜结果，堵回调竞态）----
  if (inManualSettle(now_sec)) {
    return PoseUpdateResult::kRejectedStale;
  }

  // ---- 人工验证：首个质量 good 的结果即验证通过，直接采纳确立新基准 ----
  // 不与人工位姿比距离（人工位姿本身不精确）；距离合理性由 GM 自身 max_delta 门限限定
  if (manual_verify_pending_) {
    if (quality != QualityLevel::kGood) {
      return PoseUpdateResult::kRejectedQuality;
    }
    commit(candidate_map_odom, now_sec);
    manual_verify_pending_ = false;
    return PoseUpdateResult::kAcceptedVerified;
  }

  // ---- 常规跟踪修正：质量 bad 不写入；good/degraded 过 jump guard ----
  if (quality == QualityLevel::kBad) {
    return PoseUpdateResult::kRejectedQuality;
  }
  const JumpGuardVerdict verdict =
      jump_guard_.evaluate(map_odom_, candidate_map_odom, odom_base_, now_sec);
  switch (verdict) {
    case JumpGuardVerdict::kAccept:
      commit(candidate_map_odom, now_sec);
      return PoseUpdateResult::kAccepted;
    case JumpGuardVerdict::kConfirmPending:
      return PoseUpdateResult::kConfirmPending;
    case JumpGuardVerdict::kReject:
      return PoseUpdateResult::kRejectedGuard;
  }
  return PoseUpdateResult::kRejectedGuard;
}

bool FusionCore::applyRelocalization(const Pose3d& map_base, bool require_slam, double now_sec) {
  if (!isFinite(map_base.position) || !isFinite(map_base.orientation)) {
    return false;
  }
  if (require_slam && !have_slam_) {
    return false;
  }
  // P5 修复：重定位结果是 map->base，必须反推 map->odom，不得直接当作 map->odom 写入
  const Pose3d map_odom = have_slam_ ? compose(map_base, inverse(odom_base_)) : map_base;
  commit(map_odom, now_sec);
  // 重定位成功即完成人工验证（若有）：位姿已由全局搜索给出
  manual_verify_pending_ = false;
  return true;
}

bool FusionCore::applyRecoveryMatch(const Pose3d& candidate_map_odom, double now_sec) {
  if (!isFinite(candidate_map_odom.position) || !isFinite(candidate_map_odom.orientation)) {
    return false;
  }
  // 恢复期 / 初始化：无 jump guard（恢复本来就是大跳变）
  commit(candidate_map_odom, now_sec);
  manual_verify_pending_ = false;
  return true;
}

}  // namespace localization_v3
