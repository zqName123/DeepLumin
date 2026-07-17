#include "pose_fusion/core/quality_gate.hpp"

#include <sstream>

namespace pose_fusion {

void QualityGate::setConfig(const FusionConfig& config) {
  config_ = config;
}

bool QualityGate::validatePoseObservation(const PoseObservation& obs, std::string* reason) const {
  if (!obs.valid) {
    if (reason) *reason = obs.reject_reason.empty() ? "invalid_observation" : obs.reject_reason;
    return false;
  }
  if (!std::isfinite(obs.timestamp) || obs.timestamp <= 0.0) {
    if (reason) *reason = "invalid_timestamp";
    return false;
  }
  if (!isFinite(obs.pose.position) || !isFinite(obs.pose.orientation)) {
    if (reason) *reason = "non_finite_pose";
    return false;
  }
  if (obs.source == ObservationSource::DR || obs.source == ObservationSource::SLAM) {
    if (!obs.frame_id.empty() && obs.frame_id != config_.frames.odom) {
      if (reason) *reason = "local_observation_frame_mismatch:" + obs.frame_id;
      return false;
    }
  } else if (obs.source == ObservationSource::GLOBAL_MATCHER || obs.source == ObservationSource::GNSS) {
    if (!obs.frame_id.empty() && obs.frame_id != config_.frames.map) {
      if (reason) *reason = "global_observation_frame_mismatch:" + obs.frame_id;
      return false;
    }
  } else if (!obs.frame_id.empty() && obs.frame_id != config_.frames.odom && obs.frame_id != config_.frames.map) {
    if (reason) *reason = "unexpected_frame:" + obs.frame_id;
    return false;
  }
  if (!obs.child_frame_id.empty() && obs.child_frame_id != config_.frames.base_link) {
    if (reason) *reason = "unexpected_child_frame:" + obs.child_frame_id;
    return false;
  }
  return true;
}

bool QualityGate::validateGlobalMatcher(const PoseObservation& obs, std::string* reason) const {
  if (!validatePoseObservation(obs, reason)) {
    return false;
  }
  if (!obs.success) {
    if (reason) *reason = obs.reject_reason.empty() ? "global_match_failed" : obs.reject_reason;
    return false;
  }
  if (!obs.converged && !config_.global_matcher.allow_converged_false) {
    if (reason) *reason = "global_match_not_converged";
    return false;
  }
  if (!obs.map_id.empty() && obs.map_id != config_.default_map_id) {
    if (reason) *reason = "map_id_mismatch:" + obs.map_id;
    return false;
  }
  if (obs.inlier_ratio < config_.global_matcher.min_inlier_ratio) {
    if (reason) *reason = "global_match_low_inlier_ratio";
    return false;
  }
  if (obs.inlier_rmse > config_.global_matcher.max_inlier_rmse) {
    if (reason) *reason = "global_match_high_inlier_rmse";
    return false;
  }
  if (obs.fitness_score > config_.global_matcher.max_fitness) {
    if (reason) *reason = "global_match_high_fitness";
    return false;
  }
  return true;
}

bool QualityGate::validateReset(const ResetObservation& obs, std::string* reason) const {
  if (!obs.valid) {
    if (reason) *reason = "invalid_reset";
    return false;
  }
  if (!std::isfinite(obs.timestamp) || obs.timestamp <= 0.0) {
    if (reason) *reason = "invalid_reset_timestamp";
    return false;
  }
  if (!isFinite(obs.map_base.position) || !isFinite(obs.map_base.orientation)) {
    if (reason) *reason = "non_finite_reset_pose";
    return false;
  }
  if (!obs.frame_id.empty() && obs.frame_id != config_.frames.map) {
    if (reason) *reason = "reset_frame_mismatch:" + obs.frame_id;
    return false;
  }
  if (!obs.child_frame_id.empty() && obs.child_frame_id != config_.frames.base_link) {
    if (reason) *reason = "reset_child_frame_mismatch:" + obs.child_frame_id;
    return false;
  }
  return true;
}

bool QualityGate::validateDrDelta(const Pose3d& delta, double dt, std::string* reason) const {
  if (!std::isfinite(dt) || dt <= 0.0) {
    if (reason) *reason = "dr_non_positive_dt";
    return false;
  }
  if (dt > config_.prediction.max_dr_gap) {
    if (reason) *reason = "dr_gap_too_large";
    return false;
  }
  if (delta.position.norm() > config_.prediction.max_dr_delta_translation) {
    if (reason) *reason = "dr_delta_translation_too_large";
    return false;
  }
  const double yaw = std::abs(yawFromQuat(delta.orientation));
  if (yaw > config_.prediction.max_dr_delta_yaw_deg * M_PI / 180.0) {
    if (reason) *reason = "dr_delta_yaw_too_large";
    return false;
  }
  return true;
}

bool QualityGate::validateSlamResidual(const Pose3d& predicted, const Pose3d& observed, std::string* reason) const {
  const double trans = translationNorm(predicted, observed);
  const double yaw = std::abs(yawDiff(predicted, observed));
  if (trans > config_.slam.max_position_residual) {
    if (reason) *reason = "slam_residual_translation_too_large";
    return false;
  }
  if (yaw > config_.slam.max_yaw_residual_deg * M_PI / 180.0) {
    if (reason) *reason = "slam_residual_yaw_too_large";
    return false;
  }
  return true;
}

bool QualityGate::validateGlobalResidual(const Pose3d& current_map_odom,
                                         const Pose3d& observed_map_odom,
                                         std::string* reason) const {
  const double trans = translationNorm(current_map_odom, observed_map_odom);
  const double yaw = std::abs(yawDiff(current_map_odom, observed_map_odom));
  if (trans > config_.global_matcher.max_position_residual) {
    if (reason) *reason = "global_residual_translation_too_large";
    return false;
  }
  if (yaw > config_.global_matcher.max_yaw_residual_deg * M_PI / 180.0) {
    if (reason) *reason = "global_residual_yaw_too_large";
    return false;
  }
  return true;
}

}  // namespace pose_fusion
