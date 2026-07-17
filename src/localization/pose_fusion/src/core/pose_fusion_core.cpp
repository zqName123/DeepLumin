#include "pose_fusion/core/pose_fusion_core.hpp"

#include <algorithm>
#include <cmath>

namespace pose_fusion {

bool PoseFusionCore::initialize(const FusionConfig& config) {
  config_ = config;
  current_map_id_ = config.default_map_id;
  gate_.setConfig(config_);
  initialized_config_ = true;
  policy_ = ObserverPolicyData();
  state_ = FusionState();
  state_.pose_covariance = initialCovariance(config_.initialization);
  state_machine_.setState(FusionStateName::UNINITIALIZED);
  return true;
}

void PoseFusionCore::setObserverPolicy(const ObserverPolicyData& policy) {
  policy_ = policy;
}

void PoseFusionCore::feedDrOdom(const PoseObservation& obs) {
  buffer_.setDr(obs);
  if (!initialized_config_) {
    return;
  }
  std::string reason;
  if (!gate_.validatePoseObservation(obs, &reason)) {
    recordRejected(ObservationSource::DR, reason);
    return;
  }
  if (!policy_.use_dr) {
    recordRejected(ObservationSource::DR, "policy_disabled");
    return;
  }
  if (!state_.initialized) {
    if (config_.initialization.allow_dr_initialization) {
      initializeAtCurrentDrOrOrigin(obs.timestamp);
      last_dr_ = obs;
      last_dr_time_ = obs.timestamp;
      recordUsed(ObservationSource::DR, "initialize_from_dr");
    }
    return;
  }
  propagateWithDr(obs);
}

void PoseFusionCore::feedSlamOdom(const PoseObservation& obs) {
  buffer_.setSlam(obs);
  if (!initialized_config_) {
    return;
  }
  std::string reason;
  if (!config_.slam.enabled || !policy_.use_slam) {
    recordRejected(ObservationSource::SLAM, "policy_disabled");
    return;
  }
  if (!gate_.validatePoseObservation(obs, &reason)) {
    recordRejected(ObservationSource::SLAM, reason);
    return;
  }
  if (!state_.initialized) {
    if (config_.initialization.allow_slam_initialization) {
      initializeFromSlam(obs);
      recordUsed(ObservationSource::SLAM, "initialize_from_slam");
    }
    return;
  }
  updateWithSlam(obs);
}

void PoseFusionCore::feedGlobalMatch(const PoseObservation& obs) {
  buffer_.setGlobalMatcher(obs);
  if (!state_.initialized) {
    return;
  }
  if (!config_.global_matcher.enabled || !policy_.use_global_matcher) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "policy_disabled");
    return;
  }
  updateWithGlobalMatcher(obs);
}

void PoseFusionCore::feedRelocalizationReset(const ResetObservation& reset_obs) {
  buffer_.setRelocalization(reset_obs);
  if (!policy_.use_relocalization || !policy_.allow_fusion_reset) {
    recordRejected(ObservationSource::RELOCALIZATION, "policy_disabled");
    return;
  }
  reset(reset_obs);
}

void PoseFusionCore::reset(const ResetObservation& reset_obs) {
  std::string reason;
  if (!gate_.validateReset(reset_obs, &reason)) {
    recordRejected(reset_obs.source, reason);
    return;
  }
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = reset_obs.timestamp;
  state_.odom_base = Pose3d();
  state_.map_odom = reset_obs.map_base;
  state_.pose_covariance = reset_obs.covariance;
  current_map_id_ = reset_obs.map_id.empty() ? config_.default_map_id : reset_obs.map_id;
  last_dr_.reset();
  last_slam_.reset();
  fusion_odom_from_slam_odom_.reset();
  last_relocalization_time_ = reset_obs.timestamp;
  state_machine_.setState(FusionStateName::MAP_CONSTRAINED);
  recordUsed(reset_obs.source, "reset:" + reset_obs.reason);
}

FusionOutput PoseFusionCore::tick(double now) {
  if (!state_.initialized && config_.initialization.allow_dr_initialization && buffer_.latestDr()) {
    const auto& dr = *buffer_.latestDr();
    initializeAtCurrentDrOrOrigin(dr.timestamp > 0.0 ? dr.timestamp : now);
    last_dr_ = dr;
    last_dr_time_ = dr.timestamp;
    recordUsed(ObservationSource::DR, "lazy_initialize_from_dr");
  }
  if (!state_.initialized && config_.initialization.allow_slam_initialization && buffer_.latestSlam()) {
    initializeFromSlam(*buffer_.latestSlam());
    recordUsed(ObservationSource::SLAM, "lazy_initialize_from_slam");
  }
  updateStateName(now);
  trimDiagnostics();
  return buildOutput(now);
}

FusionStatusData PoseFusionCore::status(double now) const {
  FusionStatusData data;
  data.timestamp = now;
  data.state = state_machine_.state();
  data.initialized = state_.initialized;
  data.output_valid = state_.initialized;
  data.current_map_id = current_map_id_;
  data.active_scene = policy_.scene_type;
  data.last_dr_age = last_dr_time_ > 0.0 ? now - last_dr_time_ : 1e9;
  data.last_slam_age = last_slam_time_ > 0.0 ? now - last_slam_time_ : 1e9;
  data.last_global_matcher_age = last_global_matcher_time_ > 0.0 ? now - last_global_matcher_time_ : 1e9;
  data.last_relocalization_age = last_relocalization_time_ > 0.0 ? now - last_relocalization_time_ : 1e9;
  data.is_dr_available = data.last_dr_age <= config_.buffer.max_dr_age;
  data.is_slam_available = data.last_slam_age <= config_.buffer.max_slam_age;
  data.is_global_matcher_available = data.last_global_matcher_age <= config_.buffer.max_global_matcher_age;
  data.is_relocalization_available = data.last_relocalization_age <= config_.buffer.max_relocalization_age;
  data.position_std = std::sqrt(std::max(0.0, state_.pose_covariance(0, 0)));
  data.velocity_std = 1.0;
  data.yaw_std = std::sqrt(std::max(0.0, state_.pose_covariance(5, 5)));
  data.used_sources.assign(used_sources_.begin(), used_sources_.end());
  data.rejected_sources.assign(rejected_sources_.begin(), rejected_sources_.end());
  data.reject_reasons.assign(reject_reasons_.begin(), reject_reasons_.end());
  if (!data.output_valid) {
    data.failure_reason = "not_initialized";
  }
  return data;
}

void PoseFusionCore::initializeAtCurrentDrOrOrigin(double timestamp) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = timestamp;
  state_.odom_base = Pose3d();
  state_.map_odom = Pose3d();
  state_.pose_covariance = initialCovariance(config_.initialization);
  current_map_id_ = config_.default_map_id;
  state_machine_.setState(FusionStateName::DR_ONLY);
}

void PoseFusionCore::initializeFromSlam(const PoseObservation& obs) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = obs.timestamp;
  state_.odom_base = obs.pose;
  state_.map_odom = Pose3d();
  state_.velocity_odom = obs.velocity;
  state_.pose_covariance = initialCovariance(config_.initialization);
  current_map_id_ = config_.default_map_id;
  last_slam_ = obs;
  last_slam_time_ = obs.timestamp;
  fusion_odom_from_slam_odom_ = Pose3d();
  state_machine_.setState(FusionStateName::LOCAL_FUSION);
}

void PoseFusionCore::propagateWithDr(const PoseObservation& obs) {
  if (!last_dr_) {
    last_dr_ = obs;
    last_dr_time_ = obs.timestamp;
    return;
  }
  const double dt = obs.timestamp - last_dr_->timestamp;
  const Pose3d delta = between(last_dr_->pose, obs.pose);
  std::string reason;
  if (!gate_.validateDrDelta(delta, dt, &reason)) {
    last_dr_ = obs;
    last_dr_time_ = obs.timestamp;
    recordRejected(ObservationSource::DR, reason);
    return;
  }
  state_.odom_base = compose(state_.odom_base, delta);
  state_.velocity_odom = obs.velocity;
  state_.timestamp = obs.timestamp;
  last_dr_ = obs;
  last_dr_time_ = obs.timestamp;

  const double pos_q = config_.prediction.process_noise_position * config_.prediction.process_noise_position * dt;
  const double yaw_q = std::pow(config_.prediction.process_noise_yaw_deg * M_PI / 180.0, 2.0) * dt;
  state_.pose_covariance(0, 0) += pos_q;
  state_.pose_covariance(1, 1) += pos_q;
  state_.pose_covariance(2, 2) += pos_q;
  state_.pose_covariance(5, 5) += yaw_q;
  recordUsed(ObservationSource::DR, "delta_propagation");
}

void PoseFusionCore::updateWithSlam(const PoseObservation& obs) {
  if (!fusion_odom_from_slam_odom_) {
    fusion_odom_from_slam_odom_ = compose(state_.odom_base, inverse(obs.pose));
    last_slam_ = obs;
    last_slam_time_ = obs.timestamp;
    recordUsed(ObservationSource::SLAM, "align_slam_origin");
    return;
  }
  const Pose3d observed_local = compose(*fusion_odom_from_slam_odom_, obs.pose);
  std::string reason;
  if (!gate_.validateSlamResidual(state_.odom_base, observed_local, &reason)) {
    recordRejected(ObservationSource::SLAM, reason);
    return;
  }
  const double alpha = clamp(config_.slam.smooth_gain * policy_.slam_weight, 0.0, 0.5);
  state_.odom_base = interpolate(state_.odom_base, observed_local, alpha);
  last_slam_ = obs;
  last_slam_time_ = obs.timestamp;
  recordUsed(ObservationSource::SLAM, "local_smooth_update");
}

void PoseFusionCore::updateWithGlobalMatcher(const PoseObservation& obs) {
  std::string reason;
  if (!gate_.validateGlobalMatcher(obs, &reason)) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, reason);
    return;
  }
  const Pose3d observed_map_odom = compose(obs.pose, inverse(state_.odom_base));
  if (!gate_.validateGlobalResidual(state_.map_odom, observed_map_odom, &reason)) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, reason);
    return;
  }

  const auto& cfg = config_.global_matcher;
  double alpha = cfg.base_gain;
  alpha *= clamp(obs.inlier_ratio / std::max(1e-3, cfg.target_inlier_ratio), 0.3, 1.5);
  alpha *= clamp(cfg.target_rmse / std::max(obs.inlier_rmse, 1e-3), 0.3, 1.5);
  alpha *= clamp(policy_.global_matcher_weight, 0.0, 2.0);
  if (!obs.converged) {
    alpha *= 0.5;
  }
  alpha = clamp(alpha, cfg.min_gain, cfg.max_gain);

  state_.map_odom = interpolate(state_.map_odom, observed_map_odom, alpha);
  current_map_id_ = obs.map_id.empty() ? current_map_id_ : obs.map_id;
  last_global_matcher_time_ = obs.timestamp;
  state_.pose_covariance.block<3, 3>(0, 0) *= 0.8;
  state_.pose_covariance(5, 5) *= 0.8;
  recordUsed(ObservationSource::GLOBAL_MATCHER, "map_odom_smooth_update");
}

void PoseFusionCore::recordUsed(ObservationSource source, const std::string& detail) {
  used_sources_.push_back(toString(source) + ":" + detail);
  trimDiagnostics();
}

void PoseFusionCore::recordRejected(ObservationSource source, const std::string& reason) {
  rejected_sources_.push_back(toString(source));
  reject_reasons_.push_back(reason);
  trimDiagnostics();
}

void PoseFusionCore::trimDiagnostics() {
  constexpr std::size_t kMax = 20;
  while (used_sources_.size() > kMax) used_sources_.pop_front();
  while (rejected_sources_.size() > kMax) rejected_sources_.pop_front();
  while (reject_reasons_.size() > kMax) reject_reasons_.pop_front();
}

FusionOutput PoseFusionCore::buildOutput(double now) const {
  FusionOutput out;
  out.timestamp = now;
  out.valid = state_.initialized;
  out.map_odom = state_.map_odom;
  out.odom_base = state_.odom_base;
  out.map_base = compose(state_.map_odom, state_.odom_base);
  out.velocity_map = state_.map_odom.orientation.normalized() * state_.velocity_odom;
  out.covariance = state_.pose_covariance;
  return out;
}

void PoseFusionCore::updateStateName(double now) {
  FusionStatusData data = status(now);
  state_machine_.update(data);
}

Mat6d PoseFusionCore::initialCovariance(const InitializationConfig& cfg) {
  Mat6d cov = Mat6d::Identity();
  const double pos = cfg.initial_cov_position * cfg.initial_cov_position;
  const double yaw = std::pow(cfg.initial_cov_yaw_deg * M_PI / 180.0, 2.0);
  cov(0, 0) = pos;
  cov(1, 1) = pos;
  cov(2, 2) = pos;
  cov(3, 3) = 0.1;
  cov(4, 4) = 0.1;
  cov(5, 5) = yaw;
  return cov;
}

}  // namespace pose_fusion
