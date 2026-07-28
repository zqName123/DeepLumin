#include "simple_odom_fusion/core/simple_fusion_core.hpp"

namespace simple_odom_fusion {

void SimpleFusionCore::initialize(const FusionConfig& config) {
  config_ = config;
  sync_.configure(config_.max_queue_size, config_.sync_tolerance);
  state_ = FusionState();
  last_global_correction_stamp_ = -1.0;
}

void SimpleFusionCore::feedDr(const OdomSample& sample) {
  sync_.pushDr(sample);
}

void SimpleFusionCore::feedSlam(const OdomSample& sample) {
  sync_.pushSlam(sample);
}

void SimpleFusionCore::resetWithMapBase(const GlobalObservation& observation) {
  if (!observation.success) {
    return;
  }
  if (!state_.initialized) {
    state_.initialized = true;
    state_.stamp = observation.stamp;
    state_.odom_base = Pose3d();
  }
  resetMapOdomFromMapBase(observation.map_base);
  state_.stamp = observation.stamp;
}

void SimpleFusionCore::correctWithGlobalMatcher(const GlobalObservation& observation) {
  if (!config_.enable_global_matcher_correction || !observation.success || !state_.initialized) {
    return;
  }
  if (last_global_correction_stamp_ > 0.0 &&
      observation.stamp - last_global_correction_stamp_ < config_.global_matcher_min_interval) {
    return;
  }
  const Pose3d observed_map_odom = compose(observation.map_base, inverse(state_.odom_base));
  state_.map_odom = blend(state_.map_odom, observed_map_odom, config_.global_matcher_correction_gain);
  last_global_correction_stamp_ = observation.stamp;
}

std::optional<FusionOutput> SimpleFusionCore::update() {
  std::optional<FusionOutput> latest;
  while (auto pair = sync_.popSynced()) {
    initializeLocalIfNeeded(*pair);
    if (!state_.initialized) {
      continue;
    }
    propagateLocal(*pair);
    latest = output(pair->stamp);
  }
  return latest;
}

FusionOutput SimpleFusionCore::output(double stamp) const {
  FusionOutput out;
  out.valid = state_.initialized;
  out.stamp = stamp > 0.0 ? stamp : state_.stamp;
  out.odom_base = state_.odom_base;
  out.map_odom = state_.map_odom;
  out.map_base = compose(state_.map_odom, state_.odom_base);
  out.velocity_map = state_.map_odom.orientation.normalized() * state_.velocity_odom;
  return out;
}

bool SimpleFusionCore::validDelta(const Pose3d& delta) const {
  if (!isFinite(delta.position) || !isFinite(delta.orientation)) {
    return false;
  }
  const Eigen::AngleAxisd aa(delta.orientation.normalized());
  return delta.position.norm() <= config_.max_delta_translation &&
         std::abs(aa.angle()) <= config_.max_delta_rotation_rad;
}

void SimpleFusionCore::initializeLocalIfNeeded(const SyncedPair& pair) {
  if (state_.initialized) {
    return;
  }
  if (!config_.allow_local_auto_initialization) {
    return;
  }
  state_.initialized = true;
  state_.stamp = pair.stamp;
  state_.odom_base = Pose3d();
  state_.map_odom = Pose3d();
  state_.velocity_odom = pair.dr.linear_velocity;
  state_.last_dr = pair.dr;
  state_.last_slam = pair.slam;
}

void SimpleFusionCore::propagateLocal(const SyncedPair& pair) {
  if (!state_.last_dr || !state_.last_slam) {
    state_.last_dr = pair.dr;
    state_.last_slam = pair.slam;
    state_.stamp = pair.stamp;
    return;
  }

  if (pair.stamp - state_.stamp > config_.max_pair_dt || pair.stamp <= state_.stamp) {
    state_.last_dr = pair.dr;
    state_.last_slam = pair.slam;
    state_.stamp = pair.stamp;
    return;
  }

  const Pose3d previous_odom_base = state_.odom_base;
  const Pose3d dr_delta = between(state_.last_dr->pose, pair.dr.pose);
  Pose3d dr_pred = previous_odom_base;
  if (validDelta(dr_delta)) {
    dr_pred = blend(previous_odom_base, compose(previous_odom_base, dr_delta), config_.dr_prediction_weight);
  }

  Pose3d fused_pred = dr_pred;
  if (config_.use_slam_increment) {
    const Pose3d slam_delta = between(state_.last_slam->pose, pair.slam.pose);
    if (validDelta(slam_delta)) {
      const Pose3d slam_pred = compose(previous_odom_base, slam_delta);
      fused_pred = blend(dr_pred, slam_pred, config_.slam_correction_gain);
    }
  }
  state_.odom_base = fused_pred;

  state_.velocity_odom = pair.dr.linear_velocity;
  state_.last_dr = pair.dr;
  state_.last_slam = pair.slam;
  state_.stamp = pair.stamp;
}

void SimpleFusionCore::resetMapOdomFromMapBase(const Pose3d& map_base) {
  state_.map_odom = compose(map_base, inverse(state_.odom_base));
}

}  // namespace simple_odom_fusion
