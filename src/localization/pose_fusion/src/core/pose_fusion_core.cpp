/**
 * @file pose_fusion_core.cpp
 * @brief 位姿融合算法实现：DR 递推 + SLAM 局部平滑 + Global Matcher 全局平滑 + 硬重置
 *
 * 坐标合成：
 *   map_base = map_odom ∘ odom_base
 *   - odom_base：随 DR 增量前进；SLAM 以小增益拉回
 *   - map_odom：随 GM / 重定位在 map 系校准；平时保持缓变
 */

#include "pose_fusion/core/pose_fusion_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace pose_fusion {

bool PoseFusionCore::initialize(const FusionConfig& config) {
  config_ = config;
  current_map_id_ = config.default_map_id;
  gate_.setConfig(config_);
  initialized_config_ = true;
  policy_ = ObserverPolicyData();  // 默认全开、权重 1
  state_ = FusionState();
  clearOdomBaseHistory();
  clearLocalPredictorHistory();
  state_.pose_covariance = initialCovariance(config_.initialization);
  resetEskfFromState();
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
  pushLocalPredictorHistory(obs);
  if (!policy_.use_dr) {
    recordRejected(ObservationSource::DR, "policy_disabled");
    return;
  }

  if (pending_reset_) {
    commitResetWithDr(*pending_reset_, obs);
    pending_reset_.reset();
    return;
  }

  if (!state_.initialized) {
    if (config_.initialization.allow_dr_initialization) {
      initializeAtCurrentDr(obs);
      recordUsed(ObservationSource::DR, "debug_initialize_from_dr");
    } else {
      state_machine_.setState(FusionStateName::WAIT_INITIAL_POSE);
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
  pushLocalPredictorHistory(obs);
  if (!state_.initialized) {
    if (config_.initialization.allow_slam_initialization && !pending_reset_) {
      initializeFromSlam(obs);
      recordUsed(ObservationSource::SLAM, "debug_initialize_from_slam");
    }
    return;
  }
  updateWithSlam(obs);
}

void PoseFusionCore::feedGlobalMatch(const PoseObservation& obs) {
  buffer_.setGlobalMatcher(obs);
  if (!initialized_config_) {
    return;
  }
  if (!config_.global_matcher.enabled) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "global_matcher_disabled");
    return;
  }
  // During startup, manager policy may mark GM unavailable because fusion is not initialized yet.
  // Do not let that availability policy block GM-based initialization.
  if (!state_.initialized) {
    if (config_.initialization.allow_global_matcher_initialization) {
      initializeFromGlobalMatcher(obs);
    } else {
      recordRejected(ObservationSource::GLOBAL_MATCHER, "global_matcher_initialization_disabled");
    }
    return;
  }
  if (!policy_.use_global_matcher) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "policy_disabled");
    return;
  }
  if (config_.global_matcher.use_only_for_initialization) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "global_matcher_update_disabled_after_initialization");
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

  pending_reset_ = reset_obs;
  fusion_odom_from_slam_odom_.reset();
  last_slam_.reset();
  last_slam_fusion_pose_.reset();
  last_slam_time_ = 0.0;
  if (reset_obs.source == ObservationSource::RELOCALIZATION) {
    last_relocalization_time_ = reset_obs.received_timestamp > 0.0 ? reset_obs.received_timestamp : reset_obs.timestamp;
  }

  if (last_dr_) {
    commitResetWithDr(reset_obs, *last_dr_);
    pending_reset_.reset();
    return;
  }

  if (!config_.initialization.require_predictor_for_reset) {
    commitResetAtOrigin(reset_obs);
    pending_reset_.reset();
    return;
  }

  state_ = FusionState();
  state_machine_.setState(FusionStateName::WAIT_PREDICTOR_READY);
  current_map_id_ = reset_obs.map_id.empty() ? config_.default_map_id : reset_obs.map_id;
  recordUsed(reset_obs.source, "pending_reset_wait_dr:" + reset_obs.reason);
}

FusionOutput PoseFusionCore::tick(double now) {
  if (pending_reset_ && buffer_.latestDr()) {
    commitResetWithDr(*pending_reset_, *buffer_.latestDr());
    pending_reset_.reset();
  }
  if (!state_.initialized && config_.initialization.allow_dr_initialization && buffer_.latestDr()) {
    initializeAtCurrentDr(*buffer_.latestDr());
    recordUsed(ObservationSource::DR, "lazy_debug_initialize_from_dr");
  }
  if (!state_.initialized && config_.initialization.allow_slam_initialization && !pending_reset_ && buffer_.latestSlam()) {
    initializeFromSlam(*buffer_.latestSlam());
    recordUsed(ObservationSource::SLAM, "lazy_debug_initialize_from_slam");
  }
  if (!state_.initialized && !pending_reset_ && !config_.initialization.allow_dr_initialization &&
      !config_.initialization.allow_slam_initialization) {
    state_machine_.setState(FusionStateName::WAIT_INITIAL_POSE);
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
  // 新鲜度：超过 buffer.max_*_age 则状态机降级判定时视为不可用
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

void PoseFusionCore::initializeAtCurrentDr(const PoseObservation& obs) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = obs.timestamp;
  state_.odom_base = obs.pose;
  state_.map_odom = Pose3d();
  state_.velocity_odom = obs.velocity;
  state_.pose_covariance = initialCovariance(config_.initialization);
  resetEskfFromState();
  current_map_id_ = config_.default_map_id;
  last_dr_ = obs;
  last_dr_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  clearOdomBaseHistory();
  pushOdomBaseHistory(state_.timestamp);
  fusion_odom_from_slam_odom_.reset();
  last_slam_fusion_pose_.reset();
  consecutive_slam_residual_rejects_ = 0;
  state_machine_.setState(FusionStateName::DR_ONLY);
}

void PoseFusionCore::commitResetWithDr(const ResetObservation& reset_obs, const PoseObservation& dr_obs) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = std::max(reset_obs.timestamp, dr_obs.timestamp);
  state_.odom_base = dr_obs.pose;
  state_.map_odom = compose(reset_obs.map_base, inverse(dr_obs.pose));
  state_.velocity_odom = dr_obs.velocity;
  state_.pose_covariance = reset_obs.covariance;
  resetEskfFromState();
  current_map_id_ = reset_obs.map_id.empty() ? config_.default_map_id : reset_obs.map_id;
  last_dr_ = dr_obs;
  last_dr_time_ = dr_obs.received_timestamp > 0.0 ? dr_obs.received_timestamp : dr_obs.timestamp;
  clearOdomBaseHistory();
  pushOdomBaseHistory(dr_obs.timestamp);
  last_slam_.reset();
  fusion_odom_from_slam_odom_.reset();
  last_slam_fusion_pose_.reset();
  consecutive_slam_residual_rejects_ = 0;
  state_machine_.setState(FusionStateName::RUNNING);
  recordUsed(reset_obs.source, "reset_aligned_with_dr:" + reset_obs.reason);
}

void PoseFusionCore::commitResetAtOrigin(const ResetObservation& reset_obs) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = reset_obs.timestamp;
  state_.odom_base = Pose3d();
  state_.map_odom = reset_obs.map_base;
  state_.pose_covariance = reset_obs.covariance;
  resetEskfFromState();
  current_map_id_ = reset_obs.map_id.empty() ? config_.default_map_id : reset_obs.map_id;
  last_dr_.reset();
  last_slam_.reset();
  clearOdomBaseHistory();
  pushOdomBaseHistory(state_.timestamp);
  fusion_odom_from_slam_odom_.reset();
  last_slam_fusion_pose_.reset();
  consecutive_slam_residual_rejects_ = 0;
  state_machine_.setState(FusionStateName::RUNNING);
  recordUsed(reset_obs.source, "debug_reset_at_origin:" + reset_obs.reason);
}

void PoseFusionCore::initializeFromSlam(const PoseObservation& obs) {
  state_ = FusionState();
  state_.initialized = true;
  state_.timestamp = obs.timestamp;
  state_.odom_base = obs.pose;
  state_.map_odom = Pose3d();
  state_.velocity_odom = obs.velocity;
  state_.pose_covariance = initialCovariance(config_.initialization);
  resetEskfFromState();
  current_map_id_ = config_.default_map_id;
  last_slam_ = obs;
  last_slam_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  clearOdomBaseHistory();
  pushOdomBaseHistory(state_.timestamp);
  fusion_odom_from_slam_odom_ = Pose3d();  // compatibility flag: SLAM reference is ready.
  last_slam_fusion_pose_ = state_.odom_base;
  consecutive_slam_residual_rejects_ = 0;
  state_machine_.setState(FusionStateName::LOCAL_FUSION);
}

bool PoseFusionCore::initializeFromGlobalMatcher(const PoseObservation& obs) {
  std::string reason;
  if (!gate_.validateGlobalMatcher(obs, &reason)) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "init_" + reason);
    return false;
  }

  double time_offset = 0.0;
  bool predictor_is_slam = false;
  const auto predictor = lookupLocalPredictorAt(obs.timestamp, &time_offset, &predictor_is_slam);

  state_ = FusionState();
  state_.initialized = true;
  state_.pose_covariance = initialCovariance(config_.initialization);
  current_map_id_ = obs.map_id.empty() ? config_.default_map_id : obs.map_id;
  clearOdomBaseHistory();

  if (!predictor) {
    // GM already estimates T_map_base from the current scan. If local odometry has not produced a
    // time-aligned predictor yet, use an identity local frame at the matched scan time and let the
    // first DR/SLAM observation lock the local odom origin afterwards.
    state_.timestamp = obs.timestamp;
    state_.odom_base = Pose3d();
    state_.map_odom = obs.pose;
    resetEskfFromState();
    pushOdomBaseHistory(state_.timestamp);
    last_dr_.reset();
    last_slam_.reset();
    last_slam_time_ = 0.0;
    fusion_odom_from_slam_odom_.reset();
    last_slam_fusion_pose_.reset();
    consecutive_slam_residual_rejects_ = 0;
    last_global_matcher_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
    state_machine_.setState(FusionStateName::MAP_CONSTRAINED);
    recordUsed(ObservationSource::GLOBAL_MATCHER,
               "initialize_from_global_matcher_origin_reset_slam_extrinsic");
    return true;
  }

  const PoseObservation predictor_obs = *predictor;
  state_.timestamp = std::max(obs.timestamp, predictor_obs.timestamp);
  state_.odom_base = predictor_obs.pose;
  state_.map_odom = compose(obs.pose, inverse(predictor_obs.pose));
  state_.velocity_odom = predictor_obs.velocity;
  resetEskfFromState();

  if (predictor_is_slam) {
    // GM establishes the global initialization. Do not keep the SLAM predictor as an already
    // locked extrinsic; the next SLAM observation must relock against odom_base_history by stamp.
    last_slam_.reset();
    last_slam_time_ = 0.0;
  } else {
    last_dr_ = predictor_obs;
    last_dr_time_ = predictor_obs.received_timestamp > 0.0 ? predictor_obs.received_timestamp : predictor_obs.timestamp;
  }
  fusion_odom_from_slam_odom_.reset();
  last_slam_fusion_pose_.reset();
  consecutive_slam_residual_rejects_ = 0;
  pushOdomBaseHistory(predictor_obs.timestamp);
  last_global_matcher_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  state_machine_.setState(FusionStateName::MAP_CONSTRAINED);
  recordUsed(ObservationSource::GLOBAL_MATCHER,
             "initialize_from_global_matcher_dt=" + std::to_string(time_offset) +
             "_reset_slam_extrinsic");
  return true;
}

void PoseFusionCore::propagateWithDr(const PoseObservation& obs) {
  // 首帧仅锁定参考，不递推
  if (!last_dr_) {
    last_dr_ = obs;
    last_dr_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
    return;
  }
  const double dt = obs.timestamp - last_dr_->timestamp;
  if (dt <= 1e-9) {
    // DR may publish faster than its internal filter timestamp advances. Equal timestamps are
    // duplicate state snapshots, not motion increments. Negative dt means the stream jumped back.
    if (dt < -1e-6) {
      recordRejected(ObservationSource::DR, "dr_time_went_backwards");
    }
    return;
  }
  // delta = inv(last) ∘ current：相邻两帧 DR 的相对运动
  const Pose3d delta = between(last_dr_->pose, obs.pose);
  std::string reason;
  if (!gate_.validateDrDelta(delta, dt, &reason)) {
    // 跳过本步递推，但仍推进 last_dr_，避免卡死在坏帧上
    last_dr_ = obs;
    last_dr_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
    recordRejected(ObservationSource::DR, reason);
    return;
  }
  eskf_.predict(delta, obs.velocity, dt, config_.prediction);
  syncStateFromEskf(obs.timestamp);
  pushOdomBaseHistory(state_.timestamp);
  last_dr_ = obs;
  last_dr_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  recordUsed(ObservationSource::DR, "eskf_predict_delta");
}

void PoseFusionCore::updateWithSlam(const PoseObservation& obs) {
  double curr_time_offset = 0.0;
  const auto curr_odom_base = lookupOdomBaseAt(obs.timestamp, &curr_time_offset);
  if (!curr_odom_base) {
    recordRejected(ObservationSource::SLAM, "slam_no_time_aligned_curr_odom");
    return;
  }
  if (std::abs(curr_time_offset) > config_.buffer.max_slam_time_offset) {
    recordRejected(ObservationSource::SLAM, "slam_curr_odom_time_offset_too_large");
    return;
  }

  // 首帧只建立相对增量参考。后续比较 [last_slam.t, curr_slam.t] 同一时间段的运动。
  if (!last_slam_) {
    fusion_odom_from_slam_odom_ = Pose3d();  // compatibility flag: SLAM reference is ready.
    last_slam_ = obs;
    last_slam_fusion_pose_ = *curr_odom_base;
    last_slam_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
    consecutive_slam_residual_rejects_ = 0;
    recordUsed(ObservationSource::SLAM,
               "align_slam_relative_origin_curr_dt=" + std::to_string(curr_time_offset));
    return;
  }

  const double slam_dt = obs.timestamp - last_slam_->timestamp;
  if (!std::isfinite(slam_dt) || slam_dt <= 1e-6) {
    recordRejected(ObservationSource::SLAM, "slam_non_positive_dt");
    return;
  }

  double prev_time_offset = 0.0;
  const auto prev_odom_base = lookupOdomBaseAt(last_slam_->timestamp, &prev_time_offset);
  if (!prev_odom_base) {
    resetSlamAlignment("slam_prev_no_time_aligned_odom");
    return;
  }
  if (std::abs(prev_time_offset) > config_.buffer.max_slam_time_offset) {
    resetSlamAlignment("slam_prev_odom_time_offset_too_large");
    return;
  }

  const Pose3d delta_slam = between(last_slam_->pose, obs.pose);
  const Pose3d observed_at_curr_stamp = compose(*prev_odom_base, delta_slam);

  std::string reason;
  if (!gate_.validateSlamResidual(*curr_odom_base, observed_at_curr_stamp, &reason)) {
    if (isSlamResidualReject(reason)) {
      ++consecutive_slam_residual_rejects_;
      if (config_.slam.relock_after_residual_rejects > 0 &&
          consecutive_slam_residual_rejects_ >= config_.slam.relock_after_residual_rejects) {
        resetSlamAlignment("slam_relative_relock_after_" + std::to_string(consecutive_slam_residual_rejects_) +
                           "_residual_rejects");
        return;
      }
    }
    recordRejected(ObservationSource::SLAM, reason);
    return;
  }

  // No replay yet: convert the correction at obs.timestamp into an equivalent correction on the
  // current ESKF state, instead of directly updating the current state with a past-time pose.
  const Pose3d curr_stamp_correction = between(*curr_odom_base, observed_at_curr_stamp);
  const Pose3d observed_at_state_time = compose(state_.odom_base, curr_stamp_correction);

  const double weight = clamp(policy_.slam_weight, 0.0, 5.0);
  const Mat6d observation_covariance = slamObservationCovariance(obs);
  if (!eskf_.updatePose(observed_at_state_time, observation_covariance, weight)) {
    recordRejected(ObservationSource::SLAM, "slam_relative_eskf_update_failed");
    return;
  }

  syncStateFromEskf(std::max(state_.timestamp, obs.timestamp));
  pushOdomBaseHistory(state_.timestamp);
  last_slam_ = obs;
  last_slam_fusion_pose_ = observed_at_curr_stamp;
  last_slam_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  consecutive_slam_residual_rejects_ = 0;
  recordUsed(ObservationSource::SLAM,
             "eskf_relative_update_prev_dt=" + std::to_string(prev_time_offset) +
             "_curr_dt=" + std::to_string(curr_time_offset));
}

void PoseFusionCore::updateWithGlobalMatcher(const PoseObservation& obs) {
  std::string reason;
  if (!gate_.validateGlobalMatcher(obs, &reason)) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, reason);
    return;
  }
  // GM 给出 map_base(t_cloud)；必须用同一时刻的 odom_base(t_cloud) 反解 map_odom。
  double odom_time_offset = 0.0;
  const auto odom_base_at_obs = lookupOdomBaseAt(obs.timestamp, &odom_time_offset);
  if (!odom_base_at_obs) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "global_match_no_time_aligned_odom");
    return;
  }
  if (std::abs(odom_time_offset) > config_.buffer.max_global_matcher_time_offset) {
    recordRejected(ObservationSource::GLOBAL_MATCHER, "global_match_odom_time_offset_too_large");
    return;
  }
  const Pose3d observed_map_odom = compose(obs.pose, inverse(*odom_base_at_obs));
  if (!gate_.validateGlobalResidual(state_.map_odom, observed_map_odom, &reason)) {
    const double trans = translationNorm(state_.map_odom, observed_map_odom);
    const double yaw = std::abs(yawDiff(state_.map_odom, observed_map_odom));
    const auto& cfg = config_.global_matcher;
    const bool residual_in_reanchor_range =
        trans <= cfg.hard_reanchor_max_position_residual &&
        yaw <= cfg.hard_reanchor_max_yaw_residual_deg * M_PI / 180.0;
    if (cfg.enable_hard_reanchor && residual_in_reanchor_range && isStrongGlobalMatchForReanchor(obs)) {
      state_.map_odom = observed_map_odom;
      current_map_id_ = obs.map_id.empty() ? current_map_id_ : obs.map_id;
      last_global_matcher_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
      state_.pose_covariance.block<3, 3>(0, 0) *= 0.5;
      state_.pose_covariance(5, 5) *= 0.5;
      recordUsed(ObservationSource::GLOBAL_MATCHER,
                 "hard_reanchor_" + reason + "_dt=" + std::to_string(odom_time_offset));
      return;
    }
    recordRejected(ObservationSource::GLOBAL_MATCHER, reason);
    return;
  }

  // 增益随匹配质量自适应，再乘策略权重，并限幅到 [min_gain, max_gain]
  const auto& cfg = config_.global_matcher;
  double alpha = cfg.base_gain;
  alpha *= clamp(obs.inlier_ratio / std::max(1e-3, cfg.target_inlier_ratio), 0.3, 1.5);
  alpha *= clamp(cfg.target_rmse / std::max(obs.inlier_rmse, 1e-3), 0.3, 1.5);
  alpha *= clamp(policy_.global_matcher_weight, 0.0, 2.0);
  if (!obs.converged) {
    alpha *= 0.5;  // 未收敛时更保守
  }
  alpha = clamp(alpha, cfg.min_gain, cfg.max_gain);

  state_.map_odom = interpolate(state_.map_odom, observed_map_odom, alpha);
  current_map_id_ = obs.map_id.empty() ? current_map_id_ : obs.map_id;
  last_global_matcher_time_ = obs.received_timestamp > 0.0 ? obs.received_timestamp : obs.timestamp;
  // 接受全局约束后略微收缩位置/航向协方差
  state_.pose_covariance.block<3, 3>(0, 0) *= 0.8;
  state_.pose_covariance(5, 5) *= 0.8;
  recordUsed(ObservationSource::GLOBAL_MATCHER, "map_odom_smooth_update_dt=" + std::to_string(odom_time_offset));
}

bool PoseFusionCore::isStrongGlobalMatchForReanchor(const PoseObservation& obs) const {
  const auto& cfg = config_.global_matcher;
  if (!obs.success) {
    return false;
  }
  if (!obs.converged && !cfg.allow_converged_false) {
    return false;
  }
  return obs.inlier_ratio >= cfg.hard_reanchor_min_inlier_ratio &&
         obs.inlier_rmse <= cfg.hard_reanchor_max_inlier_rmse &&
         obs.fitness_score <= cfg.hard_reanchor_max_fitness;
}

void PoseFusionCore::pushLocalPredictorHistory(const PoseObservation& obs) {
  if (!std::isfinite(obs.timestamp) || obs.timestamp <= 0.0) {
    return;
  }
  auto& history = obs.source == ObservationSource::SLAM ? slam_history_ : dr_history_;
  if (!history.empty() && std::abs(history.back().timestamp - obs.timestamp) < 1e-6) {
    history.back() = obs;
  } else {
    history.push_back(obs);
  }
  trimLocalPredictorHistory(obs.timestamp);
}

std::optional<PoseObservation> PoseFusionCore::lookupLocalPredictorAt(double timestamp,
                                                                       double* time_offset,
                                                                       bool* is_slam) const {
  if (!std::isfinite(timestamp) || timestamp <= 0.0) {
    return std::nullopt;
  }

  auto find_best = [&](const std::deque<PoseObservation>& history) -> std::optional<PoseObservation> {
    if (history.empty()) {
      return std::nullopt;
    }
    auto best = history.begin();
    double best_abs_dt = std::abs(best->timestamp - timestamp);
    for (auto it = std::next(history.begin()); it != history.end(); ++it) {
      const double abs_dt = std::abs(it->timestamp - timestamp);
      if (abs_dt < best_abs_dt) {
        best = it;
        best_abs_dt = abs_dt;
      }
    }
    if (best_abs_dt > config_.initialization.global_matcher_init_max_predictor_time_offset) {
      return std::nullopt;
    }
    return *best;
  };

  const auto dr = find_best(dr_history_);
  const auto slam = find_best(slam_history_);
  if (!dr && !slam) {
    return std::nullopt;
  }
  if (dr && (!slam || std::abs(dr->timestamp - timestamp) <= std::abs(slam->timestamp - timestamp))) {
    if (time_offset) *time_offset = dr->timestamp - timestamp;
    if (is_slam) *is_slam = false;
    return dr;
  }
  if (time_offset) *time_offset = slam->timestamp - timestamp;
  if (is_slam) *is_slam = true;
  return slam;
}

void PoseFusionCore::trimLocalPredictorHistory(double newest_timestamp) {
  const double min_time = newest_timestamp - std::max(1.0, config_.buffer.odom_history_duration);
  auto trim = [&](std::deque<PoseObservation>* history) {
    while (!history->empty() && history->front().timestamp < min_time) {
      history->pop_front();
    }
    while (history->size() > config_.buffer.max_queue_size) {
      history->pop_front();
    }
  };
  trim(&dr_history_);
  trim(&slam_history_);
}

void PoseFusionCore::clearLocalPredictorHistory() {
  dr_history_.clear();
  slam_history_.clear();
}

void PoseFusionCore::syncStateFromEskf(double timestamp) {
  state_.odom_base = eskf_.pose();
  state_.velocity_odom = eskf_.velocity();
  state_.pose_covariance = eskf_.covariance();
  if (std::isfinite(timestamp) && timestamp > 0.0) {
    state_.timestamp = timestamp;
  }
}

void PoseFusionCore::resetEskfFromState() {
  eskf_.reset(state_.odom_base, state_.velocity_odom, state_.pose_covariance);
}

Mat6d PoseFusionCore::slamObservationCovariance(const PoseObservation& obs) const {
  Mat6d cov = Mat6d::Zero();
  const double pos_var = std::pow(std::max(1e-3, config_.slam.observation_std_position), 2.0);
  const double yaw_var = std::pow(std::max(1e-3, config_.slam.observation_std_yaw_deg) * M_PI / 180.0, 2.0);
  cov.block<3, 3>(0, 0) = Mat3d::Identity() * pos_var;
  cov.block<3, 3>(3, 3) = Mat3d::Identity() * yaw_var;

  bool usable_msg_cov = true;
  for (int i = 0; i < 6; ++i) {
    usable_msg_cov = usable_msg_cov && std::isfinite(obs.covariance(i, i)) && obs.covariance(i, i) > 1e-9;
  }
  if (usable_msg_cov) {
    cov = obs.covariance;
  }
  return cov;
}

bool PoseFusionCore::isSlamResidualReject(const std::string& reason) const {
  return reason == "slam_residual_translation_too_large" || reason == "slam_residual_yaw_too_large";
}

void PoseFusionCore::resetSlamAlignment(const std::string& detail) {
  fusion_odom_from_slam_odom_.reset();
  last_slam_.reset();
  last_slam_fusion_pose_.reset();
  last_slam_time_ = 0.0;
  consecutive_slam_residual_rejects_ = 0;
  recordRejected(ObservationSource::SLAM, detail);
}

void PoseFusionCore::pushOdomBaseHistory(double timestamp) {
  if (!std::isfinite(timestamp) || timestamp <= 0.0 || !state_.initialized) {
    return;
  }
  OdomBaseSample sample;
  sample.timestamp = timestamp;
  sample.odom_base = state_.odom_base;
  if (!odom_base_history_.empty() && timestamp < odom_base_history_.back().timestamp) {
    auto it = std::lower_bound(odom_base_history_.begin(), odom_base_history_.end(), timestamp,
                               [](const OdomBaseSample& lhs, double rhs) {
                                 return lhs.timestamp < rhs;
                               });
    odom_base_history_.insert(it, sample);
  } else if (!odom_base_history_.empty() && std::abs(timestamp - odom_base_history_.back().timestamp) < 1e-6) {
    odom_base_history_.back() = sample;
  } else {
    odom_base_history_.push_back(sample);
  }

  const double min_time = timestamp - std::max(0.1, config_.buffer.odom_history_duration);
  while (!odom_base_history_.empty() && odom_base_history_.front().timestamp < min_time) {
    odom_base_history_.pop_front();
  }
  while (odom_base_history_.size() > config_.buffer.max_queue_size) {
    odom_base_history_.pop_front();
  }
}

std::optional<Pose3d> PoseFusionCore::lookupOdomBaseAt(double timestamp, double* time_offset) const {
  if (time_offset) {
    *time_offset = 1e9;
  }
  if (!std::isfinite(timestamp) || timestamp <= 0.0 || odom_base_history_.empty()) {
    return std::nullopt;
  }
  if (odom_base_history_.size() == 1) {
    if (time_offset) {
      *time_offset = odom_base_history_.front().timestamp - timestamp;
    }
    return odom_base_history_.front().odom_base;
  }

  auto upper = std::lower_bound(odom_base_history_.begin(), odom_base_history_.end(), timestamp,
                                [](const OdomBaseSample& lhs, double rhs) {
                                  return lhs.timestamp < rhs;
                                });
  if (upper == odom_base_history_.begin()) {
    if (time_offset) {
      *time_offset = upper->timestamp - timestamp;
    }
    return upper->odom_base;
  }
  if (upper == odom_base_history_.end()) {
    const auto& last = odom_base_history_.back();
    if (time_offset) {
      *time_offset = last.timestamp - timestamp;
    }
    return last.odom_base;
  }

  const auto& next = *upper;
  const auto& prev = *(upper - 1);
  const double span = next.timestamp - prev.timestamp;
  if (span <= 1e-9) {
    if (time_offset) {
      *time_offset = prev.timestamp - timestamp;
    }
    return prev.odom_base;
  }
  const double alpha = (timestamp - prev.timestamp) / span;
  if (time_offset) {
    *time_offset = std::min(std::abs(timestamp - prev.timestamp), std::abs(next.timestamp - timestamp));
  }
  return interpolate(prev.odom_base, next.odom_base, alpha);
}

void PoseFusionCore::clearOdomBaseHistory() {
  odom_base_history_.clear();
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
  // 速度从 odom 系旋到 map 系（仅旋转，忽略平移）
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
