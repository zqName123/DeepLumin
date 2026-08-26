/**
 * @file manager_core.cpp
 * @brief 定位管理器决策逻辑：场景判定、状态机、观测策略生成、重定位门控
 *
 * 状态机优先级（updateStateLocked，自上而下匹配）：
 *   1) 未收到 initialpose 且 fusion 未 initialized → WAIT_INITIAL_POSE
 *   2) fusion 超时 → FAILURE
 *   3) DR/SLAM/GNSS 全无 → FAILURE
 *   4) 有待发布 accepted_relocalization → RELOCALIZING
 *   5) 连续降级计数达阈值 / DR 或 SLAM 超时 → DEGRADED
 *   6) GM 正在运行（success 但未 converged）→ GLOBAL_MATCHING
 *   7) 否则 → LOCALIZATION
 */

#include "localization_manager/core/manager_core.hpp"

#include <algorithm>
#include <cmath>

namespace localization_manager {
namespace {

std::string joinReason(const std::string& base, const std::string& extra) {
  if (base.empty()) {
    return extra;
  }
  if (extra.empty()) {
    return base;
  }
  return base + "; " + extra;
}

}  // namespace

bool ManagerCore::initialize(const ManagerConfig& config) {
  config_ = config;
  initialized_ = true;
  state_ = ManagerState::WAIT_INITIAL_POSE;
  status_.state = state_;
  status_.active_scene = config_.default_scene;
  status_.current_map_id = config_.default_map_id;
  status_.failure_reason = "waiting for initial pose";
  policy_.scene_type = config_.default_scene;
  policy_.reason = status_.failure_reason;
  return true;
}

// ---------- 回调路径：仅刷新 ModuleHealth / 最新消息，状态转移留给 update() ----------

void ManagerCore::onDrStatus(const ModuleHealth& health) { dr_status_ = health; }
void ManagerCore::onDrOdom(const ModuleHealth& health) { dr_odom_ = health; }
void ManagerCore::onSlamStatus(const ModuleHealth& health) { slam_status_ = health; }
void ManagerCore::onSlamOdom(const ModuleHealth& health) { slam_odom_ = health; }

void ManagerCore::onFusionStatus(const ModuleHealth& health, const deeplumin_msgs::FusionStatus& msg) {
  fusion_status_ = health;
  last_fusion_msg_ = msg;
  status_.initialized = msg.initialized;
  status_.output_valid = msg.output_valid;
  current_map_id_ = msg.current_map_id.empty() ? current_map_id_ : msg.current_map_id;
}

void ManagerCore::onGlobalMatchStatus(const ModuleHealth& health) { global_matcher_status_ = health; }

void ManagerCore::onGlobalMatchResult(const ModuleHealth& health,
                                      const deeplumin_msgs::GlobalMatchResult& msg) {
  global_matcher_result_ = health;
  last_global_match_msg_ = msg;
  current_map_id_ = msg.map_id.empty() ? current_map_id_ : msg.map_id;
}

void ManagerCore::onRelocalizationResult(const ModuleHealth& health,
                                         const deeplumin_msgs::RelocalizationResult& msg) {
  relocalization_status_ = health;
  last_relocalization_msg_ = msg;
  // 通过门控后暂存，等待 onTimer 发布 accepted_relocalization
  if (relocalizationTrusted(msg)) {
    accepted_relocalization_ = msg;
    accepted_relocalization_->accepted = true;
    current_map_id_ = msg.map_id.empty() ? current_map_id_ : msg.map_id;
  }
}

void ManagerCore::onGnss(const GnssView& gnss) {
  gnss_ = gnss;
  has_gnss_ = true;
  last_gnss_time_ = gnss.stamp;
  gnss_heading_only_ = gnss.heading_valid && !gnss.position_valid;
  gnss_full_ = gnss.heading_valid && gnss.position_valid && gnss.velocity_valid;
}

void ManagerCore::onInitialPose(const ros::Time& stamp, const std::string&) {
  last_initial_pose_time_ = stamp;
  has_initial_pose_ = !stamp.isZero();
}

void ManagerCore::onSceneState(const std::string& scene) {
  manual_scene_ = scene.empty() ? config_.default_scene : scene;
  has_manual_scene_ = true;
  last_manual_scene_time_ = ros::Time::now();
}

deeplumin_msgs::RelocalizationResult ManagerCore::takeAcceptedRelocalization() {
  const auto msg = *accepted_relocalization_;
  accepted_relocalization_.reset();
  return msg;
}

void ManagerCore::update(const ros::Time& now) {
  if (!initialized_) {
    initialize(config_);
  }
  last_update_ = now;
  updateSceneLocked(now);
  updateStateLocked(now);
  updateStatusLocked(now);
  updatePolicyLocked(now);
}

void ManagerCore::updateSceneLocked(const ros::Time& now) {
  // 1) 手动场景覆盖（在超时窗口内优先）
  const bool manual_valid = has_manual_scene_ &&
      (config_.scene.source != SceneSource::AUTO_GNSS) &&
      ((now - last_manual_scene_time_).toSec() <= config_.scene.manual_override_timeout_sec);
  if (manual_valid) {
    current_scene_ = manual_scene_;
    scene_reason_ = "manual scene override";
    return;
  }

  // 2) 纯手动模式：无覆盖则用 default_scene
  if (config_.scene.source == SceneSource::MANUAL) {
    current_scene_ = has_manual_scene_ ? manual_scene_ : config_.default_scene;
    scene_reason_ = has_manual_scene_ ? "manual scene" : "default scene";
    return;
  }

  // 3) 自动 GNSS 场景判定
  if (!has_gnss_) {
    current_scene_ = "INDOOR";
    scene_reason_ = "gnss unavailable";
    return;
  }

  const bool gnss_fresh = (now - last_gnss_time_).toSec() <= config_.timeout.gnss;
  const bool gnss_good = gnss_fresh && gnss_.heading_valid &&
                         (gnss_.nsv1 + gnss_.nsv2 >= config_.scene.min_satellites);
  if (gnss_good && gnss_.position_valid && gnss_.velocity_valid) {
    current_scene_ = "OUTDOOR_OPEN";
    scene_reason_ = "gnss reliable";
    gnss_confirm_count_++;
    gnss_lost_count_ = 0;
    return;
  }
  if (gnss_good) {
    current_scene_ = "SEMI_OUTDOOR";
    scene_reason_ = "gnss heading only";
    gnss_confirm_count_++;
    gnss_lost_count_ = 0;
    return;
  }

  current_scene_ = "INDOOR";
  scene_reason_ = "gnss poor";
  gnss_lost_count_++;
}

void ManagerCore::updateStateLocked(const ros::Time& now) {
  (void)now;
  const double dr_age = dr_odom_.age_sec;
  const double slam_age = slam_odom_.age_sec;
  const double fusion_age = fusion_status_.age_sec;
  const bool dr_ok = dr_odom_.available && dr_age <= config_.timeout.dr_odom;
  const bool slam_ok = slam_odom_.available && slam_age <= config_.timeout.slam_odom;
  const bool fusion_ok = fusion_status_.available && fusion_age <= config_.timeout.fused_odom;
  // 有 initialpose 或 fusion 已 initialized 视为初始化就绪
  const bool init_ready = has_initial_pose_ || status_.initialized;

  if (!init_ready) {
    state_ = ManagerState::WAIT_INITIAL_POSE;
    failure_reason_ = "waiting for initial pose";
    return;
  }

  if (!fusion_ok && fusion_age > config_.degrade.fusion_failure_timeout_sec) {
    state_ = ManagerState::FAILURE;
    failure_reason_ = "fused odom timeout";
    return;
  }

  if (!dr_ok && !slam_ok && !has_gnss_) {
    state_ = ManagerState::FAILURE;
    failure_reason_ = "no usable localization source";
    return;
  }

  if (accepted_relocalization_) {
    state_ = ManagerState::RELOCALIZING;
    failure_reason_ = "accepted relocalization ready";
    return;
  }

  // 连续降级计数：SLAM 退化 / fusion 降级 / GM 降级
  if (!slam_ok || slam_status_.level == ModuleLevel::DEGRADED ||
      fusion_status_.level == ModuleLevel::DEGRADED ||
      global_matcher_status_.level == ModuleLevel::DEGRADED) {
    degraded_count_++;
  } else {
    healthy_count_++;
    degraded_count_ = 0;
  }

  if (degraded_count_ >= config_.degrade.enter_degraded_fail_count) {
    state_ = ManagerState::DEGRADED;
    failure_reason_ = joinReason(scene_reason_, "module degraded");
    return;
  }

  if (!dr_ok) {
    state_ = ManagerState::DEGRADED;
    failure_reason_ = joinReason(scene_reason_, "dr timeout");
    return;
  }

  if (!slam_ok) {
    state_ = ManagerState::DEGRADED;
    failure_reason_ = joinReason(scene_reason_, "slam timeout");
    return;
  }

  // GM 有结果但未收敛：认为正在全局匹配中
  if (global_matcher_result_.available &&
      global_matcher_result_.age_sec <= config_.timeout.global_matcher_result &&
      last_global_match_msg_.success && !last_global_match_msg_.converged) {
    state_ = ManagerState::GLOBAL_MATCHING;
    failure_reason_ = "global matcher running";
    return;
  }

  state_ = ManagerState::LOCALIZATION;
  failure_reason_ = scene_reason_.empty() ? "localization nominal" : scene_reason_;
}

void ManagerCore::updateStatusLocked(const ros::Time& now) {
  status_.stamp = now;
  status_.state = state_;
  status_.initialized = status_.initialized || has_initial_pose_;
  status_.output_valid = state_ != ManagerState::FAILURE;
  status_.failure_reason = failure_reason_;
  status_.current_map_id = current_map_id_;
  status_.active_scene = current_scene_;
  status_.last_dr_age = dr_odom_.age_sec;
  status_.last_slam_age = slam_odom_.age_sec;
  status_.last_global_matcher_age = global_matcher_result_.age_sec;
  status_.last_relocalization_age = relocalization_status_.age_sec;
  status_.last_gnss_age = has_gnss_ ? (now - last_gnss_time_).toSec() : 1e9;
  status_.position_std = status_.output_valid ? 1.0 : 10.0;
  status_.velocity_std = status_.output_valid ? 1.0 : 10.0;
  status_.roll_std = 1.0;
  status_.pitch_std = 1.0;
  status_.yaw_std = status_.output_valid ? 2.0 : 20.0;
  status_.used_sources.clear();
  status_.rejected_sources.clear();
  status_.reject_reasons.clear();
  if (dr_odom_.available) status_.used_sources += "dr,";
  if (slam_odom_.available) status_.used_sources += "slam,";
  if (global_matcher_result_.available) status_.used_sources += "global_matcher,";
  if (accepted_relocalization_) status_.used_sources += "relocalization,";
  if (has_gnss_) status_.used_sources += "gnss,";
  status_.is_dr_available = dr_odom_.available;
  status_.is_slam_available = slam_odom_.available;
  status_.is_global_matcher_available = global_matcher_status_.available || global_matcher_result_.available;
  status_.is_relocalization_available = relocalization_status_.available;
  status_.is_gnss_available = has_gnss_;
  status_.is_can_available = true;
  status_.is_imu_available = true;
  status_.is_lidar_available = slam_odom_.available;
}

void ManagerCore::updatePolicyLocked(const ros::Time&) {
  policy_.scene_type = current_scene_;
  policy_.reason = failure_reason_;
  const bool initializing = state_ == ManagerState::WAIT_INITIAL_POSE || state_ == ManagerState::RELOCALIZING;
  // During initialization, availability is exactly what the system is trying to establish.
  // Keep candidate observers enabled so pose_fusion can accept the first valid predictor / map constraint.
  policy_.use_dr = initializing ? true : dr_odom_.available;
  policy_.use_slam = initializing ? true : (slam_odom_.available && state_ != ManagerState::PURE_DR);
  policy_.use_global_matcher = (initializing || global_matcher_status_.available || global_matcher_result_.available) &&
                               current_scene_ != "NO_MAP";
  // 等待初值、重定位中、降级、故障时允许触发重定位/融合重置
  policy_.use_relocalization = state_ == ManagerState::WAIT_INITIAL_POSE ||
                               state_ == ManagerState::RELOCALIZING ||
                               state_ == ManagerState::DEGRADED ||
                               state_ == ManagerState::FAILURE;
  policy_.use_gnss = has_gnss_ && (current_scene_ == "OUTDOOR_OPEN" || current_scene_ == "SEMI_OUTDOOR");
  policy_.use_gnss_position = policy_.use_gnss && gnss_full_ && config_.initialization.allow_gnss_init;
  policy_.use_gnss_heading = policy_.use_gnss && gnss_.heading_valid;
  policy_.dr_weight = policy_.use_dr ? 1.0 : 0.0;
  policy_.slam_weight = policy_.use_slam ? (state_ == ManagerState::DEGRADED ? 0.5 : 1.0) : 0.0;
  policy_.global_matcher_weight = policy_.use_global_matcher ? 1.0 : 0.0;
  policy_.relocalization_weight = policy_.use_relocalization ? 1.0 : 0.0;
  policy_.gnss_position_weight = policy_.use_gnss_position ? 1.0 : 0.0;
  policy_.gnss_heading_weight = policy_.use_gnss_heading ? 1.0 : 0.0;
  policy_.allow_fusion_reset = initializing || has_initial_pose_ || accepted_relocalization_.has_value();
  policy_.allow_dr_reset = false;
  policy_.allow_global_matcher_trigger = policy_.use_global_matcher;
  policy_.allow_relocalization_trigger = policy_.use_relocalization;
}

bool ManagerCore::relocalizationTrusted(const deeplumin_msgs::RelocalizationResult& msg) const {
  if (!msg.success) {
    return false;
  }
  if (!msg.map_id.empty() && !current_map_id_.empty() && msg.map_id != current_map_id_) {
    return false;
  }
  if (msg.fitness_score > config_.relocalization.accept_fitness) {
    return false;
  }
  if (msg.inlier_ratio < config_.relocalization.min_inlier_ratio) {
    return false;
  }
  if (msg.inlier_rmse > config_.relocalization.max_inlier_rmse) {
    return false;
  }
  // 若 fusion 已可信，则重定位结果不能与当前融合位姿偏差过大
  if (fusion_status_.available && fusion_status_.quality_score > 0.5) {
    if (msg.translation_diff > config_.relocalization.max_pose_diff_when_trusted) {
      return false;
    }
    if (std::abs(msg.yaw_diff) > config_.relocalization.max_yaw_diff_deg_when_trusted * M_PI / 180.0) {
      return false;
    }
  }
  return true;
}

}  // namespace localization_manager
