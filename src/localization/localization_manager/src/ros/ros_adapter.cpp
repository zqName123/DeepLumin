/**
 * @file ros_adapter.cpp
 * @brief 参数加载与消息编解码实现
 */

#include "localization_manager/ros/ros_adapter.hpp"

#include <cmath>

namespace localization_manager_ros {
namespace {

localization_manager::ModuleLevel levelFromMsg(uint8_t level) {
  switch (level) {
    case deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL:
      return localization_manager::ModuleLevel::NORMAL;
    case deeplumin_msgs::LocalizationStatus::LEVEL_WARNING:
      return localization_manager::ModuleLevel::WARNING;
    case deeplumin_msgs::LocalizationStatus::LEVEL_DEGRADED:
      return localization_manager::ModuleLevel::DEGRADED;
    default:
      return localization_manager::ModuleLevel::FAILURE;
  }
}

template <typename MsgT>
ros::Time msgStampOrNow(const MsgT& msg, const ros::Time& now) {
  return msg.header.stamp.isZero() ? now : msg.header.stamp;
}

}  // namespace

localization_manager::ManagerConfig loadConfig(ros::NodeHandle& pnh) {
  localization_manager::ManagerConfig cfg;
  std::string scene_source = "manual";

  pnh.param<std::string>("frames/map", cfg.frames.map, cfg.frames.map);
  pnh.param<std::string>("frames/odom", cfg.frames.odom, cfg.frames.odom);
  pnh.param<std::string>("frames/base_link", cfg.frames.base_link, cfg.frames.base_link);

  pnh.param<std::string>("topics/dr_odom", cfg.topics.dr_odom, cfg.topics.dr_odom);
  pnh.param<std::string>("topics/dr_status", cfg.topics.dr_status, cfg.topics.dr_status);
  pnh.param<std::string>("topics/slam_odom", cfg.topics.slam_odom, cfg.topics.slam_odom);
  pnh.param<std::string>("topics/slam_health", cfg.topics.slam_health, cfg.topics.slam_health);
  pnh.param<std::string>("topics/fused_odom", cfg.topics.fused_odom, cfg.topics.fused_odom);
  pnh.param<std::string>("topics/fusion_status", cfg.topics.fusion_status, cfg.topics.fusion_status);
  pnh.param<std::string>("topics/global_matcher_result", cfg.topics.global_matcher_result,
                         cfg.topics.global_matcher_result);
  pnh.param<std::string>("topics/global_matcher_status", cfg.topics.global_matcher_status,
                         cfg.topics.global_matcher_status);
  pnh.param<std::string>("topics/relocalization_result", cfg.topics.relocalization_result,
                         cfg.topics.relocalization_result);
  pnh.param<std::string>("topics/gnss", cfg.topics.gnss, cfg.topics.gnss);
  pnh.param<std::string>("topics/scene_state", cfg.topics.scene_state, cfg.topics.scene_state);
  pnh.param<std::string>("topics/initialpose", cfg.topics.initialpose, cfg.topics.initialpose);
  pnh.param<std::string>("topics/manager_status", cfg.topics.manager_status, cfg.topics.manager_status);
  pnh.param<std::string>("topics/observer_policy", cfg.topics.observer_policy, cfg.topics.observer_policy);
  pnh.param<std::string>("topics/accepted_relocalization", cfg.topics.accepted_relocalization,
                         cfg.topics.accepted_relocalization);
  pnh.param<std::string>("topics/manager_initial_pose", cfg.topics.manager_initial_pose,
                         cfg.topics.manager_initial_pose);

  pnh.param<double>("runtime/rate_hz", cfg.runtime.rate_hz, cfg.runtime.rate_hz);
  pnh.param<double>("runtime/startup_grace_sec", cfg.runtime.startup_grace_sec, cfg.runtime.startup_grace_sec);
  pnh.param<bool>("runtime/publish_events", cfg.runtime.publish_events, cfg.runtime.publish_events);

  pnh.param<bool>("initialization/require_initial_pose", cfg.initialization.require_initial_pose,
                  cfg.initialization.require_initial_pose);
  pnh.param<bool>("initialization/allow_manual_initialpose", cfg.initialization.allow_manual_initialpose,
                  cfg.initialization.allow_manual_initialpose);
  pnh.param<bool>("initialization/allow_relocalization_init", cfg.initialization.allow_relocalization_init,
                  cfg.initialization.allow_relocalization_init);
  pnh.param<bool>("initialization/allow_gnss_init", cfg.initialization.allow_gnss_init,
                  cfg.initialization.allow_gnss_init);
  pnh.param<double>("initialization/initialpose_timeout_sec", cfg.initialization.initialpose_timeout_sec,
                    cfg.initialization.initialpose_timeout_sec);
  pnh.param<bool>("initialization/complete_manual_initialpose_z", cfg.initialization.complete_manual_initialpose_z,
                  cfg.initialization.complete_manual_initialpose_z);
  pnh.param<bool>("initialization/publish_raw_initialpose_directly",
                  cfg.initialization.publish_raw_initialpose_directly,
                  cfg.initialization.publish_raw_initialpose_directly);

  pnh.param<bool>("initial_pose_completion/enabled", cfg.initial_pose_completion.enabled,
                  cfg.initial_pose_completion.enabled);
  pnh.param<std::string>("initial_pose_completion/pcd_path", cfg.initial_pose_completion.pcd_path,
                         cfg.initial_pose_completion.pcd_path);
  pnh.param<std::string>("initial_pose_completion/height_method", cfg.initial_pose_completion.height_method,
                         cfg.initial_pose_completion.height_method);
  pnh.param<double>("initial_pose_completion/forward", cfg.initial_pose_completion.forward,
                    cfg.initial_pose_completion.forward);
  pnh.param<double>("initial_pose_completion/backward", cfg.initial_pose_completion.backward,
                    cfg.initial_pose_completion.backward);
  pnh.param<double>("initial_pose_completion/left", cfg.initial_pose_completion.left,
                    cfg.initial_pose_completion.left);
  pnh.param<double>("initial_pose_completion/right", cfg.initial_pose_completion.right,
                    cfg.initial_pose_completion.right);
  pnh.param<double>("initial_pose_completion/base_height_offset", cfg.initial_pose_completion.base_height_offset,
                    cfg.initial_pose_completion.base_height_offset);
  pnh.param<double>("initial_pose_completion/height_search_radius",
                    cfg.initial_pose_completion.height_search_radius,
                    cfg.initial_pose_completion.height_search_radius);
  pnh.param<int>("initial_pose_completion/min_map_points", cfg.initial_pose_completion.min_map_points,
                 cfg.initial_pose_completion.min_map_points);
  pnh.param<double>("initial_pose_completion/max_height_span", cfg.initial_pose_completion.max_height_span,
                    cfg.initial_pose_completion.max_height_span);
  pnh.param<bool>("initial_pose_completion/allow_default_z_fallback",
                  cfg.initial_pose_completion.allow_default_z_fallback,
                  cfg.initial_pose_completion.allow_default_z_fallback);
  pnh.param<double>("initial_pose_completion/default_z", cfg.initial_pose_completion.default_z,
                    cfg.initial_pose_completion.default_z);
  pnh.param<double>("initial_pose_completion/default_z_std", cfg.initial_pose_completion.default_z_std,
                    cfg.initial_pose_completion.default_z_std);
  pnh.param<double>("initial_pose_completion/map_height_z_std", cfg.initial_pose_completion.map_height_z_std,
                    cfg.initial_pose_completion.map_height_z_std);

  pnh.param<double>("timeout/dr_odom", cfg.timeout.dr_odom, cfg.timeout.dr_odom);
  pnh.param<double>("timeout/dr_status", cfg.timeout.dr_status, cfg.timeout.dr_status);
  pnh.param<double>("timeout/slam_odom", cfg.timeout.slam_odom, cfg.timeout.slam_odom);
  pnh.param<double>("timeout/slam_health", cfg.timeout.slam_health, cfg.timeout.slam_health);
  pnh.param<double>("timeout/fused_odom", cfg.timeout.fused_odom, cfg.timeout.fused_odom);
  pnh.param<double>("timeout/fusion_status", cfg.timeout.fusion_status, cfg.timeout.fusion_status);
  pnh.param<double>("timeout/global_matcher_result", cfg.timeout.global_matcher_result,
                    cfg.timeout.global_matcher_result);
  pnh.param<double>("timeout/global_matcher_status", cfg.timeout.global_matcher_status,
                    cfg.timeout.global_matcher_status);
  pnh.param<double>("timeout/relocalization_result", cfg.timeout.relocalization_result,
                    cfg.timeout.relocalization_result);
  pnh.param<double>("timeout/gnss", cfg.timeout.gnss, cfg.timeout.gnss);

  pnh.param<int>("degrade/enter_degraded_fail_count", cfg.degrade.enter_degraded_fail_count,
                 cfg.degrade.enter_degraded_fail_count);
  pnh.param<int>("degrade/exit_degraded_ok_count", cfg.degrade.exit_degraded_ok_count,
                 cfg.degrade.exit_degraded_ok_count);
  pnh.param<int>("degrade/trigger_reloc_fail_count", cfg.degrade.trigger_reloc_fail_count,
                 cfg.degrade.trigger_reloc_fail_count);
  pnh.param<double>("degrade/fusion_failure_timeout_sec", cfg.degrade.fusion_failure_timeout_sec,
                    cfg.degrade.fusion_failure_timeout_sec);
  pnh.param<double>("degrade/lidar_short_dropout_sec", cfg.degrade.lidar_short_dropout_sec,
                    cfg.degrade.lidar_short_dropout_sec);
  pnh.param<double>("degrade/lidar_long_dropout_sec", cfg.degrade.lidar_long_dropout_sec,
                    cfg.degrade.lidar_long_dropout_sec);
  pnh.param<double>("degrade/max_fusion_position_std", cfg.degrade.max_fusion_position_std,
                    cfg.degrade.max_fusion_position_std);
  pnh.param<double>("degrade/max_fusion_yaw_std_deg", cfg.degrade.max_fusion_yaw_std_deg,
                    cfg.degrade.max_fusion_yaw_std_deg);

  pnh.param<std::string>("scene/source", scene_source, scene_source);
  if (scene_source == "auto_gnss") {
    cfg.scene.source = localization_manager::SceneSource::AUTO_GNSS;
  } else if (scene_source == "auto_with_manual_override") {
    cfg.scene.source = localization_manager::SceneSource::AUTO_WITH_MANUAL_OVERRIDE;
  } else {
    cfg.scene.source = localization_manager::SceneSource::MANUAL;
  }
  pnh.param<double>("scene/manual_override_timeout_sec", cfg.scene.manual_override_timeout_sec,
                    cfg.scene.manual_override_timeout_sec);
  pnh.param<int>("scene/auto_gnss_confirm_count", cfg.scene.auto_gnss_confirm_count,
                 cfg.scene.auto_gnss_confirm_count);
  pnh.param<int>("scene/auto_gnss_lost_count", cfg.scene.auto_gnss_lost_count, cfg.scene.auto_gnss_lost_count);
  pnh.param<int>("scene/min_satellites", cfg.scene.min_satellites, cfg.scene.min_satellites);
  pnh.param<bool>("scene/allow_heading_only", cfg.scene.allow_heading_only, cfg.scene.allow_heading_only);
  pnh.param<double>("scene/jump_threshold_m", cfg.scene.jump_threshold_m, cfg.scene.jump_threshold_m);
  pnh.param<double>("scene/jump_quarantine_sec", cfg.scene.jump_quarantine_sec, cfg.scene.jump_quarantine_sec);
  pnh.param<double>("scene/min_speed_for_heading", cfg.scene.min_speed_for_heading,
                    cfg.scene.min_speed_for_heading);

  pnh.param<bool>("relocalization/require_second_confirmation", cfg.relocalization.require_second_confirmation,
                  cfg.relocalization.require_second_confirmation);
  pnh.param<double>("relocalization/accept_fitness", cfg.relocalization.accept_fitness,
                    cfg.relocalization.accept_fitness);
  pnh.param<double>("relocalization/min_inlier_ratio", cfg.relocalization.min_inlier_ratio,
                    cfg.relocalization.min_inlier_ratio);
  pnh.param<double>("relocalization/max_inlier_rmse", cfg.relocalization.max_inlier_rmse,
                    cfg.relocalization.max_inlier_rmse);
  pnh.param<double>("relocalization/max_pose_diff_when_trusted",
                    cfg.relocalization.max_pose_diff_when_trusted,
                    cfg.relocalization.max_pose_diff_when_trusted);
  pnh.param<double>("relocalization/max_yaw_diff_deg_when_trusted",
                    cfg.relocalization.max_yaw_diff_deg_when_trusted,
                    cfg.relocalization.max_yaw_diff_deg_when_trusted);

  pnh.param<std::string>("default_scene", cfg.default_scene, cfg.default_scene);
  pnh.param<std::string>("default_map_id", cfg.default_map_id, cfg.default_map_id);
  return cfg;
}

localization_manager::ModuleHealth fromLocalizationStatus(const deeplumin_msgs::LocalizationStatus& msg,
                                                          const std::string& name,
                                                          const ros::Time& now) {
  localization_manager::ModuleHealth health;
  health.name = name;
  health.last_stamp = msgStampOrNow(msg, now);
  health.age_sec = (now - health.last_stamp).toSec();
  health.level = levelFromMsg(msg.level);
  health.available = msg.level != deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE;
  health.timeout = false;
  health.quality_score = msg.quality_score;
  health.reason = msg.failure_reason;
  return health;
}

localization_manager::ModuleHealth fromFusionStatus(const deeplumin_msgs::FusionStatus& msg,
                                                    const ros::Time& now) {
  localization_manager::ModuleHealth health;
  health.name = "fusion";
  health.last_stamp = msgStampOrNow(msg, now);
  health.age_sec = (now - health.last_stamp).toSec();
  health.level = msg.level == deeplumin_msgs::FusionStatus::LEVEL_NORMAL
                   ? localization_manager::ModuleLevel::NORMAL
                   : (msg.level == deeplumin_msgs::FusionStatus::LEVEL_DEGRADED
                        ? localization_manager::ModuleLevel::DEGRADED
                        : localization_manager::ModuleLevel::FAILURE);
  health.available = msg.initialized;
  health.timeout = false;
  health.quality_score = msg.output_valid ? 1.0 : 0.0;
  health.reason = msg.failure_reason;
  return health;
}

localization_manager::ModuleHealth fromSlamHealth(const deeplumin_msgs::SlamHealth& msg,
                                                  const ros::Time& now) {
  localization_manager::ModuleHealth health;
  health.name = "slam";
  health.last_stamp = msgStampOrNow(msg, now);
  health.age_sec = (now - health.last_stamp).toSec();
  health.level = msg.is_degenerated ? localization_manager::ModuleLevel::DEGRADED
                                    : localization_manager::ModuleLevel::NORMAL;
  health.available = true;
  health.timeout = false;
  health.quality_score = msg.score;
  health.reason = msg.is_degenerated ? "slam_degenerated" : "";
  return health;
}

localization_manager::ModuleHealth fromOdom(const nav_msgs::Odometry& msg,
                                            const std::string& name,
                                            const ros::Time& now) {
  localization_manager::ModuleHealth health;
  health.name = name;
  health.last_stamp = msgStampOrNow(msg, now);
  health.age_sec = (now - health.last_stamp).toSec();
  health.level = localization_manager::ModuleLevel::NORMAL;
  health.available = true;
  health.timeout = false;
  health.quality_score = 1.0;
  return health;
}

localization_manager::ModuleHealth fromGlobalMatchStatus(const deeplumin_msgs::LocalizationStatus& msg,
                                                         const ros::Time& now) {
  return fromLocalizationStatus(msg, "global_matcher", now);
}

localization_manager::GnssView fromGnss(const deeplumin_msgs::Gpchc& msg) {
  localization_manager::GnssView view;
  view.stamp = msg.header.stamp;
  view.status = msg.status;
  view.nsv1 = msg.nsv1;
  view.nsv2 = msg.nsv2;
  view.heading_rad = (90.0 - msg.heading) * M_PI / 180.0;
  view.speed_mps = msg.v;
  view.heading_valid = msg.status > 0 && std::isfinite(msg.heading);
  view.position_valid = msg.status > 0 && std::isfinite(msg.latitude) && std::isfinite(msg.longitude) &&
                        std::abs(msg.latitude) > 1e-8 && std::abs(msg.longitude) > 1e-8;
  view.velocity_valid = msg.status > 0 && std::isfinite(msg.ve) && std::isfinite(msg.vn) &&
                        std::isfinite(msg.vu);
  return view;
}

deeplumin_msgs::LocalizationStatus toStatusMsg(const localization_manager::ManagerSnapshot& snapshot) {
  deeplumin_msgs::LocalizationStatus msg;
  msg.header.stamp = snapshot.stamp;
  msg.header.frame_id = "map";
  switch (snapshot.state) {
    case localization_manager::ManagerState::DEGRADED:
      msg.level = deeplumin_msgs::LocalizationStatus::LEVEL_DEGRADED;
      break;
    case localization_manager::ManagerState::WAIT_INITIAL_POSE:
    case localization_manager::ManagerState::RELOCALIZING:
      msg.level = deeplumin_msgs::LocalizationStatus::LEVEL_WARNING;
      break;
    case localization_manager::ManagerState::FAILURE:
      msg.level = deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE;
      break;
    default:
      msg.level = deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL;
      break;
  }
  msg.failure_reason = snapshot.failure_reason;
  msg.quality_score = snapshot.output_valid ? 1.0f : 0.0f;
  msg.is_gnss_available = snapshot.is_gnss_available;
  msg.is_slam_available = snapshot.is_slam_available;
  msg.is_dr_available = snapshot.is_dr_available;
  msg.current_mode = snapshot.active_scene + ":" + stateToString(snapshot.state);
  return msg;
}

deeplumin_msgs::ObserverPolicy toPolicyMsg(const localization_manager::ObserverPolicyData& policy) {
  deeplumin_msgs::ObserverPolicy msg;
  msg.header.stamp = ros::Time::now();
  msg.scene_type = policy.scene_type;
  msg.reason = policy.reason;
  msg.use_dr = policy.use_dr;
  msg.use_slam = policy.use_slam;
  msg.use_global_matcher = policy.use_global_matcher;
  msg.use_relocalization = policy.use_relocalization;
  msg.use_gnss = policy.use_gnss;
  msg.use_gnss_position = policy.use_gnss_position;
  msg.use_gnss_heading = policy.use_gnss_heading;
  msg.dr_weight = policy.dr_weight;
  msg.slam_weight = policy.slam_weight;
  msg.global_matcher_weight = policy.global_matcher_weight;
  msg.relocalization_weight = policy.relocalization_weight;
  msg.gnss_position_weight = policy.gnss_position_weight;
  msg.gnss_heading_weight = policy.gnss_heading_weight;
  msg.allow_fusion_reset = policy.allow_fusion_reset;
  msg.allow_dr_reset = policy.allow_dr_reset;
  msg.allow_global_matcher_trigger = policy.allow_global_matcher_trigger;
  msg.allow_relocalization_trigger = policy.allow_relocalization_trigger;
  return msg;
}

geometry_msgs::PoseWithCovarianceStamped toInitialPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& msg,
    const std::string& frame_id) {
  geometry_msgs::PoseWithCovarianceStamped out = msg;
  out.header.frame_id = frame_id;
  return out;
}

deeplumin_msgs::RelocalizationResult toAcceptedRelocalization(
    const deeplumin_msgs::RelocalizationResult& msg) {
  deeplumin_msgs::RelocalizationResult out = msg;
  out.accepted = true;
  return out;
}

std::string stateToString(const localization_manager::ManagerState state) {
  switch (state) {
    case localization_manager::ManagerState::INIT: return "INIT";
    case localization_manager::ManagerState::WAIT_INITIAL_POSE: return "WAIT_INITIAL_POSE";
    case localization_manager::ManagerState::LOCALIZATION: return "LOCALIZATION";
    case localization_manager::ManagerState::GLOBAL_MATCHING: return "GLOBAL_MATCHING";
    case localization_manager::ManagerState::RELOCALIZING: return "RELOCALIZING";
    case localization_manager::ManagerState::DEGRADED: return "DEGRADED";
    case localization_manager::ManagerState::FAILURE: return "FAILURE";
    case localization_manager::ManagerState::PURE_DR: return "PURE_DR";
    case localization_manager::ManagerState::PURE_SLAM: return "PURE_SLAM";
    case localization_manager::ManagerState::MAPPING: return "MAPPING";
  }
  return "UNKNOWN";
}

}  // namespace localization_manager_ros
