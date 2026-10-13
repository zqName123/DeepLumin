#include "localization_v3/ros/localization_v3_config.hpp"

namespace localization_v3 {

namespace {

std::string loadString(ros::NodeHandle& pnh, const std::string& key,
                       const std::string& default_value) {
  std::string value;
  pnh.param<std::string>(key, value, default_value);
  return value;
}

double loadDouble(ros::NodeHandle& pnh, const std::string& key, double default_value) {
  double value = default_value;
  pnh.param<double>(key, value, default_value);
  return value;
}

int loadInt(ros::NodeHandle& pnh, const std::string& key, int default_value) {
  int value = default_value;
  pnh.param<int>(key, value, default_value);
  return value;
}

bool loadBool(ros::NodeHandle& pnh, const std::string& key, bool default_value) {
  bool value = default_value;
  pnh.param<bool>(key, value, default_value);
  return value;
}

}  // namespace

LocalizationV3Config loadLocalizationV3Config(ros::NodeHandle& pnh) {
  LocalizationV3Config cfg;

  // ---- 坐标系 ----
  cfg.map_frame = loadString(pnh, "frames/map", cfg.map_frame);
  cfg.odom_frame = loadString(pnh, "frames/odom", cfg.odom_frame);
  cfg.base_frame = loadString(pnh, "frames/base_link", cfg.base_frame);

  // ---- 话题 ----
  cfg.slam_odom_topic = loadString(pnh, "topics/slam_odom", cfg.slam_odom_topic);
  cfg.global_matcher_result_topic =
      loadString(pnh, "topics/global_matcher_result", cfg.global_matcher_result_topic);
  cfg.initial_pose_topic = loadString(pnh, "topics/initial_pose", cfg.initial_pose_topic);
  cfg.source_cloud_topic = loadString(pnh, "topics/source_cloud", cfg.source_cloud_topic);
  cfg.fused_odom_topic = loadString(pnh, "topics/fused_odom", cfg.fused_odom_topic);
  cfg.map_to_odom_topic = loadString(pnh, "topics/map_to_odom", cfg.map_to_odom_topic);
  cfg.fused_path_topic = loadString(pnh, "topics/fused_path", cfg.fused_path_topic);
  cfg.status_topic = loadString(pnh, "topics/status", cfg.status_topic);
  cfg.current_scan_in_map_topic =
      loadString(pnh, "topics/current_scan_in_map", cfg.current_scan_in_map_topic);
  cfg.gm_anchor_topic = loadString(pnh, "topics/gm_anchor", cfg.gm_anchor_topic);
  cfg.loc_point_topic = loadString(pnh, "topics/loc_point", cfg.loc_point_topic);

  // ---- 运行时 ----
  cfg.output_rate_hz = loadDouble(pnh, "runtime/output_rate_hz", cfg.output_rate_hz);
  cfg.max_path_size = loadInt(pnh, "runtime/max_path_size", cfg.max_path_size);
  cfg.publish_tf = loadBool(pnh, "runtime/publish_tf", cfg.publish_tf);
  cfg.publish_map_to_odom_tf =
      loadBool(pnh, "runtime/publish_map_to_odom_tf", cfg.publish_map_to_odom_tf);
  cfg.publish_odom_to_base_tf =
      loadBool(pnh, "runtime/publish_odom_to_base_tf", cfg.publish_odom_to_base_tf);
  cfg.publish_path = loadBool(pnh, "runtime/publish_path", cfg.publish_path);
  cfg.publish_current_scan_in_map =
      loadBool(pnh, "runtime/publish_current_scan_in_map", cfg.publish_current_scan_in_map);
  cfg.base_yaw_offset = loadDouble(pnh, "runtime/base_yaw_offset", cfg.base_yaw_offset);

  // ---- GM 结果语义与质量门限 ----
  cfg.gm_result_pose_frame =
      loadString(pnh, "global_matcher/result_pose_frame", cfg.gm_result_pose_frame);
  cfg.quality.good_min_inlier_ratio =
      loadDouble(pnh, "global_matcher/good_min_inlier_ratio", cfg.quality.good_min_inlier_ratio);
  cfg.quality.degraded_min_inlier_ratio = loadDouble(
      pnh, "global_matcher/degraded_min_inlier_ratio", cfg.quality.degraded_min_inlier_ratio);
  cfg.quality.max_inlier_rmse =
      loadDouble(pnh, "global_matcher/max_inlier_rmse", cfg.quality.max_inlier_rmse);

  // ---- jump guard（仅 TRACKING 常规 GM 修正生效）----
  cfg.fusion.jump_guard.enabled = loadBool(pnh, "jump_guard/enabled", cfg.fusion.jump_guard.enabled);
  cfg.fusion.jump_guard.soft_max_translation =
      loadDouble(pnh, "jump_guard/soft_max_translation", cfg.fusion.jump_guard.soft_max_translation);
  cfg.fusion.jump_guard.soft_max_yaw_deg =
      loadDouble(pnh, "jump_guard/soft_max_yaw_deg", cfg.fusion.jump_guard.soft_max_yaw_deg);
  cfg.fusion.jump_guard.hard_max_translation =
      loadDouble(pnh, "jump_guard/hard_max_translation", cfg.fusion.jump_guard.hard_max_translation);
  cfg.fusion.jump_guard.hard_max_yaw_deg =
      loadDouble(pnh, "jump_guard/hard_max_yaw_deg", cfg.fusion.jump_guard.hard_max_yaw_deg);
  cfg.fusion.jump_guard.confirm_frames =
      loadInt(pnh, "jump_guard/confirm_frames", cfg.fusion.jump_guard.confirm_frames);
  cfg.fusion.jump_guard.confirm_consistency_trans = loadDouble(
      pnh, "jump_guard/confirm_consistency_trans", cfg.fusion.jump_guard.confirm_consistency_trans);
  cfg.fusion.jump_guard.confirm_consistency_yaw_deg =
      loadDouble(pnh, "jump_guard/confirm_consistency_yaw_deg",
                 cfg.fusion.jump_guard.confirm_consistency_yaw_deg);
  cfg.fusion.jump_guard.confirm_window_sec =
      loadDouble(pnh, "jump_guard/confirm_window_sec", cfg.fusion.jump_guard.confirm_window_sec);

  // ---- 人工位姿 ----
  cfg.fusion.manual_settle_sec =
      loadDouble(pnh, "manual_pose/settle_sec", cfg.fusion.manual_settle_sec);
  cfg.manual_verify_timeout_sec =
      loadDouble(pnh, "manual_pose/verify_timeout_sec", cfg.manual_verify_timeout_sec);
  cfg.manual_verify_fail_threshold =
      loadInt(pnh, "manual_pose/verify_fail_threshold", cfg.manual_verify_fail_threshold);
  cfg.publish_gm_anchor = loadBool(pnh, "manual_pose/publish_gm_anchor", cfg.publish_gm_anchor);
  cfg.initial_pose_z_completion =
      loadBool(pnh, "manual_pose/z_completion_enabled", cfg.initial_pose_z_completion);
  cfg.initial_pose_z_search_radius =
      loadDouble(pnh, "manual_pose/z_search_radius", cfg.initial_pose_z_search_radius);
  cfg.initial_pose_map_pcd_path =
      loadString(pnh, "manual_pose/map_pcd_path", cfg.initial_pose_map_pcd_path);

  // ---- 快速失效判定 ----
  cfg.tracking.gm_failure_threshold =
      loadInt(pnh, "failure_detection/gm_fail_fast_count", cfg.tracking.gm_failure_threshold);
  cfg.tracking.gm_degraded_threshold =
      loadInt(pnh, "failure_detection/degraded_count", cfg.tracking.gm_degraded_threshold);
  cfg.failure.map_miss_count =
      loadInt(pnh, "failure_detection/map_miss_count", cfg.failure.map_miss_count);
  cfg.failure.no_correction_timeout_sec = loadDouble(
      pnh, "failure_detection/no_correction_timeout_sec", cfg.failure.no_correction_timeout_sec);
  cfg.failure.lost_timeout_sec =
      loadDouble(pnh, "failure_detection/lost_timeout_sec", cfg.failure.lost_timeout_sec);
  cfg.failure.slam_timeout_sec =
      loadDouble(pnh, "failure_detection/slam_timeout_sec", cfg.failure.slam_timeout_sec);
  cfg.tracking.reloc_failure_threshold = loadInt(pnh, "failure_detection/reloc_failure_threshold",
                                                 cfg.tracking.reloc_failure_threshold);

  // ---- 恢复防抖 ----
  cfg.tracking.gm_recovery_threshold =
      loadInt(pnh, "recovery/gm_recovery_threshold", cfg.tracking.gm_recovery_threshold);
  cfg.tracking.reloc_recovery_threshold =
      loadInt(pnh, "recovery/reloc_recovery_threshold", cfg.tracking.reloc_recovery_threshold);
  cfg.tracking.tracking_min_dwell_sec =
      loadDouble(pnh, "recovery/tracking_min_dwell_sec", cfg.tracking.tracking_min_dwell_sec);
  cfg.tracking.tracking_lost_min_dwell_sec = loadDouble(pnh, "recovery/tracking_lost_min_dwell_sec",
                                                        cfg.tracking.tracking_lost_min_dwell_sec);

  // ---- 初始化策略 ----
  cfg.init.require_initial_pose =
      loadBool(pnh, "initialization/require_initial_pose", cfg.init.require_initial_pose);
  cfg.init.use_relocalization_init =
      loadBool(pnh, "initialization/use_relocalization_init", cfg.init.use_relocalization_init);
  cfg.init.use_manual_initialpose =
      loadBool(pnh, "initialization/use_manual_initialpose", cfg.init.use_manual_initialpose);
  cfg.init.use_global_matcher_init =
      loadBool(pnh, "initialization/use_global_matcher_init", cfg.init.use_global_matcher_init);

  // ---- 重定位接口与门限 ----
  cfg.relocalization_result_topic =
      loadString(pnh, "relocalization/result_topic", cfg.relocalization_result_topic);
  cfg.relocalization_command_topic =
      loadString(pnh, "relocalization/command_topic", cfg.relocalization_command_topic);
  cfg.reloc_policy.enabled = loadBool(pnh, "relocalization/enabled", cfg.reloc_policy.enabled);
  cfg.reloc_policy.auto_start = loadBool(pnh, "relocalization/auto_start", cfg.reloc_policy.auto_start);
  cfg.reloc_policy.stop_on_success =
      loadBool(pnh, "relocalization/stop_on_success", cfg.reloc_policy.stop_on_success);
  cfg.reloc_policy.stop_on_manual_pose =
      loadBool(pnh, "relocalization/stop_on_manual_pose", cfg.reloc_policy.stop_on_manual_pose);
  cfg.reloc_policy.stop_on_global_match =
      loadBool(pnh, "relocalization/stop_on_global_match", cfg.reloc_policy.stop_on_global_match);
  cfg.relocalization_require_accepted =
      loadBool(pnh, "relocalization/require_accepted", cfg.relocalization_require_accepted);
  cfg.relocalization_min_inlier_ratio = loadDouble(pnh, "relocalization/min_inlier_ratio",
                                                   cfg.relocalization_min_inlier_ratio);
  cfg.relocalization_max_inlier_rmse =
      loadDouble(pnh, "relocalization/max_inlier_rmse", cfg.relocalization_max_inlier_rmse);
  cfg.init_arbiter.fallback_timeout_sec =
      loadDouble(pnh, "relocalization/fallback_timeout", cfg.init_arbiter.fallback_timeout_sec);
  cfg.init_arbiter.reloc_result_max_age_sec = loadDouble(pnh, "relocalization/result_max_age_sec",
                                                         cfg.init_arbiter.reloc_result_max_age_sec);

  // 初始化仲裁与初始化策略共用开关
  cfg.init_arbiter.use_relocalization_init = cfg.init.use_relocalization_init;
  cfg.init_arbiter.use_global_matcher_init = cfg.init.use_global_matcher_init;

  // ---- 降级输出 ----
  cfg.allow_degraded_output =
      loadBool(pnh, "degraded/allow_degraded_output", cfg.allow_degraded_output);

  return cfg;
}

}  // namespace localization_v3
