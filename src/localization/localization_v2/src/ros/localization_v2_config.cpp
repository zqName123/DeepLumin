#include "localization_v2/ros/localization_v2_config.hpp"

namespace localization_v2 {

namespace {

bool loadBool(ros::NodeHandle& pnh, const std::string& key, bool fallback) {
  bool value = fallback;
  pnh.param<bool>(key, value, fallback);
  return value;
}

double loadDouble(ros::NodeHandle& pnh, const std::string& key, double fallback) {
  double value = fallback;
  pnh.param<double>(key, value, fallback);
  return value;
}

int loadInt(ros::NodeHandle& pnh, const std::string& key, int fallback) {
  int value = fallback;
  pnh.param<int>(key, value, fallback);
  return value;
}

std::string loadString(ros::NodeHandle& pnh, const std::string& key, const std::string& fallback) {
  std::string value = fallback;
  pnh.param<std::string>(key, value, fallback);
  return value;
}

}  // namespace

LocalizationV2Config loadLocalizationV2Config(ros::NodeHandle& pnh) {
  LocalizationV2Config cfg;

  // frames
  cfg.map_frame = loadString(pnh, "frames/map", cfg.map_frame);
  cfg.odom_frame = loadString(pnh, "frames/odom", cfg.odom_frame);
  cfg.base_frame = loadString(pnh, "frames/base_link", cfg.base_frame);

  // topics
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
  cfg.relocalization_result_topic =
      loadString(pnh, "relocalization/result_topic", cfg.relocalization_result_topic);
  cfg.relocalization_command_topic =
      loadString(pnh, "relocalization/command_topic", cfg.relocalization_command_topic);
  cfg.gm_anchor_topic = loadString(pnh, "topics/gm_anchor", cfg.gm_anchor_topic);

  // runtime
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

  // global_matcher
  cfg.gm_result_pose_frame =
      loadString(pnh, "global_matcher/result_pose_frame", cfg.gm_result_pose_frame);
  cfg.enable_jump_guard = loadBool(pnh, "global_matcher/enable_jump_guard", cfg.enable_jump_guard);
  cfg.max_jump_translation =
      loadDouble(pnh, "global_matcher/max_jump_translation", cfg.max_jump_translation);
  cfg.max_jump_yaw_deg = loadDouble(pnh, "global_matcher/max_jump_yaw_deg", cfg.max_jump_yaw_deg);

  // initialization
  cfg.require_initial_pose =
      loadBool(pnh, "initialization/require_initial_pose", cfg.require_initial_pose);
  cfg.use_relocalization_init =
      loadBool(pnh, "initialization/use_relocalization_init", cfg.use_relocalization_init);
  cfg.use_manual_initialpose =
      loadBool(pnh, "initialization/use_manual_initialpose", cfg.use_manual_initialpose);
  cfg.use_global_matcher_init =
      loadBool(pnh, "initialization/use_global_matcher_init", cfg.use_global_matcher_init);

  // relocalization
  cfg.relocalization_enabled =
      loadBool(pnh, "relocalization/enabled", cfg.relocalization_enabled);
  cfg.relocalization_auto_start =
      loadBool(pnh, "relocalization/auto_start", cfg.relocalization_auto_start);
  cfg.relocalization_stop_on_success =
      loadBool(pnh, "relocalization/stop_on_success", cfg.relocalization_stop_on_success);
  cfg.relocalization_stop_on_manual_pose =
      loadBool(pnh, "relocalization/stop_on_manual_pose", cfg.relocalization_stop_on_manual_pose);
  cfg.relocalization_stop_on_global_match =
      loadBool(pnh, "relocalization/stop_on_global_match", cfg.relocalization_stop_on_global_match);
  cfg.relocalization_require_accepted =
      loadBool(pnh, "relocalization/require_accepted", cfg.relocalization_require_accepted);
  cfg.relocalization_max_fitness_score = loadDouble(
      pnh, "relocalization/max_fitness_score", cfg.relocalization_max_fitness_score);
  cfg.relocalization_min_inlier_ratio = loadDouble(
      pnh, "relocalization/min_inlier_ratio", cfg.relocalization_min_inlier_ratio);
  cfg.relocalization_max_inlier_rmse =
      loadDouble(pnh, "relocalization/max_inlier_rmse", cfg.relocalization_max_inlier_rmse);
  cfg.relocalization_fallback_timeout =
      loadDouble(pnh, "relocalization/fallback_timeout", cfg.relocalization_fallback_timeout);

  // 人工初始位姿 z 补全
  cfg.initial_pose_z_completion =
      loadBool(pnh, "initial_pose/z_completion_enabled", cfg.initial_pose_z_completion);
  cfg.initial_pose_z_search_radius =
      loadDouble(pnh, "initial_pose/z_search_radius", cfg.initial_pose_z_search_radius);
  cfg.initial_pose_map_pcd_path =
      loadString(pnh, "initial_pose/map_pcd_path", cfg.initial_pose_map_pcd_path);

  return cfg;
}

}  // namespace localization_v2
