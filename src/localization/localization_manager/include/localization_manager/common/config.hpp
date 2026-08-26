#pragma once

/**
 * @file config.hpp
 * @brief localization_manager 配置结构体定义
 *
 * 由 localization_manager_ros::loadConfig() 从 YAML 填充。
 * 键路径与 config/localization_manager.yaml 层级一一对应。
 */

#include <string>

namespace localization_manager {

/** 场景判定来源 */
enum class SceneSource {
  MANUAL,                    ///< 默认场景 + 可选 /system/scene_state 覆盖
  AUTO_GNSS,                 ///< 完全由 GNSS 质量自动判定
  AUTO_WITH_MANUAL_OVERRIDE  ///< GNSS 自动 + 手动覆盖（带超时）
};

struct FrameConfig {
  std::string map = "map";
  std::string odom = "odom";
  std::string base_link = "base_link";
};

struct TopicConfig {
  std::string dr_odom = "/localization/dr_odom";
  std::string dr_status = "/localization/dr_status";
  std::string slam_odom = "/localization/slam_odom";
  std::string slam_health = "/localization/slam_health";
  std::string fused_odom = "/localization/fused_odom";
  std::string fusion_status = "/localization/fusion_status";
  std::string global_matcher_result = "/localization/global_matcher/result";
  std::string global_matcher_status = "/localization/global_matcher/status";
  std::string relocalization_result = "/localization/relocalization/result";
  std::string gnss = "/gnss_chc_data";
  std::string scene_state = "/system/scene_state";
  std::string initialpose = "/initialpose";
  std::string manager_status = "/localization/manager/status";
  std::string observer_policy = "/localization/manager/observer_policy";
  std::string accepted_relocalization = "/localization/manager/accepted_relocalization";
  std::string manager_initial_pose = "/localization/manager/initial_pose";
};

struct RuntimeConfig {
  double rate_hz = 10.0;
  double startup_grace_sec = 3.0;
  bool publish_events = true;
};

struct InitializationConfig {
  bool require_initial_pose = true;
  bool allow_manual_initialpose = true;
  bool allow_relocalization_init = true;
  bool allow_gnss_init = false;
  double initialpose_timeout_sec = 10.0;
  bool complete_manual_initialpose_z = true;
  bool publish_raw_initialpose_directly = false;
};

struct InitialPoseCompletionConfig {
  bool enabled = true;
  std::string pcd_path = "/home/hhy/2004ros/ysw_loc/src/map/0730map/yuhsuwan2_loc_rot_72.pcd";
  std::string height_method = "median";
  double forward = 70.0;
  double backward = 7.0;
  double left = 35.0;
  double right = 35.0;
  double base_height_offset = 0.0;
  double height_search_radius = 5.0;
  int min_map_points = 200;
  double max_height_span = 8.0;
  bool allow_default_z_fallback = false;
  double default_z = 0.0;
  double default_z_std = 5.0;
  double map_height_z_std = 1.0;
};

/** 各模块消息超时阈值（秒） */
struct TimeoutConfig {
  double dr_odom = 0.5;
  double dr_status = 1.0;
  double slam_odom = 0.5;
  double slam_health = 0.5;
  double fused_odom = 0.5;
  double fusion_status = 0.5;
  double global_matcher_result = 3.0;
  double global_matcher_status = 3.0;
  double relocalization_result = 5.0;
  double gnss = 1.0;
};

struct DegradeConfig {
  int enter_degraded_fail_count = 3;
  int exit_degraded_ok_count = 5;
  int trigger_reloc_fail_count = 5;
  double fusion_failure_timeout_sec = 5.0;
  double lidar_short_dropout_sec = 1.0;
  double lidar_long_dropout_sec = 5.0;
  double max_fusion_position_std = 5.0;
  double max_fusion_yaw_std_deg = 15.0;
};

struct ScenePolicyConfig {
  SceneSource source = SceneSource::MANUAL;
  double manual_override_timeout_sec = 30.0;
  int auto_gnss_confirm_count = 5;
  int auto_gnss_lost_count = 5;
  int min_satellites = 10;
  bool allow_heading_only = true;
  double jump_threshold_m = 3.0;
  double jump_quarantine_sec = 2.0;
  double min_speed_for_heading = 0.2;
};

/** 重定位 accept 门控参数 */
struct RelocalizationGateConfig {
  bool require_second_confirmation = false;
  double accept_fitness = 1.0;
  double min_inlier_ratio = 0.5;
  double max_inlier_rmse = 0.8;
  double max_pose_diff_when_trusted = 5.0;
  double max_yaw_diff_deg_when_trusted = 20.0;
};

struct ManagerConfig {
  FrameConfig frames;
  TopicConfig topics;
  RuntimeConfig runtime;
  InitializationConfig initialization;
  InitialPoseCompletionConfig initial_pose_completion;
  TimeoutConfig timeout;
  DegradeConfig degrade;
  ScenePolicyConfig scene;
  RelocalizationGateConfig relocalization;
  std::string default_scene = "INDOOR";
  std::string default_map_id = "default";
};

}  // namespace localization_manager
