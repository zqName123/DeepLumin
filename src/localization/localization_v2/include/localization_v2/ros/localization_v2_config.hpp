#pragma once

/**
 * @file localization_v2_config.hpp
 * @brief localization_v2 节点配置结构体与加载函数
 *
 * 配置来源：节点私有命名空间（~），由 launch 通过 <rosparam> 注入
 * config/localization_v2.yaml。
 */

#include <ros/ros.h>

#include <string>

namespace localization_v2 {

struct LocalizationV2Config {
  // ---- 坐标系 ----
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  // ---- 话题 ----
  std::string slam_odom_topic = "/localization/slam_odom";
  std::string global_matcher_result_topic = "/localization/global_matcher/result";
  std::string initial_pose_topic = "/initialpose";
  std::string source_cloud_topic = "/localization/cloud_registered_effect";
  std::string fused_odom_topic = "/localization_v2/odom";
  std::string map_to_odom_topic = "/localization_v2/map_to_odom";
  std::string fused_path_topic = "/localization_v2/path";
  std::string status_topic = "/localization_v2/status";
  std::string current_scan_in_map_topic = "/localization_v2/current_scan_in_map";
  std::string relocalization_result_topic = "/localization/relocalization/result";
  std::string relocalization_command_topic = "/localization_v2/relocalization/command";
  std::string gm_anchor_topic = "/localization_v2/gm_anchor";  ///< 初始化仲裁重定位胜出时，通知 GM 重锚（仅初始化阶段）

  // ---- 运行时 ----
  double output_rate_hz = 100.0;
  int max_path_size = 50000;
  bool publish_tf = true;
  bool publish_map_to_odom_tf = true;
  bool publish_odom_to_base_tf = false;  // slam_odometry_v2 默认已发布 odom->base，避免重复
  bool publish_path = true;
  bool publish_current_scan_in_map = true;

  // ---- global_matcher 结果语义与门控 ----
  std::string gm_result_pose_frame = "odom";  // "odom"=结果为 map->odom；"base"=结果为 map->base_link
  bool enable_jump_guard = false;
  double max_jump_translation = 5.0;
  double max_jump_yaw_deg = 50.0;

  // ---- 初始化策略 ----
  bool require_initial_pose = false;
  bool use_relocalization_init = true;
  bool use_manual_initialpose = true;
  bool use_global_matcher_init = true;

  // ---- 重定位策略 ----
  bool relocalization_enabled = true;
  bool relocalization_auto_start = true;
  bool relocalization_stop_on_success = true;
  bool relocalization_stop_on_manual_pose = true;
  bool relocalization_stop_on_global_match = true;
  bool relocalization_require_accepted = false;
  double relocalization_max_fitness_score = 1.0;
  double relocalization_min_inlier_ratio = 0.5;
  double relocalization_max_inlier_rmse = 0.8;
  double relocalization_fallback_timeout = 8.0;  ///< 双源初始化仲裁超时（s），超时后取已就绪的一源，防止阻塞

  // ---- 人工初始位姿 z 补全（可选，复用全局地图 PCD） ----
  bool initial_pose_z_completion = true;
  double initial_pose_z_search_radius = 5.0;
  std::string initial_pose_map_pcd_path;

  // ---- 跟踪失效检测 ----
  int gm_failure_threshold = 10;           ///< GM 连续失败阈值，达到此值触发跟踪丢失
  double gm_min_inlier_ratio = 0.3;        ///< GM inlier 比率下限
  double gm_max_inlier_rmse = 0.8;         ///< GM inlier RMSE 上限

  // ---- 重定位恢复 ----
  int reloc_failure_threshold = 5;         ///< 重定位连续失败阈值，达到此值触发定位失败
  double reloc_timeout_sec = 30.0;         ///< 重定位超时时间（秒）

  // ---- 降级输出 ----
  bool allow_degraded_output = true;       ///< TRACKING_LOST 时是否继续输出
};

/// 从节点私有命名空间（~）加载全部配置
LocalizationV2Config loadLocalizationV2Config(ros::NodeHandle& pnh);

}  // namespace localization_v2
