#pragma once

/**
 * @file global_matcher_v2_config.hpp
 * @brief global_matcher_v2 节点配置结构体与加载函数
 */

#include <Eigen/Dense>
#include <ros/ros.h>

#include <string>

#include "global_matcher_v2/core/map_manager.hpp"
#include "global_matcher_v2/core/matcher.hpp"

namespace global_matcher_v2 {

struct GlobalMatcherV2Config {
  // ---- 坐标系与外参 ----
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string lidar_frame = "lidar_link";
  Eigen::Matrix4f base_to_lidar = Eigen::Matrix4f::Identity();

  // ---- 话题 ----
  std::string source_cloud_topic = "/localization/cloud_registered";
  std::string initial_pose_topic = "/localization/fused_odom";
  std::string source_odom_topic = "/localization/slam_odom";
  std::string bootstrap_initial_pose_topic = "/localization/manager/initial_pose";
  std::string manual_initial_pose_topic = "/initialpose";
  std::string gm_anchor_topic = "/localization_v2/gm_anchor";  ///< localization_v2 初始化仲裁发布的重锚消息
  std::string result_topic = "/localization/global_matcher/result";
  std::string status_topic = "/localization/global_matcher/status";
  std::string source_initial_scan_topic = "/localization/global_matcher/source_scan_initial";
  std::string aligned_scan_topic = "/localization/global_matcher/aligned_scan";
  std::string target_submap_topic = "/localization/global_matcher/target_submap";
  std::string global_map_topic = "/localization/global_matcher/global_map";
  std::string initial_guess_topic = "/localization/global_matcher/initial_guess";
  std::string path_topic = "/localization/global_matcher/path";
  std::string load_map_command_topic = "/localization/global_matcher/load_map_command";
  std::string set_map_segment_command_topic = "/localization/global_matcher/set_map_segment_command";

  // ---- 地图管理 ----
  std::string map_mode = "single";  ///< single | chunked
  std::string map_pcd_path;
  std::string map_dir;
  double chunk_size = 100.0;
  int load_radius_chunks = 1;
  double load_voxel_leaf = 0.4;
  int max_loaded_chunks = 64;
  int cov_num_neighbors = 20;
  int cov_num_threads = 4;
  double vgicp_voxel_resolution = 0.5;
  double vgicp_coarse_scale = 10.0;
  double vgicp_fine_scale = 1.0;
  double global_map_publish_voxel_leaf = 5.0;
  double global_map_publish_period = 5.0;

  // ---- 运行时 ----
  bool auto_match = true;
  bool source_cloud_is_odom_frame = true;
  std::string source_cloud_frame = "auto";
  double match_rate = 10.0;
  double max_cloud_age = 1.0;
  bool use_receive_time_for_cloud_age = true;
  bool use_tracking_initial_pose = true;
  bool use_bootstrap_initial_pose = true;
  bool use_manual_initial_pose = true;
  bool use_default_initial_pose = true;
  bool allow_identity_bootstrap = true;
  // ---- 人工初值 z 补全（与 ysw_loc GM 一致） ----
  double manual_initial_pose_z_search_radius = 10.0;
  Eigen::Vector3f default_initial_translation = Eigen::Vector3f::Zero();
  Eigen::Vector3f default_initial_rpy_deg = Eigen::Vector3f::Zero();

  // ---- 裁剪（仅可视化子图用） ----
  CropBoxConfig crop;

  // ---- 匹配器 ----
  MatcherParams matcher;

  // ---- 质量门控 ----
  int min_source_points = 200;
  int min_target_points = 1000;
  double max_fitness_score = 1.0;
  double min_inlier_ratio = 0.5;
  double max_inlier_rmse = 0.8;
  double inlier_distance = 0.5;
  double max_delta_translation = 5.0;
  double max_delta_yaw_deg = 20.0;

  // ---- 结果 ----
  std::string result_pose_frame = "base";  ///< base | odom（odom=输出 map->odom）

  // ---- 发布开关（减少话题量） ----
  bool publish_global_map = true;
  bool publish_target_submap = false;
  bool publish_aligned_scan = false;
  bool publish_source_initial_scan = false;
  bool publish_initial_guess = false;
  bool publish_path = true;
  bool publish_status = true;

  MapManagerConfig mapManagerConfig() const;
};

GlobalMatcherV2Config loadGlobalMatcherV2Config(ros::NodeHandle& pnh);

}  // namespace global_matcher_v2
