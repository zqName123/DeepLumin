#pragma once

/**
 * @file config.hpp
 * @brief slam_mapping_v2 自有配置（全部由 v2 yaml 直接加载，无旧包键）
 */

#include "slam_mapping_v2/common/types.hpp"

#include <string>
#include <utility>
#include <vector>

namespace slam_mapping_v2 {

struct FrameConfig {
  std::string map = "map";
  std::string odom = "odom";
  std::string base_link = "base_link";
};

struct InputConfig {
  std::string odom_topic = "/localization/slam_odom";
  std::string scan_topic = "/localization/cloud_registered";
  std::string gnss_topic = "/gnss_chc_data";
};

struct OutputConfig {
  std::string status_topic = "/slam_mapping_v2/status";
  std::string path_topic = "/slam_mapping_v2/path";
  std::string map_topic = "/slam_mapping_v2/map";
  std::string loop_markers_topic = "/slam_mapping_v2/loop_markers";
  std::string keyframe_markers_topic = "/slam_mapping_v2/keyframes";
  std::string odom_topic = "/slam_mapping_v2/odom";
  std::string current_scan_topic = "/slam_mapping_v2/current_scan";
  bool publish_odom = false;          ///< 转发 odom（冗余，默认关）
  bool publish_current_scan = false;  ///< 转发 registered_scan（冗余，默认关）
  bool publish_debug_markers = false; ///< 回环/关键帧 marker（调试）
};

struct ServiceConfig {
  std::string start = "/slam_mapping_v2/start";
  std::string stop = "/slam_mapping_v2/stop";
  std::string save = "/slam_mapping_v2/save";
  std::string reset = "/slam_mapping_v2/reset";
  std::string get_status = "/slam_mapping_v2/get_status";
};

struct ThreadingConfig {
  double backend_interval_sec = 0.1;
  double viz_interval_sec = 0.5;
  std::size_t max_input_queue = 20;
  double time_sync_tolerance_sec = 0.1;
};

struct KeyframeConfig {
  double distance_thresh = 2.0;
  double rotation_thresh_rad = 0.0;
  double time_thresh_sec = 0.0;
  int min_points = 50;
  int min_frame_gap = 10;
};

struct GnssConfig {
  bool enable = true;
  int min_status = 33;
  int min_nsv = 0;
  double max_age_sec = 0.02;
  double min_spacing_m = 6.0;
  bool use_gps_elevation = false;
  double z_override_sigma = 0.01;
  double sigma_fixed = 0.3;
  double sigma_float = 1.5;
  double sigma_unknown = 3.0;
  double sigma_floor = 0.2;
  double robust_delta = 1.0;
  int alignment_frames = 20;
  double consistency_thresh_m = 10.0;
  Vec3d lever_arm = Vec3d::Zero();
  std::vector<std::pair<int, double>> status_sigma;
};

struct LoopClosureConfig {
  bool enable = true;
  int min_index_gap = 20;
  double time_thresh_sec = 20.0;
  int loop_kf_gap = 30;
  int min_id_interval = 20;
  int closest_id_th = 50;
  double max_range = 20.0;
  int submap_search_num = 25;
  double submap_resolution = 0.3;
  double icp_fitness_thresh = 0.3;
  double loop_noise_scale = 1.0;
  double loop_rot_noise = 0.05;
  double loop_pos_noise = 0.5;
  double loop_score_floor = 0.05;
  double odom_rot_noise = 1e-6;   ///< 里程计因子旋转噪声
  double odom_pos_noise = 1e-4;   ///< 里程计因子平移噪声
  GnssConfig gnss;
};

struct MapSaverConfig {
  double default_resolution = 0.2;
  bool save_on_stop = true;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
  bool save_loop_results = true;
  std::string map_filename = "map.pcd";
  std::string trajectory_filename = "trajectory.csv";
  std::string keyframe_pose_filename = "keyframes.csv";
  std::string keyframe_dir_name = "keyframes";
  std::string loop_result_dir_name = "loop_results";
  std::string loop_pairs_filename = "loop_pairs.csv";
  std::string loop_current_submap_filename = "latest_loop_current_submap.pcd";
  std::string loop_history_submap_filename = "latest_loop_history_submap.pcd";
};

struct MapConfig {
  double voxel_resolution = 0.5;   ///< iVox 体素分辨率
  double filter_size_map = 0.2;    ///< 插入去重距离
  double publish_voxel_leaf = 0.5; ///< 发布地图降采样
  bool publish_global_map = true;
};

struct RuntimeConfig {
  bool auto_start = false;
  std::string task_name = "manual_mapping";
  std::string output_root = "/home/hhy/2004ros/deeplumin_mapping";
  bool publish_tf = true;      ///< 发布 map→odom
  double process_rate_hz = 100.0;
};

struct MappingConfig {
  FrameConfig frames;
  InputConfig input;
  OutputConfig output;
  ServiceConfig services;
  ThreadingConfig threading;
  KeyframeConfig keyframe;
  LoopClosureConfig loop_closure;
  MapSaverConfig map_saver;
  MapConfig map;
  RuntimeConfig runtime;
};

}  // namespace slam_mapping_v2
