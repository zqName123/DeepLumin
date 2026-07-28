#pragma once

#include <localization/common/config.hpp>

#include <string>

namespace slam_mapping {

struct MappingConfig {
  bool auto_start = false;
  std::string task_name = "manual_mapping";
  std::string output_root = "/tmp/deeplumin_mapping";
  bool publish_tf = true;
  double process_rate_hz = 5000.0;
};

struct SensorConfig {
  std::string imu_topic = "/ouster/imu";
  std::string lidar_topic = "/ouster/points";
  std::string can_topic = "/can_receive_info";
  std::string gnss_topic = "/gnss_chc_data";
};

struct KeyframeConfig {
  double distance_thresh = 1.5;
  double rotation_thresh_rad = 0.2;
  double time_thresh_sec = 0.0;
  int min_points = 50;
};

struct MapSaverConfig {
  double default_resolution = 0.2;
  bool save_on_stop = true;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
  std::string map_filename = "map.pcd";
  std::string trajectory_filename = "trajectory.csv";
  std::string keyframe_pose_filename = "keyframes.csv";
  std::string keyframe_dir_name = "keyframes";
};

struct LoopClosureConfig {
  bool enable = false;
  double search_radius = 15.0;
  int min_index_gap = 20;
  int submap_search_num = 25;
  double submap_resolution = 0.3;
  double icp_fitness_thresh = 0.3;
  bool pose_graph_enable = false;
};

struct VisualizationConfig {
  bool publish_path = true;
  bool publish_keyframes = true;
  bool publish_local_map = true;
  bool publish_current_scan = true;
  bool publish_accumulated_map_preview = true;
  bool publish_loop_markers = true;
  double accumulated_map_publish_period = 2.0;
  double accumulated_map_voxel_leaf = 0.5;
};

struct OutputTopicConfig {
  std::string odom_topic = "/slam_mapping/odom";
  std::string path_topic = "/slam_mapping/path";
  std::string current_scan_topic = "/slam_mapping/current_scan";
  std::string local_map_topic = "/slam_mapping/local_map";
  std::string map_preview_topic = "/slam_mapping/map_preview";
  std::string keyframe_markers_topic = "/slam_mapping/keyframes";
  std::string loop_markers_topic = "/slam_mapping/loop_markers";
  std::string status_topic = "/slam_mapping/status";
};

struct SlamMappingConfig {
  MappingConfig mapping;
  localization::FrameConfig frames;
  SensorConfig sensors;
  localization::SlamConfig slam_backend;
  KeyframeConfig keyframe;
  MapSaverConfig map_saver;
  LoopClosureConfig loop_closure;
  VisualizationConfig visualization;
  OutputTopicConfig outputs;
};

}  // namespace slam_mapping
