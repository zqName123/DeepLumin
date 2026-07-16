#pragma once

#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/ros/eval_visualizer.hpp>

#include <ros/ros.h>

namespace relocalization_ros {

struct BagEvalConfig {
  relocalization::RelocalizationCoreConfig core;
  relocalization::EvalVisualizationConfig viz;
  std::string report_csv;
  std::string bag_file;
  std::string lidar_topic = "/ouster/points";
  int cloud_stride = 10;
  int max_queries = 50;
  double success_translation = 1.0;
  double success_yaw_deg = 5.0;
  double place_success_radius = 5.0;
  double max_pose_time_diff = 0.15;
  double bag_start_offset = 0.0;
  double bag_duration = 0.0;
  bool run_gicp = true;
  bool use_truth_pose = false;
  bool log_topk_candidates = true;
  bool query_rotate_xy_y_negx = false;
  double query_frame_yaw_deg = 0.0;
};

struct OnlineRelocalizationConfig {
  relocalization::RelocalizationCoreConfig core;
  std::string lidar_topic = "/ouster/points";
  std::string frame_id = "map";
  std::string child_frame_id = "base_link";
  std::string result_topic = "/localization/relocalization/result";
  std::string map_id = "default";
  std::string method = "scan_context_gicp";
  bool query_rotate_xy_y_negx = false;
  double query_frame_yaw_deg = 0.0;
};

BagEvalConfig loadBagEvalConfig(ros::NodeHandle& pnh);
OnlineRelocalizationConfig loadOnlineRelocalizationConfig(ros::NodeHandle& pnh);

}  // namespace relocalization_ros
