#pragma once

#include "simple_odom_fusion/core/math_utils.hpp"

#include <deque>
#include <optional>
#include <string>

namespace simple_odom_fusion {

struct Frames {
  std::string map = "map";
  std::string odom = "odom";
  std::string base_link = "base_link";
  std::string lidar = "lidar_link";
};

struct Topics {
  std::string dr_odom = "/localization/dr_odom";
  std::string slam_odom = "/localization/slam_odom";
  std::string global_matcher_result = "/localization/global_matcher/result";
  std::string initial_pose = "/initialpose";
  std::string source_cloud = "/localization/cloud_registered";
  std::string fused_odom = "/localization/simple_fused_odom";
  std::string fused_pose = "/localization/simple_fused_pose";
  std::string fused_path = "/localization/simple_fused_path";
  std::string gm_source_cloud_odom = "/localization/simple_fusion/gm_source_cloud_odom";
};

struct FusionConfig {
  Frames frames;
  Topics topics;
  double output_rate_hz = 100.0;
  std::size_t max_queue_size = 300;
  double sync_tolerance = 0.05;
  double max_pair_dt = 0.20;
  double max_delta_translation = 8.0;
  double max_delta_rotation_rad = 1.2;
  double dr_prediction_weight = 1.0;
  double slam_correction_gain = 0.25;
  bool use_slam_increment = true;
  bool publish_tf = true;
  bool publish_map_to_odom_tf = true;
  bool publish_odom_to_base_tf = true;
  bool publish_path = true;
  int max_path_size = 5000;
  bool allow_initialpose = true;
  bool allow_global_matcher_initialization = true;
  bool allow_local_auto_initialization = true;
  bool enable_global_matcher_correction = true;
  double global_matcher_correction_gain = 0.15;
  double global_matcher_min_interval = 1.0;
  bool publish_gm_source_cloud = true;
  bool gm_cloud_use_fused_pose = true;
};

struct OdomSample {
  double stamp = 0.0;
  Pose3d pose;
  Vec3d linear_velocity = Vec3d::Zero();
  std::string frame_id;
  std::string child_frame_id;
};

struct SyncedPair {
  OdomSample dr;
  OdomSample slam;
  double stamp = 0.0;
};

struct GlobalObservation {
  double stamp = 0.0;
  Pose3d map_base;
  bool success = true;
};

struct FusionState {
  bool initialized = false;
  double stamp = 0.0;
  Pose3d odom_base;
  Pose3d map_odom;
  Vec3d velocity_odom = Vec3d::Zero();
  std::optional<OdomSample> last_dr;
  std::optional<OdomSample> last_slam;
};

struct FusionOutput {
  bool valid = false;
  double stamp = 0.0;
  Pose3d odom_base;
  Pose3d map_odom;
  Pose3d map_base;
  Vec3d velocity_map = Vec3d::Zero();
};

}  // namespace simple_odom_fusion
