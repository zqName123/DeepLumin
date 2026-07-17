#pragma once

#include "pose_fusion/common/math_utils.hpp"

#include <deque>
#include <string>
#include <vector>

namespace pose_fusion {

enum class ObservationSource {
  DR,
  SLAM,
  GLOBAL_MATCHER,
  RELOCALIZATION,
  GNSS,
  MANUAL,
  UNKNOWN
};

enum class FusionStateName {
  UNINITIALIZED,
  DR_ONLY,
  LOCAL_FUSION,
  MAP_CONSTRAINED,
  DEGRADED,
  FAILURE
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
  std::string global_matcher_result = "/localization/global_matcher/result";
  std::string global_matcher_status = "/localization/global_matcher/status";
  std::string accepted_relocalization = "/localization/manager/accepted_relocalization";
  std::string observer_policy = "/localization/manager/observer_policy";
  std::string initial_pose = "/initialpose";
  std::string fused_odom = "/localization/fused_odom";
  std::string fused_pose = "/localization/fused_pose";
  std::string fused_path = "/localization/fused_path";
  std::string fusion_status = "/localization/fusion_status";
};

struct RuntimeConfig {
  double output_rate_hz = 100.0;
  bool publish_tf = true;
  bool publish_map_to_odom_tf = true;
  bool publish_odom_to_base_tf = false;
  bool publish_path = true;
  int max_path_size = 5000;
  bool allow_debug_relocalization_direct = false;
};

struct InitializationConfig {
  bool allow_initialpose = true;
  bool allow_dr_initialization = true;
  bool allow_slam_initialization = true;
  double initial_cov_position = 1.0;
  double initial_cov_yaw_deg = 5.0;
};

struct PredictionConfig {
  double max_dr_gap = 0.2;
  double max_dr_delta_translation = 5.0;
  double max_dr_delta_yaw_deg = 45.0;
  double process_noise_position = 0.05;
  double process_noise_yaw_deg = 0.5;
};

struct SlamObserverConfig {
  bool enabled = true;
  double smooth_gain = 0.08;
  double max_position_residual = 3.0;
  double max_yaw_residual_deg = 25.0;
};

struct GlobalMatcherConfig {
  bool enabled = true;
  bool allow_converged_false = true;
  double base_gain = 0.15;
  double min_gain = 0.02;
  double max_gain = 0.30;
  double target_inlier_ratio = 0.70;
  double target_rmse = 0.30;
  double min_inlier_ratio = 0.50;
  double max_inlier_rmse = 0.80;
  double max_fitness = 20.0;
  double max_position_residual = 5.0;
  double max_yaw_residual_deg = 20.0;
};

struct BufferConfig {
  double max_dr_age = 0.5;
  double max_slam_age = 1.0;
  double max_global_matcher_age = 3.0;
  double max_relocalization_age = 3.0;
};

struct FusionConfig {
  FrameConfig frames;
  TopicConfig topics;
  RuntimeConfig runtime;
  InitializationConfig initialization;
  PredictionConfig prediction;
  SlamObserverConfig slam;
  GlobalMatcherConfig global_matcher;
  BufferConfig buffer;
  std::string default_map_id = "default";
};

struct ObserverPolicyData {
  std::string scene_type = "unknown";
  std::string reason;
  bool use_dr = true;
  bool use_slam = true;
  bool use_global_matcher = true;
  bool use_relocalization = true;
  bool use_gnss = false;
  bool allow_fusion_reset = true;
  double dr_weight = 1.0;
  double slam_weight = 1.0;
  double global_matcher_weight = 1.0;
  double relocalization_weight = 1.0;
};

struct PoseObservation {
  ObservationSource source = ObservationSource::UNKNOWN;
  double timestamp = 0.0;
  Pose3d pose;
  Vec3d velocity = Vec3d::Zero();
  Mat6d covariance = Mat6d::Identity();
  std::string frame_id;
  std::string child_frame_id;
  std::string map_id;
  bool valid = false;
  bool success = true;
  bool converged = true;
  double quality_score = 1.0;
  double fitness_score = 0.0;
  double inlier_ratio = 1.0;
  double inlier_rmse = 0.0;
  double initial_translation_error = 0.0;
  double initial_yaw_error = 0.0;
  std::string reject_reason;
};

struct ResetObservation {
  ObservationSource source = ObservationSource::UNKNOWN;
  double timestamp = 0.0;
  Pose3d map_base;
  Mat6d covariance = Mat6d::Identity();
  std::string frame_id;
  std::string child_frame_id;
  std::string map_id;
  std::string reason;
  std::string approved_by;
  bool valid = false;
};

struct FusionState {
  double timestamp = 0.0;
  Pose3d odom_base;
  Pose3d map_odom;
  Vec3d velocity_odom = Vec3d::Zero();
  Mat6d pose_covariance = Mat6d::Identity();
  bool initialized = false;
};

struct FusionOutput {
  double timestamp = 0.0;
  bool valid = false;
  Pose3d map_base;
  Pose3d map_odom;
  Pose3d odom_base;
  Vec3d velocity_map = Vec3d::Zero();
  Mat6d covariance = Mat6d::Identity();
};

struct FusionStatusData {
  double timestamp = 0.0;
  FusionStateName state = FusionStateName::UNINITIALIZED;
  bool initialized = false;
  bool output_valid = false;
  std::string failure_reason;
  std::string current_map_id = "default";
  std::string active_scene = "unknown";
  double last_dr_age = 1e9;
  double last_slam_age = 1e9;
  double last_global_matcher_age = 1e9;
  double last_relocalization_age = 1e9;
  double position_std = 1.0;
  double velocity_std = 1.0;
  double yaw_std = 1.0;
  std::vector<std::string> used_sources;
  std::vector<std::string> rejected_sources;
  std::vector<std::string> reject_reasons;
  bool is_dr_available = false;
  bool is_slam_available = false;
  bool is_global_matcher_available = false;
  bool is_relocalization_available = false;
};

std::string toString(ObservationSource source);
std::string toString(FusionStateName state);

}  // namespace pose_fusion
