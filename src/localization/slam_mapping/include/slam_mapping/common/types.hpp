#pragma once

#include <localization/common/types.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace slam_mapping {

enum class RuntimeState {
  IDLE,
  READY,
  RUNNING,
  STOPPING,
  SAVING,
  FINISHED,
  FAILED,
};

struct MappingKeyframe {
  int id = -1;
  double timestamp = 0.0;
  localization::Pose pose_odom;
  localization::Pose pose_map;
  localization::PointCloudPtr cloud_body;
  std::size_t point_count = 0;
  std::uint64_t source_frame_index = 0;
  bool optimized = false;
  bool loop_closed = false;
  int loop_match_count = 0;
};

struct MappingRuntimeStatus {
  std::string task_id;
  std::string task_name;
  RuntimeState state = RuntimeState::IDLE;
  std::string message = "idle";
  bool running = false;
  bool saving = false;
  bool output_valid = false;
  bool loop_enabled = false;
  bool loop_supported = false;
  std::string output_dir;
  std::string map_path;
  std::string keyframe_dir;
  std::string keyframe_pose_path;
  std::string trajectory_path;
  std::string loop_result_path;
  std::uint64_t frame_count = 0;
  std::uint64_t keyframe_count = 0;
  std::uint64_t loop_count = 0;
  int last_loop_from = -1;
  int last_loop_to = -1;
  double last_loop_score = -1.0;
  bool loop_updated_last_frame = false;
  std::uint64_t accumulated_point_count = 0;
  // [Task B] Warmup stats are kept in runtime status for logging and service inspection.
  bool warming_up = false;
  std::uint64_t imu_warmup_samples = 0;
  std::uint64_t dropped_lidar_before_warmup = 0;
  double imu_warmup_duration_sec = 0.0;
  double start_time = 0.0;
  double last_frame_time = 0.0;
  double duration_sec = 0.0;
};

struct RuntimeStartRequest {
  std::string task_id;
  std::string task_name;
  std::string output_dir;
  std::string config_path;
  bool enable_loop_closure = false;
  bool enable_visualization = true;
  bool reset_before_start = true;
};

struct RuntimeStopRequest {
  std::string task_id;
  bool save_map = true;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
};

struct SaveRequest {
  std::string output_dir;
  std::string map_name = "map.pcd";
  double resolution = 0.2;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
  bool save_loop_results = true;
};

struct SaveResult {
  bool success = false;
  std::string message;
  std::string map_path;
  std::string keyframe_dir;
  std::string keyframe_pose_path;
  std::string trajectory_path;
  std::string loop_result_path;
};

struct LoopClosureResult {
  bool updated = false;
  int loop_count = 0;
  double latest_score = -1.0;
  std::vector<std::pair<int, int>> loop_pairs;
};

std::string toString(RuntimeState state);

}  // namespace slam_mapping
