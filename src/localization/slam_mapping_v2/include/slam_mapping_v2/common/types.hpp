#pragma once

/**
 * @file types.hpp
 * @brief slam_mapping_v2 自有数据类型（消息订阅式建图，无外部包依赖）
 */

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace slam_mapping_v2 {

using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Quatd = Eigen::Quaterniond;
using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

struct Pose {
  double timestamp = 0.0;
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
  Mat6d covariance = Mat6d::Identity();
  std::string frame_id;
};

/// 订阅消息转换后的单帧输入（odom 与 registered_scan 配对后使用）
struct FrameInput {
  double timestamp = 0.0;
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
  PointCloudPtr cloud;  ///< odom 系配准扫描
  bool valid = false;
  std::string frame_id;
};

struct GnssObservation {
  bool valid = false;
  double timestamp = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  int status = 0;
  int nsv = 0;
  double position_std = 0.0;
};

struct Keyframe {
  int id = -1;
  double timestamp = 0.0;
  Pose pose_odom;
  Pose pose_map;
  PointCloudPtr cloud;  ///< 关键帧点云（odom 系 registered_scan）
  std::size_t point_count = 0;
  std::uint64_t source_frame = 0;
  bool optimized = false;
  bool loop_closed = false;
  int loop_match_count = 0;
  GnssObservation gnss;
};

struct LoopCandidate {
  bool valid = false;
  int history_idx = -1;
  int current_idx = -1;
  double score = 0.0;
  Mat3d diff_rot = Mat3d::Identity();
  Vec3d diff_pos = Vec3d::Zero();
};

enum class RuntimeState { IDLE, READY, RUNNING, STOPPING, SAVING, FINISHED, FAILED };

struct RuntimeStatus {
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
  std::uint64_t gnss_factor_count = 0;
  std::uint64_t gnss_rejected_count = 0;
  bool gnss_aligned = false;
  double start_time = 0.0;
  double last_frame_time = 0.0;
  double duration_sec = 0.0;
};

struct StartRequest {
  std::string task_id;
  std::string task_name;
  std::string output_dir;
  bool enable_loop_closure = false;
  bool reset_before_start = true;
};

struct StopRequest {
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

struct LoopResult {
  bool updated = false;
  int loop_count = 0;
  double latest_score = -1.0;
  std::vector<std::pair<int, int>> loop_pairs;
  std::uint64_t gnss_factor_count = 0;
  std::uint64_t gnss_rejected_count = 0;
  bool gnss_aligned = false;
};

std::string toString(RuntimeState state);

}  // namespace slam_mapping_v2
