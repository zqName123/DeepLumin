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

/**
 * @brief GNSS 位置观测（从 deeplumin_msgs/Gpchc 解析，用于后端 GNSS 因子）
 *
 * 保存原始经纬高与质量字段，由后端在因子添加前完成可靠性门控和 ENU 转换，
 * 使运行时只负责"时间戳对齐"，因子逻辑保持解耦。
 */
struct GnssObservation {
  bool valid = false;        ///< 观测是否有效（经纬度非零等基本检查）
  double timestamp = 0.0;    ///< GNSS 观测时间戳（秒）
  double latitude = 0.0;     ///< 纬度（度）
  double longitude = 0.0;    ///< 经度（度）
  double altitude = 0.0;     ///< 海拔（米）
  int status = 0;            ///< 设备状态字（fixed/float 语义见设备协议）
  int nsv = 0;               ///< 参与解算的卫星数
  double position_std = 0.0; ///< 位置标准差（米）；0 表示由 status 推导
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
  GnssObservation gnss;      ///< 与该关键帧时间对齐的 GNSS 观测（默认无效）
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
  std::uint64_t gnss_factor_count = 0;    ///< 已添加的 GNSS 因子数
  std::uint64_t gnss_rejected_count = 0;  ///< 被可靠性门控拒绝的 GNSS 观测数
  bool gnss_aligned = false;              ///< ENU<->map 对齐是否已完成
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
  int gnss_factor_count = 0;    ///< 后端本次返回的 GNSS 因子累计数
  int gnss_rejected_count = 0;  ///< 后端本次返回的被拒 GNSS 观测累计数
  bool gnss_aligned = false;    ///< ENU<->map 对齐是否已完成
};

std::string toString(RuntimeState state);

}  // namespace slam_mapping
