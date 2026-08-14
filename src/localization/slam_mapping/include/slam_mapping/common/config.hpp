#pragma once

#include <localization/common/config.hpp>

#include <string>
#include <utility>
#include <vector>

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
  int min_frame_gap = 0;
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

/**
 * @brief GNSS 后端优化配置（YAML: loop_closure/gnss/*）
 *
 * 仅在 enable=true 时生效；关闭后不影响现有回环/建图流程。
 * status/nsv 语义取决于设备协议，阈值需按实际 Gpchc 数据确认。
 *
 * status_sigma 为按车型配置的 "status -> sigma" 表（白名单 + 噪声）：
 * - 表非空时，status 必须在表内才会被接受，σ 取表内值；
 * - 表为空时回退到 min_status + sigma_fixed/float/unknown 分档逻辑。
 */
struct GnssOptimizationConfig {
  bool enable = false;              ///< 是否启用 GNSS 因子
  int min_status = 4;               ///< 最低 status；低于该值拒绝
  int min_nsv = 15;                 ///< 最少卫星数；0 表示不检查
  double max_age_sec = 0.2;         ///< GNSS 与关键帧最大时间差（秒）
  double min_spacing_m = 5.0;       ///< 相邻 GNSS 因子最小间距（米），抑制冗余约束
  bool use_gps_elevation = false;   ///< false 时高程不采用 GNSS，改用里程计 z
  double z_override_sigma = 0.01;   ///< 不使用 GNSS 高程时的 z 噪声（米）
  double sigma_fixed = 0.3;         ///< 回退档位：高等级 status 位置噪声 σ（米）
  double sigma_float = 1.5;         ///< 回退档位：中等级 status 位置噪声 σ（米）
  double sigma_unknown = 3.0;       ///< 回退档位：低等级 status 位置噪声 σ（米）
  double sigma_floor = 0.2;         ///< 位置噪声下限 σ（米）
  double robust_delta = 1.0;        ///< Huber 核阈值；<=0 关闭鲁棒核
  int alignment_frames = 20;        ///< 用于估计 ENU<->map 对齐的关键帧对数
  double consistency_thresh_m = 10.0;  ///< GNSS 与当前位姿一致性阈值；<=0 关闭
  localization::Vec3d lever_arm = localization::Vec3d::Zero();  ///< 天线相对 body 平移（米）
  std::vector<std::pair<int, double>> status_sigma;  ///< status -> σ 表（白名单 + 噪声）
};

struct LoopClosureConfig {
  bool enable = false;
  std::string type = "fastlio2_sam_lc";
  double search_radius = 15.0;
  int min_index_gap = 20;
  double time_thresh_sec = 30.0;
  int loop_kf_gap = 20;
  int min_id_interval = 20;
  int closest_id_th = 50;
  double max_range = 30.0;
  int submap_search_num = 25;
  double submap_resolution = 0.3;
  double icp_fitness_thresh = 0.3;
  double odom_rot_noise = 1e-6;
  double odom_pos_noise = 1e-4;
  double loop_noise_scale = 1.0;
  double loop_rot_noise = 0.05;
  double loop_pos_noise = 0.5;
  double loop_score_floor = 0.05;
  GnssOptimizationConfig gnss;      ///< GNSS 后端优化子配置
};

struct VisualizationConfig {
  bool publish_path = true;
  bool publish_keyframes = true;
  bool publish_local_map = true;
  bool publish_current_scan = true;
  bool publish_accumulated_map_preview = true;
  bool publish_loop_markers = true;
  bool publish_gnss_markers = true;  ///< 是否发布 GNSS 因子/拒绝点 marker
  bool publish_gnss_edges = true;    ///< 是否发布 GNSS 因子-关键帧残差连线
  double accumulated_map_voxel_leaf = 0.5;
};

struct OutputTopicConfig {
  std::string odom_topic = "/slam_mapping/odom";
  std::string path_topic = "/slam_mapping/path";
  std::string current_scan_topic = "/slam_mapping/current_scan";
  std::string local_map_topic = "/slam_mapping/local_map";
  std::string map_preview_topic = "/slam_mapping/map_preview";
  std::string loop_current_submap_topic = "/slam_mapping/loop_current_submap";
  std::string loop_history_submap_topic = "/slam_mapping/loop_history_submap";
  std::string keyframe_markers_topic = "/slam_mapping/keyframes";
  std::string loop_markers_topic = "/slam_mapping/loop_markers";
  std::string gnss_markers_topic = "/slam_mapping/gnss_markers";
  std::string gnss_factor_edges_topic = "/slam_mapping/gnss_factor_edges";
  std::string status_topic = "/slam_mapping/status";
};

struct StartupConfig {
  bool enable_imu_warmup = true;
  double imu_warmup_duration = 1.0;
  int min_imu_samples = 20;
};

struct SlamMappingConfig {
  MappingConfig mapping;
  localization::FrameConfig frames;
  SensorConfig sensors;
  localization::SlamConfig slam_backend;
  StartupConfig startup;
  KeyframeConfig keyframe;
  MapSaverConfig map_saver;
  LoopClosureConfig loop_closure;
  VisualizationConfig visualization;
  OutputTopicConfig outputs;
};

}  // namespace slam_mapping
