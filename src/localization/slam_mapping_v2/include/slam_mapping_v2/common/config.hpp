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
  std::string odom_topic = "/slam_odometry/odom";
  std::string scan_topic = "/slam_odometry/cloud_registered";
  std::string gnss_topic = "/gnss_chc_data";
  std::string image_topic = "/camera/image_raw";  // 相机图像话题（视觉回环用）
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

/// 视觉回环配置
struct VisualLoopConfig {
  // 词库
  std::string vocabulary_path = "";    // ORBvoc.txt 路径

  // ORB 特征
  int nfeatures = 2000;
  float scale_factor = 1.2f;
  int nlevels = 8;
  int ini_th_fast = 20;
  int min_th_fast = 7;

  // 检索
  double min_bow_score = 0.03;
  int min_shared_words = 20;
  int max_candidates = 3;

  // 匹配
  int min_bow_matches = 50;
  double nn_ratio = 0.6;
  int ransac_min_inliers = 15;
  double ransac_probability = 0.99;
  int ransac_max_iterations = 300;

  // 连续性验证
  int min_continuity = 3;
  int max_continuity_miss = 2;

  // 线程
  int feature_extract_rate_hz = 10;
  int buffer_size = 100;

  // 相机内参（几何验证必需）
  struct CameraIntrinsic {
    double fx = 500.0;
    double fy = 500.0;
    double cx = 320.0;
    double cy = 240.0;
  } camera;

  // LiDAR 到相机的外参 T_L_C（4x4 齐次变换矩阵）
  // 默认值：相机 x 轴 = LiDAR z 轴，相机 y 轴 = -LiDAR x 轴，相机 z 轴 = -LiDAR y 轴
  Eigen::Matrix4d T_L_C = (Eigen::Matrix4d() <<
    0., 0., 1., 0.30456,
   -1., 0., 0., 0.00065,
    0.,-1., 0., 0.65376,
    0., 0., 0., 1.).finished();
};

// [回环改造]
// 回环配准器配置
struct LoopRegistrationConfig {
  std::string type = "vgicp";
  double max_correspondence_distance = 1.5; // 细层对应距离阈值
  int max_iterations = 20; // 最大迭代次数
  double transformation_epsilon = 1e-6; // 变换误差阈值
  int num_threads = 4; // 线程数
  int cov_num_neighbors = 20; // 协方差计算邻居数
  double voxel_resolution = 0.5; // 体素分辨率
  double coarse_scale = 4.0; // 粗层缩放因子
  double fine_scale = 1.0; // 细层缩放因子
  std::string robust_kernel = "huber"; // 鲁棒核函数 "cauchy"
  double robust_kernel_scale = 1.0; // 鲁棒核参数

};

// 回环内点质量门控
struct LoopInlierGateConfig {
  double inlier_distance = 0.5; 
  double min_inlier_ratio = 0.4; 
  double max_inlier_rmse = 0.3; // 内点 rmse 上限
};

// 打分/配准前区域裁剪
struct LoopCropConfig {
  bool enable = true;
  double forward = 40.0;
  double backward = 40.0;
  double left = 30.0;
  double right = 30.0;
  double up = 10.0;
  double down = 5.0;

};

struct LoopClosureConfig {
  bool enable = true;
  std::string detector_type = "laser";     // "laser" | "visual" | "hybrid"
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

  LoopRegistrationConfig registration; // 配准器
  LoopInlierGateConfig inlier_gate; // 内点门控
  LoopCropConfig crop;  // 区域裁剪

  GnssConfig gnss;
  VisualLoopConfig visual;         ///< 视觉回环配置
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
  std::string images_dir_name = "images";
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
