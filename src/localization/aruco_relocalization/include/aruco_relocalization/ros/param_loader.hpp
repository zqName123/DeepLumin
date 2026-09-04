#pragma once

/**
 * @file param_loader.hpp
 * @brief ROS 参数加载
 */

#include <aruco_relocalization/detector/marker_detector.hpp>
#include <aruco_relocalization/solver/relocalization_solver.hpp>
#include <aruco_relocalization/validator/relocalization_validator.hpp>

#include <ros/ros.h>

#include <string>

namespace aruco_reloc {

struct ExtrinsicConfig {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

/// 点云验证配置
struct PointCloudVerifyConfig {
  bool enabled = false;
  bool use_vgicp = true;  // 是否使用 VGICP 精化（false 则直接用 ArUco 位姿计算 inlier）
  std::string cloud_topic = "/localization/points";

  // 点云坐标系转换（与 relocalization 一致）
  bool rotate_xy_y_negx = true;   // 额外 -90° 旋转（lidar→base）
  double query_yaw_deg = 0.0;

  // base_link 到 lidar 的变换
  ExtrinsicConfig base_to_lidar;

  // 全局地图
  std::string global_map_pcd;

  // 裁剪参数（米）
  double crop_forward = 30.0;
  double crop_backward = 30.0;
  double crop_left = 30.0;
  double crop_right = 30.0;
  double crop_down = 5.0;
  double crop_up = 5.0;

  // 预处理
  double voxel_leaf = 0.5;
  double min_range = 3.0;
  double max_range = 50.0;
  bool z_filter_enable = true;
  double z_filter_min = -1.0;
  double z_filter_max = 5.0;

  // 内点判断
  double inlier_distance = 0.5;
  double min_inlier_ratio = 0.5;
  double max_inlier_rmse = 0.5;
};

/// 调试可视化配置
struct DebugVisualizerConfig {
  bool enabled = false;
  std::string debug_image_topic = "/aruco_reloc/debug_image";
  std::string source_cloud_topic = "/aruco_reloc/source_cloud";
  std::string submap_topic = "/aruco_reloc/submap";
  std::string aligned_cloud_topic = "/aruco_reloc/aligned_cloud";
};

struct OnlineNodeConfig {
  // 话题
  std::string image_topic = "/camera/image_raw";
  std::string result_topic = "/localization/relocalization/result";
  std::string pose_topic = "/aruco_reloc/pose";
  std::string status_topic = "/aruco_reloc/status";
  std::string command_topic = "/localization_v2/relocalization/command";
  std::string debug_image_topic = "/aruco_reloc/debug_image";

  // 坐标系
  std::string frame_id = "map";
  std::string child_frame_id = "base_link";

  // 图像旋转（0: 不旋转, 1: 顺时针90°, 2: 180°, 3: 逆时针90°）
  int image_rotation = 0;

  // 路径
  std::string marker_map_path;

  // 外参（分离 base->lidar 和 lidar->camera）
  ExtrinsicConfig base_to_lidar;
  ExtrinsicConfig lidar_to_camera;
  bool use_tf_extrinsic = false;

  // 子模块配置
  DetectorConfig detector;
  SolverConfig solver;
  ValidatorConfig validator;

  // 点云验证配置
  PointCloudVerifyConfig pc_verify;

  // 可视化配置
  DebugVisualizerConfig visualizer;
};

struct OfflineMappingConfig {
  // 话题
  std::string path_topic = "/slam_mapping_v2/path";
  std::string image_topic = "/camera/image_raw";
  std::string camera_info_topic = "/camera/camera_info";

  // 输出
  std::string output_path;

  // 外参（lidar_to_camera）
  ExtrinsicConfig extrinsic;

  // lidar_base 到 base_link 的变换
  ExtrinsicConfig lidar_base_to_base;

  // 图像旋转（0: 不旋转, 1: 顺时针90°, 2: 180°, 3: 逆时针90°）
  int image_rotation = 3;  // 默认逆时针90°，与 slam_mapping_v2 保持一致

  // 同步参数
  double time_sync_tolerance = 0.5;  // 秒

  // 子模块配置
  DetectorConfig detector;

  // 聚合参数
  int min_observations = 5;
  double max_reproj_error = 5.0;  // pixels
};

/// 加载在线节点配置
OnlineNodeConfig loadOnlineNodeConfig(ros::NodeHandle& pnh);

/// 加载离线建图节点配置
OfflineMappingConfig loadOfflineMappingConfig(ros::NodeHandle& pnh);

/// 从参数加载相机内参
bool loadCameraIntrinsics(ros::NodeHandle& pnh, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);

/// 从参数加载外参
Eigen::Isometry3d loadExtrinsic(ros::NodeHandle& pnh, const std::string& key);

}  // namespace aruco_reloc
