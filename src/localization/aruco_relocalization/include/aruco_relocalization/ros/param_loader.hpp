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

struct OnlineNodeConfig {
  // 话题
  std::string image_topic = "/camera/image_raw";
  std::string camera_info_topic = "/camera/camera_info";
  std::string slam_odom_topic = "/localization/slam_odom";
  std::string result_topic = "/aruco_reloc/result";
  std::string pose_topic = "/aruco_reloc/pose";
  std::string status_topic = "/aruco_reloc/status";
  std::string command_topic = "/aruco_reloc/command";
  std::string debug_image_topic = "/aruco_reloc/debug_image";

  // 坐标系
  std::string frame_id = "map";
  std::string child_frame_id = "base_link";

  // 路径
  std::string marker_map_path;

  // 外参
  ExtrinsicConfig extrinsic;
  bool use_tf_extrinsic = false;

  // 子模块配置
  DetectorConfig detector;
  SolverConfig solver;
  ValidatorConfig validator;
};

struct OfflineMappingConfig {
  // 话题
  std::string path_topic = "/slam_mapping_v2/path";
  std::string image_topic = "/camera/image_raw";
  std::string camera_info_topic = "/camera/camera_info";

  // 输出
  std::string output_path;

  // 外参
  ExtrinsicConfig extrinsic;

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
