#pragma once

/**
 * @file config.hpp
 * @brief 分组配置加载（ROS 参数 → core DrConfig + 话题/外参/开关）
 *
 * 配置按功能分组：imu / wheel / gnss / localization / output / time_sync / frames / topics / extrinsics
 */

#include <ros/ros.h>

#include <string>

#include "dr_odometry_v2/core/types.hpp"
#include "dr_odometry_v2/ros/map_viz.hpp"

namespace dr_odometry_v2_ros {

using dr_odometry_v2::DrConfig;

struct TopicConfig {
  std::string imu = "/ouster/imu";
  std::string can = "/can_receive_info";
  std::string gnss = "/gnss_chc_data";
  std::string localization_pose = "/localization/fused_odom";
  std::string localization_status = "/localization/status";
  std::string odom = "/dr_odometry/odom";
  std::string status = "/dr_odometry/status";
  std::string path = "/dr_odometry/path";
};

struct FrameConfig {
  std::string odom = "odom";
  std::string base_link = "base_link";
};

struct Extrinsic {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

struct ExtrinsicConfig {
  Extrinsic base_to_imu;
  Extrinsic base_to_gnss;
  Extrinsic base_to_can;
};

struct LoadedConfig {
  DrConfig core;
  TopicConfig topics;
  FrameConfig frames;
  ExtrinsicConfig extrinsics;
  MapVizPublisher::Config map_viz;
};

LoadedConfig loadConfig(ros::NodeHandle& pnh);

}  // namespace dr_odometry_v2_ros
