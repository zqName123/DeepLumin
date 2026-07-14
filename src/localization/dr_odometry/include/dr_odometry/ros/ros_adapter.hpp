#pragma once

/**
 * @file ros_adapter.hpp
 * @brief ROS ↔ 核心算法层的适配接口（参数加载、消息转换）。
 *
 * 设计意图：DrEskf 与 types 不依赖 ROS；本头文件是唯一允许拉 ROS/deeplumin_msgs
 * 的薄适配层，节点 .cpp 通过这里与滤波核心交互。
 */

#include "dr_odometry/core/types.hpp"

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/LocalizationStatus.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace dr_odometry_ros {

/** @brief 输入/输出话题名，对应 yaml 中 topics/*。 */
struct TopicConfig {
  std::string imu = "/ouster/imu";
  std::string can = "/can_receive_info";
  std::string gnss = "/gnss_chc_data";
  std::string odom = "/localization/dr_odom";
  std::string status = "/localization/dr_status";
  std::string path = "/localization/dr_path";
};

/** @brief 坐标系名，对应 yaml 中 frames/*。 */
struct FrameConfig {
  std::string odom = "odom";
  std::string base_link = "base_link";
  std::string imu = "os_imu";
  std::string gnss = "gnss_sensor";
  std::string can = "local1";
};

/** @brief 从参数服务器加载 dr/* → DrConfig。 */
dr_odometry::DrConfig loadDrConfig(ros::NodeHandle& nh);
/** @brief 从参数服务器加载 topics/*。 */
TopicConfig loadTopicConfig(ros::NodeHandle& nh);
/** @brief 从参数服务器加载 frames/*。 */
FrameConfig loadFrameConfig(ros::NodeHandle& nh);

/** @brief sensor_msgs/Imu → 内部 ImuData（时间戳、gyro、accel）。 */
dr_odometry::ImuData fromRos(const sensor_msgs::Imu& msg);

/**
 * @brief CanReceiveInfo → CanData。
 * @param speed_is_kmh true 时将 speed 从 km/h 转为 m/s；gear≈2 视为倒车取负速。
 */
dr_odometry::CanData fromRos(const deeplumin_msgs::CanReceiveInfo& msg, bool speed_is_kmh);

/**
 * @brief Gpchc → GnssData。
 * heading：导航角（0=北，顺时针）→ ENU yaw（0=东，逆时针）；
 * 并填充 heading/position/velocity 有效标志供滤波门控。
 */
dr_odometry::GnssData fromRos(const deeplumin_msgs::Gpchc& msg);

/** @brief 内部 OdomResult → nav_msgs/Odometry（含 6x6 pose 协方差）。 */
nav_msgs::Odometry toRosOdom(const dr_odometry::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame);

/** @brief 内部 OdomResult → geometry_msgs/TransformStamped（odom→base_link）。 */
geometry_msgs::TransformStamped toRosTransform(const dr_odometry::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame);

/** @brief DrHealth → LocalizationStatus（当前无 IMU 判 FAILURE，有则 NORMAL）。 */
deeplumin_msgs::LocalizationStatus toStatusMsg(const dr_odometry::DrHealth& health,
                                               const ros::Time& stamp);

}  // namespace dr_odometry_ros
