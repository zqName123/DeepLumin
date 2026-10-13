#pragma once

/**
 * @file converters.hpp
 * @brief ROS 消息 ↔ core 数据结构转换（零业务逻辑）
 */

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/LocalizationStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "dr_odometry_v2/core/types.hpp"
#include "dr_odometry_v2/ros/config.hpp"

namespace dr_odometry_v2_ros {

using namespace dr_odometry_v2;  // NOLINT(build/namespaces)

/// IMU：msg → core（含 base_to_imu 旋转外参）
ImuSample fromRos(const sensor_msgs::Imu& msg, const Extrinsic& base_to_imu);

/// CAN 车速 → 轮速样本（kmh 换算 / 倒档取负 / valid 开关）
WheelSample fromRos(const deeplumin_msgs::CanReceiveInfo& msg, const WheelConfig& cfg);

/// GPCHC → GNSS 样本（heading/pitch/roll、acc、ve/vn/vu、lat/lon/alt）
GnssSample fromRos(const deeplumin_msgs::Gpchc& msg);

/// 定位输出（map->base + body 速度）→ 位姿观测样本
PoseSample fromRos(const nav_msgs::Odometry& msg);

/// status 门控信息（是否允许定位观测）
struct LocalizationGateInfo {
  bool tracking = false;
  bool output_valid = false;
  double slam_age = 1e9;
};
LocalizationGateInfo fromRos(const deeplumin_msgs::FusionStatus& msg);

/// core 输出 → ROS
nav_msgs::Odometry toRos(const OdomResult& odom, const std::string& frame,
                         const std::string& child);
deeplumin_msgs::LocalizationStatus toStatusMsg(const DrHealth& health, double now);

}  // namespace dr_odometry_v2_ros
