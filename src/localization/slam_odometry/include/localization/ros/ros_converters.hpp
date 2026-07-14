#pragma once

#include <localization/common/config.hpp>
#include <localization/common/types.hpp>
#include <localization/ros/sensor_topics.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace localization_ros {

localization::FrameConfig loadFrameConfig(ros::NodeHandle& nh);
localization::SlamConfig loadSlamConfig(ros::NodeHandle& nh);

localization::ImuData fromRos(const sensor_msgs::Imu& msg);
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg);
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg,
                                            const localization::SlamConfig& cfg);
localization::TimestampedPointCloud fromRosOuster(const sensor_msgs::PointCloud2& msg,
                                                  const localization::SlamConfig& cfg);
localization::TimestampedPointCloud fromRosLidar(const sensor_msgs::PointCloud2& msg,
                                                 const localization::SlamConfig& cfg);

nav_msgs::Odometry toRosOdom(const localization::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame);
sensor_msgs::PointCloud2 toRosPointCloud(const localization::TimestampedPointCloud& pc);
geometry_msgs::TransformStamped toRosTransform(const localization::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame);

}  // namespace localization_ros
