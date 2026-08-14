#pragma once

#include "slam_mapping/common/types.hpp"

#include <localization/common/config.hpp>

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/GetMappingRuntimeStatus.h>
#include <deeplumin_msgs/MappingStatus.h>
#include <deeplumin_msgs/SaveMappingRuntime.h>
#include <deeplumin_msgs/StartMappingRuntime.h>
#include <deeplumin_msgs/StopMappingRuntime.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace slam_mapping_ros {

localization::ImuData fromRos(const sensor_msgs::Imu& msg);
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg);
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg,
                                            const localization::SlamConfig& cfg);
localization::CanVelocityData fromRos(const deeplumin_msgs::CanReceiveInfo& msg);
localization::GnssAttitudeData fromRos(const deeplumin_msgs::Gpchc& msg);
slam_mapping::GnssObservation fromRosGnss(const deeplumin_msgs::Gpchc& msg);

nav_msgs::Odometry toRosOdom(const localization::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame);
sensor_msgs::PointCloud2 toRosPointCloud(const localization::TimestampedPointCloud& pc);
geometry_msgs::TransformStamped toRosTransform(const localization::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame);
deeplumin_msgs::MappingStatus toRosStatus(const slam_mapping::MappingRuntimeStatus& status,
                                          const ros::Time& stamp);

slam_mapping::RuntimeStartRequest fromRos(const deeplumin_msgs::StartMappingRuntime::Request& req);
slam_mapping::RuntimeStopRequest fromRos(const deeplumin_msgs::StopMappingRuntime::Request& req);
slam_mapping::SaveRequest fromRos(const deeplumin_msgs::SaveMappingRuntime::Request& req);

void fillRos(const slam_mapping::MappingRuntimeStatus& status,
             deeplumin_msgs::GetMappingRuntimeStatus::Response* res,
             const ros::Time& stamp);
void fillRos(const slam_mapping::SaveResult& result,
             deeplumin_msgs::SaveMappingRuntime::Response* res);

}  // namespace slam_mapping_ros
