#pragma once

/**
 * @file converters.hpp
 * @brief 自有消息转换（odom/scan/Gpchc → v2 类型；状态 → MappingStatus）
 */

#include "slam_mapping_v2/common/types.hpp"

#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/GetMappingRuntimeStatus.h>
#include <deeplumin_msgs/MappingStatus.h>
#include <deeplumin_msgs/SaveMappingRuntime.h>
#include <deeplumin_msgs/StartMappingRuntime.h>
#include <deeplumin_msgs/StopMappingRuntime.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace slam_mapping_v2_ros {

slam_mapping_v2::FrameInput fromOdomMsg(const nav_msgs::Odometry& msg);
slam_mapping_v2::FrameInput fromScanMsg(const sensor_msgs::PointCloud2& msg);
slam_mapping_v2::GnssObservation fromGnssMsg(const deeplumin_msgs::Gpchc& msg);

deeplumin_msgs::MappingStatus toStatusMsg(const slam_mapping_v2::RuntimeStatus& status, const ros::Time& stamp);

slam_mapping_v2::StartRequest fromStartReq(const deeplumin_msgs::StartMappingRuntime::Request& req);
slam_mapping_v2::StopRequest fromStopReq(const deeplumin_msgs::StopMappingRuntime::Request& req);
slam_mapping_v2::SaveRequest fromSaveReq(const deeplumin_msgs::SaveMappingRuntime::Request& req);
void fillStatusRes(const slam_mapping_v2::RuntimeStatus& status,
                   deeplumin_msgs::GetMappingRuntimeStatus::Response* res,
                   const ros::Time& stamp);
void fillSaveRes(const slam_mapping_v2::SaveResult& result,
                 deeplumin_msgs::SaveMappingRuntime::Response* res);

}  // namespace slam_mapping_v2_ros
