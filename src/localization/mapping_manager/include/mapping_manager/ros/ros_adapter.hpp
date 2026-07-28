#pragma once

#include "mapping_manager/common/config.hpp"
#include "mapping_manager/common/types.hpp"

#include <deeplumin_msgs/CheckMappingEnvironment.h>
#include <deeplumin_msgs/GetMappingStatus.h>
#include <deeplumin_msgs/MappingEnvironmentItem.h>
#include <deeplumin_msgs/MappingEnvironmentStatus.h>
#include <deeplumin_msgs/MappingRecordStatus.h>
#include <deeplumin_msgs/MappingStatus.h>
#include <deeplumin_msgs/StartBagRecording.h>
#include <deeplumin_msgs/StartMapping.h>
#include <deeplumin_msgs/StopBagRecording.h>
#include <deeplumin_msgs/StopMapping.h>
#include <ros/ros.h>

namespace mapping_manager_ros {

mapping_manager::ManagerConfig loadConfig(ros::NodeHandle& pnh);

deeplumin_msgs::MappingEnvironmentItem toRos(const mapping_manager::EnvironmentItem& src);
deeplumin_msgs::MappingEnvironmentStatus toRos(const mapping_manager::EnvironmentStatusData& src,
                                               const ros::Time& stamp);
deeplumin_msgs::MappingStatus toRos(const mapping_manager::MappingStatusData& src,
                                    const ros::Time& stamp);
deeplumin_msgs::MappingRecordStatus toRos(const mapping_manager::RecordStatusData& src,
                                          const ros::Time& stamp);

mapping_manager::StartMappingRequestData fromRos(const deeplumin_msgs::StartMapping::Request& req);
mapping_manager::StopMappingRequestData fromRos(const deeplumin_msgs::StopMapping::Request& req);
mapping_manager::StartBagRecordingRequestData fromRos(const deeplumin_msgs::StartBagRecording::Request& req);
mapping_manager::StopBagRecordingRequestData fromRos(const deeplumin_msgs::StopBagRecording::Request& req);

void fillRos(const mapping_manager::EnvironmentStatusData& src,
             deeplumin_msgs::CheckMappingEnvironment::Response* res);
void fillRos(const mapping_manager::StartMappingResponseData& src,
             deeplumin_msgs::StartMapping::Response* res);
void fillRos(const mapping_manager::StopMappingResponseData& src,
             deeplumin_msgs::StopMapping::Response* res);
void fillRos(const mapping_manager::StartBagRecordingResponseData& src,
             deeplumin_msgs::StartBagRecording::Response* res);
void fillRos(const mapping_manager::StopBagRecordingResponseData& src,
             deeplumin_msgs::StopBagRecording::Response* res);
mapping_manager::MappingStatusData fromRos(const deeplumin_msgs::MappingStatus& msg);

void fillRos(const mapping_manager::GetStatusResponseData& src,
             deeplumin_msgs::GetMappingStatus::Response* res,
             const ros::Time& stamp);

}  // namespace mapping_manager_ros
