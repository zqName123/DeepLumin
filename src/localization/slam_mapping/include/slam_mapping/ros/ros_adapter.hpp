#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/core/mapping_runtime.hpp"
#include "slam_mapping/ros/visualization_publisher.hpp"

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/GetMappingRuntimeStatus.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/SaveMappingRuntime.h>
#include <deeplumin_msgs/StartMappingRuntime.h>
#include <deeplumin_msgs/StopMappingRuntime.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

namespace slam_mapping_ros {

class SlamMappingRosAdapter {
 public:
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();
  double processRateHz() const;
  // [NEW][shutdown_autosave] Save once on process exit only if runtime is still running.
  void shutdown();

 private:
  void onImu(const sensor_msgs::Imu::ConstPtr& msg);
  void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg);
  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg);

  bool handleStart(deeplumin_msgs::StartMappingRuntime::Request& req,
                   deeplumin_msgs::StartMappingRuntime::Response& res);
  bool handleStop(deeplumin_msgs::StopMappingRuntime::Request& req,
                  deeplumin_msgs::StopMappingRuntime::Response& res);
  bool handleSave(deeplumin_msgs::SaveMappingRuntime::Request& req,
                  deeplumin_msgs::SaveMappingRuntime::Response& res);
  bool handleReset(std_srvs::Trigger::Request& req,
                   std_srvs::Trigger::Response& res);
  bool handleGetStatus(deeplumin_msgs::GetMappingRuntimeStatus::Request& req,
                       deeplumin_msgs::GetMappingRuntimeStatus::Response& res);

  void publishOutputs();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  slam_mapping::SlamMappingConfig config_;
  slam_mapping::MappingRuntime runtime_;
  VisualizationPublisher viz_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber imu_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber can_sub_;
  ros::Subscriber gnss_sub_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer get_status_srv_;
};

}  // namespace slam_mapping_ros
