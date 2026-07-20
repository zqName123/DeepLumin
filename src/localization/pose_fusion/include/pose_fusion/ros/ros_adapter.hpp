#pragma once

/**
 * @file ros_adapter.hpp
 * @brief ROS 消息 ↔ 融合内部类型的双向转换（节点与 core 之间的边界）
 *
 * from*：订阅回调中把 ROS 消息变成 PoseObservation / ResetObservation / Policy
 * to*  ：定时器中把 FusionOutput / FusionStatusData 变成可发布消息或 TF
 */

#include "pose_fusion/common/types.hpp"

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/ObserverPolicy.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace pose_fusion_ros {

pose_fusion::PoseObservation fromOdom(const nav_msgs::Odometry& msg,
                                      pose_fusion::ObservationSource source);
pose_fusion::PoseObservation fromGlobalMatch(const deeplumin_msgs::GlobalMatchResult& msg);
pose_fusion::ResetObservation fromInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg,
                                              const std::string& map_id);
pose_fusion::ResetObservation fromRelocalization(const deeplumin_msgs::RelocalizationResult& msg);
pose_fusion::ObserverPolicyData fromObserverPolicy(const deeplumin_msgs::ObserverPolicy& msg);

nav_msgs::Odometry toOdomMsg(const pose_fusion::FusionOutput& output,
                             const std::string& map_frame,
                             const std::string& base_frame);
geometry_msgs::PoseWithCovarianceStamped toPoseMsg(const pose_fusion::FusionOutput& output,
                                                   const std::string& map_frame);
deeplumin_msgs::FusionStatus toStatusMsg(const pose_fusion::FusionStatusData& status,
                                          const std::string& map_frame);
geometry_msgs::PoseStamped toPathPose(const pose_fusion::FusionOutput& output,
                                      const std::string& map_frame);
geometry_msgs::TransformStamped toTransformMsg(const pose_fusion::Pose3d& pose,
                                               const std::string& parent_frame,
                                               const std::string& child_frame,
                                               const ros::Time& stamp);

}  // namespace pose_fusion_ros
