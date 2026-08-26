#pragma once

/**
 * @file ros_adapter.hpp
 * @brief ROS 消息 ↔ localization_manager 内部类型的双向转换
 *
 * from*：订阅回调中将 ROS 消息转为 ModuleHealth / GnssView
 * to*  ：onTimer 中将 ManagerSnapshot / ObserverPolicyData 转为可发布消息
 */

#include "localization_manager/common/config.hpp"
#include "localization_manager/common/types.hpp"

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/LocalizationStatus.h>
#include <deeplumin_msgs/ObserverPolicy.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <deeplumin_msgs/SlamHealth.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace localization_manager_ros {

/** 从 ~ 私有参数服务器读取 ManagerConfig */
localization_manager::ManagerConfig loadConfig(ros::NodeHandle& pnh);

localization_manager::ModuleHealth fromLocalizationStatus(const deeplumin_msgs::LocalizationStatus& msg,
                                                          const std::string& name,
                                                          const ros::Time& now);
localization_manager::ModuleHealth fromFusionStatus(const deeplumin_msgs::FusionStatus& msg,
                                                    const ros::Time& now);
localization_manager::ModuleHealth fromSlamHealth(const deeplumin_msgs::SlamHealth& msg,
                                                  const ros::Time& now);
/** 里程计类消息：仅用于刷新 last_stamp / age_sec / available */
localization_manager::ModuleHealth fromOdom(const nav_msgs::Odometry& msg,
                                            const std::string& name,
                                            const ros::Time& now);
localization_manager::ModuleHealth fromGlobalMatchStatus(const deeplumin_msgs::LocalizationStatus& msg,
                                                         const ros::Time& now);

localization_manager::GnssView fromGnss(const deeplumin_msgs::Gpchc& msg);

deeplumin_msgs::LocalizationStatus toStatusMsg(const localization_manager::ManagerSnapshot& snapshot);
deeplumin_msgs::ObserverPolicy toPolicyMsg(const localization_manager::ObserverPolicyData& policy);
/** 将 initialpose 的 frame_id 规范为 config.frames.map */
geometry_msgs::PoseWithCovarianceStamped toInitialPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& msg,
    const std::string& frame_id);
deeplumin_msgs::RelocalizationResult toAcceptedRelocalization(
    const deeplumin_msgs::RelocalizationResult& msg);

std::string stateToString(const localization_manager::ManagerState state);

}  // namespace localization_manager_ros
