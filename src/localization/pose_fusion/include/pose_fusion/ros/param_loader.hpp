#pragma once

/**
 * @file param_loader.hpp
 * @brief 从 ROS 私有参数服务器（~）读取 FusionConfig
 *
 * 参数来源：pose_fusion.launch 中 <rosparam file="pose_fusion.yaml" command="load"/>
 */

#include "pose_fusion/common/types.hpp"

#include <ros/ros.h>

namespace pose_fusion_ros {

/** @param pnh 节点私有句柄（构造 PoseFusionNode 时的 "~"） */
pose_fusion::FusionConfig loadFusionConfig(ros::NodeHandle& pnh);

}  // namespace pose_fusion_ros
