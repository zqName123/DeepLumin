#pragma once

#include "pose_fusion/common/types.hpp"

#include <ros/ros.h>

namespace pose_fusion_ros {

pose_fusion::FusionConfig loadFusionConfig(ros::NodeHandle& pnh);

}  // namespace pose_fusion_ros
