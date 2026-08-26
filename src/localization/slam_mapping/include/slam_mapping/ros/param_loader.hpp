#pragma once

#include "slam_mapping/common/config.hpp"

#include <ros/ros.h>

namespace slam_mapping_ros {

slam_mapping::SlamMappingConfig loadConfig(ros::NodeHandle& pnh);

}  // namespace slam_mapping_ros
