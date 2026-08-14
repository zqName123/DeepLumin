#pragma once

/**
 * @file config_loader.hpp
 * @brief v2 配置加载（只读 v2 自有键，无旧包键）
 */

#include "slam_mapping_v2/common/config.hpp"

#include <ros/ros.h>

namespace slam_mapping_v2_ros {

slam_mapping_v2::MappingConfig loadConfig(ros::NodeHandle& pnh);

}  // namespace slam_mapping_v2_ros
