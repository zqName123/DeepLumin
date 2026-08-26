#pragma once

/**
 * @file tf_manager.hpp
 * @brief 按配置发布 map→odom / odom→base_link TF
 *
 * 全系统默认只发 map→odom，避免与 DR/SLAM 的 odom→base 调试 TF 冲突。
 */

#include "pose_fusion/common/types.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace pose_fusion_ros {

class TfManager {
 public:
  void publish(const pose_fusion::FusionOutput& output,
               const pose_fusion::FusionConfig& config,
               const ros::Time& stamp);

 private:
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace pose_fusion_ros
