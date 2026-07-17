#pragma once

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
