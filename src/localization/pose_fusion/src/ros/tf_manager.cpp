#include "pose_fusion/ros/tf_manager.hpp"
#include "pose_fusion/ros/ros_adapter.hpp"

#include <vector>

namespace pose_fusion_ros {

void TfManager::publish(const pose_fusion::FusionOutput& output,
                        const pose_fusion::FusionConfig& config,
                        const ros::Time& stamp) {
  if (!output.valid || !config.runtime.publish_tf) {
    return;
  }
  std::vector<geometry_msgs::TransformStamped> transforms;
  if (config.runtime.publish_map_to_odom_tf) {
    transforms.push_back(toTransformMsg(output.map_odom, config.frames.map, config.frames.odom, stamp));
  }
  if (config.runtime.publish_odom_to_base_tf) {
    transforms.push_back(toTransformMsg(output.odom_base, config.frames.odom, config.frames.base_link, stamp));
  }
  if (!transforms.empty()) {
    broadcaster_.sendTransform(transforms);
  }
}

}  // namespace pose_fusion_ros
