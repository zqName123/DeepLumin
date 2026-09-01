#include "slam_mapping/ros/ros_adapter.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  slam_mapping_ros::SlamMappingRosAdapter node;
  if (!node.initialize(nh, pnh)) {
    ROS_ERROR("[slam_mapping] initialize failed");
    return 1;
  }

  ros::Rate rate(node.processRateHz());
  while (ros::ok()) {
    ros::spinOnce();
    node.spinOnce();
    rate.sleep();
  }

  // [NEW][shutdown_autosave] Save unfinished standalone runs exactly once on exit.
  node.shutdown();
  return 0;
}
