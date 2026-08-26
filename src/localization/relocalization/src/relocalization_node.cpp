#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "relocalization");
  ROS_ERROR("Use global_relocalization_node or bag_eval_node. Launch files are provided in the relocalization package.");
  return 1;
}
