#include <ros/ros.h>

#include "lio_odometry/ros/ros_adapter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_odometry");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    slam_odometry_ros::RosAdapter adapter;

    if (!adapter.Init(nh, pnh)) {
        ROS_ERROR("Failed to initialize lio_odometry");
        return -1;
    }

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        adapter.tick();
        rate.sleep();
    }

    return 0;
}