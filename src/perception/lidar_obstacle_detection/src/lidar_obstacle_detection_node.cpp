#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "lidar_obstacle_detection");
    ros::spin();
    return 0;
}
