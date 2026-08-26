#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_driver");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "lidar_driver");
    ros::spin();
    return 0;
}
