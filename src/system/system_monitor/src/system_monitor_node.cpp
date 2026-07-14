#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_monitor");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "system_monitor");
    ros::spin();
    return 0;
}
