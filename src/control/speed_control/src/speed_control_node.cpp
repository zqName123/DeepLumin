#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_control");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "speed_control");
    ros::spin();
    return 0;
}
