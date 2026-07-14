#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_control");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "steering_control");
    ros::spin();
    return 0;
}
