#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parameter_server");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "parameter_server");
    ros::spin();
    return 0;
}
