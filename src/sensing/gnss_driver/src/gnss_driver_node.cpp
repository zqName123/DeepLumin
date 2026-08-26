#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_driver");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "gnss_driver");
    ros::spin();
    return 0;
}
