#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_manager");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "localization_manager");
    ros::spin();
    return 0;
}
