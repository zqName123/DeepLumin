#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_logger");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "data_logger");
    ros::spin();
    return 0;
}
