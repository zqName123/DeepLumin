#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_driver");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "camera_driver");
    ros::spin();
    return 0;
}
