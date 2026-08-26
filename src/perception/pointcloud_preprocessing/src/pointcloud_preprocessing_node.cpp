#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_preprocessing");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "pointcloud_preprocessing");
    ros::spin();
    return 0;
}
