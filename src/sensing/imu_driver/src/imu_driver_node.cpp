#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "imu_driver");
    ros::spin();
    return 0;
}
