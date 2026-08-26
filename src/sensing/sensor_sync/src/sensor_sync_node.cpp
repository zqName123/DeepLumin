#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_sync");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "sensor_sync");
    ros::spin();
    return 0;
}
