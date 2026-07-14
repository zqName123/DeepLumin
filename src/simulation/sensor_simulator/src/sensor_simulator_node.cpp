#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_simulator");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "sensor_simulator");
    ros::spin();
    return 0;
}
