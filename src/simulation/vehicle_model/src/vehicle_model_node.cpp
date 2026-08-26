#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_model");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "vehicle_model");
    ros::spin();
    return 0;
}
