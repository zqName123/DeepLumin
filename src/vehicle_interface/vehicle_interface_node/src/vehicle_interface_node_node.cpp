#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_interface_node");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "vehicle_interface_node");
    ros::spin();
    return 0;
}
