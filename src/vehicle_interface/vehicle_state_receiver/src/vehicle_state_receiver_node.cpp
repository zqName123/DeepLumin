#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_state_receiver");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "vehicle_state_receiver");
    ros::spin();
    return 0;
}
