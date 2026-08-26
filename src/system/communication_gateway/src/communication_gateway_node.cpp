#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "communication_gateway");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "communication_gateway");
    ros::spin();
    return 0;
}
