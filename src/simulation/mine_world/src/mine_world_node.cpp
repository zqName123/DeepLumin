#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mine_world");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "mine_world");
    ros::spin();
    return 0;
}
