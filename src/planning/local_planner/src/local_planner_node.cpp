#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "local_planner");
    ros::spin();
    return 0;
}
