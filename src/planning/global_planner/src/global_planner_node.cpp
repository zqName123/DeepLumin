#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "global_planner");
    ros::spin();
    return 0;
}
