#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_matcher");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "global_matcher");
    ros::spin();
    return 0;
}
