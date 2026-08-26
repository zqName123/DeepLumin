#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_smoother");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "path_smoother");
    ros::spin();
    return 0;
}
