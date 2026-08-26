#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_predictor");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "obstacle_predictor");
    ros::spin();
    return 0;
}
