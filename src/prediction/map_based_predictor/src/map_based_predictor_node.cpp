#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_based_predictor");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "map_based_predictor");
    ros::spin();
    return 0;
}
