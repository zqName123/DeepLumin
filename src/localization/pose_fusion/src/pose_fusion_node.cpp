#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_fusion");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "pose_fusion");
    ros::spin();
    return 0;
}
