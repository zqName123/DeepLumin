#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_obstacle_detection");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "camera_obstacle_detection");
    ros::spin();
    return 0;
}
