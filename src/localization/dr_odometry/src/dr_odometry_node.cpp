#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dr_odometry");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "dr_odometry");
    ros::spin();
    return 0;
}
