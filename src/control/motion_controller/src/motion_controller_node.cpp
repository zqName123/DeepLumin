#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle nh;
    ROS_INFO("[%s] Node started", "motion_controller");
    ros::spin();
    return 0;
}
