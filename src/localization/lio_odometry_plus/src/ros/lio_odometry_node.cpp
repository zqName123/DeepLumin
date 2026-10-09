#include <ros/ros.h>

#include "lio_odometry/ros/ros_adapter.h"
#include "syslog/logger.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_odometry");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string log_dir;
    pnh.param<std::string>("log_dir", log_dir, "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
    SYS_LOG_INIT_FULL(log_dir,
                        syslog::LogSeverity::INFO,
                        50,
                        "lio_odometry4");

    slam_odometry_ros::RosAdapter adapter;

    if (!adapter.Init(nh, pnh)) {
        // ROS_ERROR("Failed to initialize lio_odometry");
        SYS_LOG(syslog::LogSeverity::ERROR, true) 
                << "Failed to initialize lio_odometry";
        return -1;
    }

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        adapter.tick();
        rate.sleep();
    }

    return 0;
}