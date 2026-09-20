#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>

#include "lio_odometry/core/laser_odometry_core.h"
#include "lio_odometry/core/correction_handler.h"
#include "lio_odometry/ros/config.h"

namespace slam_odometry_ros {

/**
 * @brief ROS适配器类，负责处理ROS相关任务：订阅，发布，数据处理在订阅回调中
 * 
 */
class RosAdapter {
   public:
    RosAdapter() = default;
    ~RosAdapter() = default;

    bool Init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void tick();

   private:
    void OnLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void OnImu(const sensor_msgs::Imu::ConstPtr& msg);
    void OnWheelSpeed(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg);
    void OnGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg);
    void OnCorrection(const nav_msgs::Odometry::ConstPtr& msg);

    void PublishOdometry(const slam_odometry::NavState& state, double timestamp);
    void PublishScans();
    void PublishTF(const slam_odometry::NavState& state, double timestamp);

    ros::Subscriber imu_sub_, lidar_sub_, wheel_sub_, gnss_sub_, correction_sub_;
    ros::Publisher odom_pub_, reg_scan_pub_, effect_scan_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    NodeConfig config_;
    std::unique_ptr<slam_odometry::LaserOdometryCore> core_;
    std::unique_ptr<slam_odometry::CorrectionHandler> correction_handler_;

    slam_odometry::LidarType lidar_type_ = slam_odometry::LidarType::OUST64;
    int frame_count_ = 0;
};

}  // namespace slam_odometry_ros