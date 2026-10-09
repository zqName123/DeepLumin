#include "lio_odometry/ros/ros_adapter.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include "syslog/logger.h"

namespace slam_odometry_ros {

bool RosAdapter::Init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // 屏蔽pclerror一下的日志
    std::string config_path;
    pnh.param<std::string>("config_path", config_path, "");
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh, pnh);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();

    std::cout << "lio_odometry_voxelmap: HealthChecker enabled and activated." << std::endl;

    // 加载配置文件
    if (!config_.LoadFromYAML(config_path)) {
        // ROS_ERROR("Failed to load node config from %s", config_path.c_str());
        SYS_LOG(syslog::LogSeverity::ERROR, true)
                << "Failed to load node config from " << config_path;
        return false;
    }

    config_.config_yaml = pnh.param<std::string>("faster_lio_config_yaml", config_path);
    if (config_.config_yaml.empty()) {
        // ROS_ERROR("faster_lio_config_yaml is empty");
        SYS_LOG(syslog::LogSeverity::ERROR, true)
                << "faster_lio_config_yaml is empty";
        return false;
    }

    core_ = std::make_unique<slam_odometry::LaserOdometryCore>();
    if (!core_->Init(config_.config_yaml)) {
        // ROS_ERROR("Failed to init LaserOdometryCore");
        SYS_LOG(syslog::LogSeverity::ERROR, true)
                << "Failed to init LaserOdometryCore";
        return false;
    }
    // 获取lidar类型
    lidar_type_ = core_->GetLidarType();
    // 回传修正类
    correction_handler_ = std::make_unique<slam_odometry::CorrectionHandler>();
    // 是否使用IMU数据
    if (config_.external.use_imu) {
        imu_sub_ = nh.subscribe(config_.external.imu_topic, 200000, &RosAdapter::OnImu, this);
        // ROS_INFO("Subscribed IMU: %s", config_.external.imu_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed IMU: " << config_.external.imu_topic;
    }

    lidar_sub_ = nh.subscribe(config_.external.lidar_topic, 200000, &RosAdapter::OnLidar, this);
    // ROS_INFO("Subscribed LiDAR: %s", config_.external.lidar_topic.c_str());
    SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed LiDAR: " << config_.external.lidar_topic;

    if (config_.external.use_wheel_speed) {
        wheel_sub_ = nh.subscribe(config_.external.wheel_topic, 1000, &RosAdapter::OnWheelSpeed, this);
        // ROS_INFO("Subscribed Wheel: %s", config_.external.wheel_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed Wheel: " << config_.external.wheel_topic;
    }

    if (config_.external.use_gnss) {
        gnss_sub_ = nh.subscribe(config_.external.gnss_topic, 100, &RosAdapter::OnGnss, this);
        // ROS_INFO("Subscribed GNSS: %s", config_.external.gnss_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed GNSS: " << config_.external.gnss_topic;
    }

    if (config_.external.use_correction) {
        correction_sub_ =
            nh.subscribe(config_.external.correction_topic, 100, &RosAdapter::OnCorrection, this);
        // ROS_INFO("Subscribed Correction: %s", config_.external.correction_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed Correction: " << config_.external.correction_topic;
    }

    if (config_.outputs.publish_odom) {
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(config_.outputs.odom_topic, 100);
        // ROS_INFO("Publishing odom: %s", config_.outputs.odom_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Publishing odom: " << config_.outputs.odom_topic;
    }

    reg_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.outputs.registered_scan_topic, 10);
    effect_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.outputs.effect_scan_topic, 10);

    // ROS_INFO("[lio_odometry] Init complete");
    SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[lio_odometry_voxelmap] Init complete";
    return true;
}

void RosAdapter::tick() {
    if (!core_) {
        return;
    }
    if (!core_->Run()) {
        return;
    }

    const auto& state = core_->GetState();
    double timestamp = state.timestamp_;

    if (config_.outputs.publish_odom && state.pose_is_ok_) {
        PublishOdometry(state, timestamp);
        // PublishTF(state, timestamp);
    }
    if (config_.outputs.publish_tf) {
        PublishTF(state, timestamp);
    }

    PublishScans();
}

void RosAdapter::OnLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!core_) {
        return;
    }

    double timestamp = msg->header.stamp.toSec();

    switch (lidar_type_) {
        case slam_odometry::LidarType::OUST64: {
            auto cloud = boost::make_shared<pcl::PointCloud<slam_odometry::OusterPoint>>();
            pcl::fromROSMsg(*msg, *cloud);
            core_->ProcessPointCloud(cloud, timestamp);
            break;
        }
        case slam_odometry::LidarType::VELO32: {
            auto cloud = boost::make_shared<pcl::PointCloud<slam_odometry::VelodynePoint>>();
            pcl::fromROSMsg(*msg, *cloud);
            core_->ProcessPointCloud(cloud, timestamp);
            break;
        }
        default: {
            auto cloud = boost::make_shared<slam_odometry::PointCloudType>();
            pcl::fromROSMsg(*msg, *cloud);
            core_->ProcessPointCloud(cloud, timestamp);
            break;
        }
    }
}

void RosAdapter::OnImu(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!core_) {
        return;
    }
    if (msg->header.stamp.toSec() - last_timestamp_imu_ > 0.05) {
        std::cout << "imu stamp > 0.05 " << std::endl;
    }
    last_timestamp_imu_ = msg->header.stamp.toSec();
    auto imu = std::make_shared<slam_odometry::IMU>();
    imu->timestamp = msg->header.stamp.toSec();
    imu->angular_velocity << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;
    imu->linear_acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    core_->ProcessIMU(imu);
}

void RosAdapter::OnWheelSpeed(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    if (!core_) {
        return;
    }
    double timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                           ? msg->header.stamp.toSec()
                           : ros::Time::now().toSec();
    core_->FeedWheelSpeed(timestamp, msg->speed, static_cast<int>(msg->gear));
}

void RosAdapter::OnGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    if (!core_) {
        return;
    }
    // 在这里进行简单的gnss质量判断
    int status = msg->status;
    int nsv = msg->nsv1 + msg->nsv2;
    // if (status < 33 || nsv < 4) {
    //     std::cout << "gnss status < 33 or nsv < 4 " << std::endl;
    //     return;
    // }
    core_->FeedGnssAttitude(msg->heading, msg->pitch, msg->roll, status, nsv);
}

void RosAdapter::OnCorrection(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!core_ || !correction_handler_) {
        return;
    }
    slam_odometry::CorrectionInput correction;
    correction.valid = true;
    correction.timestamp = msg->header.stamp.toSec();
    correction.t_map_odom << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    correction.q_map_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    correction_handler_->PushCorrection(correction);
}

void RosAdapter::PublishOdometry(const slam_odometry::NavState& state, double timestamp) {
    const auto& base_to_imu = config_.extrinsics.base_to_imu;
    Eigen::Matrix4d T_base_imu = base_to_imu.ToMatrix();

    Eigen::Matrix4d T_odom_imu = Eigen::Matrix4d::Identity();
    T_odom_imu.block<3, 3>(0, 0) = state.rot_.matrix();
    T_odom_imu.block<3, 1>(0, 3) = state.pos_;

    Eigen::Matrix4d T_odom_base = T_odom_imu * T_base_imu.inverse();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time(timestamp);
    odom_msg.header.frame_id = config_.frames.odom;
    odom_msg.child_frame_id = config_.frames.base_link;

    odom_msg.pose.pose.position.x = T_odom_base(0, 3);
    odom_msg.pose.pose.position.y = T_odom_base(1, 3);
    odom_msg.pose.pose.position.z = T_odom_base(2, 3);

    Eigen::Quaterniond q(T_odom_base.block<3, 3>(0, 0));
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();

    odom_msg.twist.twist.linear.x = state.vel_[0];
    odom_msg.twist.twist.linear.y = state.vel_[1];
    odom_msg.twist.twist.linear.z = state.vel_[2];
    odom_msg.twist.twist.angular.x = state.bg_[0];
    odom_msg.twist.twist.angular.y = state.bg_[1];
    odom_msg.twist.twist.angular.z = state.bg_[2];

    odom_pub_.publish(odom_msg);
}

void RosAdapter::PublishScans() {
    auto reg_scan = core_->GetScanDownWorld();
    if (reg_scan && !reg_scan->empty() && reg_scan_pub_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*reg_scan, msg);
        msg.header.stamp = ros::Time(core_->GetState().timestamp_);
        msg.header.frame_id = config_.frames.odom;
        reg_scan_pub_.publish(msg);
    }

    auto effect_scan = core_->GetEffectScan();
    if (effect_scan && !effect_scan->empty() && effect_scan_pub_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*effect_scan, msg);
        msg.header.stamp = ros::Time(core_->GetState().timestamp_);
        msg.header.frame_id = config_.frames.odom;
        effect_scan_pub_.publish(msg);
    }
}

void RosAdapter::PublishTF(const slam_odometry::NavState& state, double timestamp) {
    if (!config_.outputs.publish_tf) {
        return;
    }
    const auto& base_to_imu = config_.extrinsics.base_to_imu;
    Eigen::Matrix4d T_base_imu = base_to_imu.ToMatrix();
    Eigen::Matrix4d T_odom_imu = Eigen::Matrix4d::Identity();
    T_odom_imu.block<3, 3>(0, 0) = state.rot_.matrix();
    T_odom_imu.block<3, 1>(0, 3) = state.pos_;
    Eigen::Matrix4d T_odom_base = T_odom_imu * T_base_imu.inverse();

    Eigen::Quaterniond q(T_odom_base.block<3, 3>(0, 0));
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(T_odom_base(0, 3), T_odom_base(1, 3), T_odom_base(2, 3)));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, ros::Time(timestamp), config_.frames.odom,
                             config_.frames.base_link));
}

}  // namespace slam_odometry_ros