#include "lio_odometry/ros/ros_adapter.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>

namespace slam_odometry_ros {

bool RosAdapter::Init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // 屏蔽pclerror一下的日志
    std::string config_path;
    pnh.param<std::string>("config_path", config_path, "");
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh, pnh);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();

    std::cout << "lio_odometry: HealthChecker enabled and activated." << std::endl;

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

    lidar_type_ = core_->GetLidarType();

    correction_handler_ = std::make_unique<slam_odometry::CorrectionHandler>();

    // 订阅队列收敛（Stage 1）：队列单位是条不是字节，LiDAR 旧值 200000 × ~2MB 理论上限数十 GB。
    // enable_override=false 时逐字恢复旧常量，用于运行时回退（docs/odom_buffer_memory_fix.md）。
    const int q_imu = config_.queue.enable_override ? config_.queue.imu : 200000;
    const int q_lidar = config_.queue.enable_override ? config_.queue.lidar : 200000;
    const int q_wheel = config_.queue.enable_override ? config_.queue.wheel : 1000;
    const int q_gnss = config_.queue.enable_override ? config_.queue.gnss : 100;
    const int q_correction = config_.queue.enable_override ? config_.queue.correction : 100;

    if (config_.external.use_imu) {
        imu_sub_ = nh.subscribe(config_.external.imu_topic, q_imu, &RosAdapter::OnImu, this);
        // ROS_INFO("Subscribed IMU: %s", config_.external.imu_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "Subscribed IMU: " << config_.external.imu_topic << " queue=" << q_imu;
    }

    lidar_sub_ = nh.subscribe(config_.external.lidar_topic, q_lidar, &RosAdapter::OnLidar, this);
    // ROS_INFO("Subscribed LiDAR: %s", config_.external.lidar_topic.c_str());
    SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "Subscribed LiDAR: " << config_.external.lidar_topic << " queue=" << q_lidar;

    if (config_.external.use_wheel_speed) {
        wheel_sub_ = nh.subscribe(config_.external.wheel_topic, q_wheel, &RosAdapter::OnWheelSpeed, this);
        // ROS_INFO("Subscribed Wheel: %s", config_.external.wheel_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "Subscribed Wheel: " << config_.external.wheel_topic << " queue=" << q_wheel;
    }

    if (config_.external.use_gnss) {
        gnss_sub_ = nh.subscribe(config_.external.gnss_topic, q_gnss, &RosAdapter::OnGnss, this);
        // ROS_INFO("Subscribed GNSS: %s", config_.external.gnss_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "Subscribed GNSS: " << config_.external.gnss_topic << " queue=" << q_gnss;
    }

    if (config_.external.use_correction) {
        correction_sub_ =
            nh.subscribe(config_.external.correction_topic, q_correction, &RosAdapter::OnCorrection, this);
        // ROS_INFO("Subscribed Correction: %s", config_.external.correction_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "Subscribed Correction: " << config_.external.correction_topic << " queue=" << q_correction;
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
                << "[lio_odometry] Init complete";
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
        std::cout << "-------------   publish tf --------------" << std::endl;
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
    // 时间戳必须与 LiDAR 同源。
    // 原实现对 stamp=0 的帧回退到 ros::Time::now()（墙钟），导致轮速进入另一个时间纪元：
    // bag 回放时年龄/窗口判定全部失效（约束 100% 不生效），实车也可能有时钟恒定偏移。
    // 现改为显式丢弃 + 告警：静默失效才是 bug，显式告警才可排查。
    const bool stamp_valid = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0);
    if (!stamp_valid) {
        ++dropped_wheel_bad_stamp_;
        if (dropped_wheel_bad_stamp_ == 1 || dropped_wheel_bad_stamp_ % 100 == 0) {
            SYS_LOG(syslog::LogSeverity::WARNING, true)
                << "[WHEEL] drop frame: invalid header.stamp (total=" << dropped_wheel_bad_stamp_
                << "). CAN 驱动需填写与 LiDAR 同源的 header.stamp。";
        }
        return;
    }
    // CanReceiveInfo.valid 是现成的质量位，之前完全没用
    // if (!msg->valid) {
    //     ++dropped_wheel_invalid_;
    //     return;
    // }
    core_->FeedWheelSpeed(msg->header.stamp.toSec(), msg->speed, static_cast<int>(msg->gear));
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
    core_->FeedGnssAttitude(msg->header.stamp.toSec(), msg->heading, msg->pitch, msg->roll, status, nsv);
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