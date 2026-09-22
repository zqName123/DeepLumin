/**
 * @file slam_odometry_v2_node.cc
 * @brief slam_odometry_v2 ROS 节点
 *
 * 基于 faster_lio_ws 原始算法的纯里程计节点：
 * - 订阅 IMU / LiDAR / 轮速 / GNSS 姿态，原始消息直通 faster_lio 核心（与
 *   slam_odometry 当前外接形式 fastlio2 一致，不做节点侧二次预处理）
 * - 发布 odom / local_map / registered_scan / effect_scan / TF
 * - 仅里程计：全局建图、关键帧、GNSS 优化、回环由 slam_mapping_v2 负责
 *
 * 算法核心由 faster_lio_ws 源码移植（-O3 优化编译），保证运行效果一致。
 */

#include "faster_lio/laser_odometry.h"
#include "faster_lio/utils.h"
#include "slam_odometry_v2/ros/slam_odometry_v2_config.hpp"

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <deeplumin_health_checker/health_checker/health_checker.h>
#include <syslog/logger.h>

namespace slam_odometry_v2 {

namespace {

/// 节点侧里程计结果（与 slam_odometry 的 localization::OdomResult 对应）
struct OdomResult {
    double timestamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
};

Eigen::Matrix4d sensorExtrinsicToMatrix(const SensorExtrinsic& ext) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = ext.rotation;
    t.block<3, 1>(0, 3) = ext.translation;
    return t;
}

Eigen::Matrix4d odomToMatrix(const OdomResult& odom) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = odom.orientation.normalized().toRotationMatrix();
    t.block<3, 1>(0, 3) = odom.position;
    return t;
}

OdomResult matrixToOdom(const OdomResult& src, const Eigen::Matrix4d& transform) {
    OdomResult out = src;
    out.position = transform.block<3, 1>(0, 3);
    out.orientation = Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
    return out;
}

/// odom -> base = odom -> sensor * T_base_sensor^{-1}（与 slam_odometry 节点一致）
OdomResult transformSensorOdomToBase(const OdomResult& sensor_odom,
                                     const SensorExtrinsic& base_to_sensor) {
    if (!sensor_odom.valid) {
        return sensor_odom;
    }
    const Eigen::Matrix4d t_odom_sensor = odomToMatrix(sensor_odom);
    const Eigen::Matrix4d t_base_sensor = sensorExtrinsicToMatrix(base_to_sensor);
    return matrixToOdom(sensor_odom, t_odom_sensor * t_base_sensor.inverse());
}

}  // namespace

class SlamOdometryV2Node {
   public:
    explicit SlamOdometryV2Node(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // 屏蔽pclerror一下的日志   
        
        // ---- 健康监测------
        health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh_, pnh_);
        health_checker_->ENABLE();
        health_checker_->NODE_ACTIVATE();
        // 1. 加载配置
        config_ = loadNodeConfig(pnh_);

        if (config_.slam.faster_lio_config_yaml.empty()) {
            // ROS_ERROR("[slam_odometry_v2] node/faster_lio_config_yaml is empty, cannot init faster-lio");
            SYS_LOG(syslog::LogSeverity::ERROR, true) << "node/faster_lio_config_yaml is empty, cannot init faster-lio";
            ros::shutdown();
            return;
        }

        // 2. 初始化 faster-lio 里程计核心（faster_lio_ws 原始算法）
        odom_ = std::make_unique<faster_lio::LaserOdometry>();
        if (!odom_->InitWithoutROS(config_.slam.faster_lio_config_yaml)) {
            // ROS_ERROR("[slam_odometry_v2] faster-lio InitWithoutROS failed: %s",
            //           config_.slam.faster_lio_config_yaml.c_str());
            SYS_LOG(syslog::LogSeverity::ERROR, true) << "faster-lio InitWithoutROS failed: " << config_.slam.faster_lio_config_yaml;
            odom_.reset();
            ros::shutdown();
            return;
        }
        // 用节点配置覆盖轮速/GNSS 辅助参数（单一开关源）
        odom_->SetVelocityGnssParams(
            config_.slam.use_vel, config_.slam.use_gnss_attitude,
            config_.slam.can_vel_diff_thresh_kmh, config_.slam.can_timeout_sec,
            config_.slam.gnss_attitude_timeout_sec, config_.slam.attitude_diff_thresh_deg,
            config_.slam.attitude_correction_gain);

        // 设置观测参数
        odom_->SetObservationParams(
            config_.slam.wheel_velocity_noise, config_.slam.gnss_attitude_noise);

        // 3. 订阅传感器
        imu_sub_ = nh_.subscribe(config_.sensors.imu, 200000, &SlamOdometryV2Node::onImu, this);
        lidar_sub_ = nh_.subscribe(config_.sensors.lidar, 200000, &SlamOdometryV2Node::onLidar, this);
        if (config_.slam.use_vel) {
            can_sub_ = nh_.subscribe(config_.sensors.wheel, 1000, &SlamOdometryV2Node::onCan, this);
        }
        // GNSS 订阅独立于轮速开关（否则关轮速做 GNSS 隔离调试时数据源一起消失）
        if (config_.slam.use_gnss_attitude) {
            gnss_sub_ = nh_.subscribe(config_.sensors.gnss, 100, &SlamOdometryV2Node::onGnss, this);
        }

        // 4. 发布结果（话题与 slam_odometry 当前外接形式一致）
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.outputs.odom, 100);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.outputs.local_map, 10);
        registered_scan_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(config_.outputs.registered_scan, 10);
        effect_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.outputs.effect_scan, 10);

        // ROS_INFO("[slam_odometry_v2] imu=%s lidar=%s can=%s gnss=%s odom=%s local_map=%s registered_scan=%s effect_scan=%s",
        //          config_.sensors.imu.c_str(), config_.sensors.lidar.c_str(),
        //          config_.sensors.wheel.c_str(), config_.sensors.gnss.c_str(),
        //          config_.outputs.odom.c_str(), config_.outputs.local_map.c_str(),
        //          config_.outputs.registered_scan.c_str(), config_.outputs.effect_scan.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) << "imu=" << config_.sensors.imu << " lidar=" << config_.sensors.lidar << " can=" << config_.sensors.wheel << " gnss=" << config_.sensors.gnss << " odom=" << config_.outputs.odom << " local_map=" << config_.outputs.local_map << " registered_scan=" << config_.outputs.registered_scan << " effect_scan=" << config_.outputs.effect_scan;


        // ROS_INFO("[slam_odometry_v2] use_vel=%s use_gnss_attitude=%s publish_tf=%s faster_lio_yaml=%s",
        //          config_.slam.use_vel ? "true" : "false",
        //          config_.slam.use_gnss_attitude ? "true" : "false",
        //          config_.publish_tf ? "true" : "false",
        //          config_.slam.faster_lio_config_yaml.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) << "use_vel=" << (config_.slam.use_vel ? "true" : "false") << " use_gnss_attitude=" << (config_.slam.use_gnss_attitude ? "true" : "false") << " publish_tf=" << (config_.publish_tf ? "true" : "false") << " faster_lio_yaml=" << config_.slam.faster_lio_config_yaml;
    }

    void tick() {
        if (!odom_) {
            return;
        }
        if (!odom_->Run()) {
            return;
        }

        // 里程计（IMU 系）-> base_link（与 slam_odometry 节点一致）
        OdomResult odom_sensor;
        nav_msgs::Odometry odom_msg;
        if (odom_->GetOdometry(odom_msg)) {
            odom_sensor.timestamp = odom_msg.header.stamp.toSec();
            odom_sensor.position << odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
            odom_sensor.orientation = Eigen::Quaterniond(
                odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
            odom_sensor.valid = true;
        }
        const OdomResult odom = transformSensorOdomToBase(odom_sensor, config_.extrinsics.base_to_imu);

        // 发布里程计 + TF
        if (odom.valid) {
            nav_msgs::Odometry out;
            out.header.stamp = ros::Time(odom.timestamp);
            out.header.frame_id = config_.frames.odom;
            out.child_frame_id = config_.frames.base_link;
            out.pose.pose.position.x = odom.position.x();
            out.pose.pose.position.y = odom.position.y();
            out.pose.pose.position.z = odom.position.z();
            out.pose.pose.orientation.w = odom.orientation.w();
            out.pose.pose.orientation.x = odom.orientation.x();
            out.pose.pose.orientation.y = odom.orientation.y();
            out.pose.pose.orientation.z = odom.orientation.z();
            odom_pub_.publish(out);

            if (config_.publish_tf) {
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(odom.position.x(), odom.position.y(), odom.position.z()));
                transform.setRotation(tf::Quaternion(odom.orientation.x(), odom.orientation.y(),
                                                     odom.orientation.z(), odom.orientation.w()));
                tf_broadcaster_.sendTransform(tf::StampedTransform(
                    transform, ros::Time(odom.timestamp), config_.frames.odom, config_.frames.base_link));
            }
        }

        // 局部地图（iVox 世界系点，frame=odom）：节流发布，
        // 仅在有订阅者且满足发布间隔时才全量拷贝+发布（避免地图增长导致每帧变慢）
        const int local_map_interval = std::max(0, config_.local_map_publish_interval);
        if (local_map_pub_.getNumSubscribers() > 0 &&
            (local_map_interval == 0 || local_map_frame_count_ % local_map_interval == 0)) {
            PointCloudType::Ptr local_map;
            if (odom_->GetLocalMap(local_map)) {
                const double stamp = odom.valid ? odom.timestamp : odom_->lastLidarTime();
                publishCloud(local_map, stamp, config_.frames.odom, local_map_pub_);
            }
        }
        ++local_map_frame_count_;

        // 配准扫描 / 有效对应点（IMU 系 -> odom 系，与 slam_odometry 节点同一变换链）
        PointCloudType::Ptr scan_imu;
        if (odom.valid && odom_->GetScanInImu(scan_imu)) {
            publishScanInOdom(odom, scan_imu, registered_scan_pub_);
        }
        PointCloudType::Ptr effect_imu;
        if (odom.valid && odom_->GetEffectInImu(effect_imu)) {
            publishScanInOdom(odom, effect_imu, effect_scan_pub_);
        }
    }

   private:
    void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!odom_) {
            return;
        }
        // 原始消息直通（与 faster_lio_ws 独立运行一致）
        odom_->IMUCallBack(msg);
        updateImuWarmup(msg->header.stamp.toSec());
    }

    void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!odom_) {
            return;
        }
        if (!canFeedLidar()) {
            ++dropped_lidar_before_warmup_;
            if (dropped_lidar_before_warmup_ == 1 || dropped_lidar_before_warmup_ % 20 == 0) {
                // ROS_INFO("[slam_odometry_v2] drop lidar before IMU warmup: dropped=%lu imu_samples=%lu lidar_t=%.9f",
                //          static_cast<unsigned long>(dropped_lidar_before_warmup_),
                //          static_cast<unsigned long>(imu_warmup_samples_), msg->header.stamp.toSec());
                SYS_LOG(syslog::LogSeverity::INFO, true) << "drop lidar before IMU warmup: dropped=" << dropped_lidar_before_warmup_ << " imu_samples=" << imu_warmup_samples_ << " lidar_t=" << msg->header.stamp.toSec();
            }
            return;
        }
        // 原始消息直通：faster-lio 自带点云预处理（Ouster t 字段等）
        odom_->StandardPCLCallBack(msg);
    }

    void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
        if (!odom_ || !config_.slam.use_vel) {
            return;
        }
        faster_lio::LaserOdometry::CanVelocityData can;
        can.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                            ? msg->header.stamp.toSec()
                            : ros::Time::now().toSec();
        can.speed_kmh = (msg->gear == 2.0) ? -msg->speed : msg->speed;
        can.valid = msg->valid;
        odom_->FeedCanVelocity(can);
    }

    void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
        // GNSS 姿态数据源只由 use_gnss_attitude 门控（不与轮速 use_vel 耦合，
        // 否则关轮速做 GNSS 隔离调试时数据源也会被切断）
        if (!odom_ || !config_.slam.use_gnss_attitude) {
            return;
        }
        faster_lio::LaserOdometry::GnssAttitudeData gnss;
        gnss.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                             ? msg->header.stamp.toSec()
                             : ros::Time::now().toSec();
        gnss.heading_deg = msg->heading;
        gnss.pitch_deg = msg->pitch;
        gnss.roll_deg = msg->roll;
        gnss.status = msg->status;  // GNSS 状态
        gnss.nsv = msg->nsv1 + msg->nsv2;  // 可见卫星数量
        gnss.valid = true;
        odom_->FeedGnssAttitude(gnss);

        // 调试日志
        // LOG_FIRST_N(INFO, 20) << "[GNSS] received: heading=" << msg->heading
        //                       << " pitch=" << msg->pitch << " roll=" << msg->roll
        //                       << " status=" << msg->status << " nsv=" << gnss.nsv;
    }

    void publishCloud(const PointCloudType::Ptr& cloud, double stamp,
                      const std::string& frame_id, const ros::Publisher& pub) {
        if (!cloud || cloud->empty()) {
            return;
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time(stamp);
        msg.header.frame_id = frame_id;
        pub.publish(msg);
    }

    /// body/IMU 系点云变换到 odom 系并发布（与 slam_odometry 节点同一变换链）
    void publishScanInOdom(const OdomResult& odom, const PointCloudType::Ptr& scan,
                           const ros::Publisher& pub) {
        if (!scan || scan->empty()) {
            return;
        }
        // T_odom_lidar = T_odom_base * T_base_lidar
        const Eigen::Matrix4d t_odom_lidar =
            odomToMatrix(odom) * sensorExtrinsicToMatrix(config_.extrinsics.base_to_lidar);
        const Eigen::Matrix3d R = t_odom_lidar.block<3, 3>(0, 0);
        const Eigen::Vector3d t = t_odom_lidar.block<3, 1>(0, 3);

        PointCloudType out;
        out.reserve(scan->size());
        for (const auto& pt : scan->points) {
            const Eigen::Vector3d pw = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
            PointType o = pt;
            o.x = static_cast<float>(pw.x());
            o.y = static_cast<float>(pw.y());
            o.z = static_cast<float>(pw.z());
            out.push_back(o);
        }
        PointCloudType::Ptr out_ptr(new PointCloudType(out));
        publishCloud(out_ptr, odom.timestamp,
                     config_.frames.odom, pub);
    }

    void updateImuWarmup(double stamp) {
        if (first_imu_stamp_ <= 0.0) {
            first_imu_stamp_ = stamp;
        }
        last_imu_stamp_ = stamp;
        ++imu_warmup_samples_;
        if (!config_.startup.enable_imu_warmup || imu_warmup_ready_) {
            return;
        }
        const double duration = last_imu_stamp_ - first_imu_stamp_;
        if (duration >= config_.startup.imu_warmup_duration &&
            imu_warmup_samples_ >= static_cast<std::uint64_t>(std::max(1, config_.startup.min_imu_samples))) {
            imu_warmup_ready_ = true;
            // ROS_INFO("[slam_odometry_v2] IMU warmup ready: samples=%lu duration=%.3fs dropped_lidar=%lu",
            //          static_cast<unsigned long>(imu_warmup_samples_), duration,
            //          static_cast<unsigned long>(dropped_lidar_before_warmup_));
            SYS_LOG(syslog::LogSeverity::INFO, true) << "IMU warmup ready: samples=" << imu_warmup_samples_ << " duration=" << duration << " dropped_lidar=" << dropped_lidar_before_warmup_;
        }
    }

    bool canFeedLidar() const {
        return !config_.startup.enable_imu_warmup || imu_warmup_ready_;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    NodeConfig config_;
    std::unique_ptr<faster_lio::LaserOdometry> odom_;

    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber can_sub_;
    ros::Subscriber gnss_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher registered_scan_pub_;
    ros::Publisher effect_scan_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_;

    double first_imu_stamp_ = 0.0;
    double last_imu_stamp_ = 0.0;
    std::uint64_t imu_warmup_samples_ = 0;
    std::uint64_t dropped_lidar_before_warmup_ = 0;
    bool imu_warmup_ready_ = false;
    std::uint64_t local_map_frame_count_ = 0;  ///< local_map 节流计数器
};

}  // namespace slam_odometry_v2

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_odometry_v2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string log_dir;
    pnh.param<std::string>("log_dir", log_dir, "");
    SYS_LOG_INIT_FULL(log_dir,
                    syslog::LogSeverity::INFO,
                    50,
                    "slam_odometry_v2");

    slam_odometry_v2::SlamOdometryV2Node node(nh);

    ros::Rate rate(500);
    while (ros::ok()) {
        ros::spinOnce();
        node.tick();
        rate.sleep();
    }

    faster_lio::Timer::PrintAll();
    ROS_INFO("[slam_odometry_v2] shutdown");
    return 0;
}