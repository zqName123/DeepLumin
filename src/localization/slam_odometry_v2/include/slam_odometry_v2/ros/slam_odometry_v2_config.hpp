#pragma once

/**
 * @file slam_odometry_v2_config.hpp
 * @brief slam_odometry_v2 节点配置（自包含，不依赖 slam_odometry 包）
 *
 * 结构与 slam_odometry 的 localization/sensors/outputs/slam 一致，
 * 保证与当前外接形式（fastlio2）的输入输出对齐。
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>

namespace slam_odometry_v2 {

struct FrameConfig {
    std::string map = "map";
    std::string odom = "odom";
    std::string base_link = "base_link";
    std::string imu = "imu_link";
    std::string lidar = "lidar_link";
};

struct SensorExtrinsic {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

struct ExtrinsicConfig {
    SensorExtrinsic base_to_imu;
    SensorExtrinsic base_to_lidar;
};

struct SensorTopicConfig {
    std::string imu = "/ouster/imu";
    std::string lidar = "/ouster/points";
    std::string wheel = "/can_receive_info";
    std::string gnss = "/gnss_chc_data";
};

struct OutputTopicConfig {
    std::string odom = "/slam_odometry/odom";
    std::string local_map = "/slam_odometry/local_map";
    std::string registered_scan = "/slam_odometry/cloud_registered";
    std::string effect_scan = "/slam_odometry/cloud_effect";
};

struct SlamConfig {
    std::string faster_lio_config_yaml;  ///< faster-lio（faster_lio_ws 原始算法）yaml 路径
    bool use_vel = false;
    bool use_gnss_attitude = false;
    double can_vel_diff_thresh_kmh = 2.0;
    double can_timeout_sec = 0.2;
    double gnss_attitude_timeout_sec = 1.0;
    double attitude_diff_thresh_deg = 2.0;
    double attitude_correction_gain = 0.7;

    // 观测更新参数
    double wheel_velocity_noise = 0.1089;    ///< 轮速 3D 伪观测噪声方差 (m²/s²)，σ=0.33 对齐 lm_slam
    double gnss_attitude_noise = 1e-3;       ///< GNSS 相对姿态噪声方差 (rad²)，σ≈1.8° 对齐 lm_slam
};

struct StartupConfig {
    bool enable_imu_warmup = true;
    double imu_warmup_duration = 1.0;
    int min_imu_samples = 20;
};

/// 订阅队列长度（单位：条）。
/// enable_override=false 时逐字恢复旧常量（imu/lidar=200000, wheel=1000, gnss/correction=100），
/// 用于运行时回退，无需重编译。详见 src/localization/docs/odom_buffer_memory_fix.md
struct QueueSizeConfig {
    bool enable_override = true;
    int imu = 200;         // 200Hz × 1s
    int lidar = 10;        // 10Hz × 1s，单帧 ~2MB，必须小
    int wheel = 100;
    int gnss = 50;
    int correction = 50;
};

struct NodeConfig {
    FrameConfig frames;
    ExtrinsicConfig extrinsics;
    SensorTopicConfig sensors;
    OutputTopicConfig outputs;
    SlamConfig slam;
    StartupConfig startup;
    QueueSizeConfig queue;
    bool publish_tf = true;
    int local_map_publish_interval = 10;  ///< local_map 发布间隔（帧），0=每帧
};

/// 从节点私有命名空间（~）加载全部配置
NodeConfig loadNodeConfig(ros::NodeHandle& pnh);

}  // namespace slam_odometry_v2