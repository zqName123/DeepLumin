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
    std::string odom = "/localization/slam_odom";
    std::string local_map = "/localization/local_map";
    std::string registered_scan = "/localization/cloud_registered";
    std::string effect_scan = "/localization/cloud_registered_effect";
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
    double wheel_velocity_noise = 0.01;      ///< 轮速观测噪声方差 (m²/s²)
    double non_holonomic_noise = 0.01;       ///< 非完整性约束噪声方差 (m²/s²)
    double gnss_attitude_noise_rad2 = 0.03;  ///< GNSS 姿态观测噪声方差 (rad²)，与 lm_slam 对齐
    int min_gnss_status = 33;                ///< 最小 GNSS 状态（RTK 固定解）
    int min_gnss_nsv = 8;                    ///< 最小 GNSS 卫星数量
    int min_lidar_points = 100;              ///< 最小点云数量
};

struct StartupConfig {
    bool enable_imu_warmup = true;
    double imu_warmup_duration = 1.0;
    int min_imu_samples = 20;
};

struct NodeConfig {
    FrameConfig frames;
    ExtrinsicConfig extrinsics;
    SensorTopicConfig sensors;
    OutputTopicConfig outputs;
    SlamConfig slam;
    StartupConfig startup;
    bool publish_tf = true;
    int local_map_publish_interval = 10;  ///< local_map 发布间隔（帧），0=每帧
};

/// 从节点私有命名空间（~）加载全部配置
NodeConfig loadNodeConfig(ros::NodeHandle& pnh);

}  // namespace slam_odometry_v2