#pragma once

#include <Eigen/Dense>
#include <string>
#include <yaml-cpp/yaml.h>

namespace slam_odometry_ros {

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

    Eigen::Matrix4d ToMatrix() const {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        t.block<3, 3>(0, 0) = rotation;
        t.block<3, 1>(0, 3) = translation;
        return t;
    }
};

struct ExtrinsicConfig {
    SensorExtrinsic base_to_imu;
    SensorExtrinsic base_to_lidar;
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
    std::string config_yaml;

    FrameConfig frames;
    ExtrinsicConfig extrinsics;
    QueueSizeConfig queue;

    struct External {
        std::string imu_topic = "/ouster/imu";
        std::string lidar_topic = "/ouster/points";
        std::string wheel_topic = "/can_receive";
        std::string gnss_topic = "/gnss_chc";
        std::string correction_topic = "/slam_mapping_v2/correction";
        bool use_imu = true;
        bool use_wheel_speed = false;
        bool use_gnss = false;
        bool use_correction = false;
    } external;

    struct Outputs {
        bool publish_odom = true;
        std::string odom_topic = "/slam_odometry/odom";
        std::string registered_scan_topic = "/slam_odometry/cloud_registered";
        std::string effect_scan_topic = "/slam_odometry/cloud_effect";
        bool publish_tf = true;
        int local_map_publish_interval = 10;
    } outputs;

    bool LoadFromYAML(const std::string& path);
};

}  // namespace slam_odometry_ros