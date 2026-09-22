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

struct NodeConfig {
    std::string config_yaml;

    FrameConfig frames;
    ExtrinsicConfig extrinsics;

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