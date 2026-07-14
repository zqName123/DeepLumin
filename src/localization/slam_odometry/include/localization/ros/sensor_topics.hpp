/**
 * @file sensor_topics.hpp
 * @brief 传感器输入 / 定位输出话题名配置（DR 调试节点会用到 imu / wheel / dr）
 *
 * 在 dr_debug_node 构造阶段调用：
 *   loadSensorTopicsConfig(pnh_)  → 订阅哪个 IMU、轮速话题
 *   loadOutputTopicsConfig(pnh_)  → 发布哪个 dr_odom 话题
 *
 * YAML 对应段：sensors/* 、outputs/*（见 localization_ysw.yaml）
 */
#pragma once

#include <ros/ros.h>
#include <string>

namespace localization_ros {

/** 传感器输入话题（dr_debug 只用 imu + wheel） */
struct SensorTopicsConfig {
    std::string imu = "/ouster/imu";              ///< IMU（默认对齐 ysw 实车）
    std::string lidar = "/ouster/points";         ///< 点云（本节点不用）
    std::string wheel = "/can_receive_info";      ///< 轮速/档位 CAN 消息
    std::string gnss = "/sensor/gnss/fix";
    std::string scene = "/system/scene_state";
};

/** 定位输出话题（dr_debug 只用 dr） */
struct OutputTopicsConfig {
    std::string dr = "/localization/dr_odom";     ///< DR 主输出
    std::string slam = "/localization/slam_odom";
    std::string fused = "/localization/fused_odom";
    std::string covariance = "/localization/covariance";
    std::string status = "/localization/status";
    std::string path = "/localization/path";
    std::string local_map = "/localization/local_map";
    std::string registered_scan = "/localization/cloud_registered";
    std::string global_map = "/localization/global_map";
    std::string loop_markers = "/localization/loop_markers";
};

SensorTopicsConfig loadSensorTopicsConfig(ros::NodeHandle& nh);
OutputTopicsConfig loadOutputTopicsConfig(ros::NodeHandle& nh);

}  // namespace localization_ros
