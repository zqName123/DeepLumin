/**
 * @file sensor_topics.cpp
 * @brief 从 YAML 加载传感器输入话题与定位输出话题名
 *
 * 对应配置段：sensors/* 与 outputs/*
 * 实车/ysw 联调：localization_ysw.yaml（/ouster/imu、/can_receive_info）
 * 通用模板：localization.yaml（/sensor/*）
 *
 * dr_debug_node 调用链：
 *   DrDebugNode 构造 → loadSensorTopicsConfig / loadOutputTopicsConfig
 *                   → subscribe(imu/wheel) / advertise(dr)
 */
#include <localization/ros/sensor_topics.hpp>

namespace localization_ros {

SensorTopicsConfig loadSensorTopicsConfig(ros::NodeHandle& nh) {
    SensorTopicsConfig cfg;
    // 第二个参数是默认值：YAML 缺项时保留结构体默认（ysw 话题名）
    nh.param<std::string>("sensors/imu_topic", cfg.imu, cfg.imu);
    nh.param<std::string>("sensors/lidar_topic", cfg.lidar, cfg.lidar);
    nh.param<std::string>("sensors/wheel_topic", cfg.wheel, cfg.wheel);
    nh.param<std::string>("sensors/gnss_topic", cfg.gnss, cfg.gnss);
    nh.param<std::string>("sensors/scene_topic", cfg.scene, cfg.scene);
    return cfg;
}

OutputTopicsConfig loadOutputTopicsConfig(ros::NodeHandle& nh) {
    OutputTopicsConfig cfg;
    nh.param<std::string>("outputs/dr_topic", cfg.dr, cfg.dr);
    nh.param<std::string>("outputs/slam_topic", cfg.slam, cfg.slam);
    nh.param<std::string>("outputs/fused_topic", cfg.fused, cfg.fused);
    nh.param<std::string>("outputs/covariance_topic", cfg.covariance, cfg.covariance);
    nh.param<std::string>("outputs/status_topic", cfg.status, cfg.status);
    nh.param<std::string>("outputs/path_topic", cfg.path, cfg.path);
    nh.param<std::string>("outputs/local_map_topic", cfg.local_map, cfg.local_map);
    nh.param<std::string>("outputs/registered_scan_topic", cfg.registered_scan, cfg.registered_scan);
    nh.param<std::string>("outputs/global_map_topic", cfg.global_map, cfg.global_map);
    nh.param<std::string>("outputs/loop_markers_topic", cfg.loop_markers, cfg.loop_markers);
    return cfg;
}

}  // namespace localization_ros
