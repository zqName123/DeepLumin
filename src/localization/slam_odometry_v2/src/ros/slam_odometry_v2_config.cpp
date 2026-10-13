#include "slam_odometry_v2/ros/slam_odometry_v2_config.hpp"

#include <ros/ros.h>
#include <string>
#include <vector>

namespace slam_odometry_v2 {

namespace {

SensorExtrinsic loadExtrinsic(ros::NodeHandle& nh, const std::string& prefix) {
    SensorExtrinsic ext;
    std::vector<double> t;
    std::vector<double> r;
    nh.getParam(prefix + "/translation", t);
    nh.getParam(prefix + "/rotation", r);
    if (t.size() == 3) {
        ext.translation << t[0], t[1], t[2];
    }
    if (r.size() == 9) {
        ext.rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
    }
    return ext;
}

}  // namespace

NodeConfig loadNodeConfig(ros::NodeHandle& pnh) {
    NodeConfig cfg;

    // localization/frames
    pnh.param<std::string>("localization/frames/map", cfg.frames.map, "map");
    pnh.param<std::string>("localization/frames/odom", cfg.frames.odom, "odom");
    pnh.param<std::string>("localization/frames/base_link", cfg.frames.base_link, "base_link");
    pnh.param<std::string>("localization/frames/imu", cfg.frames.imu, "imu_link");
    pnh.param<std::string>("localization/frames/lidar", cfg.frames.lidar, "lidar_link");

    // localization/extrinsics
    cfg.extrinsics.base_to_imu = loadExtrinsic(pnh, "localization/extrinsics/base_to_imu");
    cfg.extrinsics.base_to_lidar = loadExtrinsic(pnh, "localization/extrinsics/base_to_lidar");

    // sensors
    pnh.param<std::string>("sensors/imu_topic", cfg.sensors.imu, "/ouster/imu");
    pnh.param<std::string>("sensors/lidar_topic", cfg.sensors.lidar, "/ouster/points");
    pnh.param<std::string>("sensors/wheel_topic", cfg.sensors.wheel, "/can_receive_info");
    pnh.param<std::string>("sensors/gnss_topic", cfg.sensors.gnss, "/gnss_chc_data");

    // outputs（与 slam_odometry 当前外接形式默认一致）
    pnh.param<std::string>("outputs/odom_topic", cfg.outputs.odom, "/slam_odometry/odom");
    pnh.param<std::string>("outputs/local_map_topic", cfg.outputs.local_map, "/slam_odometry/local_map");
    pnh.param<std::string>("outputs/registered_scan_topic", cfg.outputs.registered_scan,
                           "/slam_odometry/cloud_registered");
    pnh.param<std::string>("outputs/effect_scan_topic", cfg.outputs.effect_scan,
                           "/slam_odometry/cloud_effect");

    // slam
    pnh.param<std::string>("node/faster_lio_config_yaml", cfg.slam.faster_lio_config_yaml, "");
    pnh.param<bool>("vel_gnss/use_vel", cfg.slam.use_vel, false);
    pnh.param<bool>("vel_gnss/use_gnss_attitude", cfg.slam.use_gnss_attitude, false);
    pnh.param<double>("vel_gnss/can_vel_diff_thresh_kmh", cfg.slam.can_vel_diff_thresh_kmh, 2.0);
    pnh.param<double>("vel_gnss/can_timeout_sec", cfg.slam.can_timeout_sec, 0.2);
    pnh.param<double>("vel_gnss/gnss_attitude_timeout_sec", cfg.slam.gnss_attitude_timeout_sec, 1.0);
    pnh.param<double>("vel_gnss/attitude_diff_thresh_deg", cfg.slam.attitude_diff_thresh_deg, 2.0);
    pnh.param<double>("vel_gnss/attitude_correction_gain", cfg.slam.attitude_correction_gain, 0.7);

    // 观测更新参数（固定 R，对齐 lm_slam）
    pnh.param<double>("vel_gnss/wheel_velocity_noise", cfg.slam.wheel_velocity_noise, 0.1089);
    pnh.param<double>("vel_gnss/gnss_attitude_noise", cfg.slam.gnss_attitude_noise, 1e-3);

    pnh.param<bool>("node/publish_tf", cfg.publish_tf, true);
    pnh.param<int>("node/local_map_publish_interval", cfg.local_map_publish_interval, 10);

    // startup
    pnh.param<bool>("node/startup/enable_imu_warmup", cfg.startup.enable_imu_warmup, true);
    pnh.param<double>("node/startup/imu_warmup_duration", cfg.startup.imu_warmup_duration, 1.0);
    pnh.param<int>("node/startup/min_imu_samples", cfg.startup.min_imu_samples, 20);

    // 订阅队列（Stage 1）：enable_override=false 时恢复旧常量
    pnh.param<bool>("queue_size/enable_override", cfg.queue.enable_override, true);
    pnh.param<int>("queue_size/imu", cfg.queue.imu, 200);
    pnh.param<int>("queue_size/lidar", cfg.queue.lidar, 10);
    pnh.param<int>("queue_size/wheel", cfg.queue.wheel, 100);
    pnh.param<int>("queue_size/gnss", cfg.queue.gnss, 50);
    pnh.param<int>("queue_size/correction", cfg.queue.correction, 50);

    return cfg;
}

}  // namespace slam_odometry_v2