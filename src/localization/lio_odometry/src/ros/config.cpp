#include "lio_odometry/ros/config.h"

#include <ros/ros.h>

namespace slam_odometry_ros {

namespace {

SensorExtrinsic LoadExtrinsic(const YAML::Node& node) {
    SensorExtrinsic ext;
    if (node["translation"] && node["translation"].IsSequence() && node["translation"].size() == 3) {
        ext.translation << node["translation"][0].as<double>(), node["translation"][1].as<double>(),
            node["translation"][2].as<double>();
    }
    if (node["rotation"] && node["rotation"].IsSequence() && node["rotation"].size() == 9) {
        ext.rotation << node["rotation"][0].as<double>(), node["rotation"][1].as<double>(),
            node["rotation"][2].as<double>(), node["rotation"][3].as<double>(),
            node["rotation"][4].as<double>(), node["rotation"][5].as<double>(),
            node["rotation"][6].as<double>(), node["rotation"][7].as<double>(),
            node["rotation"][8].as<double>();
    }
    return ext;
}

}  // namespace

bool NodeConfig::LoadFromYAML(const std::string& path) {
    try {
        auto yaml = YAML::LoadFile(path);

        if (yaml["lio_odometry"]["frames"]) {
            auto f = yaml["lio_odometry"]["frames"];
            frames.map = f["map"].as<std::string>(frames.map);
            frames.odom = f["odom"].as<std::string>(frames.odom);
            frames.base_link = f["base_link"].as<std::string>(frames.base_link);
            frames.imu = f["imu"].as<std::string>(frames.imu);
            frames.lidar = f["lidar"].as<std::string>(frames.lidar);
        }

        if (yaml["lio_odometry"]["extrinsics"]) {
            auto e = yaml["lio_odometry"]["extrinsics"];
            extrinsics.base_to_imu = LoadExtrinsic(e["base_to_imu"]);
            extrinsics.base_to_lidar = LoadExtrinsic(e["base_to_lidar"]);
        }

        if (yaml["lio_odometry"]["external"]) {
            auto ext = yaml["lio_odometry"]["external"];
            external.imu_topic = ext["imu_topic"].as<std::string>(external.imu_topic);
            external.lidar_topic = ext["lidar_topic"].as<std::string>(external.lidar_topic);
            external.wheel_topic = ext["wheel_topic"].as<std::string>(external.wheel_topic);
            external.gnss_topic = ext["gnss_topic"].as<std::string>(external.gnss_topic);
            external.correction_topic =
                ext["correction_topic"].as<std::string>(external.correction_topic);
            external.use_imu = ext["use_imu"].as<bool>(external.use_imu);
            external.use_correction = ext["use_correction"].as<bool>(external.use_correction);
        }

        if (yaml["vel_gnss"]) {
            auto vg = yaml["vel_gnss"];
            external.use_wheel_speed = vg["use_wheel_speed"].as<bool>(external.use_wheel_speed);
            external.use_gnss = vg["use_gnss"].as<bool>(external.use_gnss);
        }

        if (yaml["lio_odometry"]["outputs"]) {
            auto out = yaml["lio_odometry"]["outputs"];
            outputs.publish_odom = out["publish_odom"].as<bool>(outputs.publish_odom);
            outputs.odom_topic = out["odom_topic"].as<std::string>(outputs.odom_topic);
            outputs.registered_scan_topic =
                out["registered_scan_topic"].as<std::string>(outputs.registered_scan_topic);
            outputs.effect_scan_topic =
                out["effect_scan_topic"].as<std::string>(outputs.effect_scan_topic);
            outputs.publish_tf = out["publish_tf"].as<bool>(outputs.publish_tf);
            outputs.local_map_publish_interval =
                out["local_map_publish_interval"].as<int>(outputs.local_map_publish_interval);
        }

        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load node config from %s: %s", path.c_str(), e.what());
        return false;
    }
}

}  // namespace slam_odometry_ros