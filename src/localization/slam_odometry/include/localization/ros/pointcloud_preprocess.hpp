#pragma once

#include <localization/common/config.hpp>
#include <localization/common/types.hpp>
#include <localization/ros/ouster_point.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

namespace localization_ros {

// 与 ysw_loc faster-lio pointcloud_preprocess.h 中 LidarType 对齐
enum class LidarType { AVIA = 1, VELO32 = 2, OUST64 = 3 };

/**
 * @brief 点云预处理（从 ysw_loc faster-lio PointCloudPreprocess 移植）
 *
 * 统一 Livox / Velodyne / Ouster 点格式为 localization::PointCloud，
 * curvature 字段存储 per-point 时间偏移（单位 ms），供 Faster-LIO 去畸变。
 */
class PointCloudPreprocess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;

    void set(const localization::SlamConfig& cfg);
    void process(const sensor_msgs::PointCloud2& msg, localization::PointCloudPtr& pcl_out);

    double& blind() { return blind_; }
    int& pointFilterNum() { return point_filter_num_; }
    int& numScans() { return num_scans_; }
    float& timeScale() { return time_scale_; }
    LidarType lidarType() const { return lidar_type_; }

private:
    void oust64Handler(const sensor_msgs::PointCloud2& msg);
    void velodyneHandler(const sensor_msgs::PointCloud2& msg);

    localization::PointCloud cloud_out_; // 输出点云

    LidarType lidar_type_ = LidarType::OUST64;
    int point_filter_num_ = 2;
    int num_scans_ = 64;
    double blind_ = 3.0;
    float time_scale_ = 1e-3f;
    bool given_offset_time_ = false;

};

}  // namespace localization_ros
