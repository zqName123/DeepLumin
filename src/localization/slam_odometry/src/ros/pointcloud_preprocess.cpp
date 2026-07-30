/**
 * @file pointcloud_preprocess.cpp
 * @brief 点云预处理（从 ysw_loc faster-lio pointcloud_preprocess.cc 移植）
 */
#include <localization/ros/pointcloud_preprocess.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace localization_ros {
namespace {

// [NEW][parking_bag_adaptation] Convert scan rate to angular speed in deg/ms.
double scanRateToOmegaDegPerMs(const double scan_rate) {
    if (scan_rate <= 1e-6) {
        return 3.61;
    }
    return 360.0 / (1000.0 / scan_rate);
}

}  // namespace

/**
    设置点云预处理配置
*/
void PointCloudPreprocess::set(const localization::SlamConfig& cfg) {
    switch (cfg.lidar_type) {
        case 1:
            lidar_type_ = LidarType::AVIA;
            break;
        case 2:
            lidar_type_ = LidarType::VELO32;
            break;
        case 3:
            lidar_type_ = LidarType::OUST64;
            break;
        case 4:  // [NEW][parking_bag_adaptation]
            lidar_type_ = LidarType::ROBOSENSE16;
            break;
        default:
            lidar_type_ = LidarType::OUST64;
            break;
    }
    blind_ = cfg.blind;  // 盲区距离
    point_filter_num_ = cfg.point_filter_num;  // 点云滤波数量
    num_scans_ = cfg.scan_line;  // 扫描线数
    time_scale_ = static_cast<float>(cfg.time_scale);  // 时间尺度
    scan_rate_ = cfg.scan_rate;  // [NEW][parking_bag_adaptation] 用于估计每点时间偏移
}

void PointCloudPreprocess::process(const sensor_msgs::PointCloud2& msg, localization::PointCloudPtr& pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            oust64Handler(msg);
            break;
        case LidarType::VELO32:
            velodyneHandler(msg);
            break;
        case LidarType::ROBOSENSE16:  // [NEW][parking_bag_adaptation]
            robosense16Handler(msg);
            break;
        case LidarType::AVIA:
        default:
            ROS_ERROR("[PointCloudPreprocess] unsupported lidar type for PointCloud2 input");
            cloud_out_.clear();
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::oust64Handler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();

    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.size());
    cloud_out_.reserve(plsize);

    for (int i = 0; i < plsize; ++i) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        const auto& src = pl_orig.points[i];
        const double range = src.x * src.x + src.y * src.y + src.z * src.z;
        if (range < blind_ * blind_) {
            continue;
        }

        localization::PointType added_pt;
        // 注意：此处 x←src.y, y←-src.x 是针对 ysw 安装方式的坐标轴旋转（绕Z轴+90°）。
        // 若实际传感器安装无旋转（即 Ouster 正向朝前），应改回：
        //   added_pt.x = src.x; added_pt.y = src.y;
        // 同时 extrinsic_R 必须与此坐标系定义保持一致：
        //   若此处旋转已补偿安装角，则 extrinsic_R 可为单位矩阵；
        //   否则应在 extrinsic_R 中配置真实的 LiDAR→IMU 旋转。
        added_pt.x = src.y;
        added_pt.y = -src.x;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = src.t / 1e6f;  // 单位：ms，与 imu_processing 的 curvature/1000 对应
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.points.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

// [NEW][parking_bag_adaptation] Handle Robosense-16 organized clouds without ring/time fields.
void PointCloudPreprocess::robosense16Handler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();

    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int height = static_cast<int>(pl_orig.height);
    const int width = static_cast<int>(pl_orig.width);
    const int plsize = static_cast<int>(pl_orig.points.size());
    cloud_out_.reserve(plsize);

    if (height <= 1 || width <= 0) {
        ROS_WARN_THROTTLE(1.0, "[PointCloudPreprocess][parking_bag_adaptation] /rslidar_points is not organized, fallback to XYZI-only ordering");
    }

    const int rings = std::max(1, height);
    const int cols = std::max(1, width);
    const double omega_l = scanRateToOmegaDegPerMs(scan_rate_);
    std::vector<bool> is_first(rings, true);
    std::vector<double> yaw_fp(rings, 0.0);
    std::vector<float> time_last(rings, 0.0f);

    for (int idx = 0; idx < plsize; ++idx) {
        const auto& src = pl_orig.points[idx];
        if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z)) {
            continue;
        }

        localization::PointType added_pt;
        added_pt.normal_x = 0.0f;
        added_pt.normal_y = 0.0f;
        added_pt.normal_z = 0.0f;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;

        const double range = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
        if (range <= blind_ * blind_) {
            continue;
        }

        const int ring = std::min(rings - 1, idx / cols);
        const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.29577951308232;
        float offset_ms = 0.0f;
        if (is_first[ring]) {
            yaw_fp[ring] = yaw_angle;
            is_first[ring] = false;
        } else {
            if (yaw_angle <= yaw_fp[ring]) {
                offset_ms = static_cast<float>((yaw_fp[ring] - yaw_angle) / omega_l);
            } else {
                offset_ms = static_cast<float>((yaw_fp[ring] - yaw_angle + 360.0) / omega_l);
            }
            if (offset_ms < time_last[ring]) {
                offset_ms += static_cast<float>(360.0 / omega_l);
            }
        }
        time_last[ring] = offset_ms;
        added_pt.curvature = offset_ms;

        const int col = idx % cols;
        const int linear_index = ring * cols + col;
        if (linear_index % point_filter_num_ == 0) {
            cloud_out_.points.push_back(added_pt);
        }
    }

    cloud_out_.width = cloud_out_.points.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::velodyneHandler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig); // 
    const int plsize = static_cast<int>(pl_orig.points.size());
    cloud_out_.reserve(plsize);

    const double omega_l = 3.61;
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);
    std::vector<float> yaw_last(num_scans_, 0.0);
    std::vector<float> time_last(num_scans_, 0.0);

    if (plsize > 0 && pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
    }

    for (int i = 0; i < plsize; ++i) {
        localization::PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale_;

        if (!given_offset_time_) {
            const int layer = pl_orig.points[i].ring;
            const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0f;
                yaw_last[layer] = static_cast<float>(yaw_angle);
                time_last[layer] = added_pt.curvature;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle) / omega_l);
            } else {
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle + 360.0) / omega_l);
            }
            if (added_pt.curvature < time_last[layer]) {
                added_pt.curvature += static_cast<float>(360.0 / omega_l);
            }
            yaw_last[layer] = static_cast<float>(yaw_angle);
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num_ == 0) {
            const double range = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range > blind_ * blind_) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.points.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace localization_ros
