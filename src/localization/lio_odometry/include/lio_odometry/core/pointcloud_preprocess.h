#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lio_odometry/core/types.h"

namespace slam_odometry {

enum class LidarType { AVIA = 1, VELO32, OUST64 };

class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    void Process(const CloudPtr& cloud_in, CloudPtr& pcl_out);

    void Process(const pcl::PointCloud<OusterPoint>::Ptr& cloud_in, CloudPtr& pcl_out);
    void Process(const pcl::PointCloud<VelodynePoint>::Ptr& cloud_in, CloudPtr& pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    double& Blind() { return blind_; }
    int& NumScans() { return num_scans_; }
    int& PointFilterNum() { return point_filter_num_; }
    float& TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

   private:
    PointCloudType cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
};

}  // namespace slam_odometry