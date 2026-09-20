#include "lio_odometry/core/pointcloud_preprocess.h"

namespace slam_odometry {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const CloudPtr& cloud_in, CloudPtr& pcl_out) {
    cloud_out_.clear();

    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        if (static_cast<int>(i) % point_filter_num_ != 0) {
            continue;
        }

        const auto& pt = cloud_in->points[i];
        double range = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.intensity = pt.intensity;
        added_pt.time = pt.time;

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const pcl::PointCloud<OusterPoint>::Ptr& cloud_in, CloudPtr& pcl_out) {
    cloud_out_.clear();
    cloud_out_.reserve(cloud_in->size());

    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        if (static_cast<int>(i) % point_filter_num_ != 0) {
            continue;
        }

        double range = cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y +
                       cloud_in->points[i].z * cloud_in->points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        PointType added_pt;
        added_pt.x = cloud_in->points[i].x;
        added_pt.y = cloud_in->points[i].y;
        added_pt.z = cloud_in->points[i].z;
        added_pt.intensity = cloud_in->points[i].intensity;
        added_pt.time = cloud_in->points[i].t / 1e6;

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const pcl::PointCloud<VelodynePoint>::Ptr& cloud_in, CloudPtr& pcl_out) {
    cloud_out_.clear();
    cloud_out_.reserve(cloud_in->size());

    double omega_l = 3.61;
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);
    std::vector<float> yaw_last(num_scans_, 0.0);
    std::vector<float> time_last(num_scans_, 0.0);

    int plsize = cloud_in->size();
    if (plsize > 0 && cloud_in->points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.x = cloud_in->points[i].x;
        added_pt.y = cloud_in->points[i].y;
        added_pt.z = cloud_in->points[i].z;
        added_pt.intensity = cloud_in->points[i].intensity;
        added_pt.time = cloud_in->points[i].time * time_scale_;

        if (!given_offset_time_) {
            int layer = cloud_in->points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
                (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

}  // namespace slam_odometry