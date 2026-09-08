#include "slam_mapping_v2/core/global_map.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace slam_mapping_v2 {

void GlobalMap::configure(const MapConfig& cfg) { cfg_ = cfg; }

void GlobalMap::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  cached_map_.reset();
}

void GlobalMap::rebuild(const std::vector<Keyframe>& keyframes) {
  PointCloudPtr out(new PointCloud());
  for (const auto& kf : keyframes) {
    if (!kf.cloud || kf.cloud->empty()) continue;
    const Eigen::Matrix4f tf =
        (poseToMatrix(kf.pose_map) * poseToMatrix(kf.pose_odom).inverse()).cast<float>();
    PointCloud tmp;
    pcl::transformPointCloud(*kf.cloud, tmp, tf);
    *out += tmp;
  }
  if (cfg_.publish_voxel_leaf > 0.0 && !out->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(cfg_.publish_voxel_leaf, cfg_.publish_voxel_leaf, cfg_.publish_voxel_leaf);
    voxel.setInputCloud(out);
    PointCloudPtr filtered(new PointCloud());
    voxel.filter(*filtered);
    out = filtered;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  cached_map_ = std::move(out);
}

void GlobalMap::append(const Keyframe& kf) {
  if (!kf.cloud || kf.cloud->empty()) return;

  // 与 rebuild 相同的变换：T(pose_map) * T(pose_odom)^{-1}
  const Eigen::Matrix4f tf =
      (poseToMatrix(kf.pose_map) * poseToMatrix(kf.pose_odom).inverse()).cast<float>();
  PointCloud tmp;
  pcl::transformPointCloud(*kf.cloud, tmp, tf);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!cached_map_) {
    cached_map_.reset(new PointCloud());
  }
  *cached_map_ += tmp;

  // 增量降采样，控制可视化点数
  if (cfg_.publish_voxel_leaf > 0.0 && !cached_map_->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(cfg_.publish_voxel_leaf, cfg_.publish_voxel_leaf, cfg_.publish_voxel_leaf);
    voxel.setInputCloud(cached_map_);
    PointCloudPtr filtered(new PointCloud());
    voxel.filter(*filtered);
    cached_map_ = std::move(filtered);
  }
}

PointCloudPtr GlobalMap::cloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_map_ ? cached_map_ : PointCloudPtr(new PointCloud());
}

std::size_t GlobalMap::numPoints() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_map_ ? cached_map_->size() : 0;
}

}  // namespace slam_mapping_v2