#include "slam_mapping_v2/core/global_map.hpp"

#include "ivox3d.h"

#include <pcl/filters/voxel_grid.h>

#include <vector>

namespace slam_mapping_v2 {

using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;

void GlobalMap::configure(const MapConfig& cfg) {
  cfg_ = cfg;
  IVoxType::Options options;
  options.resolution_ = static_cast<float>(cfg_.voxel_resolution);
  options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  ivox_ = std::make_shared<IVoxType>(options);
}

void GlobalMap::reset() {
  IVoxType::Options options;
  options.resolution_ = static_cast<float>(cfg_.voxel_resolution);
  options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  ivox_ = std::make_shared<IVoxType>(options);
}

void GlobalMap::insert(const FrameInput& f) {
  if (!ivox_ || !f.cloud || f.cloud->empty()) {
    return;
  }
  IVoxType::PointVector to_add;
  to_add.reserve(f.cloud->size());
  const double filter = cfg_.filter_size_map > 0.0 ? cfg_.filter_size_map : 0.2;
  const double filter_sq = filter * filter;
  IVoxType::PointVector nearest;
  for (const auto& pt : f.cloud->points) {
    if (ivox_->GetClosestPoint(pt, nearest, 1, filter) && !nearest.empty()) {
      const double dx = nearest[0].x - pt.x;
      const double dy = nearest[0].y - pt.y;
      const double dz = nearest[0].z - pt.z;
      if (dx * dx + dy * dy + dz * dz < filter_sq) {
        continue;  // 已有更近点，跳过
      }
    }
    to_add.push_back(pt);
  }
  if (!to_add.empty()) {
    ivox_->AddPoints(to_add);
  }
}

PointCloudPtr GlobalMap::cloud(double voxel_leaf) const {
  PointCloudPtr out(new PointCloud());
  if (!ivox_ || ivox_->NumPoints() == 0) {
    return out;
  }
  IVoxType::PointVector pts;
  ivox_->GetAllPoints(pts);
  out->points.assign(pts.begin(), pts.end());
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  if (voxel_leaf > 0.0 && !out->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
    voxel.setInputCloud(out);
    PointCloudPtr filtered(new PointCloud());
    voxel.filter(*filtered);
    return filtered;
  }
  return out;
}

std::size_t GlobalMap::numPoints() const { return ivox_ ? ivox_->NumPoints() : 0; }

}  // namespace slam_mapping_v2
