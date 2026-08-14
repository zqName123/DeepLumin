#pragma once
#include "ivox3d.h"

/**
 * @file global_map.hpp
 * @brief 全局地图：faster-lio iVox 增量建图（O(1)/帧，替代关键帧全量重建）
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

#include <memory>

namespace slam_mapping_v2 {

class GlobalMap {
 public:
  void configure(const MapConfig& cfg);
  void reset();

  /// 增量插入一帧 odom 系配准扫描（体素去重，规则同 faster-lio MapIncremental）
  void insert(const FrameInput& f);

  /// 导出全部地图点（可选降采样，voxel_leaf<=0 不降采样）
  PointCloudPtr cloud(double voxel_leaf = 0.0) const;

  std::size_t numPoints() const;

 private:
  MapConfig cfg_;
  std::shared_ptr<faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>> ivox_;
};

}  // namespace slam_mapping_v2
