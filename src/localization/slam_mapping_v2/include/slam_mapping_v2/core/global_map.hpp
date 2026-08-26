#pragma once

/**
 * @file global_map.hpp
 * @brief 全局地图：map 坐标系下的降采样关键帧缓存
 *
 * 职责：
 * - 从关键帧列表重建 map 系点云（逐关键帧变换 + VoxelGrid 降采样）
 * - 提供已缓存的 map 系点云供可视化线程直接发布
 *
 * 线程安全：rebuild() 与 cloud() 可从不同线程调用。
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

#include <mutex>

namespace slam_mapping_v2 {

class GlobalMap {
 public:
  void configure(const MapConfig& cfg);
  void reset();

  /**
   * @brief 从关键帧列表重建 map 系降采样地图
   *
   * 对每个关键帧计算 T(pose_map) * T(pose_odom)^{-1}，将 odom 系点云
   * 变换到 map 系后累加，最后用 VoxelGrid 降采样。回环优化后调用。
   */
  void rebuild(const std::vector<Keyframe>& keyframes);

  /// 导出已缓存的 map 系地图（无需再变换，可直接发布）
  PointCloudPtr cloud() const;

  std::size_t numPoints() const;

 private:
  MapConfig cfg_;
  PointCloudPtr cached_map_;
  mutable std::mutex mutex_;
};

}  // namespace slam_mapping_v2
