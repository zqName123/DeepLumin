#pragma once

/**
 * @file loop_detector_kdtree_gicp.hpp
 * @brief 默认回环检测器：历史位姿 kdtree 空间邻近 + 子图 GICP 精配准验证
 */

#include "slam_mapping_v2/core/i_loop_detector.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <utility>
#include <vector>

namespace slam_mapping_v2 {

class KdtreeGicpLoopDetector : public ILoopDetector {
 public:
  void configure(const LoopClosureConfig& cfg) override;
  void reset() override;
  LoopCandidate detect(const std::vector<Keyframe>& kfs) override;
  const std::vector<std::pair<int, int>>& loopPairs() const override { return loop_history_; }
  PointCloudPtr latestCurrentSubmap() const override { return latest_current_submap_; }
  PointCloudPtr latestHistorySubmap() const override { return latest_history_submap_; }

 private:
  bool shouldCheck(const std::vector<Keyframe>& kfs, int current_idx) const;
  bool isCandidateNearExistingLoop(int current_idx, int history_idx) const;
  PointCloudPtr buildSubmap(const std::vector<Keyframe>& kfs, int center_index,
                            int search_num) const;
  int findBestHistoryCandidateIndex(const std::vector<Keyframe>& kfs, int current_idx);
  bool alignCandidateSubmaps(const std::vector<Keyframe>& kfs, int current_idx, int history_idx,
                             double* score, Mat3d* diff_rot, Vec3d* diff_pos);

  LoopClosureConfig cfg_;
  int last_loop_check_index_ = -1;
  std::vector<std::pair<int, int>> loop_history_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_history_poses_;
  pcl::VoxelGrid<PointType> submap_voxel_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType>::Ptr icp_;
  PointCloudPtr latest_current_submap_;
  PointCloudPtr latest_history_submap_;
};

}  // namespace slam_mapping_v2
