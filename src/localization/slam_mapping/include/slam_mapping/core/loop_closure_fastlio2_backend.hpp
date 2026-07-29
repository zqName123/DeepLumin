#pragma once

#include "slam_mapping/core/i_loop_closure_backend.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

namespace slam_mapping {

class Fastlio2SamLcLoopClosureBackend : public ILoopClosureBackend {
 public:
  std::string name() const override { return "fastlio2_sam_lc"; }
  bool supported() const override { return true; }
  void configure(const LoopClosureConfig& cfg) override;
  void reset() override;
  LoopClosureResult update(std::vector<MappingKeyframe>* keyframes) override;

 private:
  localization::PointCloudPtr buildSubmap(const std::vector<MappingKeyframe>& keyframes,
                                          int center_index,
                                          int search_num) const;
  localization::PointCloudPtr addNormals(const localization::PointCloudPtr& cloud) const;
  bool detectCandidate(const std::vector<MappingKeyframe>& keyframes,
                       int* current_idx,
                       int* history_idx,
                       double* score,
                       Eigen::Matrix3d* diff_rot,
                       Eigen::Vector3d* diff_pos);
  void addOdomFactors(const std::vector<MappingKeyframe>& keyframes);
  void addLoopFactor(int pre_idx,
                     int cur_idx,
                     double score,
                     const Eigen::Matrix3d& diff_rot,
                     const Eigen::Vector3d& diff_pos);
  void optimize(std::vector<MappingKeyframe>* keyframes, int latest_index);

  LoopClosureConfig config_;
  int previous_index_ = 0;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initialized_estimate_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_history_poses_;
  pcl::VoxelGrid<localization::PointType> submap_voxel_;
  pcl::IterativeClosestPointWithNormals<localization::PointType, localization::PointType>::Ptr icp_;
  std::vector<std::pair<int, int>> loop_history_;
};

}  // namespace slam_mapping
