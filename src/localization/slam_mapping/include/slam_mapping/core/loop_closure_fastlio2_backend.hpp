#pragma once

#include "slam_mapping/core/i_loop_closure_backend.hpp"
#include "slam_mapping/core/gnss_optimizer.hpp"

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
  localization::PointCloudPtr latestCurrentSubmap() const override { return latest_current_submap_; }
  localization::PointCloudPtr latestHistorySubmap() const override { return latest_history_submap_; }
  std::pair<int, int> latestLoopPair() const override { return latest_loop_pair_; }
  std::vector<Eigen::Vector3d> gnssFactorPositions() const override {
    return gnss_optimizer_.acceptedPositions();
  }
  std::vector<Eigen::Vector3d> gnssRejectedPositions() const override {
    return gnss_optimizer_.rejectedPositions();
  }
  std::vector<std::pair<int, Eigen::Vector3d>> gnssAcceptedFactors() const override {
    return gnss_optimizer_.acceptedFactors();
  }
  std::vector<std::pair<int, Eigen::Vector3d>> gnssRejectedFactors() const override {
    return gnss_optimizer_.rejectedFactors();
  }
  bool gnssMapEnuTransform(Eigen::Matrix3d* rot, Eigen::Vector3d* trans) const override {
    return gnss_optimizer_.getMapEnuTransform(rot, trans);
  }
  bool gnssEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const override {
    return gnss_optimizer_.getEnuOrigin(lat_deg, lon_deg, alt);
  }

 private:
  localization::PointCloudPtr buildSubmap(const std::vector<MappingKeyframe>& keyframes,
                                          int center_index,
                                          int search_num) const;
  localization::PointCloudPtr buildCurrentSubmap(const std::vector<MappingKeyframe>& keyframes,
                                                 int current_idx) const;
  localization::PointCloudPtr buildHistorySubmap(const std::vector<MappingKeyframe>& keyframes,
                                                 int history_idx) const;
  localization::PointCloudPtr addNormals(const localization::PointCloudPtr& cloud) const;
  bool shouldDetectLoopForLatestKeyframe(const std::vector<MappingKeyframe>& keyframes,
                                         int current_idx) const;
  bool isCandidateNearExistingLoop(int current_idx, int history_idx) const;
  int findBestHistoryCandidateIndex(const std::vector<MappingKeyframe>& keyframes,
                                    int current_idx);
  bool alignCandidateSubmaps(const std::vector<MappingKeyframe>& keyframes,
                             int current_idx,
                             int history_idx,
                             double* score,
                             Eigen::Matrix3d* diff_rot,
                             Eigen::Vector3d* diff_pos);
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
  void markLoopClosed(std::vector<MappingKeyframe>* keyframes,
                      int history_idx,
                      int current_idx);
  void optimize(std::vector<MappingKeyframe>* keyframes, int latest_index);

  LoopClosureConfig config_;
  GnssOptimizer gnss_optimizer_;  ///< GNSS 后端因子管理器（解耦独立模块）
  int previous_index_ = 0;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initialized_estimate_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_history_poses_;
  pcl::VoxelGrid<localization::PointType> submap_voxel_;
  pcl::IterativeClosestPointWithNormals<localization::PointType, localization::PointType>::Ptr icp_;
  std::vector<std::pair<int, int>> loop_history_;
  int last_loop_check_index_ = -1;
  localization::PointCloudPtr latest_current_submap_;
  localization::PointCloudPtr latest_history_submap_;
  std::pair<int, int> latest_loop_pair_{-1, -1};
};

}  // namespace slam_mapping
