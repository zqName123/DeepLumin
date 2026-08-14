#pragma once

/**
 * @file pose_graph_optimizer.hpp
 * @brief GTSAM ISAM2 位姿图优化（具体类，不绑定回环检测器）
 *
 * 增量添加里程计因子（相邻关键帧）与回环因子（检测器产出），
 * 执行 ISAM2 更新并回写优化后 pose_map。
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <vector>

namespace slam_mapping_v2 {

class PoseGraphOptimizer {
 public:
  void configure(const LoopClosureConfig& cfg);
  void reset();

  /// 为 [previous_index_, size-1] 的新关键帧增量添加里程计因子与初始估计
  void addOdomFactors(const std::vector<Keyframe>& kfs);
  /// 添加回环因子（噪声随 ICP 得分自适应）
  void addLoopFactor(const LoopCandidate& c);
  /// ISAM2 更新并回写 pose_map
  void optimize(std::vector<Keyframe>* kfs);

  gtsam::NonlinearFactorGraph& graph() { return graph_; }

 private:
  LoopClosureConfig cfg_;
  int previous_index_ = 0;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initialized_estimate_;
};

}  // namespace slam_mapping_v2
