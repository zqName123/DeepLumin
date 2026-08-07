#pragma once

/**
 * @file matcher_preprocess.hpp
 * @brief matcher（GICP / VGICP）共享的预处理与候选选择辅助
 *        从 relocalization_framework 移植，行为保持一致。
 */

#include <relocalization/core/common.hpp>
#include <relocalization/core/keyframe_database.hpp>
#include <relocalization/interface/i_descriptor.hpp>
#include <relocalization/interface/i_global_matcher.hpp>

#include <cmath>
#include <limits>

namespace relocalization {
namespace matcher_preprocess {

inline CloudPtr removeGroundRelative(const CloudConstPtr& cloud, const Eigen::Matrix4d& cand_inv,
                                     double rel_z) {
  CloudPtr filtered(new CloudT);
  if (!cloud) {
    return filtered;
  }
  filtered->reserve(cloud->size());
  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    const Eigen::Vector4d local = cand_inv * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
    if (local.z() < rel_z) {
      continue;
    }
    filtered->push_back(pt);
  }
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

inline CloudPtr filterCloudByZForGicp(const CloudConstPtr& cloud, double min_z, double max_z,
                                      bool near_ceiling_only, double near_radius) {
  CloudPtr filtered(new CloudT);
  if (!cloud) {
    return filtered;
  }
  filtered->reserve(cloud->size());
  const double near_radius_sq = near_radius * near_radius;
  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    if (pt.z < static_cast<float>(min_z)) {
      continue;
    }
    const double r_sq = static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y;
    const bool apply_ceiling = !near_ceiling_only || r_sq <= near_radius_sq;
    if (apply_ceiling && pt.z > static_cast<float>(max_z)) {
      continue;
    }
    filtered->push_back(pt);
  }
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

/**
 * @brief target 获取（与 relocalization_framework 一致）：
 *   target_source=="crop" → 全局地图按候选位姿朝向裁剪（crop/forward|backward|left|right|down|up）
 *   裁剪为空 → 关键帧子图拼接；再为空 → 全局地图兜底裁剪
 */
inline CloudPtr buildTargetForCandidate(const GlobalMatcherConfig& config,
                                        const KeyframeDatabase& database,
                                        const DescriptorCandidate& candidate) {
  CloudPtr target;
  if (config.target_source == "crop") {
    target = database.cropOriented(candidate.pose, config.crop_forward, config.crop_backward,
                                   config.crop_left, config.crop_right, config.crop_down,
                                   config.crop_up);
  }
  if (!target || target->empty()) {
    target = database.buildSubmapAround(candidate.keyframe_id);
  }
  if (!target || target->empty()) {
    target = database.cropGlobalMap(candidate.pose);
  }
  return target;
}

/**
 * @brief 候选空间门控：以 top1（candidates.front()）位姿为参考，
 *        只保留 XY 距离 ≤ radius 的候选；radius≤0 表示不启用。
 */
inline std::vector<DescriptorCandidate> gateCandidatesByDistance(
    const std::vector<DescriptorCandidate>& candidates, double radius) {
  std::vector<DescriptorCandidate> out;
  if (candidates.empty()) {
    return out;
  }
  if (radius <= 0.0) {
    return candidates;
  }
  const Pose& ref = candidates.front().pose;
  out.push_back(candidates.front());
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    const double d = (candidates[i].pose.t.head<2>() - ref.t.head<2>()).norm();
    if (d <= radius) {
      out.push_back(candidates[i]);
    }
  }
  return out;
}

inline bool fitnessOk(const MatchResult& result, const GlobalMatcherConfig& config) {
  return result.fitness <= config.accept_fitness;
}

inline bool inlierRatioOk(const MatchResult& result, const GlobalMatcherConfig& config) {
  return config.min_inlier_ratio <= 0.0 || result.inlier_ratio >= config.min_inlier_ratio;
}

inline bool inlierRmseOk(const MatchResult& result, const GlobalMatcherConfig& config) {
  return config.max_inlier_rmse <= 0.0 || result.inlier_rmse <= config.max_inlier_rmse;
}

inline int acceptedGateCount(const MatchResult& result, const GlobalMatcherConfig& config) {
  int count = 0;
  if (result.converged) {
    ++count;
  }
  if (fitnessOk(result, config)) {
    ++count;
  }
  if (inlierRatioOk(result, config)) {
    ++count;
  }
  if (inlierRmseOk(result, config)) {
    ++count;
  }
  return count;
}

inline bool betterMatchResult(const MatchResult& lhs, const MatchResult& rhs,
                              const GlobalMatcherConfig& config) {
  const int lhs_gates = acceptedGateCount(lhs, config);
  const int rhs_gates = acceptedGateCount(rhs, config);
  if (lhs_gates != rhs_gates) {
    return lhs_gates > rhs_gates;
  }
  if (std::abs(lhs.inlier_ratio - rhs.inlier_ratio) > 1e-6) {
    return lhs.inlier_ratio > rhs.inlier_ratio;
  }
  if (std::abs(lhs.inlier_rmse - rhs.inlier_rmse) > 1e-6) {
    return lhs.inlier_rmse < rhs.inlier_rmse;
  }
  if (std::abs(lhs.fitness - rhs.fitness) > 1e-6) {
    return lhs.fitness < rhs.fitness;
  }
  return lhs.scan_context_score < rhs.scan_context_score;
}

}  // namespace matcher_preprocess
}  // namespace relocalization
