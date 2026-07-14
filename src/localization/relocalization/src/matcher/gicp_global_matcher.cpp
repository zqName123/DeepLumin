/**
 * @file gicp_relocalizer.cpp
 * @brief GICP 精配准模块
 *
 * Scan Context 只提供粗定位（候选地点 + yaw 初值），
 * GICP 负责将当前点云精确对齐到候选局部子图，输出最终 T_map_lidar。
 *
 * 单候选流程：
 *   1. 以候选关键帧为中心构建局部子图（邻域关键帧拼接）
 *   2. 子图为空时 fallback 到全局地图裁剪
 *   3. 用候选位姿 + Scan Context yaw_diff 构造 4x4 初值
 *   4. PCL GICP 迭代优化
 *   5. 检查 converged + fitness + inlier_ratio + inlier_rmse
 *
 * 多候选时优先选通过综合验收的候选；组内按内点率高、RMSE 低、fitness 低排序。
 */

#include <relocalization/matcher/gicp_global_matcher.hpp>

#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <chrono>
#include <limits>
#include <cmath>

namespace relocalization {

namespace {

// TEMP_GICP_Z_FILTER: GICP 输入专用 z 过滤。后续若不需要，删除本函数和 GicpConfig 中相关字段。
CloudPtr filterCloudByZForGicp(const CloudConstPtr& cloud,
                               double min_z,
                               double max_z,
                               bool near_ceiling_only,
                               double near_radius) {
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
    if (pt.z < min_z) {
      continue;
    }
    const double r_sq = static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y;
    const bool apply_ceiling = !near_ceiling_only || r_sq <= near_radius_sq;
    if (apply_ceiling && pt.z > max_z) {
      continue;
    }
    filtered->push_back(pt);
  }
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

bool fitnessOk(const MatchResult& result, const GicpConfig& config) {
  return result.fitness <= config.accept_fitness;
}

bool inlierRatioOk(const MatchResult& result, const GicpConfig& config) {
  return config.min_inlier_ratio <= 0.0 || result.inlier_ratio >= config.min_inlier_ratio;
}

bool inlierRmseOk(const MatchResult& result, const GicpConfig& config) {
  return config.max_inlier_rmse <= 0.0 || result.inlier_rmse <= config.max_inlier_rmse;
}

int acceptedGateCount(const MatchResult& result, const GicpConfig& config) {
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

bool betterMatchResult(const MatchResult& lhs,
                                const MatchResult& rhs,
                                const GicpConfig& config) {
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

}  // namespace

GicpGlobalMatcher::GicpGlobalMatcher(const GicpConfig& config) : config_(config) {}

/**
 * @brief 对 Top-K 候选逐一执行 GICP，返回最优结果
 *
 * 选择策略：优先返回 success=true 的候选；同为成功或同为失败时，使用同一套质量排序：
 *   1. 通过的验收门槛更多（converged / fitness / inlier_ratio / inlier_rmse）
 *   2. inlier_ratio 更高
 *   3. inlier_rmse 更低
 *   4. fitness 更低
 *   5. SC score 更低
 */
MatchResult GicpGlobalMatcher::alignCandidates(const CloudConstPtr& source,
                                                      const std::vector<DescriptorCandidate>& candidates,
                                                      const KeyframeDatabase& database) const {
  MatchResult best_success;
  MatchResult best_failure;
  bool has_success = false;
  bool has_failure = false;
  int idx = 0;
  for (const auto& candidate : candidates) {
    std::cout  << "[GICP] start candidate " << idx
      << "/" << candidates.size()
      << " id=" << candidate.keyframe_id
      << " sc_score=" << candidate.score << std::endl;
    MatchResult result = alignOne(source, candidate, database);
    std::cout << "[GICP] done candidate " << idx
      << " id=" << candidate.keyframe_id
      << " success=" << result.success
      << " converged=" << result.converged
      << " fitness=" << result.fitness
      << " inlier_ratio=" << result.inlier_ratio
      << " pts=" << result.gicp_source_points
      << "/" << result.gicp_target_points
      << " elapsed_ms=" << result.elapsed_ms << std::endl;
    ++idx;

    if (result.success) {
      if (!has_success || betterMatchResult(result, best_success, config_)) {
        best_success = result;
        has_success = true;
      }
    } else {
      if (!has_failure || betterMatchResult(result, best_failure, config_)) {
        best_failure = result;
        has_failure = true;
      }
    }
  }

  if (has_success) {
    return best_success;
  }
  return has_failure ? best_failure : MatchResult();
}

/**
 * @brief 对单个 Scan Context 候选执行 GICP 精配准
 *
 * @param source   当前帧预处理后的点云（LiDAR 坐标系）
 * @param candidate Scan Context 检索结果（含位姿初值和 yaw 修正）
 * @param database  关键帧数据库（用于构建 target 子图）
 */
MatchResult GicpGlobalMatcher::alignOne(const CloudConstPtr& source,
                                               const DescriptorCandidate& candidate,
                                               const KeyframeDatabase& database) const {
  MatchResult result;
  result.candidate_id = candidate.keyframe_id;
  result.candidate_pose = candidate.pose;
  result.scan_context_score = candidate.score;
  result.scan_context_sector_shift = candidate.sector_shift;
  result.scan_context_yaw_diff = candidate.yaw_diff;

  const auto start = std::chrono::steady_clock::now();

  // ----- 构建 GICP target：候选关键帧附近的局部子图 -----
  CloudPtr target = database.buildSubmapAround(candidate.keyframe_id);
  if (!target || target->empty()) {
    // fallback：从全局地图按候选位姿裁剪局部区域
    target = database.cropGlobalMap(candidate.pose);
  }
  if (!source || source->empty() || !target || target->empty()) {
    return result;
  }

  CloudConstPtr gicp_source = source;
  CloudConstPtr gicp_target = target;
  CloudPtr filtered_source;
  CloudPtr filtered_target;
  // TEMP_GICP_Z_FILTER: 仅过滤 GICP 输入，不影响 SC 检索；后续可按标记整体移除。
  if (config_.z_filter_enable) {
    filtered_source = filterCloudByZForGicp(source, config_.z_filter_min, config_.z_filter_max,
                                            config_.near_ceiling_filter_enable,
                                            config_.near_ceiling_radius);
    filtered_target = filterCloudByZForGicp(target, config_.z_filter_min, config_.z_filter_max,
                                            config_.near_ceiling_filter_enable,
                                            config_.near_ceiling_radius);
    gicp_source = filtered_source;
    gicp_target = filtered_target;
  }

  // filtered_source = voxelDownsampleCloud(source, 0.3);
  // filtered_target = voxelDownsampleCloud(target, 0.3);
  // gicp_source = filtered_source;
  // gicp_target = filtered_target;

  result.gicp_source_points = gicp_source ? static_cast<int>(gicp_source->size()) : 0;
  result.gicp_target_points = gicp_target ? static_cast<int>(gicp_target->size()) : 0;
  result.target.reset(new CloudT);
  if (gicp_target) {
    *result.target = *gicp_target;
  }
  if (!gicp_source || gicp_source->empty() || !gicp_target || gicp_target->empty()) {
    return result;
  }

  // ----- 构造 GICP 初值 T_map_lidar -----
  // 位置：候选关键帧的全局位姿平移
  // 朝向：候选关键帧 yaw + Scan Context 估计的 yaw_diff
  const double candidate_yaw = yawFromQuaternion(candidate.pose.q);
  const double init_yaw = normalizeAngle(candidate_yaw + candidate.yaw_diff);
  result.candidate_yaw = candidate_yaw;
  result.initial_yaw = init_yaw;
  Eigen::Matrix4d initial_guess = poseFromXYYaw(
      candidate.pose.t.x(), candidate.pose.t.y(), candidate.pose.t.z(), init_yaw);
  result.initial_guess = initial_guess;

  // ----- 配置并执行 PCL GICP -----
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  gicp.setMaximumIterations(config_.max_iterations);
  gicp.setTransformationEpsilon(config_.transformation_epsilon);
  gicp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  
  gicp.setInputSource(gicp_source);   // TEMP_GICP_Z_FILTER: 当前帧过滤后点云（待对齐）
  gicp.setInputTarget(gicp_target);    // TEMP_GICP_Z_FILTER: 过滤后局部子图（参考地图）

  CloudT aligned;
  gicp.align(aligned, initial_guess.cast<float>());

  result.converged = gicp.hasConverged();
  // fitness：对应点平均距离的平方，越小表示对齐越好
  result.fitness = gicp.getFitnessScore(config_.max_correspondence_distance);
  // 最终变换：将 source 点云变换到 map 坐标系
  result.transform = gicp.getFinalTransformation().cast<double>();
  result.final_yaw = yawFromQuaternion(Eigen::Quaterniond(result.transform.block<3, 3>(0, 0)));
  const Eigen::Matrix4d delta_from_initial = result.transform * result.initial_guess.inverse();
  result.delta_translation = delta_from_initial.block<3, 1>(0, 3).norm();
  result.delta_yaw = normalizeAngle(result.final_yaw - result.initial_yaw);
  // TEMP_GICP_Z_FILTER: GICP 可使用过滤点云求解，但可视化/输出仍显示完整 source 的最终位姿。
  result.aligned = transformCloud(source, result.transform);

  // 额外几何验真：统计 aligned source 到 target 最近邻的内点率与内点 RMSE。
  // PCL fitness 只反映平均最近邻误差，重复结构可能低 fitness 但整体错配；
  // 内点率能约束有多少 source 点真正被 target 支撑。
  if (!aligned.empty() && gicp_target && !gicp_target->empty() && config_.inlier_distance > 0.0) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(gicp_target);
    const double inlier_dist_sq = config_.inlier_distance * config_.inlier_distance;
    double sum_inlier_sq = 0.0;
    std::vector<int> indices(1);
    std::vector<float> sq_dists(1);

    for (const auto& pt : aligned.points) {
      if (!pcl::isFinite(pt)) {
        continue;
      }
      if (kdtree.nearestKSearch(pt, 1, indices, sq_dists) <= 0) {
        continue;
      }
      ++result.correspondence_count;
      const double d2 = static_cast<double>(sq_dists[0]);
      if (d2 <= inlier_dist_sq) {
        ++result.inlier_count;
        sum_inlier_sq += d2;
      }
    }

    if (result.correspondence_count > 0) {
      result.inlier_ratio = static_cast<double>(result.inlier_count) /
                            static_cast<double>(result.correspondence_count);
    }
    if (result.inlier_count > 0) {
      result.inlier_rmse = std::sqrt(sum_inlier_sq / static_cast<double>(result.inlier_count));
    }
  }

  result.success = result.converged && fitnessOk(result, config_) &&
                   inlierRatioOk(result, config_) && inlierRmseOk(result, config_);

  const auto end = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  return result;
}


std::string GicpGlobalMatcher::summary() const {
  return "GICP max_iterations=" + std::to_string(config_.max_iterations) +
         " max_corr=" + std::to_string(config_.max_correspondence_distance) +
         " accept_fitness=" + std::to_string(config_.accept_fitness) +
         " min_inlier_ratio=" + std::to_string(config_.min_inlier_ratio) +
         " max_inlier_rmse=" + std::to_string(config_.max_inlier_rmse);
}

}  // namespace relocalization
