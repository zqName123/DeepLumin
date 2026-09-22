/**
 * @file vgicp_global_matcher.cpp
 * @brief VGICP 精配准（small_gicp），从 relocalization_framework 移植
 */

#include <relocalization/matcher/vgicp_global_matcher.hpp>

#include <relocalization/matcher/matcher_preprocess.hpp>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

namespace relocalization {

namespace {

small_gicp::PointCloud::Ptr toSmallGicpCloud(const CloudConstPtr& cloud) {
  std::vector<Eigen::Vector4f> pts;
  if (cloud) {
    pts.reserve(cloud->size());
    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      pts.emplace_back(p.x, p.y, p.z, 1.0f);
    }
  }
  return std::make_shared<small_gicp::PointCloud>(pts);
}

}  // namespace

VgicpGlobalMatcher::VgicpGlobalMatcher(const GlobalMatcherConfig& config) : config_(config) {}

MatchResult VgicpGlobalMatcher::alignOne(const CloudConstPtr& source, const CloudConstPtr& target,
                                         const DescriptorCandidate& candidate,
                                         const Eigen::Matrix4d& initial_guess) const {
  MatchResult result;
  result.candidate_id = candidate.keyframe_id;
  result.candidate_pose = candidate.pose;
  result.scan_context_score = candidate.score;
  result.scan_context_sector_shift = candidate.sector_shift;
  result.scan_context_yaw_diff = candidate.yaw_diff;
  result.initial_guess = initial_guess;

  if (!source || source->empty() || !target || target->empty()) {
    return result;
  }

  const auto start = std::chrono::steady_clock::now();

  // 预处理：相对地面剔除 + z 带过滤（与 relocalization_framework 一致）
  CloudConstPtr prep_source = source;
  CloudConstPtr prep_target = target;
  CloudPtr rel_source;
  CloudPtr rel_target;
  if (config_.ground_remove_relative) {
    const Eigen::Matrix4d cand_inv = candidate.pose.matrix().inverse();
    rel_source =
        matcher_preprocess::removeGroundRelative(source, cand_inv, config_.ground_remove_rel_z);
    rel_target =
        matcher_preprocess::removeGroundRelative(target, cand_inv, config_.ground_remove_rel_z);
    prep_source = rel_source;
    prep_target = rel_target;
  }
  CloudConstPtr gicp_source = prep_source;
  CloudConstPtr gicp_target = prep_target;
  CloudPtr filtered_source;
  CloudPtr filtered_target;
  if (config_.z_filter_enable) {
    filtered_source = matcher_preprocess::filterCloudByZForGicp(
        prep_source, config_.z_filter_min, config_.z_filter_max,
        config_.near_ceiling_filter_enable, config_.near_ceiling_radius);
    filtered_target = matcher_preprocess::filterCloudByZForGicp(
        prep_target, config_.z_filter_min, config_.z_filter_max,
        config_.near_ceiling_filter_enable, config_.near_ceiling_radius);
    gicp_source = filtered_source;
    gicp_target = filtered_target;
  }

  result.gicp_source_points = gicp_source ? static_cast<int>(gicp_source->size()) : 0;
  result.gicp_target_points = gicp_target ? static_cast<int>(gicp_target->size()) : 0;
  result.target.reset(new CloudT);
  if (gicp_target) {
    *result.target = *gicp_target;
  }
  if (!gicp_source || gicp_source->empty() || !gicp_target || gicp_target->empty()) {
    return result;
  }

  result.candidate_yaw = yawFromQuaternion(candidate.pose.q);
  result.initial_yaw = normalizeAngle(
      result.candidate_yaw +
      (config_.use_descriptor_yaw_correction ? candidate.yaw_diff : 0.0));
  result.initial_guess = initial_guess;

  // target：两级 GaussianVoxelMap
  auto target_pc = toSmallGicpCloud(gicp_target);
  auto source_pc = toSmallGicpCloud(gicp_source);
  if (!target_pc || target_pc->size() < 4 || !source_pc || source_pc->size() < 4) {
    return result;
  }
  small_gicp::estimate_covariances_omp(*target_pc, config_.vgicp_num_neighbors,
                                       config_.vgicp_num_threads);
  const double base_res = std::max(0.05, config_.vgicp_voxel_resolution);
  const double coarse_scale = std::max(1.0, config_.vgicp_coarse_scale);
  const double fine_scale = std::max(0.1, config_.vgicp_fine_scale);
  auto coarse_map = std::make_shared<small_gicp::GaussianVoxelMap>(base_res * coarse_scale);
  coarse_map->set_search_offsets(27);
  coarse_map->insert(*target_pc);
  auto fine_map = std::make_shared<small_gicp::GaussianVoxelMap>(base_res * fine_scale);
  fine_map->set_search_offsets(27);
  fine_map->insert(*target_pc);

  small_gicp::estimate_covariances_omp(*source_pc, config_.vgicp_num_neighbors,
                                       config_.vgicp_num_threads);

  // 第一层：粗配准
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> coarse_reg;
  coarse_reg.reduction.num_threads = config_.vgicp_num_threads;
  const double coarse_max_dist = config_.max_correspondence_distance * coarse_scale;
  coarse_reg.rejector.max_dist_sq = coarse_max_dist * coarse_max_dist;
  coarse_reg.criteria.translation_eps = config_.transformation_epsilon;
  coarse_reg.criteria.rotation_eps = config_.transformation_epsilon;
  coarse_reg.optimizer.max_iterations = config_.max_iterations;
  Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
  init.matrix() = initial_guess;
  const auto coarse_result = coarse_reg.align(*coarse_map, *source_pc, *coarse_map, init);

  // 第二层：精匹配（以粗配准结果为初值）
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> fine_reg;
  fine_reg.reduction.num_threads = config_.vgicp_num_threads;
  const double fine_max_dist = config_.max_correspondence_distance * fine_scale;
  fine_reg.rejector.max_dist_sq = fine_max_dist * fine_max_dist;
  fine_reg.criteria.translation_eps = config_.transformation_epsilon;
  fine_reg.criteria.rotation_eps = config_.transformation_epsilon;
  fine_reg.optimizer.max_iterations = config_.max_iterations;
  const Eigen::Isometry3d coarse_T = coarse_result.T_target_source;
  const auto fine_result = fine_reg.align(*fine_map, *source_pc, *fine_map, coarse_T);

  result.converged = fine_result.converged;
  result.transform = fine_result.T_target_source.matrix();
  result.final_yaw =
      yawFromQuaternion(Eigen::Quaterniond(result.transform.block<3, 3>(0, 0)));
  const Eigen::Matrix4d delta_from_initial = result.transform * initial_guess.inverse();
  result.delta_translation = delta_from_initial.block<3, 1>(0, 3).norm();
  result.delta_yaw = normalizeAngle(result.final_yaw - result.initial_yaw);

  result.aligned.reset(new CloudT);
  pcl::transformPointCloud(*gicp_source, *result.aligned, result.transform.cast<float>());

  // inlier / fitness（PCL 语义平均最近邻平方距离）
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(gicp_target);
  const double inlier_dist_sq = config_.inlier_distance * config_.inlier_distance;
  double sum_inlier_sq = 0.0;
  double sum_all_sq = 0.0;
  std::vector<int> indices(1);
  std::vector<float> sq_dists(1);
  for (const auto& pt : result.aligned->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    if (kdtree.nearestKSearch(pt, 1, indices, sq_dists) <= 0) {
      continue;
    }
    ++result.correspondence_count;
    sum_all_sq += static_cast<double>(sq_dists[0]);
    if (sq_dists[0] <= static_cast<float>(inlier_dist_sq)) {
      ++result.inlier_count;
      sum_inlier_sq += static_cast<double>(sq_dists[0]);
    }
  }
  if (result.correspondence_count > 0) {
    result.inlier_ratio = static_cast<double>(result.inlier_count) /
                          static_cast<double>(result.correspondence_count);
    result.fitness = sum_all_sq / static_cast<double>(result.correspondence_count);
  }
  if (result.inlier_count > 0) {
    result.inlier_rmse = std::sqrt(sum_inlier_sq / static_cast<double>(result.inlier_count));
  }

  // VGICP 无 PCL fitness：成功以内点率 / RMSE 为主
  result.success = result.converged && matcher_preprocess::inlierRatioOk(result, config_) &&
                   matcher_preprocess::inlierRmseOk(result, config_);

  const auto end = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  return result;
}

MatchResult VgicpGlobalMatcher::alignCandidates(const CloudConstPtr& source,
                                                const std::vector<DescriptorCandidate>& candidates,
                                                const KeyframeDatabase& database) const {
  MatchResult best_success;
  MatchResult best_failure;
  bool has_success = false;
  bool has_failure = false;

  const double gate_radius =
      config_.candidate_gate_enable ? config_.candidate_gate_radius : 0.0;
  const std::vector<DescriptorCandidate> gated =
      matcher_preprocess::gateCandidatesByDistance(candidates, gate_radius);
  // std::cout << "[VGICP] candidates=" << candidates.size()
  //           << " gated=" << (candidates.size() - gated.size())
  //           << " gate_radius=" << gate_radius << std::endl;
  SYS_LOG(syslog::LogSeverity::INFO, false) 
            << "VGICP] candidates=" << candidates.size() << " gated=" << (candidates.size() - gated.size())
               << " gate_radius=" << gate_radius;

  const auto candidates_start = std::chrono::steady_clock::now();
  std::size_t cand_idx = 0;
  for (const auto& candidate : gated) {
    const auto cand_start = std::chrono::steady_clock::now();
    CloudPtr target =
        matcher_preprocess::buildTargetForCandidate(config_, database, candidate);
    const double submap_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cand_start)
            .count();

    const double candidate_yaw = yawFromQuaternion(candidate.pose.q);
    const double init_yaw =
        config_.use_descriptor_yaw_correction
            ? normalizeAngle(candidate_yaw + candidate.yaw_diff)
            : candidate_yaw;
    const Eigen::Matrix4d initial_guess =
        poseFromXYYaw(candidate.pose.t.x(), candidate.pose.t.y(), candidate.pose.t.z(), init_yaw);

    MatchResult result = alignOne(source, target, candidate, initial_guess);
    const double cand_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cand_start)
            .count();
    // std::cout << "[VGICP] cand=" << cand_idx << "/" << gated.size()
    //           << " id=" << candidate.keyframe_id << " score=" << candidate.score
    //           << " submap_ms=" << submap_ms << " align_ms=" << result.elapsed_ms
    //           << " total_ms=" << cand_ms << " success=" << result.success
    //           << " converged=" << result.converged << " fitness=" << result.fitness
    //           << " inlier=" << result.inlier_ratio << std::endl;
    SYS_LOG(syslog::LogSeverity::INFO, false) 
            << "VGICP] cand=" << cand_idx << "/" << gated.size() << " id=" << candidate.keyframe_id << " score=" << candidate.score
               << " submap_ms=" << submap_ms << " align_ms=" << result.elapsed_ms
               << " total_ms=" << cand_ms << " success=" << result.success
               << " converged=" << result.converged << " fitness=" << result.fitness
               << " inlier=" << result.inlier_ratio;
    ++cand_idx;
    if (result.success) {
      if (!has_success || matcher_preprocess::betterMatchResult(result, best_success, config_)) {
        best_success = result;
        has_success = true;
      }
    } else {
      if (!has_failure || matcher_preprocess::betterMatchResult(result, best_failure, config_)) {
        best_failure = result;
        has_failure = true;
      }
    }
  }

  const double candidates_total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                candidates_start)
          .count();
  // std::cout << "[VGICP] alignCandidates n=" << gated.size()
  //           << " total_ms=" << candidates_total_ms
  //           << " best_success=" << (has_success ? best_success.candidate_id : -1)
  //           << " best_failure=" << (has_failure ? best_failure.candidate_id : -1) << std::endl;
  SYS_LOG(syslog::LogSeverity::INFO, true) 
            << "VGICP] alignCandidates n=" << gated.size() << " total_ms=" << candidates_total_ms
               << " best_success=" << (has_success ? best_success.candidate_id : -1)
               << " best_failure=" << (has_failure ? best_failure.candidate_id : -1);


  if (has_success) {
    return best_success;
  }
  return has_failure ? best_failure : MatchResult();
}

std::string VgicpGlobalMatcher::summary() const {
  std::ostringstream ss;
  ss << "vgicp voxel=" << config_.vgicp_voxel_resolution
     << " coarse=" << config_.vgicp_coarse_scale << " fine=" << config_.vgicp_fine_scale
     << " target=" << config_.target_source
     << " gate=" << (config_.candidate_gate_enable ? config_.candidate_gate_radius : 0.0);
  return ss.str();
}

}  // namespace relocalization
