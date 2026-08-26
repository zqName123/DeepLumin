#pragma once

/**
 * @file matcher_impl.hpp
 * @brief 匹配器模板实现（仅被 matcher.cpp 包含）
 */

#include <algorithm>

#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

#include "global_matcher_v2/core/matcher.hpp"

namespace global_matcher_v2 {
namespace detail {

// ---- 因子参数注入（鲁棒核宽度）----
inline void configureFactor(small_gicp::GICPFactor::Setting&, double) {}
inline void configureFactor(small_gicp::ICPFactor::Setting&, double) {}

inline void configureFactor(typename small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>::Setting& s, double scale) { s.robust_kernel.c = scale; }
inline void configureFactor(typename small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>::Setting& s, double scale) { s.robust_kernel.c = scale; }
inline void configureFactor(typename small_gicp::RobustFactor<small_gicp::Huber, small_gicp::ICPFactor>::Setting& s, double scale) { s.robust_kernel.c = scale; }
inline void configureFactor(typename small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::ICPFactor>::Setting& s, double scale) { s.robust_kernel.c = scale; }

template <typename Factor>
void configureRegistration(small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP>& reg,
                           const MatcherParams& params, double max_dist) {
  reg.reduction.num_threads = std::max(1, params.num_threads);
  reg.rejector.max_dist_sq = max_dist * max_dist;
  reg.criteria.translation_eps = params.transformation_epsilon;
  reg.criteria.rotation_eps = params.transformation_epsilon;
  reg.optimizer.max_iterations = std::max(1, params.max_iterations);
  configureFactor(reg.point_factor, params.robust_kernel_scale);
}

/// 点云 target（icp / gicp）
template <typename Factor>
small_gicp::RegistrationResult alignWithTree(
    const small_gicp::PointCloud& target, const small_gicp::PointCloud& source,
    const small_gicp::KdTree<small_gicp::PointCloud>& tree, const Eigen::Isometry3d& init,
    const MatcherParams& params, double max_dist) {
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> reg;
  configureRegistration(reg, params, max_dist);
  return reg.align(target, source, tree, init);
}

/// 体素 target（vgicp）
template <typename Factor>
small_gicp::RegistrationResult alignWithVoxelMap(
    const small_gicp::GaussianVoxelMap& voxelmap, const small_gicp::PointCloud& source,
    const Eigen::Isometry3d& init, const MatcherParams& params, double max_dist) {
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> reg;
  configureRegistration(reg, params, max_dist);
  return reg.align(voxelmap, source, voxelmap, init);
}

}  // namespace detail
}  // namespace global_matcher_v2
