#include "global_matcher_v2/core/matcher.hpp"

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <small_gicp/util/normal_estimation_omp.hpp>

#include "global_matcher_v2/core/matcher_impl.hpp"

namespace global_matcher_v2 {

namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

CloudPtr downsample(const CloudT& input, double leaf) {
  CloudPtr output(new CloudT());
  if (input.empty()) {
    return output;
  }
  if (leaf <= 1e-4) {
    *output = input;
    return output;
  }
  // 质心体素降采样（与 map_manager 一致）：PCL VoxelGrid 语义、哈希实现
  const double inv_leaf = 1.0 / leaf;
  // 质心体素降采样：与 PCL VoxelGrid 语义一致（体素内取质心），
  // 哈希实现，避免 PCL int32 体素数量限制
  struct VoxelAccum {
    std::size_t out_index;
    double sx, sy, sz;
    std::size_t count;
  };
  std::unordered_map<std::uint64_t, VoxelAccum> voxel_map;
  voxel_map.reserve(input.size() / 4);
  output->reserve(input.size() / 8);
  for (const auto& p : input.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x * inv_leaf));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y * inv_leaf));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z * inv_leaf));
    const std::uint64_t key = (static_cast<std::uint64_t>(ix) & 0x1FFFFF) |
                              ((static_cast<std::uint64_t>(iy) & 0x1FFFFF) << 21) |
                              ((static_cast<std::uint64_t>(iz) & 0x1FFFFF) << 42);
    auto it = voxel_map.find(key);
    if (it == voxel_map.end()) {
      const std::size_t idx = output->size();
      output->push_back(p);
      voxel_map.emplace(key, VoxelAccum{idx, p.x, p.y, p.z, 1u});
    } else {
      it->second.sx += p.x;
      it->second.sy += p.y;
      it->second.sz += p.z;
      ++it->second.count;
    }
  }
  for (const auto& kv : voxel_map) {
    const VoxelAccum& acc = kv.second;
    pcl::PointXYZ& pt = output->points[acc.out_index];
    pt.x = static_cast<float>(acc.sx / static_cast<double>(acc.count));
    pt.y = static_cast<float>(acc.sy / static_cast<double>(acc.count));
    pt.z = static_cast<float>(acc.sz / static_cast<double>(acc.count));
  }
  return output;
}

small_gicp::PointCloud::Ptr toSmallGicpWithCov(const CloudT& input, int num_neighbors,
                                               int num_threads) {
  auto pc = std::make_shared<small_gicp::PointCloud>();
  pc->resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    const auto& p = input.points[i];
    pc->point(i) << p.x, p.y, p.z, 1.0;
  }
  small_gicp::estimate_covariances_omp(*pc, std::max(3, num_neighbors),
                                       std::max(1, num_threads));
  return pc;
}

small_gicp::PointCloud::Ptr toSmallGicp(const CloudT& input) {
  auto pc = std::make_shared<small_gicp::PointCloud>();
  pc->resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    const auto& p = input.points[i];
    pc->point(i) << p.x, p.y, p.z, 1.0;
  }
  return pc;
}

struct StageResult {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  double fitness = std::numeric_limits<double>::infinity();
  bool converged = false;
};

// ---- small_gicp：icp / gicp / vgicp（含鲁棒核） ----

StageResult runGicpTree(const CloudT& source, const small_gicp::PointCloud& target,
                        const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                        const Eigen::Isometry3d& init, const MatcherParams& params,
                        double max_dist) {
  const auto source_sg = toSmallGicpWithCov(source, params.cov_num_neighbors,
                                            params.cov_num_threads);
  small_gicp::RegistrationResult result;
  if (params.robust_kernel == "huber") {
    result = detail::alignWithTree<
        small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>>(
        target, *source_sg, kdtree, init, params, max_dist);
  } else if (params.robust_kernel == "cauchy") {
    result = detail::alignWithTree<
        small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>>(
        target, *source_sg, kdtree, init, params, max_dist);
  } else {
    result = detail::alignWithTree<small_gicp::GICPFactor>(target, *source_sg, kdtree, init,
                                                           params, max_dist);
  }
  StageResult out;
  out.fitness = result.error;
  out.converged = result.converged;
  out.T = result.converged ? result.T_target_source.matrix().cast<float>() : init.matrix().cast<float>();
  return out;
}

StageResult runIcpTree(const CloudT& source, const small_gicp::PointCloud& target,
                       const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                       const Eigen::Isometry3d& init, const MatcherParams& params,
                       double max_dist) {
  const auto source_sg = toSmallGicp(source);
  small_gicp::RegistrationResult result;
  if (params.robust_kernel == "huber") {
    result = detail::alignWithTree<
        small_gicp::RobustFactor<small_gicp::Huber, small_gicp::ICPFactor>>(
        target, *source_sg, kdtree, init, params, max_dist);
  } else if (params.robust_kernel == "cauchy") {
    result = detail::alignWithTree<
        small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::ICPFactor>>(
        target, *source_sg, kdtree, init, params, max_dist);
  } else {
    result =
        detail::alignWithTree<small_gicp::ICPFactor>(target, *source_sg, kdtree, init, params,
                                                     max_dist);
  }
  StageResult out;
  out.fitness = result.error;
  out.converged = result.converged;
  out.T = result.converged ? result.T_target_source.matrix().cast<float>() : init.matrix().cast<float>();
  return out;
}

StageResult runVgicp(const CloudT& source, const small_gicp::GaussianVoxelMap& voxelmap,
                     const Eigen::Isometry3d& init, const MatcherParams& params,
                     double max_dist) {
  const auto source_sg = toSmallGicpWithCov(source, params.cov_num_neighbors,
                                            params.cov_num_threads);
  small_gicp::RegistrationResult result;
  if (params.robust_kernel == "huber") {
    result = detail::alignWithVoxelMap<
        small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>>(
        voxelmap, *source_sg, init, params, max_dist);
  } else if (params.robust_kernel == "cauchy") {
    result = detail::alignWithVoxelMap<
        small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>>(
        voxelmap, *source_sg, init, params, max_dist);
  } else {
    result = detail::alignWithVoxelMap<small_gicp::GICPFactor>(voxelmap, *source_sg, init,
                                                               params, max_dist);
  }
  StageResult out;
  out.fitness = result.error;
  out.converged = result.converged;
  out.T = result.converged ? result.T_target_source.matrix().cast<float>() : init.matrix().cast<float>();
  return out;
}

// ---- fast_gicp ----

StageResult runFastGicp(const CloudT& source, const CloudT& target, const Eigen::Isometry3d& init,
                        const MatcherParams& params, double max_dist) {
  fast_gicp::FastGICP<PointT, PointT> gicp;
  gicp.setNumThreads(std::max(1, params.fast_gicp_num_threads));
  gicp.setCorrespondenceRandomness(std::max(1, params.fast_gicp_correspondence_randomness));
  gicp.setInputSource(source.makeShared());
  gicp.setInputTarget(target.makeShared());
  gicp.setMaximumIterations(std::max(1, params.max_iterations));
  gicp.setTransformationEpsilon(params.transformation_epsilon);
  gicp.setMaxCorrespondenceDistance(max_dist);
  pcl::PointCloud<PointT> aligned;
  gicp.align(aligned, init.matrix().cast<float>());
  StageResult out;
  out.fitness = gicp.getFitnessScore();
  out.converged = gicp.hasConverged();
  out.T = out.converged ? gicp.getFinalTransformation() : init.matrix().cast<float>();
  return out;
}

StageResult runCoarse(const CloudT& source, const small_gicp::PointCloud& target,
                      const CloudT& target_pcl,
                      const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                      const small_gicp::GaussianVoxelMap* voxelmap,
                      const Eigen::Isometry3d& init, const MatcherParams& params,
                      double max_dist, double target_leaf) {
  const std::string& type = params.type;
  if (type == "fast_gicp" || type == "fast_gicp_vgicp") {
    return runFastGicp(source, *downsample(target_pcl, target_leaf), init, params, max_dist);
  }
  if (type == "gicp") {
    return runGicpTree(source, target, kdtree, init, params, max_dist);
  }
  if (type == "vgicp") {
    if (voxelmap && voxelmap->size() > 0) {
      return runVgicp(source, *voxelmap, init, params, max_dist);
    }
    return runGicpTree(source, target, kdtree, init, params, max_dist);
  }
  // icp
  return runIcpTree(source, target, kdtree, init, params, max_dist);
}

StageResult runFine(const CloudT& source, const small_gicp::PointCloud& target,
                    const CloudT& target_pcl,
                    const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                    const small_gicp::GaussianVoxelMap* voxelmap,
                    const Eigen::Isometry3d& init, const MatcherParams& params,
                    double max_dist, double target_leaf) {
  const std::string& type = params.type;
  if (type == "fast_gicp_vgicp" || type == "vgicp") {
    if (voxelmap && voxelmap->size() > 0) {
      return runVgicp(source, *voxelmap, init, params, max_dist);
    }
    return runGicpTree(source, target, kdtree, init, params, max_dist);
  }
  if (type == "fast_gicp") {
    return runFastGicp(source, *downsample(target_pcl, target_leaf), init, params, max_dist);
  }
  if (type == "gicp") {
    return runGicpTree(source, target, kdtree, init, params, max_dist);
  }
  return runIcpTree(source, target, kdtree, init, params, max_dist);
}

}  // namespace

MatchOutput PointCloudMatcher::match(const CloudT& source_pcl,
                                     const small_gicp::PointCloud& target,
                                     const CloudT& target_pcl,
                                     const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                                     const small_gicp::GaussianVoxelMap* coarse_voxelmap,
                                     const small_gicp::GaussianVoxelMap* fine_voxelmap,
                                     const Eigen::Matrix4f& init_T) const {
  MatchOutput out;
  const double coarse_dist = params_.coarse_max_correspondence_distance > 0.0
                                 ? params_.coarse_max_correspondence_distance
                                 : params_.max_correspondence_distance * params_.coarse_scale;
  const double fine_dist = params_.fine_max_correspondence_distance > 0.0
                               ? params_.fine_max_correspondence_distance
                               : params_.max_correspondence_distance * params_.fine_scale;

  Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
  init.matrix() = init_T.cast<double>();

  // ---- 粗配准 ----
  if (params_.coarse_scale > 1.0 + 1e-6) {
    const CloudPtr src_coarse =
        downsample(source_pcl, params_.source_voxel_leaf * params_.coarse_scale);
    const StageResult coarse = runCoarse(
        *src_coarse, target, target_pcl, kdtree, coarse_voxelmap, init, params_, coarse_dist,
        params_.target_voxel_leaf * params_.coarse_scale);
    out.coarse = coarse.T;
    out.coarse_fitness = coarse.fitness;
    out.coarse_converged = coarse.converged;
  } else {
    out.coarse = init_T;
    out.coarse_converged = true;
  }

  // ---- 精细配准 ----
  Eigen::Isometry3d fine_init = Eigen::Isometry3d::Identity();
  fine_init.matrix() = out.coarse.cast<double>();
  const CloudPtr src_fine = downsample(source_pcl, params_.source_voxel_leaf * params_.fine_scale);
  const StageResult fine = runFine(*src_fine, target, target_pcl, kdtree, fine_voxelmap,
                                   fine_init, params_, fine_dist,
                                   params_.target_voxel_leaf * params_.fine_scale);
  out.final = fine.T;
  out.fine_fitness = fine.fitness;
  out.fine_converged = fine.converged;
  return out;
}

}  // namespace global_matcher_v2
