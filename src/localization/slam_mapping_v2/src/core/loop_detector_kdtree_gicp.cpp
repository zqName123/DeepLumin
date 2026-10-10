#include "slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include "syslog/logger.h"

namespace slam_mapping_v2 {

namespace {

constexpr int kMinNormalEstimationPoints = 15;

bool isFiniteQuaternion(const Quatd& q) {
  return std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w()) && q.norm() > 1e-9;
}

bool isFinitePose(const Pose& p) {
  return std::isfinite(p.position.x()) && std::isfinite(p.position.y()) &&
         std::isfinite(p.position.z()) && isFiniteQuaternion(p.orientation);
}

bool isFinitePoint(const PointType& pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) &&
         std::isfinite(pt.intensity) && std::isfinite(pt.normal_x) &&
         std::isfinite(pt.normal_y) && std::isfinite(pt.normal_z) && std::isfinite(pt.curvature);
}

PointCloudPtr sanitizeCloud(const PointCloudPtr& cloud) {
  PointCloudPtr out(new PointCloud());
  if (!cloud) return out;
  out->reserve(cloud->size());
  for (const auto& pt : cloud->points) {
    if (isFinitePoint(pt)) out->push_back(pt);
  }
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

void debugLoopLog(const std::string& msg) {
  // std::cerr << "[slam_mapping_v2][loop_debug] " << msg << std::endl;
  SYS_LOG(syslog::LogSeverity::INFO, true) << "loop_debug: " << msg;
}

// 区域裁剪
PointCloudPtr cropCloudInLocalFrame(const PointCloudPtr& cloud, const Eigen::Matrix4d& T_map_local, const LoopCropConfig& c) {
  PointCloudPtr out(new PointCloud());
  if (!cloud) return out;
  out->reserve(cloud->size()); // 大小保持一致
  const Eigen::Matrix4d T_local_map = T_map_local.inverse();
  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    const Eigen::Vector4d p = T_local_map * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0); // 转到本地坐标系
    if (p.x() < -c.backward ||p.x() > c.forward) continue;
    if (p.y() < -c.right || p.y() > c.left) continue;
    if (p.z() < -c.down || p.z() > c.up) continue;
    out->push_back(pt);
  }
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;

}

// 内点统计
struct InlierStats {
  double ratio = 0.0; 
  double rmse = 0.0;
  int inliers = 0;
  int total = 0;
};

// 计算内点率
InlierStats computeInlierStats(const PointCloud& source_aligned, const PointCloudPtr& target, double inlier_distance) {
  InlierStats s;
  if (source_aligned.empty() || !target || target->empty()) return s;
  // 创建kdtree
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(target);
  const float dist_sq_th = static_cast<float>(inlier_distance * inlier_distance);
  std::vector<int> ids(1);
  std::vector<float> sq(1);
  double sum_sq = 0.0;
  for (const auto& pt : source_aligned.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (kdtree.nearestKSearch(pt, 1, ids, sq) <= 0) continue; // 没有对应点
    ++s.total;
    if (sq[0] <= dist_sq_th) {
      ++s.inliers;
      sum_sq += static_cast<double>(sq[0]); // 累计内点距离平方
    }
  }

  if (s.total > 0) {
    s.ratio = static_cast<double>(s.inliers) / static_cast<double>(s.total);
  }
  if (s.inliers > 0) {
    s.rmse = std::sqrt(sum_sq / static_cast<double>(s.inliers));
  }
  return s;

}

// 体素降采样
PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, double leaf) {
  if (!cloud || cloud->empty() || leaf <= 1e-4) return cloud;
  PointCloudPtr out(new PointCloud());
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return sanitizeCloud(out); // 过滤掉非有限点


 }

// small gicp 点云转换
small_gicp::PointCloud::Ptr toSmallGicpCloud(const PointCloudPtr& cloud) {
  std::vector<Eigen::Vector4f> pts;
  if (cloud) {
    pts.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      pts.push_back({pt.x, pt.y, pt.z, 1.0});
    }
    
  }
  return std::make_shared<small_gicp::PointCloud>(pts);

}

// small gicp 因子参数注入，鲁棒核宽度
inline void configureFactor(small_gicp::GICPFactor::Setting&, double){}
inline void configureFactor(
  small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>::Setting& s,
  double c) {
    s.robust_kernel.c = c;
  }

inline void configureFactor(
  small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>::Setting& s,
  double c) {
    s.robust_kernel.c = c;
  }
  
template <typename Factor>
void configureRegistration(small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP>& reg,
                           const LoopRegistrationConfig& rc, double max_dist) {
  reg.reduction.num_threads = std::max(1, rc.num_threads);
  reg.rejector.max_dist_sq = max_dist * max_dist;
  reg.criteria.translation_eps = rc.transformation_epsilon;
  reg.criteria.rotation_eps = rc.transformation_epsilon;
  reg.optimizer.max_iterations = std::max(1, rc.max_iterations);
  configureFactor(reg.point_factor, rc.robust_kernel_scale);
}


// vgicp 两级配准，粗层配准，细层配准，
template <typename Factor>
small_gicp::RegistrationResult alignTwoStageVgicp(const small_gicp::PointCloud& source,
                                                  const small_gicp::PointCloud& target,
                                                  const Eigen::Isometry3d& init,
                                                  const LoopRegistrationConfig& rc) {
  const double base = std::max(0.05, rc.voxel_resolution);
  const double coarse_scale = std::max(1.0, rc.coarse_scale);
  const double fine_scale = std::max(0.1, rc.fine_scale);

  // ---------- 第一层：粗配准（粗体素 + 放大对应距离） ----------
  auto coarse_map = std::make_shared<small_gicp::GaussianVoxelMap>(base * coarse_scale);
  coarse_map->set_search_offsets(27);
  coarse_map->insert(target);
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> coarse_reg;
  configureRegistration(coarse_reg, rc, rc.max_correspondence_distance * coarse_scale);
  const auto coarse_res = coarse_reg.align(*coarse_map, source, *coarse_map, init);

  // ---------- 第二层：精配准（细体素 + 标准对应距离） ----------
  auto fine_map = std::make_shared<small_gicp::GaussianVoxelMap>(base * fine_scale);
  fine_map->set_search_offsets(27);
  fine_map->insert(target);
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> fine_reg;
  configureRegistration(fine_reg, rc, rc.max_correspondence_distance);
  return fine_reg.align(*fine_map, source, *fine_map, coarse_res.T_target_source);

  // 只保留一层（精配准）时：注释掉【第一层】整段，并把上面 align 的第 4 个参数
  // coarse_res.T_target_source 改回 init 即可。
}

// gicp 
template <typename Factor>
small_gicp::RegistrationResult alignTwoStageGicpTree(
    const small_gicp::PointCloud& source, const small_gicp::PointCloud& target,
    const small_gicp::KdTree<small_gicp::PointCloud>& tree,
    const small_gicp::PointCloud& coarse_source, const small_gicp::PointCloud& coarse_target,
    const small_gicp::KdTree<small_gicp::PointCloud>& coarse_tree,
    const Eigen::Isometry3d& init, const LoopRegistrationConfig& rc) {
  // ---------- 第一层：粗配准（降采样点云 + 放大对应距离） ----------
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> coarse_reg;
  configureRegistration(coarse_reg, rc,
                        rc.max_correspondence_distance * std::max(1.0, rc.coarse_scale));
  const auto coarse_res = coarse_reg.align(coarse_target, coarse_source, coarse_tree, init);

  // ---------- 第二层：精配准（原始分辨率） ----------
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> fine_reg;
  configureRegistration(fine_reg, rc, rc.max_correspondence_distance);
  return fine_reg.align(target, source, tree, coarse_res.T_target_source);

  // 只保留一层（精配准）时：注释掉【第一层】整段，并把上面 align 的第 4 个参数
  // coarse_res.T_target_source 改回 init 即可。
}

// small_gicp入口
bool runSmallGicp(const PointCloudPtr& source_pcl, const PointCloudPtr& target_pcl,
                  const LoopRegistrationConfig& rc, Eigen::Matrix4d* T_out, bool* converged_out) {

  if (!T_out || !converged_out) return false;
  auto source = toSmallGicpCloud(source_pcl);
  auto target = toSmallGicpCloud(target_pcl);
  if (!source || !target || source->size() < 10 || target->size() < 10) return false;

  const int k = std::max(3, rc.cov_num_neighbors);
  const int nthreads = std::max(1, rc.num_threads);
  small_gicp::estimate_covariances_omp(*target, k, nthreads);
  small_gicp::estimate_covariances_omp(*source, k, nthreads);

  const Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
  small_gicp::RegistrationResult res;

  if (rc.type != "gicp") {
    // ---------- VGICP 两级 ----------
    if (rc.robust_kernel == "huber") {
      res = alignTwoStageVgicp<
          small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>>(
          *source, *target, init, rc);
    } else if (rc.robust_kernel == "cauchy") {
      res = alignTwoStageVgicp<
          small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>>(
          *source, *target, init, rc);
    } else {
      res = alignTwoStageVgicp<small_gicp::GICPFactor>(*source, *target, init, rc);
    }
  } else {
    // ---------- GICP 两级（KdTree）----------
    // 细层：原始分辨率目标树
    small_gicp::KdTree<small_gicp::PointCloud> fine_tree(target,
                                                        small_gicp::KdTreeBuilderOMP(nthreads));

    // 粗层：体素降采样点云（leaf = voxel_resolution * coarse_scale）+ 其目标树
    const double coarse_leaf =
        std::max(0.05, rc.voxel_resolution * std::max(1.0, rc.coarse_scale));
    auto source_coarse = toSmallGicpCloud(voxelDownsample(source_pcl, coarse_leaf));
    auto target_coarse = toSmallGicpCloud(voxelDownsample(target_pcl, coarse_leaf));
    if (!source_coarse || !target_coarse || source_coarse->size() < 10 ||
        target_coarse->size() < 10) {
      return false;
    }
    small_gicp::estimate_covariances_omp(*target_coarse, k, nthreads);
    small_gicp::estimate_covariances_omp(*source_coarse, k, nthreads);
    small_gicp::KdTree<small_gicp::PointCloud> coarse_tree(
        target_coarse, small_gicp::KdTreeBuilderOMP(nthreads));

    if (rc.robust_kernel == "huber") {
      res = alignTwoStageGicpTree<small_gicp::RobustFactor<small_gicp::Huber,
                                                           small_gicp::GICPFactor>>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    } else if (rc.robust_kernel == "cauchy") {
      res = alignTwoStageGicpTree<small_gicp::RobustFactor<small_gicp::Cauchy,
                                                           small_gicp::GICPFactor>>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    } else {
      res = alignTwoStageGicpTree<small_gicp::GICPFactor>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    }
  }

  *T_out = res.T_target_source.matrix();
  *converged_out = res.converged;
  return T_out->array().isFinite().all();

}



}  // namespace

void KdtreeGicpLoopDetector::configure(const LoopClosureConfig& cfg) {
  cfg_ = cfg;
  kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  submap_voxel_.setLeafSize(cfg_.submap_resolution, cfg_.submap_resolution, cfg_.submap_resolution);
  icp_.reset(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
  icp_->setMaxCorrespondenceDistance(cfg_.registration.max_correspondence_distance);
  icp_->setMaximumIterations(cfg_.registration.max_iterations);
  icp_->setTransformationEpsilon(cfg_.registration.transformation_epsilon);
  icp_->setEuclideanFitnessEpsilon(cfg_.registration.transformation_epsilon);
  icp_->setRANSACIterations(0);
  reset();
}

void KdtreeGicpLoopDetector::reset() {
  last_loop_check_index_ = -1;
  loop_history_.clear();
  latest_current_submap_.reset();
  latest_history_submap_.reset();
}

bool KdtreeGicpLoopDetector::shouldCheck(const std::vector<Keyframe>& kfs, int current_idx) const {
  if (current_idx < 0 || current_idx >= static_cast<int>(kfs.size())) return false;
  const int min_required_gap = std::max(cfg_.min_index_gap, cfg_.closest_id_th);
  if (current_idx < min_required_gap) return false;
  if (cfg_.loop_kf_gap > 0 && last_loop_check_index_ >= 0 &&
      (current_idx - last_loop_check_index_) < cfg_.loop_kf_gap) {
    return false;
  }
  // std::cout << "[Loop] shouldCheck: " << current_idx << std::endl;
  SYS_LOG(syslog::LogSeverity::INFO, true) << "shouldCheck: " << current_idx;
  return true;
}

bool KdtreeGicpLoopDetector::isCandidateNearExistingLoop(int current_idx,
                                                         int history_idx) const {
  if (cfg_.min_id_interval <= 0) return false;
  for (const auto& lp : loop_history_) {
    if (std::abs(lp.first - history_idx) < cfg_.min_id_interval &&
        std::abs(lp.second - current_idx) < std::max(1, cfg_.loop_kf_gap)) {
      return true;
    }
  }
  return false;
}

PointCloudPtr KdtreeGicpLoopDetector::buildSubmap(const std::vector<Keyframe>& kfs,
                                                  int center_index, int search_num) const {
  PointCloudPtr cloud(new PointCloud());
  if (kfs.empty()) return cloud;
  const int min_index = std::max(0, center_index - search_num);
  const int max_index =
      std::min(static_cast<int>(kfs.size()) - 1, center_index + search_num);
  for (int i = min_index; i <= max_index; ++i) {
    const auto& kf = kfs[i];
    if (!kf.cloud || kf.cloud->empty() || !isFinitePose(kf.pose_map)) continue;
    // 关键帧点云为 odom 系：子图用 T_map_odom = T(pose_map) * T(pose_odom)^-1
    const Eigen::Matrix4f T =
        (poseToMatrix(kf.pose_map) * poseToMatrix(kf.pose_odom).inverse()).cast<float>();
    PointCloud temp;
    pcl::transformPointCloud(*kf.cloud, temp, T);
    *cloud += temp;
  }
  if (cloud->empty()) return cloud;
  cloud = sanitizeCloud(cloud);
  if (cloud->empty()) return cloud;
  PointCloudPtr filtered(new PointCloud());
  pcl::VoxelGrid<PointType> voxel = submap_voxel_;
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);
  return sanitizeCloud(filtered);
}

int KdtreeGicpLoopDetector::findBestHistoryCandidateIndex(const std::vector<Keyframe>& kfs,
                                                          int current_idx) {
  cloud_history_poses_->clear();
  for (const auto& kf : kfs) {
    pcl::PointXYZ p;
    if (!isFinitePose(kf.pose_map)) {
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
    } else {
      p.x = kf.pose_map.position.x();
      p.y = kf.pose_map.position.y();
      p.z = kf.pose_map.position.z();
    }
    cloud_history_poses_->push_back(p);
  }
  if (cloud_history_poses_->empty() || !pcl::isFinite(cloud_history_poses_->back())) {
    return -1;
  }
  const double search_radius = cfg_.max_range > 0.0 ? cfg_.max_range : 15.0;
  kdtree_history_poses_->setInputCloud(cloud_history_poses_);
  std::vector<int> ids;
  std::vector<float> sqdists;
  kdtree_history_poses_->radiusSearch(cloud_history_poses_->back(), search_radius, ids, sqdists, 0);

  // ---- 诊断日志 ----
  {
    const auto& cur = cloud_history_poses_->back();
    debugLoopLog("radiusSearch: current=(" + std::to_string(cur.x) + "," +
                 std::to_string(cur.y) + "," + std::to_string(cur.z) +
                 ") radius=" + std::to_string(search_radius) +
                 " candidates=" + std::to_string(ids.size()));
    // 找第一个不是当前帧的历史候选
    for (std::size_t j = 0; j < ids.size(); ++j) {
      if (ids[j] >= 0 && ids[j] < current_idx) {
        debugLoopLog("  nearest history: id=" + std::to_string(ids[j]) +
                     " sqdist=" + std::to_string(sqdists[j]) +
                     " required_gap=" + std::to_string(std::max(cfg_.min_index_gap, cfg_.closest_id_th)) +
                     " time_diff=" + std::to_string(std::abs(kfs[ids[j]].timestamp - kfs.back().timestamp)));
        break;
      }
    }
  }
  // ---- 诊断日志结束 ----

  int best_history_idx = -1;
  float best_sqdist = std::numeric_limits<float>::max();
  const int required_index_gap = std::max(cfg_.min_index_gap, cfg_.closest_id_th);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const int id = ids[i];
    if (id < 0 || id >= current_idx) continue;
    if (!pcl::isFinite(cloud_history_poses_->at(id))) continue;
    if ((current_idx - id) < required_index_gap) continue;
    if (cfg_.time_thresh_sec > 0.0 &&
        std::abs(kfs[id].timestamp - kfs.back().timestamp) <= cfg_.time_thresh_sec) {
      continue;
    }
    if (std::find(loop_history_.begin(), loop_history_.end(),
                  std::make_pair(id, current_idx)) != loop_history_.end()) {
      continue;
    }
    if (isCandidateNearExistingLoop(current_idx, id)) continue;
    const float sqdist =
        i < sqdists.size() ? sqdists[i] : std::numeric_limits<float>::max();
    if (sqdist < best_sqdist) {
      best_sqdist = sqdist;
      best_history_idx = id;
    }
  }
  return best_history_idx;
}

// 采用small_gicp进行配准，采用内点率和内丹rmse进行判断
bool KdtreeGicpLoopDetector::alignCandidateSubmaps(const std::vector<Keyframe>& kfs,
                                                   int current_idx, int history_idx,
                                                   double* score, Mat3d* diff_rot,
                                                   Vec3d* diff_pos) {
  if (!score || !diff_rot || !diff_pos) return false;
  auto cur_submap = buildSubmap(kfs, current_idx, 5);
  auto hist_submap = buildSubmap(kfs, history_idx, cfg_.submap_search_num);

  // ---- 区域裁剪：配准与打分前只保留候选历史关键帧附近的盒式区域 ----
  // 两侧都裁到同一个框 → 只比较双方覆盖的交集，去掉远端无对应点
  if (cfg_.crop.enable) {
    const Eigen::Matrix4d T_hist = poseToMatrix(kfs[history_idx].pose_map);
    cur_submap = cropCloudInLocalFrame(cur_submap, T_hist, cfg_.crop);
    hist_submap = cropCloudInLocalFrame(hist_submap, T_hist, cfg_.crop);
  }

  latest_current_submap_ = cur_submap;
  latest_history_submap_ = hist_submap;
  if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) return false;

  const auto& rc = cfg_.registration;
  Eigen::Matrix4d T_hist_cur = Eigen::Matrix4d::Identity();
  bool converged = false;

  if (rc.type == "pcl_icp") {
    // ---------- 对照路径：原 PCL 点到面 ICP（需法向量） ----------
    auto addNormals = [](const PointCloudPtr& cloud) -> PointCloudPtr {
      auto filtered = sanitizeCloud(cloud);
      if (!filtered || filtered->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
        return PointCloudPtr(new PointCloud());
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud(*filtered, *xyz);
      std::vector<int> idx;
      pcl::removeNaNFromPointCloud(*xyz, *xyz, idx);
      if (xyz->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
        return PointCloudPtr(new PointCloud());
      }
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
      tree->setInputCloud(xyz);
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
      ne.setInputCloud(xyz);
      ne.setSearchMethod(tree);
      ne.setKSearch(std::min<int>(kMinNormalEstimationPoints, static_cast<int>(xyz->size())));
      ne.compute(*normals);
      PointCloudPtr out(new PointCloud());
      pcl::concatenateFields(*xyz, *normals, *out);
      return sanitizeCloud(out);
    };

    auto cur_norm = sanitizeCloud(addNormals(cur_submap));
    auto hist_norm = sanitizeCloud(addNormals(hist_submap));
    if (!cur_norm || !hist_norm ||
        cur_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
        hist_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
      return false;
    }
    PointCloud aligned;
    icp_->setInputSource(cur_norm);
    icp_->setInputTarget(hist_norm);
    icp_->align(aligned, Eigen::Matrix4f::Identity());
    converged = icp_->hasConverged();
    T_hist_cur = icp_->getFinalTransformation().cast<double>();
  } else {
    // ---------- small_gicp：VGICP 两级 / GICP 两级（可带鲁棒核） ----------
    // 两侧子图都在 map 系，漂移为零时 T=I，故用单位阵作初值；
    // 粗配准层负责扩大收敛域以吸收漂移。
    if (!runSmallGicp(cur_submap, hist_submap, rc, &T_hist_cur, &converged)) {
      return false;
    }
  }

  if (!converged || !T_hist_cur.array().isFinite().all()) {
    debugLoopLog("align rejected: converged=" + std::to_string(converged));
    return false;
  }

  // ---- 内点质量门控：内点率 + 内点 RMSE ----
  PointCloud source_aligned;
  pcl::transformPointCloud(*cur_submap, source_aligned, T_hist_cur.cast<float>());
  const auto& gate = cfg_.inlier_gate;
  const InlierStats st = computeInlierStats(source_aligned, hist_submap, gate.inlier_distance);

  debugLoopLog("align result: converged=" + std::to_string(converged) +
               " inlier_ratio=" + std::to_string(st.ratio) +
               " inlier_rmse=" + std::to_string(st.rmse) +
               " inliers=" + std::to_string(st.inliers) + "/" + std::to_string(st.total) +
               " src_pts=" + std::to_string(cur_submap->size()) +
               " tgt_pts=" + std::to_string(hist_submap->size()));

  if (st.total <= 0 || st.ratio < gate.min_inlier_ratio || st.rmse > gate.max_inlier_rmse) {
    debugLoopLog("loop rejected by inlier gate: ratio=" + std::to_string(st.ratio) +
                 " (min " + std::to_string(gate.min_inlier_ratio) +
                 ") rmse=" + std::to_string(st.rmse) +
                 " (max " + std::to_string(gate.max_inlier_rmse) + ")");
    return false;
  }

  // score 语义"越小越好"：用内点 RMSE（米）
  *score = std::max(st.rmse, 1e-6);

  const auto& pre = kfs[history_idx];
  const auto& cur = kfs[current_idx];
  const Mat3d r_pre = pre.pose_map.orientation.normalized().toRotationMatrix();
  const Mat3d r_cur = cur.pose_map.orientation.normalized().toRotationMatrix();
  *diff_rot = r_pre.transpose() * T_hist_cur.block<3, 3>(0, 0) * r_cur;
  *diff_pos =
      r_pre.transpose() *
      (T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.position + T_hist_cur.block<3, 1>(0, 3) -
       pre.pose_map.position);
  return diff_rot->array().isFinite().all() && diff_pos->array().isFinite().all();
}





// bool KdtreeGicpLoopDetector::alignCandidateSubmaps(const std::vector<Keyframe>& kfs,
//                                                    int current_idx, int history_idx,
//                                                    double* score, Mat3d* diff_rot,
//                                                    Vec3d* diff_pos) {
//   if (!score || !diff_rot || !diff_pos) return false;
//   auto cur_submap = buildSubmap(kfs, current_idx, 0);  //0： 当前帧只使用当前帧
//   auto hist_submap = buildSubmap(kfs, history_idx, cfg_.submap_search_num); // 构建历史关键帧的子地图，搜索范围为 cfg_.submap_search_num 个关键帧，每个关键帧使用 1 个点云
//   latest_current_submap_ = cur_submap;
//   latest_history_submap_ = hist_submap;
//   if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) return false;

//   // 法向量估计（ICP with Normals 需要）
//   auto addNormals = [](const PointCloudPtr& cloud) -> PointCloudPtr { // 对点云进行法向量估计
//     auto filtered = sanitizeCloud(cloud); // 对点云进行去噪和滤波
//     if (!filtered || filtered->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
//       return PointCloudPtr(new PointCloud());
//     }
//     pcl::PointCloud<pcl::PointXYZI>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::copyPointCloud(*filtered, *xyz);
//     std::vector<int> idx;
//     pcl::removeNaNFromPointCloud(*xyz, *xyz, idx); // 移除包含 NaN 的点
//     if (xyz->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
//       return PointCloudPtr(new PointCloud()); // 如果移除 NaN 后点云为空，返回空点云
//     }
//     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
//     tree->setInputCloud(xyz);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
//     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
//     ne.setInputCloud(xyz);
//     ne.setSearchMethod(tree);
//     ne.setKSearch(std::min<int>(kMinNormalEstimationPoints, static_cast<int>(xyz->size())));
//     ne.compute(*normals);
//     PointCloudPtr out(new PointCloud());
//     pcl::concatenateFields(*xyz, *normals, *out);
//     return sanitizeCloud(out);
//   };

//   auto cur_norm = sanitizeCloud(addNormals(cur_submap));
//   auto hist_norm = sanitizeCloud(addNormals(hist_submap));
//   if (!cur_norm || !hist_norm ||
//       cur_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
//       hist_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
//     return false;
//   }

//   // 用两个关键帧的 pose_map 位姿差作为 ICP 初始猜测，
//   // 解决回到起点时漂移大、单位阵初始值差导致 ICP 收敛失败的问题
//   const Eigen::Matrix4f T_cur = poseToMatrix(kfs[current_idx].pose_map).cast<float>();
//   const Eigen::Matrix4f T_hist = poseToMatrix(kfs[history_idx].pose_map).cast<float>();
//   const Eigen::Matrix4f init_guess = T_hist.inverse() * T_cur;

//   PointCloud aligned;
//   icp_->setInputSource(cur_norm);
//   icp_->setInputTarget(hist_norm);
//   // icp_->align(aligned, init_guess);
//   icp_->align(aligned, Eigen::Matrix4f::Identity());
//   *score = icp_->getFitnessScore();

//   // 统计内点率：变换后源点云中有多少点在目标点云 maxCorrespondenceDistance 内有对应
//   float inlier_ratio = 0.0f;
//   {
//     PointCloud transformed;
//     pcl::transformPointCloud(*cur_norm, transformed, icp_->getFinalTransformation());
//     pcl::KdTreeFLANN<PointType> target_tree;
//     target_tree.setInputCloud(hist_norm);
//     const float max_dist_sq = 20.0f * 20.0f;  // 与 setMaxCorrespondenceDistance 一致
//     int inlier_count = 0;
//     std::vector<int> nn_ids(1);
//     std::vector<float> nn_dists(1);
//     for (const auto& pt : transformed.points) {
//       if (target_tree.nearestKSearch(pt, 1, nn_ids, nn_dists) > 0 &&
//           nn_dists[0] < max_dist_sq) {
//         ++inlier_count;
//       }
//     }
//     inlier_ratio = transformed.empty() ? 0.0f
//         : static_cast<float>(inlier_count) / static_cast<float>(transformed.size());
//   }
//   debugLoopLog("ICP result: score=" + std::to_string(*score) +
//                " converged=" + std::to_string(icp_->hasConverged()) +
//                " inlier_ratio=" + std::to_string(inlier_ratio) +
//                " src_pts=" + std::to_string(cur_norm->size()) +
//                " tgt_pts=" + std::to_string(hist_norm->size()));

//   if (!icp_->hasConverged() || !std::isfinite(*score) || *score > cfg_.icp_fitness_thresh) {
//     debugLoopLog("ICP rejected: converged=" + std::to_string(icp_->hasConverged()) +
//                  " score=" + std::to_string(*score) +
//                  " thresh=" + std::to_string(cfg_.icp_fitness_thresh));
//     return false;
//   }
//   const Eigen::Matrix4d T_hist_cur = icp_->getFinalTransformation().cast<double>();
//   if (!T_hist_cur.array().isFinite().all()) return false;

//   const auto& pre = kfs[history_idx];
//   const auto& cur = kfs[current_idx];
//   const Mat3d r_pre = pre.pose_map.orientation.normalized().toRotationMatrix();
//   const Mat3d r_cur = cur.pose_map.orientation.normalized().toRotationMatrix();
//   *diff_rot = r_pre.transpose() * T_hist_cur.block<3, 3>(0, 0) * r_cur;
//   *diff_pos =
//       r_pre.transpose() *
//       (T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.position + T_hist_cur.block<3, 1>(0, 3) -
//        pre.pose_map.position);
//   return diff_rot->array().isFinite().all() && diff_pos->array().isFinite().all();
// }

// 回环检测入口
LoopCandidate KdtreeGicpLoopDetector::detect(const std::vector<Keyframe>& kfs) {
  LoopCandidate out;
  if (kfs.empty()) return out;
  const int current_idx = static_cast<int>(kfs.size()) - 1; // 获取当前关键帧的索引，从0开始计数
  if (!shouldCheck(kfs, current_idx)) return out; // 如果当前关键帧与上一次检查的关键帧距离过远，不进行回环检测
  last_loop_check_index_ = current_idx; // 更新上一次检查的关键帧索引

  const int history_idx = findBestHistoryCandidateIndex(kfs, current_idx); // 按照当前关键帧与历史关键帧的距离排序，返回距离最近的索引
  if (history_idx < 0) return out;

  double score = 0.0;
  Mat3d diff_rot = Mat3d::Identity();
  Vec3d diff_pos = Vec3d::Zero();
  // 对齐当前关键帧和历史关键帧的子地图
  if (!alignCandidateSubmaps(kfs, current_idx, history_idx, &score, &diff_rot, &diff_pos)) {
    return out;
  }
  out.valid = true;
  out.history_idx = history_idx;
  out.current_idx = current_idx;
  out.score = score;
  out.diff_rot = diff_rot;
  out.diff_pos = diff_pos;
  loop_history_.emplace_back(history_idx, current_idx);
  debugLoopLog("loop detected " + std::to_string(history_idx) + "->" +
               std::to_string(current_idx) + " score=" + std::to_string(score));
  return out;
}

}  // namespace slam_mapping_v2
