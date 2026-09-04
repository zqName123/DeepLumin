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
  std::cerr << "[slam_mapping_v2][loop_debug] " << msg << std::endl;
}

}  // namespace

void KdtreeGicpLoopDetector::configure(const LoopClosureConfig& cfg) {
  cfg_ = cfg;
  kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  submap_voxel_.setLeafSize(cfg_.submap_resolution, cfg_.submap_resolution, cfg_.submap_resolution);
  icp_.reset(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
  icp_->setMaxCorrespondenceDistance(20.0);
  icp_->setMaximumIterations(50);
  icp_->setTransformationEpsilon(1e-6);
  icp_->setEuclideanFitnessEpsilon(1e-6);
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
  std::cout << "[Loop] shouldCheck: " << current_idx << std::endl;
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

bool KdtreeGicpLoopDetector::alignCandidateSubmaps(const std::vector<Keyframe>& kfs,
                                                   int current_idx, int history_idx,
                                                   double* score, Mat3d* diff_rot,
                                                   Vec3d* diff_pos) {
  if (!score || !diff_rot || !diff_pos) return false;
  auto cur_submap = buildSubmap(kfs, current_idx, 0);
  auto hist_submap = buildSubmap(kfs, history_idx, cfg_.submap_search_num);
  latest_current_submap_ = cur_submap;
  latest_history_submap_ = hist_submap;
  if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) return false;

  // 法向量估计（ICP with Normals 需要）
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

  // 用两个关键帧的 pose_map 位姿差作为 ICP 初始猜测，
  // 解决回到起点时漂移大、单位阵初始值差导致 ICP 收敛失败的问题
  const Eigen::Matrix4f T_cur = poseToMatrix(kfs[current_idx].pose_map).cast<float>();
  const Eigen::Matrix4f T_hist = poseToMatrix(kfs[history_idx].pose_map).cast<float>();
  const Eigen::Matrix4f init_guess = T_hist.inverse() * T_cur;

  PointCloud aligned;
  icp_->setInputSource(cur_norm);
  icp_->setInputTarget(hist_norm);
  // icp_->align(aligned, init_guess);
  icp_->align(aligned, Eigen::Matrix4f::Identity());
  *score = icp_->getFitnessScore();

  // 统计内点率：变换后源点云中有多少点在目标点云 maxCorrespondenceDistance 内有对应
  float inlier_ratio = 0.0f;
  {
    PointCloud transformed;
    pcl::transformPointCloud(*cur_norm, transformed, icp_->getFinalTransformation());
    pcl::KdTreeFLANN<PointType> target_tree;
    target_tree.setInputCloud(hist_norm);
    const float max_dist_sq = 20.0f * 20.0f;  // 与 setMaxCorrespondenceDistance 一致
    int inlier_count = 0;
    std::vector<int> nn_ids(1);
    std::vector<float> nn_dists(1);
    for (const auto& pt : transformed.points) {
      if (target_tree.nearestKSearch(pt, 1, nn_ids, nn_dists) > 0 &&
          nn_dists[0] < max_dist_sq) {
        ++inlier_count;
      }
    }
    inlier_ratio = transformed.empty() ? 0.0f
        : static_cast<float>(inlier_count) / static_cast<float>(transformed.size());
  }
  debugLoopLog("ICP result: score=" + std::to_string(*score) +
               " converged=" + std::to_string(icp_->hasConverged()) +
               " inlier_ratio=" + std::to_string(inlier_ratio) +
               " src_pts=" + std::to_string(cur_norm->size()) +
               " tgt_pts=" + std::to_string(hist_norm->size()));

  if (!icp_->hasConverged() || !std::isfinite(*score) || *score > cfg_.icp_fitness_thresh) {
    debugLoopLog("ICP rejected: converged=" + std::to_string(icp_->hasConverged()) +
                 " score=" + std::to_string(*score) +
                 " thresh=" + std::to_string(cfg_.icp_fitness_thresh));
    return false;
  }
  const Eigen::Matrix4d T_hist_cur = icp_->getFinalTransformation().cast<double>();
  if (!T_hist_cur.array().isFinite().all()) return false;

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

LoopCandidate KdtreeGicpLoopDetector::detect(const std::vector<Keyframe>& kfs) {
  LoopCandidate out;
  if (kfs.empty()) return out;
  const int current_idx = static_cast<int>(kfs.size()) - 1;
  if (!shouldCheck(kfs, current_idx)) return out;
  last_loop_check_index_ = current_idx;

  const int history_idx = findBestHistoryCandidateIndex(kfs, current_idx);
  if (history_idx < 0) return out;

  double score = 0.0;
  Mat3d diff_rot = Mat3d::Identity();
  Vec3d diff_pos = Vec3d::Zero();
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
