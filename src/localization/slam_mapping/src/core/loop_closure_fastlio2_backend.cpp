#include "slam_mapping/core/loop_closure_fastlio2_backend.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

namespace slam_mapping {
namespace {

constexpr int kMinNormalEstimationPoints = 15;

bool isFiniteQuaternion(const localization::Quatd& q) {
  return std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w()) && q.norm() > 1e-9;
}

bool isFinitePose(const localization::Pose& pose) {
  return std::isfinite(pose.position.x()) && std::isfinite(pose.position.y()) &&
         std::isfinite(pose.position.z()) && isFiniteQuaternion(pose.orientation);
}

bool isFinitePoint(const localization::PointType& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(point.intensity) && std::isfinite(point.normal_x) &&
         std::isfinite(point.normal_y) && std::isfinite(point.normal_z) &&
         std::isfinite(point.curvature);
}

bool hasOnlyFinitePoints(const localization::PointCloudPtr& cloud) {
  if (!cloud) return false;
  for (const auto& point : cloud->points) {
    if (!isFinitePoint(point)) {
      return false;
    }
  }
  return true;
}

std::size_t countInvalidPoints(const localization::PointCloudPtr& cloud) {
  if (!cloud) return 0;
  std::size_t invalid = 0;
  for (const auto& point : cloud->points) {
    if (!isFinitePoint(point)) {
      ++invalid;
    }
  }
  return invalid;
}

std::string summarizeCloud(const char* name, const localization::PointCloudPtr& cloud) {
  std::ostringstream oss;
  oss << name << " ptr=" << (cloud ? "set" : "null");
  if (cloud) {
    oss << " size=" << cloud->size()
        << " invalid=" << countInvalidPoints(cloud)
        << " dense=" << cloud->is_dense;
  }
  return oss.str();
}

void debugLoopLog(const std::string& msg) {
  std::cerr << "[slam_mapping][loop_debug] " << msg << std::endl;
}

localization::PointCloudPtr sanitizeCloud(const localization::PointCloudPtr& cloud) {
  localization::PointCloudPtr out(new localization::PointCloud());
  if (!cloud) return out;
  out->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (isFinitePoint(point)) {
      out->push_back(point);
    }
  }
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

localization::PointCloudPtr sanitizeIcpInputCloud(const localization::PointCloudPtr& cloud) {
  auto out = sanitizeCloud(cloud);
  if (!out || out->empty()) return out;
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  if (!hasOnlyFinitePoints(out)) {
    return localization::PointCloudPtr(new localization::PointCloud());
  }
  return out;
}

localization::Pose poseFromKeyframeOdom(const MappingKeyframe& kf) {
  return kf.pose_odom;
}

Eigen::Matrix4d poseToMatrix(const localization::Pose& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = pose.position;
  return T;
}

localization::Pose matrixToPose(const Eigen::Matrix4d& T, double timestamp) {
  localization::Pose pose;
  pose.timestamp = timestamp;
  pose.position = T.block<3, 1>(0, 3);
  pose.orientation = localization::Quatd(T.block<3, 3>(0, 0)).normalized();
  return pose;
}

}  // namespace

void Fastlio2SamLcLoopClosureBackend::configure(const LoopClosureConfig& cfg) {
  config_ = cfg;
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam2_ = std::make_shared<gtsam::ISAM2>(params);
  kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  submap_voxel_.setLeafSize(config_.submap_resolution, config_.submap_resolution, config_.submap_resolution);
  icp_.reset(new pcl::IterativeClosestPointWithNormals<localization::PointType, localization::PointType>());
  icp_->setMaxCorrespondenceDistance(20.0);
  icp_->setMaximumIterations(50);
  icp_->setTransformationEpsilon(1e-6);
  icp_->setEuclideanFitnessEpsilon(1e-6);
  icp_->setRANSACIterations(0);
  previous_index_ = 0;
  loop_history_.clear();
  graph_.resize(0);
  initialized_estimate_.clear();
}

void Fastlio2SamLcLoopClosureBackend::reset() {
  previous_index_ = 0;
  loop_history_.clear();
  graph_.resize(0);
  initialized_estimate_.clear();
  if (isam2_) {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
  }
}

localization::PointCloudPtr Fastlio2SamLcLoopClosureBackend::addNormals(const localization::PointCloudPtr& cloud) const {
  debugLoopLog(std::string("addNormals input ") + summarizeCloud("cloud", cloud));
  auto filtered = sanitizeCloud(cloud);
  debugLoopLog(std::string("addNormals sanitized ") + summarizeCloud("filtered", filtered));
  if (!filtered || filtered->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
    return localization::PointCloudPtr(new localization::PointCloud());
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*filtered, *xyz);
  std::vector<int> xyz_indices;
  pcl::removeNaNFromPointCloud(*xyz, *xyz, xyz_indices);
  if (xyz->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
    debugLoopLog("addNormals xyz too small after removeNaN size=" + std::to_string(xyz->size()));
    return localization::PointCloudPtr(new localization::PointCloud());
  }
  debugLoopLog("addNormals before xyz tree setInputCloud size=" + std::to_string(xyz->size()));
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  tree->setInputCloud(xyz);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(xyz);
  ne.setSearchMethod(tree);
  ne.setKSearch(std::min<int>(kMinNormalEstimationPoints, static_cast<int>(xyz->size())));
  ne.compute(*normals);
  localization::PointCloudPtr out(new localization::PointCloud());
  pcl::concatenateFields(*xyz, *normals, *out);
  auto icp_ready = sanitizeIcpInputCloud(out);
  debugLoopLog(std::string("addNormals output ") + summarizeCloud("icp_ready", icp_ready));
  return icp_ready;
}

localization::PointCloudPtr Fastlio2SamLcLoopClosureBackend::buildSubmap(const std::vector<MappingKeyframe>& keyframes,
                                                                         int center_index,
                                                                         int search_num) const {
  localization::PointCloudPtr cloud(new localization::PointCloud());
  if (keyframes.empty()) return cloud;
  const int min_index = std::max(0, center_index - search_num);
  const int max_index = std::min(static_cast<int>(keyframes.size()) - 1, center_index + search_num);
  for (int i = min_index; i <= max_index; ++i) {
    const auto& kf = keyframes[i];
    if (!kf.cloud_body || kf.cloud_body->empty() || !isFinitePose(kf.pose_map)) continue;
    const Eigen::Matrix4f T = poseToMatrix(kf.pose_map).cast<float>();
    localization::PointCloud temp;
    pcl::transformPointCloud(*kf.cloud_body, temp, T);
    *cloud += temp;
  }
  if (!cloud->empty()) {
    debugLoopLog(std::string("buildSubmap raw ") + summarizeCloud("cloud", cloud));
    cloud = sanitizeCloud(cloud);
    if (cloud->empty()) return cloud;
    localization::PointCloudPtr filtered(new localization::PointCloud());
    pcl::VoxelGrid<localization::PointType> voxel = submap_voxel_;
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    auto sanitized = sanitizeCloud(filtered);
    debugLoopLog(std::string("buildSubmap filtered ") + summarizeCloud("sanitized", sanitized));
    return sanitized;
  }
  return cloud;
}

bool Fastlio2SamLcLoopClosureBackend::detectCandidate(const std::vector<MappingKeyframe>& keyframes,
                                                      int* current_idx,
                                                      int* history_idx,
                                                      double* score,
                                                      Eigen::Matrix3d* diff_rot,
                                                      Eigen::Vector3d* diff_pos) {
  if (!current_idx || !history_idx || !score || !diff_rot || !diff_pos) return false;
  if (keyframes.size() < static_cast<std::size_t>(config_.min_index_gap + 1)) return false;

  cloud_history_poses_->clear();
  for (const auto& kf : keyframes) {
    pcl::PointXYZ p;
    if (!isFinitePose(kf.pose_map)) {
      p.x = std::numeric_limits<float>::quiet_NaN();
      p.y = std::numeric_limits<float>::quiet_NaN();
      p.z = std::numeric_limits<float>::quiet_NaN();
    } else {
      p.x = kf.pose_map.position.x();
      p.y = kf.pose_map.position.y();
      p.z = kf.pose_map.position.z();
    }
    cloud_history_poses_->push_back(p);
  }

  *current_idx = static_cast<int>(keyframes.size()) - 1;
  *history_idx = -1;
  if (!pcl::isFinite(cloud_history_poses_->back())) {
    debugLoopLog("detectCandidate latest history pose invalid before radiusSearch");
    return false;
  }
  debugLoopLog("detectCandidate before history radiusSearch poses=" + std::to_string(cloud_history_poses_->size()) +
               " current_idx=" + std::to_string(*current_idx) +
               " search_radius=" + std::to_string(config_.search_radius));
  kdtree_history_poses_->setInputCloud(cloud_history_poses_);

  std::vector<int> ids;
  std::vector<float> sqdists;
  kdtree_history_poses_->radiusSearch(cloud_history_poses_->back(), config_.search_radius, ids, sqdists, 0);
  debugLoopLog("detectCandidate radiusSearch result candidates=" + std::to_string(ids.size()));
  for (const int id : ids) {
    if (id < 0 || id >= *current_idx) continue;
    if (!pcl::isFinite(cloud_history_poses_->at(id))) continue;
    if (std::abs(keyframes[id].timestamp - keyframes.back().timestamp) > config_.time_thresh_sec &&
        (*current_idx - id) >= config_.min_index_gap) {
      *history_idx = id;
      break;
    }
  }
  if (*history_idx < 0) return false;
  if (std::find(loop_history_.begin(), loop_history_.end(), std::make_pair(*history_idx, *current_idx)) !=
      loop_history_.end()) {
    return false;
  }

  auto cur_submap = buildSubmap(keyframes, *current_idx, 0);
  auto hist_submap = buildSubmap(keyframes, *history_idx, config_.submap_search_num);
  if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) return false;

  auto cur_norm = sanitizeIcpInputCloud(addNormals(cur_submap));
  auto hist_norm = sanitizeIcpInputCloud(addNormals(hist_submap));
  debugLoopLog(std::string("detectCandidate ICP source ") + summarizeCloud("cur_norm", cur_norm));
  debugLoopLog(std::string("detectCandidate ICP target ") + summarizeCloud("hist_norm", hist_norm));
  if (!cur_norm || !hist_norm ||
      cur_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
      hist_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
      !hasOnlyFinitePoints(cur_norm) || !hasOnlyFinitePoints(hist_norm)) {
    return false;
  }
  localization::PointCloud aligned;
  debugLoopLog("detectCandidate before ICP setInputSource/setInputTarget");
  icp_->setInputSource(cur_norm);
  icp_->setInputTarget(hist_norm);
  debugLoopLog("detectCandidate before ICP align");
  icp_->align(aligned, Eigen::Matrix4f::Identity());
  debugLoopLog(std::string("detectCandidate after ICP align ") + summarizeCloud("aligned", localization::PointCloudPtr(new localization::PointCloud(aligned))));
  *score = icp_->getFitnessScore();
  if (!icp_->hasConverged() || !std::isfinite(*score) || *score > config_.icp_fitness_thresh) return false;

  const Eigen::Matrix4d T_hist_cur = icp_->getFinalTransformation().cast<double>();
  if (!T_hist_cur.array().isFinite().all()) return false;
  const auto& pre = keyframes[*history_idx];
  const auto& cur = keyframes[*current_idx];
  *diff_rot = pre.pose_map.orientation.normalized().toRotationMatrix().transpose() *
              T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.orientation.normalized().toRotationMatrix();
  *diff_pos = pre.pose_map.orientation.normalized().toRotationMatrix().transpose() *
              (T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.position + T_hist_cur.block<3, 1>(0, 3) - pre.pose_map.position);
  if (!diff_rot->array().isFinite().all() || !diff_pos->array().isFinite().all()) return false;
  return true;
}

void Fastlio2SamLcLoopClosureBackend::addOdomFactors(const std::vector<MappingKeyframe>& keyframes) {
  const int latest_index = static_cast<int>(keyframes.size()) - 1;
  for (int i = previous_index_; i < latest_index; ++i) {
    const auto& p1 = keyframes[i];
    const auto& p2 = keyframes[i + 1];
    if (i == 0) {
      initialized_estimate_.insert(i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                                                   gtsam::Point3(p1.pose_odom.position)));
      auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(i,
                                                  gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                                                               gtsam::Point3(p1.pose_odom.position)),
                                                  noise));
    }
    initialized_estimate_.insert(i + 1,
                                 gtsam::Pose3(gtsam::Rot3(p2.pose_odom.orientation.normalized().toRotationMatrix()),
                                              gtsam::Point3(p2.pose_odom.position)));
    const Eigen::Matrix3d R12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                p2.pose_odom.orientation.normalized().toRotationMatrix();
    const Eigen::Vector3d t12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                (p2.pose_odom.position - p1.pose_odom.position);
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << config_.odom_rot_noise, config_.odom_rot_noise, config_.odom_rot_noise,
         config_.odom_pos_noise, config_.odom_pos_noise, config_.odom_pos_noise).finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1,
                                                  gtsam::Pose3(gtsam::Rot3(R12), gtsam::Point3(t12)), noise));
  }
  previous_index_ = latest_index;
}

void Fastlio2SamLcLoopClosureBackend::addLoopFactor(int pre_idx,
                                                    int cur_idx,
                                                    double score,
                                                    const Eigen::Matrix3d& diff_rot,
                                                    const Eigen::Vector3d& diff_pos) {
  if (!std::isfinite(score) || score <= 0.0 || !diff_rot.array().isFinite().all() || !diff_pos.array().isFinite().all()) {
    return;
  }
  const gtsam::Pose3 pose_between{gtsam::Rot3(diff_rot), gtsam::Point3(diff_pos)};
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(pre_idx, cur_idx, pose_between,
                                                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * score * config_.loop_noise_scale)));
}

void Fastlio2SamLcLoopClosureBackend::optimize(std::vector<MappingKeyframe>* keyframes, int latest_index) {
  if (!keyframes || !isam2_) return;
  isam2_->update(graph_, initialized_estimate_);
  isam2_->update();
  graph_.resize(0);
  initialized_estimate_.clear();
  auto optimized = isam2_->calculateBestEstimate();
  for (int i = 0; i <= latest_index && i < static_cast<int>(keyframes->size()); ++i) {
    const auto pose = optimized.at<gtsam::Pose3>(i);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = pose.rotation().matrix().cast<double>();
    T(0,3) = pose.translation().x();
    T(1,3) = pose.translation().y();
    T(2,3) = pose.translation().z();
    (*keyframes)[i].pose_map = matrixToPose(T, (*keyframes)[i].timestamp);
    (*keyframes)[i].pose_map.frame_id = (*keyframes)[i].pose_odom.frame_id;
  }
}

LoopClosureResult Fastlio2SamLcLoopClosureBackend::update(std::vector<MappingKeyframe>* keyframes) {
  LoopClosureResult out;
  if (!config_.enable || !keyframes || keyframes->empty()) return out;
  const int latest_index = static_cast<int>(keyframes->size()) - 1;
  addOdomFactors(*keyframes);

  int current_idx = -1;
  int history_idx = -1;
  double score = 0.0;
  Eigen::Matrix3d diff_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d diff_pos = Eigen::Vector3d::Zero();
  if (detectCandidate(*keyframes, &current_idx, &history_idx, &score, &diff_rot, &diff_pos)) {
    loop_history_.emplace_back(history_idx, current_idx);
    addLoopFactor(history_idx, current_idx, score, diff_rot, diff_pos);
    optimize(keyframes, latest_index);
    out.updated = true;
    out.latest_score = score;
  } else if (!initialized_estimate_.empty() || graph_.size() > 0) {
    optimize(keyframes, latest_index);
  }
  out.loop_pairs = loop_history_;
  out.loop_count = static_cast<int>(loop_history_.size());
  return out;
}

}  // namespace slam_mapping
