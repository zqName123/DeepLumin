#include "slam_mapping_v2/core/pose_graph_optimizer.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>

namespace slam_mapping_v2 {

namespace {

gtsam::SharedNoiseModel buildLoopNoiseModel(const LoopClosureConfig& cfg, double score) {
  const double fitness_th = cfg.icp_fitness_thresh > 1e-9 ? cfg.icp_fitness_thresh : 1.0;
  const double clamped_score = std::max(cfg.loop_score_floor, std::min(score, fitness_th));
  const double ratio = std::max(0.1, std::min(clamped_score / fitness_th, 1.0));
  const double rot_sigma = std::max(1e-6, cfg.loop_rot_noise * ratio * cfg.loop_noise_scale);
  const double pos_sigma = std::max(1e-6, cfg.loop_pos_noise * ratio * cfg.loop_noise_scale);
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma, pos_sigma, pos_sigma, pos_sigma)
          .finished());
}

}  // namespace

void PoseGraphOptimizer::configure(const LoopClosureConfig& cfg) {
  cfg_ = cfg;
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam2_ = std::make_shared<gtsam::ISAM2>(params);
  reset();
}

void PoseGraphOptimizer::reset() {
  previous_index_ = 0;
  graph_.resize(0);
  initialized_estimate_.clear();
  if (isam2_) {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
  }
}

void PoseGraphOptimizer::addOdomFactors(const std::vector<Keyframe>& kfs) {
  const int latest_index = static_cast<int>(kfs.size()) - 1;
  for (int i = previous_index_; i < latest_index; ++i) {
    const auto& p1 = kfs[i];
    const auto& p2 = kfs[i + 1];
    if (i == 0) {
      initialized_estimate_.insert(
          i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                          gtsam::Point3(p1.pose_odom.position)));
      auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
          i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                          gtsam::Point3(p1.pose_odom.position)),
          noise));
    }
    initialized_estimate_.insert(
        i + 1, gtsam::Pose3(gtsam::Rot3(p2.pose_odom.orientation.normalized().toRotationMatrix()),
                            gtsam::Point3(p2.pose_odom.position)));
    const Eigen::Matrix3d R12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                p2.pose_odom.orientation.normalized().toRotationMatrix();
    const Eigen::Vector3d t12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                (p2.pose_odom.position - p1.pose_odom.position);
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << cfg_.odom_rot_noise, cfg_.odom_rot_noise, cfg_.odom_rot_noise,
         cfg_.odom_pos_noise, cfg_.odom_pos_noise, cfg_.odom_pos_noise).finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1,
                                                  gtsam::Pose3(gtsam::Rot3(R12),
                                                               gtsam::Point3(t12)),
                                                  noise));
  }
  previous_index_ = latest_index;
}

void PoseGraphOptimizer::addLoopFactor(const LoopCandidate& c) {
  if (!c.valid || !std::isfinite(c.score) || c.score <= 0.0 ||
      !c.diff_rot.array().isFinite().all() || !c.diff_pos.array().isFinite().all()) {
    return;
  }
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      c.history_idx, c.current_idx,
      gtsam::Pose3(gtsam::Rot3(c.diff_rot), gtsam::Point3(c.diff_pos)),
      buildLoopNoiseModel(cfg_, c.score)));
}

void PoseGraphOptimizer::optimize(std::vector<Keyframe>* kfs) {
  if (!kfs || !isam2_) return;
  if (graph_.size() == 0 && initialized_estimate_.empty()) return;  // 无新因子（如仅 1 个关键帧）跳过
  isam2_->update(graph_, initialized_estimate_);
  isam2_->update();
  graph_.resize(0);
  initialized_estimate_.clear();
  const auto optimized = isam2_->calculateBestEstimate();
  const int latest_index = static_cast<int>(kfs->size()) - 1;
  for (int i = 0; i <= latest_index && i < static_cast<int>(kfs->size()); ++i) {
    if (!optimized.exists(i)) continue;
    const auto pose = optimized.at<gtsam::Pose3>(i);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = pose.rotation().matrix().cast<double>();
    T(0, 3) = pose.translation().x();
    T(1, 3) = pose.translation().y();
    T(2, 3) = pose.translation().z();
    (*kfs)[i].pose_map.position = T.block<3, 1>(0, 3);
    (*kfs)[i].pose_map.orientation = Quatd(T.block<3, 3>(0, 0)).normalized();
    (*kfs)[i].pose_map.frame_id = (*kfs)[i].pose_odom.frame_id;
    (*kfs)[i].optimized = true;
  }
}

}  // namespace slam_mapping_v2
