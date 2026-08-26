#pragma once

#include <relocalization/core/common.hpp>
#include <relocalization/interface/i_descriptor.hpp>

namespace relocalization {

class KeyframeDatabase;

struct GlobalMatcherConfig {
  int max_iterations = 64;
  double max_correspondence_distance = 2.0;
  double transformation_epsilon = 0.01;
  double euclidean_fitness_epsilon = 0.01;
  double voxel_leaf_source = 0.5;
  double voxel_leaf_target = 0.5;
  double accept_fitness = 0.5;
  double inlier_distance = 0.5;
  double min_inlier_ratio = 0.6;
  double max_inlier_rmse = 0.5;
  bool z_filter_enable = false;
  double z_filter_min = -1000.0;
  double z_filter_max = 1000.0;
  bool near_ceiling_filter_enable = false;
  double near_ceiling_radius = 15.0;
  // 以下字段从 relocalization_framework 移植（默认值与框架当前 yaml 一致）
  // target 来源："crop"（global_matcher 风格全局地图朝向裁剪）| "submap"（关键帧子图拼接）
  std::string target_source = "crop";
  double crop_forward = 70.0;
  double crop_backward = 50.0;
  double crop_left = 40.0;
  double crop_right = 40.0;
  double crop_down = 10.0;
  double crop_up = 10.0;
  // GICP/VGICP 初值 yaw：true=候选位姿 yaw + 描述子 yaw_diff；false=直接用候选位姿 yaw
  bool use_descriptor_yaw_correction = true;
  // 相对地面剔除：候选帧局部系 z < ground_remove_rel_z 剔除
  bool ground_remove_relative = false;
  double ground_remove_rel_z = -1.0;
  // 候选空间门控：只保留 top1 周围 XY 距离 ≤ radius 的候选；radius≤0 关闭
  bool candidate_gate_enable = true;
  double candidate_gate_radius = 20.0;
  // VGICP（small_gicp）参数
  double vgicp_voxel_resolution = 0.5;
  int vgicp_num_neighbors = 30;
  int vgicp_num_threads = 8;
  double vgicp_coarse_scale = 10.0;
  double vgicp_fine_scale = 1.0;
};

struct MatchResult {
  bool success = false;
  bool converged = false;
  int candidate_id = -1;
  Pose candidate_pose;
  double scan_context_score = 1e9;
  int scan_context_sector_shift = 0;
  double scan_context_yaw_diff = 0.0;
  double candidate_yaw = 0.0;
  double initial_yaw = 0.0;
  double final_yaw = 0.0;
  double delta_translation = 0.0;
  double delta_yaw = 0.0;
  double fitness = 1e9;
  double inlier_ratio = 0.0;
  double inlier_rmse = 1e9;
  int inlier_count = 0;
  int correspondence_count = 0;
  int gicp_source_points = 0;
  int gicp_target_points = 0;
  double elapsed_ms = 0.0;
  Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  CloudPtr aligned;
  CloudPtr target;
};

class IGlobalMatcher {
 public:
  virtual ~IGlobalMatcher() = default;

  virtual MatchResult alignCandidates(const CloudConstPtr& source,
                                      const std::vector<DescriptorCandidate>& candidates,
                                      const KeyframeDatabase& database) const = 0;
  virtual std::string summary() const = 0;
};

}  // namespace relocalization
