/**
 * @file pointcloud_verifier.cpp
 * @brief 点云验证器实现（含 VGICP 匹配和非 VGICP 模式）
 */

#include <aruco_relocalization/verifier/pointcloud_verifier.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace aruco_reloc {

namespace {

/// 转换为 small_gicp 点云格式
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

/// 计算两个位姿之间的旋转角度差（度）
double rotationDiffDeg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  Eigen::Matrix3d R_delta = a.rotation().transpose() * b.rotation();
  double trace = R_delta.trace();
  trace = std::max(-1.0, std::min(3.0, trace));
  return std::acos((trace - 1.0) / 2.0) * 180.0 / M_PI;
}

}  // namespace

bool PointCloudVerifier::initialize(const PointCloudVerifyConfig& config) {
  config_ = config;

  if (!config_.enabled) {
    ROS_INFO("[aruco_reloc] pointcloud verifier disabled");
    return true;
  }

  // 加载全局地图
  if (!loadGlobalMap(config_.global_map_pcd)) {
    ROS_ERROR("[aruco_reloc] failed to load global map: %s", config_.global_map_pcd.c_str());
    return false;
  }

  ROS_INFO("[aruco_reloc] pointcloud verifier initialized: map=%zu points, "
           "use_vgicp=%d, inlier_dist=%.2f, min_ratio=%.2f, max_rmse=%.2f",
           global_map_->size(), config_.use_vgicp, config_.inlier_distance,
           config_.min_inlier_ratio, config_.max_inlier_rmse);
  return true;
}

bool PointCloudVerifier::loadGlobalMap(const std::string& pcd_path) {
  if (pcd_path.empty()) {
    ROS_ERROR("[aruco_reloc] global_map_pcd path is empty");
    return false;
  }

  CloudPtr raw(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(pcd_path, *raw) != 0) {
    ROS_ERROR("[aruco_reloc] failed to load PCD: %s", pcd_path.c_str());
    return false;
  }

  if (raw->empty()) {
    ROS_ERROR("[aruco_reloc] global map is empty");
    return false;
  }

  // 体素下采样
  const double map_voxel = std::max(config_.voxel_leaf, 0.5);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(map_voxel),
                    static_cast<float>(map_voxel),
                    static_cast<float>(map_voxel));
  voxel.setInputCloud(raw);
  global_map_.reset(new CloudT);
  voxel.filter(*global_map_);

  map_loaded_ = true;
  ROS_INFO("[aruco_reloc] loaded global map: %zu -> %zu points (voxel=%.2f)",
           raw->size(), global_map_->size(), map_voxel);
  return true;
}

void PointCloudVerifier::onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  double timestamp = msg->header.stamp.toSec();
  cloud_buffer_.push_back({timestamp, msg});
  // 保持缓冲区大小
  while (cloud_buffer_.size() > kMaxCloudBufferSize) {
    cloud_buffer_.pop_front();
  }
}

PointCloudVerifyResult PointCloudVerifier::verify(const Eigen::Isometry3d& T_map_base,
                                                  double image_timestamp) {
  PointCloudVerifyResult result;

  if (!config_.enabled || !map_loaded_) {
    result.valid = true;
    result.T_map_base_refined = T_map_base;
    return result;
  }

  const auto start = std::chrono::steady_clock::now();

  try {

  // 从缓冲区获取与图像时间戳最接近的点云
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_buffer_.empty()) {
      result.reject_reason = "no_pointcloud";
      return result;
    }

    if (image_timestamp > 0) {
      // 查找时间戳最接近的点云
      double best_diff = 1e9;
      for (const auto& tc : cloud_buffer_) {
        double diff = std::abs(tc.timestamp - image_timestamp);
        if (diff < best_diff) {
          best_diff = diff;
          cloud_msg = tc.msg;
        }
      }
      if (best_diff > kMaxCloudTimeDiff) {
        ROS_WARN_THROTTLE(1.0, "[aruco_reloc] pointcloud time diff too large: %.3fs", best_diff);
        result.reject_reason = "pointcloud_time_diff_too_large";
        return result;
      }
    } else {
      // 没有图像时间戳，使用最新点云
      cloud_msg = cloud_buffer_.back().msg;
    }
  }

  if (!cloud_msg) {
    result.reject_reason = "no_pointcloud";
    return result;
  }

  // 转换为 PCL
  CloudPtr raw(new CloudT);
  pcl::fromROSMsg(*cloud_msg, *raw);

  if (raw->empty()) {
    result.reject_reason = "empty_pointcloud";
    return result;
  }

  // 1. 坐标系旋转（lidar → base_link）
  double yaw_rad = config_.query_yaw_deg * M_PI / 180.0;
  if (config_.rotate_xy_y_negx) {
    yaw_rad += -M_PI / 2.0;
  }
  CloudPtr cloud_base = std::abs(yaw_rad) > 1e-9 ? rotateCloudYaw(raw, yaw_rad) : raw;

  // 2. base_to_lidar 变换
  Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
  T_base_lidar.translation() = config_.base_to_lidar.translation;
  T_base_lidar.rotate(config_.base_to_lidar.rotation);
  if (!T_base_lidar.matrix().isIdentity(1e-6)) {
    CloudPtr transformed(new CloudT);
    pcl::transformPointCloud(*cloud_base, *transformed, T_base_lidar.matrix().cast<float>());
    cloud_base = transformed;
  }

  // 3. 预处理
  CloudPtr cloud_prep = preprocessCloud(cloud_base); // 原点云也下采样了

  if (cloud_prep->size() < 10) {
    result.reject_reason = "too_few_points_after_preprocess";
    return result;
  }

  // 4. 裁剪局部子图
  CloudPtr submap = cropLocalSubmap(T_map_base);
  result.submap = submap;  // 保存用于可视化

  if (!submap || submap->size() < 10) {
    result.reject_reason = "empty_submap";
    return result;
  }

  // 5. 根据 use_vgicp 决定是否使用 VGICP
  Eigen::Isometry3d final_pose = T_map_base;

  if (config_.use_vgicp) {
    // VGICP 匹配
    auto vgicp_result = runVgicp(cloud_prep, submap, T_map_base);
    result.vgicp_converged = vgicp_result.converged;
    result.T_map_base_refined = vgicp_result.T_map_base;
    result.delta_translation = vgicp_result.delta_translation;
    result.delta_rotation_deg = vgicp_result.delta_rotation_deg;
    result.aligned_source = vgicp_result.aligned_source;

    if (!vgicp_result.converged) {
      result.reject_reason = "vgicp_not_converged";
      // 即使 VGICP 不收敛，也计算 inlier（用原始位姿）
      final_pose = T_map_base;
    } else {
      final_pose = vgicp_result.T_map_base;
    }
  } else {
    // 非 VGICP 模式：直接用 ArUco 位姿
    result.T_map_base_refined = T_map_base;
    result.vgicp_converged = true;  // 标记为有效
  }

  // 6. 变换点云到 map 系（用于可视化和 inlier 计算）
  CloudPtr cloud_map = transformToMap(cloud_prep, final_pose);
  result.source_in_map = cloud_map;  // 保存用于可视化

  // 7. 最近邻搜索计算 inlier
  CloudPtr submap_for_inlier = submap;
  if (config_.use_vgicp && result.vgicp_converged) {
    // VGICP 成功时，用精化后的位姿重新裁剪子图
    CloudPtr submap_refined = cropLocalSubmap(final_pose);
    if (submap_refined && submap_refined->size() >= 10) {
      submap_for_inlier = submap_refined;
    }
  }

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(submap_for_inlier);

  const double inlier_dist_sq = config_.inlier_distance * config_.inlier_distance;
  double sum_inlier_sq = 0.0;
  int inlier_count = 0;
  int correspondence_count = 0;

  std::vector<int> indices(1);
  std::vector<float> sq_dists(1);

  for (const auto& pt : cloud_map->points) {
    if (!pcl::isFinite(pt)) continue;
    if (kdtree.nearestKSearch(pt, 1, indices, sq_dists) <= 0) continue;
    ++correspondence_count;
    if (sq_dists[0] <= static_cast<float>(inlier_dist_sq)) {
      ++inlier_count;
      sum_inlier_sq += static_cast<double>(sq_dists[0]);
    }
  }

  // 8. 计算结果
  result.correspondence_count = correspondence_count;
  result.inlier_count = inlier_count;

  if (correspondence_count > 0) {
    result.inlier_ratio = static_cast<double>(inlier_count) /
                          static_cast<double>(correspondence_count);
  }
  if (inlier_count > 0) {
    result.inlier_rmse = std::sqrt(sum_inlier_sq / static_cast<double>(inlier_count));
  }

  // 9. 阈值判断
  if (result.inlier_ratio < config_.min_inlier_ratio) {
    result.reject_reason = "inlier_ratio_too_low (" +
                           std::to_string(result.inlier_ratio) + "/" +
                           std::to_string(config_.min_inlier_ratio) + ")";
  } else if (result.inlier_rmse > config_.max_inlier_rmse) {
    result.reject_reason = "inlier_rmse_too_high (" +
                           std::to_string(result.inlier_rmse) + "/" +
                           std::to_string(config_.max_inlier_rmse) + ")";
  } else {
    result.valid = true;
  }

  const auto end = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

  } catch (const std::exception& e) {
    ROS_ERROR("[aruco_reloc] pointcloud verify exception: %s", e.what());
    result.reject_reason = std::string("exception: ") + e.what();
    result.valid = false;
  } catch (...) {
    ROS_ERROR("[aruco_reloc] pointcloud verify unknown exception");
    result.reject_reason = "unknown_exception";
    result.valid = false;
  }

  return result;
}

PointCloudVerifier::VgicpResult PointCloudVerifier::runVgicp(
    const CloudConstPtr& source, const CloudConstPtr& target,
    const Eigen::Isometry3d& initial_guess) const {
  VgicpResult result;

  if (!source || source->empty() || !target || target->empty()) {
    return result;
  }

  // 检查初始位姿是否有效
  if (!initial_guess.matrix().allFinite()) {
    ROS_WARN("[aruco_reloc] vgicp: initial_guess has NaN/Inf, skip");
    return result;
  }

  const auto start = std::chrono::steady_clock::now();

  try {

  // 转换为 small_gicp 格式
  auto source_pc = toSmallGicpCloud(source);
  auto target_pc = toSmallGicpCloud(target);

  if (!source_pc || source_pc->size() < 4 || !target_pc || target_pc->size() < 4) {
    return result;
  }

  // 估计法向量
  const int num_neighbors = 30;
  const int num_threads = 4;
  small_gicp::estimate_covariances_omp(*target_pc, num_neighbors, num_threads);
  small_gicp::estimate_covariances_omp(*source_pc, num_neighbors, num_threads);

  // VGICP 两级配准参数
  const double base_res = std::max(0.05, config_.voxel_leaf);
  const double coarse_scale = 10.0;
  const double fine_scale = 1.0;
  const double max_correspondence_distance = 2.0;
  const double transformation_epsilon = 1e-5;
  const int max_iterations = 64;

  // 构建两级 GaussianVoxelMap
  auto coarse_map = std::make_shared<small_gicp::GaussianVoxelMap>(base_res * coarse_scale);
  coarse_map->set_search_offsets(27);
  coarse_map->insert(*target_pc);

  auto fine_map = std::make_shared<small_gicp::GaussianVoxelMap>(base_res * fine_scale);
  fine_map->set_search_offsets(27);
  fine_map->insert(*target_pc);

  // 第一层：粗配准
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> coarse_reg;
  coarse_reg.reduction.num_threads = num_threads;
  const double coarse_max_dist = max_correspondence_distance * coarse_scale;
  coarse_reg.rejector.max_dist_sq = coarse_max_dist * coarse_max_dist;
  coarse_reg.criteria.translation_eps = transformation_epsilon;
  coarse_reg.criteria.rotation_eps = transformation_epsilon;
  coarse_reg.optimizer.max_iterations = max_iterations;

  Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
  init.matrix() = initial_guess.matrix();
  const auto coarse_result = coarse_reg.align(*coarse_map, *source_pc, *coarse_map, init);

  // 第二层：精匹配
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> fine_reg;
  fine_reg.reduction.num_threads = num_threads;
  const double fine_max_dist = max_correspondence_distance * fine_scale;
  fine_reg.rejector.max_dist_sq = fine_max_dist * fine_max_dist;
  fine_reg.criteria.translation_eps = transformation_epsilon;
  fine_reg.criteria.rotation_eps = transformation_epsilon;
  fine_reg.optimizer.max_iterations = max_iterations;

  const Eigen::Isometry3d coarse_T = coarse_result.T_target_source;
  const auto fine_result = fine_reg.align(*fine_map, *source_pc, *fine_map, coarse_T);

  // 提取结果
  result.converged = fine_result.converged;
  result.T_map_base = Eigen::Isometry3d::Identity();
  result.T_map_base.matrix() = fine_result.T_target_source.matrix();

  // 计算修正量
  const Eigen::Matrix4d delta = result.T_map_base.matrix() * initial_guess.matrix().inverse();
  result.delta_translation = delta.block<3, 1>(0, 3).norm();
  result.delta_rotation_deg = rotationDiffDeg(result.T_map_base, initial_guess);

  // 保存匹配后的点云用于可视化
  result.aligned_source.reset(new CloudT);
  pcl::transformPointCloud(*source, *result.aligned_source, result.T_map_base.matrix().cast<float>());

  const auto end = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

  } catch (const std::exception& e) {
    ROS_ERROR("[aruco_reloc] vgicp exception: %s", e.what());
    result.converged = false;
  } catch (...) {
    ROS_ERROR("[aruco_reloc] vgicp unknown exception");
    result.converged = false;
  }

  return result;
}

CloudPtr PointCloudVerifier::rotateCloudYaw(const CloudConstPtr& cloud,
                                            double yaw_rad) const {
  CloudPtr out(new CloudT);
  if (!cloud) return out;
  *out = *cloud;
  const float c = static_cast<float>(std::cos(yaw_rad));
  const float s = static_cast<float>(std::sin(yaw_rad));
  for (auto& pt : out->points) {
    const float x = pt.x;
    const float y = pt.y;
    pt.x = c * x - s * y;
    pt.y = s * x + c * y;
  }
  return out;
}

CloudPtr PointCloudVerifier::transformToMap(const CloudConstPtr& cloud_base,
                                            const Eigen::Isometry3d& T_map_base) const {
  CloudPtr out(new CloudT);
  if (!cloud_base) return out;
  pcl::transformPointCloud(*cloud_base, *out, T_map_base.matrix().cast<float>());
  return out;
}

CloudPtr PointCloudVerifier::cropLocalSubmap(const Eigen::Isometry3d& T_map_base) const {
  CloudPtr cropped(new CloudT);
  if (!global_map_ || global_map_->empty()) return cropped;

  pcl::CropBox<PointT> crop;
  crop.setInputCloud(global_map_);
  crop.setMin(Eigen::Vector4f(static_cast<float>(-config_.crop_backward),
                               static_cast<float>(-config_.crop_right),
                               static_cast<float>(-config_.crop_down),
                               1.0f));
  crop.setMax(Eigen::Vector4f(static_cast<float>(config_.crop_forward),
                               static_cast<float>(config_.crop_left),
                               static_cast<float>(config_.crop_up),
                               1.0f));

  Eigen::Affine3f transform;
  transform.matrix() = T_map_base.matrix().cast<float>().inverse();
  crop.setTransform(transform);
  crop.filter(*cropped);

  return cropped;
}

CloudPtr PointCloudVerifier::preprocessCloud(const CloudConstPtr& cloud) const {
  CloudPtr filtered(new CloudT);
  if (!cloud) return filtered;
  filtered->reserve(cloud->size());

  const double min_sq = config_.min_range * config_.min_range;
  const double max_sq = config_.max_range * config_.max_range;

  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) continue;
    const double r_sq = static_cast<double>(pt.x) * pt.x +
                        static_cast<double>(pt.y) * pt.y;
    if (r_sq < min_sq || r_sq > max_sq) continue;
    if (config_.z_filter_enable) {
      if (pt.z < static_cast<float>(config_.z_filter_min) ||
          pt.z > static_cast<float>(config_.z_filter_max)) continue;
    }
    filtered->push_back(pt);
  }

  if (config_.voxel_leaf > 1e-6 && !filtered->empty()) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(static_cast<float>(config_.voxel_leaf),
                      static_cast<float>(config_.voxel_leaf),
                      static_cast<float>(config_.voxel_leaf));
    voxel.setInputCloud(filtered);
    CloudPtr downsampled(new CloudT);
    voxel.filter(*downsampled);
    if (!downsampled->empty()) return downsampled;
  }

  return filtered;
}

}  // namespace aruco_reloc
