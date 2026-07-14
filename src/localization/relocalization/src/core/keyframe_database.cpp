/**
 * @file keyframe_database.cpp
 * @brief 关键帧数据库管理
 *
 * 职责：
 *   - 加载 TUM 格式位姿文件，建立 keyframe_id -> Pose 索引
 *   - 按需加载关键帧 PCD 并预处理
 *   - 为 GICP 构建候选局部子图（空间邻域关键帧拼接）
 *   - 子图构建失败时从全局地图裁剪 fallback 区域
 *
 * 数据约定：
 *   keyframe_id = optimized_poses_tum.txt 行号 = key_point_frame/<id>.pcd 文件名数字
 */

#include <relocalization/core/keyframe_database.hpp>

#include <pcl/filters/crop_box.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace relocalization {

/**
 * @brief 加载关键帧位姿和全局地图
 *
 * 位姿文件每行对应一个 keyframe_id（从 0 递增）。
 * 全局地图可选加载，用于子图 fallback。
 */
bool KeyframeDatabase::load(const KeyframeDatabaseConfig& config) {
  config_ = config;
  poses_ = loadTumPoses(config.pose_file);
  pose_index_.clear();
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    poses_[i].id = static_cast<int>(i);
    pose_index_[poses_[i].id] = i;
  }

  if (!config.global_map_pcd.empty()) {
    try {
      // 全局地图在 map 坐标系，不能用 preprocess 的 min/max_range（相对雷达原点裁剪）
      // 否则只保留原点附近 100m 内的点，远处 query 裁剪结果为空
      const double map_voxel = std::max(config.preprocess.voxel_leaf, 0.5);
      global_map_ = voxelDownsampleCloud(loadPcd(config.global_map_pcd), map_voxel);
      ROS_INFO_STREAM("Loaded global map: " << (global_map_ ? global_map_->size() : 0)
                      << " points, voxel_leaf=" << map_voxel);
    } catch (...) {
      global_map_.reset();
    }
  }
  return !poses_.empty();
}

bool KeyframeDatabase::hasPose(int id) const {
  return pose_index_.find(id) != pose_index_.end();
}

Pose KeyframeDatabase::pose(int id) const {
  const auto it = pose_index_.find(id);
  if (it == pose_index_.end()) {
    throw std::runtime_error("missing pose for keyframe id: " + std::to_string(id));
  }
  return poses_.at(it->second);
}

/// 加载并预处理单个关键帧点云：key_point_frame/<id>.pcd
CloudPtr KeyframeDatabase::loadKeyframeCloud(int id) const {
  return preprocessCloud(loadPcd(keyframePath(id)), config_.preprocess);
}

/**
 * @brief 以 center_id 为中心构建 GICP 用的局部子图
 *
 * 步骤：
 *   1. 在 XY 平面搜索距 center 不超过 submap_radius 的所有关键帧
 *   2. 按距离排序，取最近的 neighbor_keyframes 个
 *   3. 将每个关键帧点云变换到 map 坐标系后拼接
 *   4. 对拼接结果做体素下采样（不再做距离裁剪）
 *
 * 与 FASTLIO2_SAM_LC 的区别：
 *   本实现用空间半径搜索（适合全局重定位），
 *   FASTLIO2 用序列索引 ±N（适合回环检测）。
 */
CloudPtr KeyframeDatabase::buildSubmapAround(int center_id) const {
  if (config_.submap_mode == "index") {
    return buildSubmapAroundIndex(center_id, config_.submap_before_frames,
                                  config_.submap_after_frames);
  }
  if (config_.submap_mode == "hybrid") {
    return buildSubmapAroundIndex(center_id, config_.submap_before_frames,
                                  config_.submap_after_frames, config_.submap_radius);
  }

  CloudPtr submap(new CloudT);
  if (!hasPose(center_id)) {
    return submap;
  }

  const Pose center = pose(center_id);
  std::vector<std::pair<double, int>> neighbors;
  neighbors.reserve(poses_.size());

  // 空间邻域搜索：XY 平面欧氏距离
  for (const auto& p : poses_) {
    const double dist = (p.t.head<2>() - center.t.head<2>()).norm();
    if (dist <= config_.submap_radius) {
      neighbors.emplace_back(dist, p.id);
    }
  }
  std::sort(neighbors.begin(), neighbors.end());
  if (config_.neighbor_keyframes > 0 && static_cast<int>(neighbors.size()) > config_.neighbor_keyframes) {
    neighbors.resize(static_cast<std::size_t>(config_.neighbor_keyframes));
  }

  // 拼接邻域关键帧点云到 map 坐标系
  for (const auto& item : neighbors) {
    try {
      const Pose p = pose(item.second);
      CloudPtr cloud = loadKeyframeCloud(item.second);
      CloudPtr world = transformCloud(cloud, p.matrix());
      *submap += *world;
    } catch (...) {
      continue;
    }
  }

  // 关键帧已在 body 系下采样；合并后的世界系点云不再做细小体素，避免大坐标 + 小 leaf 导致 VoxelGrid 溢出
  return submap;
}

CloudPtr KeyframeDatabase::buildSubmapAroundIndex(int center_id, int before_frames, int after_frames,
                                                  double radius_gate) const {
  CloudPtr submap(new CloudT);
  if (!hasPose(center_id)) {
    return submap;
  }

  const Pose center = pose(center_id);
  const int before = std::max(0, before_frames);
  const int after = std::max(0, after_frames);
  const int start_id = std::max(0, center_id - before);
  const int end_id = std::min(static_cast<int>(poses_.size()) - 1, center_id + after);

  for (int id = start_id; id <= end_id; ++id) {
    try {
      if (!hasPose(id)) {
        continue;
      }
      const Pose p = pose(id);
      if (radius_gate > 0.0 && (p.t.head<2>() - center.t.head<2>()).norm() > radius_gate) {
        continue;
      }
      CloudPtr cloud = loadKeyframeCloud(id);
      CloudPtr world = transformCloud(cloud, p.matrix());
      *submap += *world;
    } catch (...) {
      continue;
    }
  }

  return submap;
}

/**
 * @brief 构建中心关键帧局部坐标系下的邻域子图，用于 Scan Context 子图建库
 *
 * 对中心关键帧 i 和邻居关键帧 j：
 *   P_j_local -> T_map_j -> map -> T_i_map -> P_i_local
 * 即：P_in_center = inverse(T_map_i) * T_map_j * P_j。
 * 这样生成的描述子仍以中心关键帧局部坐标系为极坐标参考，而不是 map 坐标系。
 */
CloudPtr KeyframeDatabase::buildLocalSubmapAround(int center_id, double radius, int max_keyframes) const {
  CloudPtr submap(new CloudT);
  if (!hasPose(center_id)) {
    return submap;
  }

  const Pose center = pose(center_id);
  std::vector<std::pair<double, int>> neighbors;
  neighbors.reserve(poses_.size());

  for (const auto& p : poses_) {
    const double dist = (p.t.head<2>() - center.t.head<2>()).norm();
    if (dist <= radius) {
      neighbors.emplace_back(dist, p.id);
    }
  }
  std::sort(neighbors.begin(), neighbors.end());
  if (max_keyframes > 0 && static_cast<int>(neighbors.size()) > max_keyframes) {
    neighbors.resize(static_cast<std::size_t>(max_keyframes));
  }

  const Eigen::Matrix4d center_inv = center.matrix().inverse();
  for (const auto& item : neighbors) {
    try {
      const Pose p = pose(item.second);
      CloudPtr cloud = loadKeyframeCloud(item.second);
      CloudPtr local = transformCloud(cloud, center_inv * p.matrix());
      *submap += *local;
    } catch (...) {
      continue;
    }
  }

  return submap;
}

CloudPtr KeyframeDatabase::buildLocalSubmapAroundIndex(int center_id, int before_frames, int after_frames,
                                                          double radius_gate) const {
  CloudPtr submap(new CloudT);
  if (!hasPose(center_id)) {
    return submap;
  }

  const Pose center = pose(center_id);
  const Eigen::Matrix4d center_inv = center.matrix().inverse();
  const int before = std::max(0, before_frames);
  const int after = std::max(0, after_frames);
  const int start_id = std::max(0, center_id - before);
  const int end_id = std::min(static_cast<int>(poses_.size()) - 1, center_id + after);

  for (int id = start_id; id <= end_id; ++id) {
    try {
      if (!hasPose(id)) {
        continue;
      }
      const Pose p = pose(id);
      if (radius_gate > 0.0 && (p.t.head<2>() - center.t.head<2>()).norm() > radius_gate) {
        continue;
      }
      CloudPtr cloud = loadKeyframeCloud(id);
      CloudPtr local = transformCloud(cloud, center_inv * p.matrix());
      *submap += *local;
    } catch (...) {
      continue;
    }
  }

  return submap;
}

/**
 * @brief 从全局地图中裁剪候选位姿附近的立方体区域（子图 fallback）
 *
 * 当关键帧邻域子图为空（如关键帧 PCD 缺失）时使用。
 * 半径使用 config_.submap_radius。
 */
CloudPtr KeyframeDatabase::cropGlobalMap(const Pose& center) const {
  return cropGlobalMap(center, config_.submap_radius);
}

/**
 * @brief 以指定半径从全局地图裁剪立方体区域
 *
 * 用于 EvalVisualizer：围绕 query 真值位姿裁剪局部地图，
 * 在 RViz 中作为灰色背景，与红/绿 query 点云叠加对比。
 *
 * @param center  裁剪中心（通常为 query 真值位姿）
 * @param radius  立方体半边长 (m)
 */
CloudPtr KeyframeDatabase::cropGlobalMap(const Pose& center, double radius) const {
  CloudPtr cropped(new CloudT);
  if (!global_map_ || global_map_->empty()) {
    return cropped;
  }
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(global_map_);
  const float r = static_cast<float>(radius);
  crop.setMin(Eigen::Vector4f(static_cast<float>(center.t.x() - r),
                              static_cast<float>(center.t.y() - r),
                              static_cast<float>(center.t.z() - r),
                              1.0f));
  crop.setMax(Eigen::Vector4f(static_cast<float>(center.t.x() + r),
                              static_cast<float>(center.t.y() + r),
                              static_cast<float>(center.t.z() + r),
                              1.0f));
  crop.filter(*cropped);
  return cropped;
}

/**
 * @brief 将 query 附近关键帧拼接为局部 3D 地图（RViz 背景用）
 *
 * 与 cropGlobalMap 使用相同的立方体范围，但数据来自各关键帧原始 LiDAR 扫描，
 * 保留高度信息；适用于 global_map_pcd 为 2D 投影（z=0）的场景。
 */
CloudPtr KeyframeDatabase::buildKeyframeMapAround(const Pose& center, double radius,
                                                    int max_keyframes) const {
  CloudPtr map(new CloudT);
  std::vector<std::pair<double, int>> neighbors;
  neighbors.reserve(poses_.size());

  const float r = static_cast<float>(radius);
  for (const auto& p : poses_) {
    if (std::abs(p.t.x() - center.t.x()) > r || std::abs(p.t.y() - center.t.y()) > r ||
        std::abs(p.t.z() - center.t.z()) > r) {
      continue;
    }
    neighbors.emplace_back((p.t - center.t).norm(), p.id);
  }
  std::sort(neighbors.begin(), neighbors.end());
  if (max_keyframes > 0 && static_cast<int>(neighbors.size()) > max_keyframes) {
    neighbors.resize(static_cast<std::size_t>(max_keyframes));
  }

  for (const auto& item : neighbors) {
    try {
      const Pose p = pose(item.second);
      CloudPtr cloud = loadKeyframeCloud(item.second);
      CloudPtr world = transformCloud(cloud, p.matrix());
      *map += *world;
    } catch (...) {
      continue;
    }
  }
  return map;
}

std::string KeyframeDatabase::keyframePath(int id) const {
  return joinPath(config_.keyframe_dir, std::to_string(id) + ".pcd");
}

}  // namespace relocalization
