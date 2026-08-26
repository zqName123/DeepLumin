/**
 * @file loop_closure_fastlio2_backend.cpp
 * @brief Fast-LIO2 + SAM (Smoothing and Mapping) 回环检测与后端优化实现
 *
 * 本文件实现了基于 Fast-LIO2 框架的回环检测与后端图优化模块。
 * 核心功能包括：
 * - 子图构建：从历史关键帧中构建局部子地图用于回环检测
 * - 回环检测：通过 KD-Tree 空间搜索 + ICP 配准检测回环
 * - 图优化：使用 GTSAM ISAM2 增量式平滑器进行后端优化
 * - GNSS 融合：集成 GNSS 位置因子辅助全局定位
 *
 * 算法流程（update 函数）：
 * 1. 添加里程计因子（相邻关键帧间的相对位姿约束）
 * 2. 添加 GNSS 位置因子（若启用）
 * 3. 检测回环候选（空间邻近搜索 + ICP 配准验证）
 * 4. 添加回环因子（通过 ICP 得分自适应噪声）
 * 5. 执行 ISAM2 优化并更新关键帧位姿
 *
 * @see Fastlio2SamLcLoopClosureBackend
 * @see ILoopClosureBackend
 */

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

/**
 * @brief 法向量估计所需的最少点数阈值
 *
 * ICP with Normals 需要足够的点来估计局部法向量，
 * 低于此阈值时跳过法向量计算以避免数值不稳定。
 */
constexpr int kMinNormalEstimationPoints = 15;

/**
 * @brief 检查四元数是否有限且有效
 * @param q 输入四元数
 * @return true 四元数所有分量有限且模长大于阈值
 *
 * 用于位姿有效性检查，防止 NaN/Inf 污染优化问题。
 */
bool isFiniteQuaternion(const localization::Quatd& q) {
  return std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
         std::isfinite(q.w()) && q.norm() > 1e-9;
}

/**
 * @brief 检查位姿是否有限且有效
 * @param pose 输入位姿
 * @return true 位置和朝向都有效
 *
 * 综合检查位置和四元数的有效性。
 */
bool isFinitePose(const localization::Pose& pose) {
  return std::isfinite(pose.position.x()) && std::isfinite(pose.position.y()) &&
         std::isfinite(pose.position.z()) && isFiniteQuaternion(pose.orientation);
}

/**
 * @brief 检查点云中的单个点是否有限
 * @param point 输入点
 * @return true 点的坐标、强度、法向量、曲率都有限
 *
 * PointType 包含 xyz、intensity、normal_xyz、curvature 等字段，
 * 需要全部检查以避免 ICP 中出现 NaN。
 */
bool isFinitePoint(const localization::PointType& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(point.intensity) && std::isfinite(point.normal_x) &&
         std::isfinite(point.normal_y) && std::isfinite(point.normal_z) &&
         std::isfinite(point.curvature);
}

/**
 * @brief 检查点云中所有点是否都有限
 * @param cloud 输入点云指针
 * @return true 所有点都有限或点云为空
 */
bool hasOnlyFinitePoints(const localization::PointCloudPtr& cloud) {
  if (!cloud) return false;
  for (const auto& point : cloud->points) {
    if (!isFinitePoint(point)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 统计点云中无效点的数量
 * @param cloud 输入点云指针
 * @return 无效点的数量
 */
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

/**
 * @brief 生成点云的摘要信息字符串（用于调试日志）
 * @param name 点云名称标识
 * @param cloud 输入点云指针
 * @return 格式化的摘要字符串
 *
 * 输出格式："name ptr=set size=N invalid=M dense=T/F"
 */
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

/**
 * @brief 输出回环调试日志
 * @param msg 日志消息
 *
 * 使用 std::cerr 直接输出到 stderr，避免 ROS 日志缓冲导致的时序问题。
 * 日志前缀统一为 [slam_mapping][loop_debug]。
 */
void debugLoopLog(const std::string& msg) {
  std::cerr << "[slam_mapping][loop_debug] " << msg << std::endl;
}

/**
 * @brief 清理点云中的无效点
 * @param cloud 输入点云指针
 * @return 清理后的点云指针（新分配）
 *
 * 遍历所有点，只保留 isFinitePoint 为真的点。
 * 输出点云标记为 is_dense=true。
 */
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

/**
 * @brief 为 ICP 输入准备点云（额外检查所有点是否有限）
 * @param cloud 输入点云指针
 * @return 准备好的点云指针，若存在无效点则返回空点云
 *
 * 在 sanitizeCloud 基础上增加二次检查，确保 ICP 输入绝对干净。
 */
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

/**
 * @brief 从关键帧中提取里程计位姿
 * @param kf 输入关键帧
 * @return 关键帧的里程计位姿（pose_odom）
 */
localization::Pose poseFromKeyframeOdom(const MappingKeyframe& kf) {
  return kf.pose_odom;
}

/**
 * @brief 将位姿转换为 4x4 齐次变换矩阵
 * @param pose 输入位姿
 * @return 4x4 齐次变换矩阵
 *
 * 旋转部分使用四元数归一化后转旋转矩阵，确保正交性。
 */
Eigen::Matrix4d poseToMatrix(const localization::Pose& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = pose.position;
  return T;
}

/**
 * @brief 将 4x4 齐次变换矩阵转换为位姿
 * @param T 输入齐次变换矩阵
 * @param timestamp 时间戳
 * @return 转换后的位姿
 *
 * 旋转部分从矩阵提取后重新归一化为四元数，避免数值误差累积。
 */
localization::Pose matrixToPose(const Eigen::Matrix4d& T, double timestamp) {
  localization::Pose pose;
  pose.timestamp = timestamp;
  pose.position = T.block<3, 1>(0, 3);
  pose.orientation = localization::Quatd(T.block<3, 3>(0, 0)).normalized();
  return pose;
}

/**
 * @brief 构建回环因子的噪声模型（自适应 ICP 得分）
 * @param cfg 回环检测配置
 * @param score ICP 配准得分（fitness score）
 * @return GTSAM 噪声模型
 *
 * 噪声模型根据 ICP 得分自适应调整：
 * - score 越低（配准越好），噪声越小（约束越可信）
 * - score 越高（配准越差），噪声越大（约束越不确定）
 * - 使用 loop_score_floor 保证最小噪声，避免过约束
 * - 使用 loop_noise_scale 作为全局缩放因子
 *
 * 噪声参数计算：
 *   ratio = clamped_score / fitness_thresh  (范围 [0.1, 1.0])
 *   rot_sigma = loop_rot_noise * ratio * loop_noise_scale
 *   pos_sigma = loop_pos_noise * ratio * loop_noise_scale
 */
gtsam::SharedNoiseModel buildLoopNoiseModel(const LoopClosureConfig& cfg, double score) {
  const double fitness_th = cfg.icp_fitness_thresh > 1e-9 ? cfg.icp_fitness_thresh : 1.0;
  const double clamped_score = std::max(cfg.loop_score_floor, std::min(score, fitness_th));
  const double ratio = std::max(0.1, std::min(clamped_score / fitness_th, 1.0));
  const double rot_sigma = std::max(1e-6, cfg.loop_rot_noise * ratio * cfg.loop_noise_scale);
  const double pos_sigma = std::max(1e-6, cfg.loop_pos_noise * ratio * cfg.loop_noise_scale);
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma,
       pos_sigma, pos_sigma, pos_sigma).finished());
}

}  // namespace

/**
 * @brief 配置回环检测后端
 * @param cfg 回环检测配置参数
 *
 * 初始化所有组件：
 * 1. GTSAM ISAM2 增量式平滑器（relinearizeThreshold=0.01, relinearizeSkip=1）
 * 2. PCL KD-Tree 用于历史位姿空间搜索
 * 3. PCL VoxelGrid 用于子图降采样
 * 4. PCL ICP with Normals 用于子图配准
 * 5. GNSS 优化器
 */
void Fastlio2SamLcLoopClosureBackend::configure(const LoopClosureConfig& cfg) {
  config_ = cfg;
  // 初始化 GTSAM ISAM2 增量式平滑器
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;  // 重新线性化阈值
  params.relinearizeSkip = 1;          // 每次 update 都检查是否需要重新线性化
  isam2_ = std::make_shared<gtsam::ISAM2>(params);
  
  // 初始化 KD-Tree 用于历史位姿的空间邻近搜索
  kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  
  // 配置子图降采样参数
  submap_voxel_.setLeafSize(config_.submap_resolution, config_.submap_resolution, config_.submap_resolution);
  
  // 初始化 ICP with Normals 配准器
  icp_.reset(new pcl::IterativeClosestPointWithNormals<localization::PointType, localization::PointType>());
  icp_->setMaxCorrespondenceDistance(20.0);    // 最大对应点距离（米）
  icp_->setMaximumIterations(50);               // 最大迭代次数
  icp_->setTransformationEpsilon(1e-6);         // 变换收敛阈值
  icp_->setEuclideanFitnessEpsilon(1e-6);      // 均方误差收敛阈值
  icp_->setRANSACIterations(0);                 // 禁用 RANSAC（使用所有点）
  
  // 配置 GNSS 优化器
  gnss_optimizer_.configure(cfg.gnss);
  
  // 重置所有状态变量
  previous_index_ = 0;
  loop_history_.clear();
  graph_.resize(0);
  initialized_estimate_.clear();
  last_loop_check_index_ = -1;
  latest_current_submap_.reset();
  latest_history_submap_.reset();
  latest_loop_pair_ = {-1, -1};
}

/**
 * @brief 重置回环检测后端状态
 *
 * 清空所有历史数据并重新初始化 ISAM2，用于重新开始建图。
 */
void Fastlio2SamLcLoopClosureBackend::reset() {
  gnss_optimizer_.reset();
  previous_index_ = 0;
  loop_history_.clear();
  graph_.resize(0);
  initialized_estimate_.clear();
  last_loop_check_index_ = -1;
  latest_current_submap_.reset();
  latest_history_submap_.reset();
  latest_loop_pair_ = {-1, -1};
  if (isam2_) {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
  }
}

/**
 * @brief 为点云添加法向量估计
 * @param cloud 输入点云指针
 * @return 带法向量的点云指针
 *
 * 处理流程：
 * 1. 清理无效点（sanitizeCloud）
 * 2. 转换为 pcl::PointXYZI（ICP with Normals 需要 intensity 字段）
 * 3. 使用 K 近邻法估计法向量（k=min(15, 点数)）
 * 4. 合并 xyz + intensity + normals 为输出点云
 * 5. 二次清理确保 ICP 输入绝对干净
 */
localization::PointCloudPtr Fastlio2SamLcLoopClosureBackend::addNormals(const localization::PointCloudPtr& cloud) const {
  debugLoopLog(std::string("addNormals input ") + summarizeCloud("cloud", cloud));
  auto filtered = sanitizeCloud(cloud);
  debugLoopLog(std::string("addNormals sanitized ") + summarizeCloud("filtered", filtered));
  if (!filtered || filtered->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
    return localization::PointCloudPtr(new localization::PointCloud());
  }
  // 转换为 PointXYZI（ICP with Normals 需要 intensity 字段存储法向量信息）
  pcl::PointCloud<pcl::PointXYZI>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*filtered, *xyz);
  std::vector<int> xyz_indices;
  pcl::removeNaNFromPointCloud(*xyz, *xyz, xyz_indices);
  if (xyz->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
    debugLoopLog("addNormals xyz too small after removeNaN size=" + std::to_string(xyz->size()));
    return localization::PointCloudPtr(new localization::PointCloud());
  }
  debugLoopLog("addNormals before xyz tree setInputCloud size=" + std::to_string(xyz->size()));
  // 构建 KD-Tree 用于法向量估计的邻域搜索
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  tree->setInputCloud(xyz);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(xyz);
  ne.setSearchMethod(tree);
  ne.setKSearch(std::min<int>(kMinNormalEstimationPoints, static_cast<int>(xyz->size())));
  ne.compute(*normals);
  // 合并原始点云和法向量
  localization::PointCloudPtr out(new localization::PointCloud());
  pcl::concatenateFields(*xyz, *normals, *out);
  auto icp_ready = sanitizeIcpInputCloud(out);
  debugLoopLog(std::string("addNormals output ") + summarizeCloud("icp_ready", icp_ready));
  return icp_ready;
}

/**
 * @brief 判断是否应该对最新关键帧执行回环检测
 * @param keyframes 所有关键帧列表
 * @param current_idx 当前最新关键帧索引
 * @return true 应该执行回环检测
 *
 * 检查条件：
 * 1. 当前索引有效
 * 2. 当前索引 >= min_index_gap（避免检测过近的关键帧）
 * 3. 距离上次回环检测的关键帧间隔 >= loop_kf_gap
 */
bool Fastlio2SamLcLoopClosureBackend::shouldDetectLoopForLatestKeyframe(const std::vector<MappingKeyframe>& keyframes,
                                                                 int current_idx) const {
  if (current_idx < 0 || current_idx >= static_cast<int>(keyframes.size())) {
    return false;
  }
  const int min_required_gap = std::max(config_.min_index_gap, config_.closest_id_th);
  if (current_idx < min_required_gap) {
    return false;
  }
  if (config_.loop_kf_gap > 0 && last_loop_check_index_ >= 0 &&
      (current_idx - last_loop_check_index_) < config_.loop_kf_gap) {
    return false;
  }
  return true;
}

/**
 * @brief 检查候选回环是否已在历史回环附近
 * @param current_idx 当前关键帧索引
 * @param history_idx 历史候选关键帧索引
 * @return true 候选在历史回环附近（应跳过）
 *
 * 防止在已检测到回环的邻近区域重复检测，避免冗余约束。
 */
bool Fastlio2SamLcLoopClosureBackend::isCandidateNearExistingLoop(int current_idx, int history_idx) const {
  if (config_.min_id_interval <= 0) {
    return false;
  }
  for (const auto& loop_pair : loop_history_) {
    if (std::abs(loop_pair.first - history_idx) < config_.min_id_interval &&
        std::abs(loop_pair.second - current_idx) < std::max(1, config_.loop_kf_gap)) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 构建当前帧子图（仅包含当前关键帧自身点云）
 * @param keyframes 所有关键帧列表
 * @param current_idx 当前关键帧索引
 * @return 当前帧子图点云
 */
localization::PointCloudPtr Fastlio2SamLcLoopClosureBackend::buildCurrentSubmap(const std::vector<MappingKeyframe>& keyframes,
                                                                                 int current_idx) const {
  return buildSubmap(keyframes, current_idx, 0);  // search_num=0 表示只取当前帧
}

/**
 * @brief 构建历史帧子图（包含历史关键帧前后各 search_num 帧）
 * @param keyframes 所有关键帧列表
 * @param history_idx 历史关键帧索引
 * @return 历史帧子图点云
 */
localization::PointCloudPtr Fastlio2SamLcLoopClosureBackend::buildHistorySubmap(const std::vector<MappingKeyframe>& keyframes,
                                                                                 int history_idx) const {
  return buildSubmap(keyframes, history_idx, config_.submap_search_num);
}

/**
 * @brief 构建子地图（通用实现）
 * @param keyframes 所有关键帧列表
 * @param center_index 中心关键帧索引
 * @param search_num 向两侧扩展的帧数
 * @return 构建好的子地图点云
 *
 * 子图构建流程：
 * 1. 确定索引范围 [center - search_num, center + search_num]
 * 2. 遍历范围内关键帧，将 body 系点云转换到 map 系
 * 3. 合并所有点云
 * 4. 清理无效点
 * 5. VoxelGrid 降采样
 * 6. 再次清理
 */
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
    // 将 body 系点云转换到 map 系（使用优化后的 pose_map）
    const Eigen::Matrix4f T = poseToMatrix(kf.pose_map).cast<float>();
    localization::PointCloud temp;
    pcl::transformPointCloud(*kf.cloud_body, temp, T);
    *cloud += temp;
  }
  if (!cloud->empty()) {
    debugLoopLog(std::string("buildSubmap raw ") + summarizeCloud("cloud", cloud));
    cloud = sanitizeCloud(cloud);
    if (cloud->empty()) return cloud;
    // VoxelGrid 降采样以减少 ICP 计算量
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

/**
 * @brief 查找最佳历史回环候选关键帧
 * @param keyframes 所有关键帧列表
 * @param current_idx 当前最新关键帧索引
 * @return 最佳历史候选索引，若未找到返回 -1
 *
 * 搜索流程：
 * 1. 构建历史位姿 KD-Tree（使用 pose_map 位置）
 * 2. 半径搜索当前位置附近的历史关键帧
 * 3. 过滤不满足条件的候选：
 *    - 索引无效或 >= current_idx
 *    - 位姿无效（NaN）
 *    - 与当前帧索引间隔 < min_index_gap
 *    - 时间差 < time_thresh_sec（避免检测连续帧）
 *    - 已在 loop_history_ 中（避免重复检测）
 *    - 在已有回环附近（min_id_interval）
 * 4. 返回距离最近的有效候选
 */
int Fastlio2SamLcLoopClosureBackend::findBestHistoryCandidateIndex(const std::vector<MappingKeyframe>& keyframes,
                                                                      int current_idx) {
  // 构建历史位姿点云用于 KD-Tree 搜索
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

  if (cloud_history_poses_->empty() || !pcl::isFinite(cloud_history_poses_->back())) {
    debugLoopLog("findBestHistoryCandidateIndex latest history pose invalid before radiusSearch");
    return -1;
  }

  const double search_radius = config_.max_range > 0.0 ? config_.max_range : config_.search_radius;
  debugLoopLog("findBestHistoryCandidateIndex before history radiusSearch poses=" +
               std::to_string(cloud_history_poses_->size()) +
               " current_idx=" + std::to_string(current_idx) +
               " search_radius=" + std::to_string(search_radius));
  kdtree_history_poses_->setInputCloud(cloud_history_poses_);

  std::vector<int> ids;
  std::vector<float> sqdists;
  kdtree_history_poses_->radiusSearch(cloud_history_poses_->back(), search_radius, ids, sqdists, 0);
  debugLoopLog("findBestHistoryCandidateIndex radiusSearch result candidates=" + std::to_string(ids.size()));

  int best_history_idx = -1;
  float best_sqdist = std::numeric_limits<float>::max();
  const int required_index_gap = std::max(config_.min_index_gap, config_.closest_id_th);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const int id = ids[i];
    if (id < 0 || id >= current_idx) {
      continue;
    }
    if (!pcl::isFinite(cloud_history_poses_->at(id))) {
      continue;
    }
    if ((current_idx - id) < required_index_gap) {
      continue;
    }
    if (config_.time_thresh_sec > 0.0 &&
        std::abs(keyframes[id].timestamp - keyframes.back().timestamp) <= config_.time_thresh_sec) {
      continue;
    }
    if (std::find(loop_history_.begin(), loop_history_.end(), std::make_pair(id, current_idx)) != loop_history_.end()) {
      continue;
    }
    if (isCandidateNearExistingLoop(current_idx, id)) {
      continue;
    }

    const float sqdist = i < sqdists.size() ? sqdists[i] : std::numeric_limits<float>::max();
    if (sqdist < best_sqdist) {
      best_sqdist = sqdist;
      best_history_idx = id;
    }
  }

  return best_history_idx;
}

/**
 * @brief ICP 配准候选子图并计算相对位姿
 * @param keyframes 所有关键帧列表
 * @param current_idx 当前关键帧索引
 * @param history_idx 历史候选关键帧索引
 * @param score 输出 ICP 配准得分
 * @param diff_rot 输出历史到当前的相对旋转
 * @param diff_pos 输出历史到当前的相对平移
 * @return true ICP 配准成功且通过阈值检查
 *
 * 配准流程：
 * 1. 构建当前和历史子图
 * 2. 为子图添加法向量
 * 3. 执行 ICP with Normals 配准
 * 4. 检查收敛性和 fitness score
 * 5. 计算历史关键帧到当前关键帧的相对位姿变换
 */
bool Fastlio2SamLcLoopClosureBackend::alignCandidateSubmaps(const std::vector<MappingKeyframe>& keyframes,
                                                            int current_idx,
                                                            int history_idx,
                                                            double* score,
                                                            Eigen::Matrix3d* diff_rot,
                                                            Eigen::Vector3d* diff_pos) {
  if (!score || !diff_rot || !diff_pos) {
    return false;
  }

  auto cur_submap = buildCurrentSubmap(keyframes, current_idx);
  auto hist_submap = buildHistorySubmap(keyframes, history_idx);
  latest_current_submap_ = cur_submap;
  latest_history_submap_ = hist_submap;
  if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) {
    return false;
  }

  // 为子图添加法向量（ICP with Normals 需要）
  auto cur_norm = sanitizeIcpInputCloud(addNormals(cur_submap));
  auto hist_norm = sanitizeIcpInputCloud(addNormals(hist_submap));
  debugLoopLog(std::string("alignCandidateSubmaps ICP source ") + summarizeCloud("cur_norm", cur_norm));
  debugLoopLog(std::string("alignCandidateSubmaps ICP target ") + summarizeCloud("hist_norm", hist_norm));
  if (!cur_norm || !hist_norm ||
      cur_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
      hist_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
      !hasOnlyFinitePoints(cur_norm) || !hasOnlyFinitePoints(hist_norm)) {
    return false;
  }

  localization::PointCloud aligned;
  debugLoopLog("alignCandidateSubmaps before ICP setInputSource/setInputTarget");
  icp_->setInputSource(cur_norm);
  icp_->setInputTarget(hist_norm);
  debugLoopLog("alignCandidateSubmaps before ICP align");
  icp_->align(aligned, Eigen::Matrix4f::Identity());
  debugLoopLog(std::string("alignCandidateSubmaps after ICP align ") +
               summarizeCloud("aligned", localization::PointCloudPtr(new localization::PointCloud(aligned))));
  *score = icp_->getFitnessScore();
  if (!icp_->hasConverged() || !std::isfinite(*score) || *score > config_.icp_fitness_thresh) {
    return false;
  }

  // 获取 ICP 结果：T_history_current（将当前帧点云对齐到历史帧的变换）
  const Eigen::Matrix4d T_hist_cur = icp_->getFinalTransformation().cast<double>();
  if (!T_hist_cur.array().isFinite().all()) {
    return false;
  }

  // 计算历史关键帧到当前关键帧的相对位姿（在 map 系下）
  // diff_rot = R_history^T * T_hist_cur.R * R_current
  // diff_pos = R_history^T * (T_hist_cur.R * t_current + T_hist_cur.t - t_history)
  const auto& pre = keyframes[history_idx];
  const auto& cur = keyframes[current_idx];
  *diff_rot = pre.pose_map.orientation.normalized().toRotationMatrix().transpose() *
              T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.orientation.normalized().toRotationMatrix();
  *diff_pos = pre.pose_map.orientation.normalized().toRotationMatrix().transpose() *
              (T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.position + T_hist_cur.block<3, 1>(0, 3) - pre.pose_map.position);
  if (!diff_rot->array().isFinite().all() || !diff_pos->array().isFinite().all()) {
    return false;
  }

  return true;
}

/**
 * @brief 检测回环候选
 * @param keyframes 所有关键帧列表
 * @param current_idx 输出当前关键帧索引
 * @param history_idx 输出历史候选关键帧索引
 * @param score 输出 ICP 配准得分
 * @param diff_rot 输出相对旋转
 * @param diff_pos 输出相对平移
 * @return true 检测到有效回环
 *
 * 检测流程：
 * 1. 设置 current_idx 为最新关键帧
 * 2. 检查是否应该检测（shouldDetectLoopForLatestKeyframe）
 * 3. 搜索最佳历史候选（findBestHistoryCandidateIndex）
 * 4. ICP 配准验证（alignCandidateSubmaps）
 */
bool Fastlio2SamLcLoopClosureBackend::detectCandidate(const std::vector<MappingKeyframe>& keyframes,
                                                      int* current_idx,
                                                      int* history_idx,
                                                      double* score,
                                                      Eigen::Matrix3d* diff_rot,
                                                      Eigen::Vector3d* diff_pos) {
  if (!current_idx || !history_idx || !score || !diff_rot || !diff_pos) {
    return false;
  }

  *current_idx = static_cast<int>(keyframes.size()) - 1;
  *history_idx = -1;
  if (!shouldDetectLoopForLatestKeyframe(keyframes, *current_idx)) {
    return false;
  }
  last_loop_check_index_ = *current_idx;

  *history_idx = findBestHistoryCandidateIndex(keyframes, *current_idx);
  if (*history_idx < 0) {
    return false;
  }

  return alignCandidateSubmaps(keyframes, *current_idx, *history_idx, score, diff_rot, diff_pos);
}

/**
 * @brief 添加里程计因子到因子图
 * @param keyframes 所有关键帧列表
 *
 * 为相邻关键帧之间添加 BetweenFactor：
 * - 第 0 帧添加 PriorFactor（强约束，方差 1e-12）
 * - 其余帧添加 BetweenFactor，使用 pose_odom 计算相对位姿
 * - 噪声模型使用配置的 odom_rot_noise 和 odom_pos_noise
 *
 * 注意：使用 pose_odom（里程计原始位姿）而非 pose_map（优化后位姿）
 * 因为里程计因子应该反映前端里程计的原始观测。
 */
void Fastlio2SamLcLoopClosureBackend::addOdomFactors(const std::vector<MappingKeyframe>& keyframes) {
  const int latest_index = static_cast<int>(keyframes.size()) - 1;
  for (int i = previous_index_; i < latest_index; ++i) {
    const auto& p1 = keyframes[i];
    const auto& p2 = keyframes[i + 1];
    if (i == 0) {
      // 首帧添加先验因子（强约束，固定坐标原点）
      initialized_estimate_.insert(i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                                                   gtsam::Point3(p1.pose_odom.position)));
      auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(i,
                                                  gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                                                               gtsam::Point3(p1.pose_odom.position)),
                                                  noise));
    }
    // 插入新关键帧的初始估计
    initialized_estimate_.insert(i + 1,
                                 gtsam::Pose3(gtsam::Rot3(p2.pose_odom.orientation.normalized().toRotationMatrix()),
                                              gtsam::Point3(p2.pose_odom.position)));
    // 计算相邻关键帧间的相对位姿
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
  debugLoopLog("addOdomFactors done");
}

/**
 * @brief 添加回环因子到因子图
 * @param pre_idx 历史关键帧索引
 * @param cur_idx 当前关键帧索引
 * @param score ICP 配准得分
 * @param diff_rot 历史到当前的相对旋转
 * @param diff_pos 历史到当前的相对平移
 *
 * 使用自适应噪声模型：score 越低（配准越好），噪声越小，约束越可信。
 * 若输入参数无效（NaN/Inf），则跳过添加。
 */
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
                                                buildLoopNoiseModel(config_, score)));
  debugLoopLog("addLoopFactor done score=" + std::to_string(score));
}

/**
 * @brief 标记关键帧为已回环
 * @param keyframes 关键帧列表（会被修改）
 * @param history_idx 历史关键帧索引
 * @param current_idx 当前关键帧索引
 *
 * 设置 loop_closed=true 并增加 loop_match_count，用于可视化显示。
 */
void Fastlio2SamLcLoopClosureBackend::markLoopClosed(std::vector<MappingKeyframe>* keyframes,
                                                     int history_idx,
                                                     int current_idx) {
  if (!keyframes) {
    return;
  }
  for (const int idx : {history_idx, current_idx}) {
    if (idx < 0 || idx >= static_cast<int>(keyframes->size())) {
      continue;
    }
    auto& kf = (*keyframes)[idx];
    kf.loop_closed = true;
    kf.loop_match_count += 1;
  }
}

/**
 * @brief 执行 ISAM2 优化并更新关键帧位姿
 * @param keyframes 关键帧列表（会被修改）
 * @param latest_index 本次处理的最新关键帧索引
 *
 * 优化流程：
 * 1. 将新因子和初始估计传入 ISAM2
 * 2. 执行 update（两次，确保收敛）
 * 3. 清空本次的因子和估计（已合并到 ISAM2 内部）
 * 4. 获取优化后的所有位姿
 * 5. 更新关键帧的 pose_map（优化后全局位姿）
 * 6. 标记 optimized=true
 */
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
    (*keyframes)[i].optimized = true;
  }
  debugLoopLog("optimize done");
}

/**
 * @brief 回环检测与后端优化的主入口
 * @param keyframes 关键帧列表（会被修改，优化后位姿更新到 pose_map）
 * @return 回环检测结果
 *
 * 主流程：
 * 1. 检查是否启用回环或 GNSS
 * 2. 添加里程计因子（所有新增的关键帧对）
 * 3. 若启用 GNSS，添加 GNSS 位置因子
 * 4. 若启用回环，检测回环候选：
 *    a. 空间邻近搜索找候选
 *    b. ICP 配准验证
 *    c. 若通过，添加回环因子并优化
 * 5. 若无回环但有新因子，执行纯里程计优化
 * 6. 返回回环统计信息
 */
LoopClosureResult Fastlio2SamLcLoopClosureBackend::update(std::vector<MappingKeyframe>* keyframes) {
  LoopClosureResult out;
  if ((!config_.enable && !config_.gnss.enable) || !keyframes || keyframes->empty()) return out;
  const int latest_index = static_cast<int>(keyframes->size()) - 1;
  latest_loop_pair_ = {-1, -1};

  // 里程计因子：回环与 GNSS 因子都依赖相邻位姿约束（仅 GNSS 时同样需要）
  addOdomFactors(*keyframes);

  // GNSS 位置因子（独立于回环检测流程）
  if (config_.gnss.enable) {
    gnss_optimizer_.update(*keyframes, &graph_, latest_index);
  }

  int current_idx = -1;
  int history_idx = -1;
  double score = -1.0;
  Eigen::Matrix3d diff_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d diff_pos = Eigen::Vector3d::Zero();
  if (config_.enable &&
      detectCandidate(*keyframes, &current_idx, &history_idx, &score, &diff_rot, &diff_pos)) {
    // 检测到有效回环
    latest_loop_pair_ = {history_idx, current_idx};
    loop_history_.emplace_back(history_idx, current_idx);
    markLoopClosed(keyframes, history_idx, current_idx);
    addLoopFactor(history_idx, current_idx, score, diff_rot, diff_pos);
    optimize(keyframes, latest_index);
    out.updated = true;
  } else if (!initialized_estimate_.empty() || graph_.size() > 0) {
    // 无回环但有新因子，执行优化以平滑轨迹
    optimize(keyframes, latest_index);
  }
  out.latest_score = score;
  out.loop_pairs = loop_history_;
  out.loop_count = static_cast<int>(loop_history_.size());
  out.gnss_factor_count = gnss_optimizer_.factorCount();
  out.gnss_rejected_count = gnss_optimizer_.rejectedCount();
  out.gnss_aligned = gnss_optimizer_.aligned();
  return out;
}

}  // namespace slam_mapping