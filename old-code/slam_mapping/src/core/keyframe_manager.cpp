/**
 * @file keyframe_manager.cpp
 * @brief 关键帧管理器实现
 *
 * 该模块负责 SLAM 系统中的关键帧选取和管理，是回环检测和位姿图优化的基础。
 *
 * 核心功能：
 * - 关键帧选取：基于距离、旋转、时间间隔等条件决定是否将当前帧添加为关键帧
 * - 关键帧存储：保存关键帧的位姿（里程计坐标系和地图坐标系）、点云、时间戳等
 * - 关键帧查询：提供只读和可变的访问接口
 *
 * 关键帧选取策略：
 * 1. 有效性检查：里程计有效、点云非空且点数足够
 * 2. 首帧强制添加：第一帧总是作为关键帧
 * 3. 最小帧间隔：避免关键帧过于密集（可配置）
 * 4. 距离阈值：当前帧与上一关键帧的平移距离超过阈值
 * 5. 旋转阈值：当前帧与上一关键帧的旋转角度超过阈值
 * 6. 时间阈值：距离上一关键帧的时间超过阈值（可配置）
 *
 * 坐标系说明：
 * - pose_odom: 里程计坐标系下的位姿（来自 SLAM 后端）
 * - pose_map: 地图坐标系下的位姿（经回环优化后更新）
 * - cloud_body: 本体坐标系下的点云（用于回环检测和地图构建）
 *
 * @see slam_mapping::KeyframeManager
 * @see slam_mapping::MappingKeyframe
 */

// 模块头文件
#include "slam_mapping/core/keyframe_manager.hpp"

// 标准库
#include <cmath>  // 数学函数（std::atan2, std::abs）

namespace slam_mapping {

/**
 * @brief 配置关键帧管理器
 *
 * @param cfg 关键帧配置参数
 *
 * 配置参数包括：
 * - min_points: 最小点数阈值
 * - min_frame_gap: 最小帧间隔
 * - distance_thresh: 距离阈值（米）
 * - rotation_thresh_rad: 旋转阈值（弧度）
 * - time_thresh_sec: 时间阈值（秒）
 */
void KeyframeManager::configure(const KeyframeConfig& cfg) { config_ = cfg; }

/**
 * @brief 重置关键帧管理器
 *
 * 清空所有关键帧，重置计数器：
 * - keyframes_: 关键帧列表
 * - next_frame_index_: 下一帧索引
 * - last_keyframe_frame_index_: 上一关键帧的帧索引
 */
void KeyframeManager::reset() {
  keyframes_.clear();
  next_frame_index_ = 0;
  last_keyframe_frame_index_ = 0;
}

/**
 * @brief 检查是否应将当前帧添加为关键帧
 *
 * @param odom 当前帧里程计结果
 * @param scan 当前帧点云
 * @return true 应该添加
 * @return false 不应该添加
 *
 * 调用 shouldAddAtFrame，使用内部维护的 next_frame_index_ 作为帧索引。
 */
bool KeyframeManager::shouldAdd(const localization::OdomResult& odom,
                                const localization::TimestampedPointCloud& scan) const {
  return shouldAddAtFrame(odom, scan, next_frame_index_);
}

/**
 * @brief 检查是否应将指定帧添加为关键帧（核心逻辑）
 *
 * @param odom 当前帧里程计结果
 * @param scan 当前帧点云
 * @param frame_index 当前帧索引
 * @return true 应该添加
 * @return false 不应该添加
 *
 * 选取条件（按优先级）：
 * 1. 数据有效性：
 *    - 里程计有效（odom.valid = true）
 *    - 点云非空
 *    - 点数 >= min_points
 * 2. 首帧强制：若关键帧列表为空，直接添加
 * 3. 最小帧间隔：若配置了 min_frame_gap，当前帧与上一关键帧的间隔必须 >= min_frame_gap
 * 4. 几何变化：满足以下任一条件：
 *    - 平移距离 > distance_thresh
 *    - 旋转角度 > rotation_thresh_rad
 *    - 时间差 > time_thresh_sec（若配置了时间阈值）
 */
bool KeyframeManager::shouldAddAtFrame(const localization::OdomResult& odom,
                                       const localization::TimestampedPointCloud& scan,
                                       std::uint64_t frame_index) const {
  // 条件 1: 数据有效性检查
  if (!odom.valid || !scan.cloud || static_cast<int>(scan.cloud->size()) < config_.min_points) {
    return false;
  }

  // 条件 2: 首帧强制添加
  if (keyframes_.empty()) {
    return true;
  }

  // 条件 3: 最小帧间隔检查
  if (config_.min_frame_gap > 0) {
    const auto frame_gap = frame_index - last_keyframe_frame_index_;
    if (frame_gap < static_cast<std::uint64_t>(config_.min_frame_gap)) {
      return false;
    }
  }

  // 构建当前位姿
  localization::Pose current_pose;
  current_pose.timestamp = odom.timestamp;
  current_pose.position = odom.position;
  current_pose.orientation = odom.orientation;
  current_pose.covariance = odom.covariance;
  current_pose.frame_id = odom.frame_id;

  // 获取上一关键帧
  const auto& last = keyframes_.back();

  // 计算相对距离和旋转
  const double dist = relativeDistance(last.pose_odom, current_pose);
  const double rot = relativeRotationRad(last.pose_odom, current_pose);
  const double dt = odom.timestamp - last.timestamp;

  // 条件 4: 几何变化检查
  return dist > config_.distance_thresh ||           // 平移距离超过阈值
         rot > config_.rotation_thresh_rad ||          // 旋转角度超过阈值
         (config_.time_thresh_sec > 0.0 && dt > config_.time_thresh_sec);  // 时间超过阈值
}

/**
 * @brief 添加关键帧
 *
 * @param odom 当前帧里程计结果
 * @param scan 当前帧点云
 * @return true 添加成功
 * @return false 添加失败（不满足选取条件）
 *
 * 添加流程：
 * 1. 递增帧索引
 * 2. 检查是否应该添加
 * 3. 构建关键帧结构
 * 4. 添加到列表
 * 5. 更新上一关键帧索引
 */
bool KeyframeManager::add(const localization::OdomResult& odom,
                          const localization::TimestampedPointCloud& scan) {
  // 获取当前帧索引并递增
  const auto frame_index = next_frame_index_++;

  // 检查是否应该添加
  if (!shouldAddAtFrame(odom, scan, frame_index)) return false;

  // 构建关键帧
  MappingKeyframe kf;
  kf.id = static_cast<int>(keyframes_.size());  // 关键帧 ID（按添加顺序）
  kf.timestamp = odom.timestamp;

  // 里程计坐标系位姿（来自 SLAM 后端）
  kf.pose_odom.timestamp = odom.timestamp;
  kf.pose_odom.position = odom.position;
  kf.pose_odom.orientation = odom.orientation;
  kf.pose_odom.covariance = odom.covariance;
  kf.pose_odom.frame_id = odom.frame_id;

  // 地图坐标系位姿（初始与里程计位姿相同，经回环优化后更新）
  kf.pose_map = kf.pose_odom;

  // 本体坐标系点云（拷贝）
  kf.cloud_body.reset(new localization::PointCloud(*scan.cloud));
  kf.point_count = scan.cloud->size();

  // 源帧索引（用于追踪该关键帧来自哪一帧）
  kf.source_frame_index = frame_index;

  // 回环相关标志（初始状态）
  kf.optimized = false;       // 是否经过位姿图优化
  kf.loop_closed = false;     // 是否参与回环
  kf.loop_match_count = 0;    // 回环匹配计数

  // 添加到列表
  keyframes_.push_back(kf);

  // 更新上一关键帧索引
  last_keyframe_frame_index_ = frame_index;

  return true;
}

// ==================== 查询接口 ====================

/**
 * @brief 获取所有关键帧（只读）
 */
const std::vector<MappingKeyframe>& KeyframeManager::keyframes() const { return keyframes_; }

/**
 * @brief 获取所有关键帧（可变）
 *
 * @note 主要用于回环检测后端修改关键帧的 pose_map
 */
std::vector<MappingKeyframe>& KeyframeManager::mutableKeyframes() { return keyframes_; }

/**
 * @brief 获取最新关键帧（只读）
 * @return nullptr 若关键帧列表为空
 */
const MappingKeyframe* KeyframeManager::latest() const { return keyframes_.empty() ? nullptr : &keyframes_.back(); }

/**
 * @brief 获取最新关键帧（可变）
 * @return nullptr 若关键帧列表为空
 */
MappingKeyframe* KeyframeManager::latestMutable() { return keyframes_.empty() ? nullptr : &keyframes_.back(); }

/**
 * @brief 获取最新关键帧的索引
 * @return -1 若关键帧列表为空
 */
int KeyframeManager::latestIndex() const { return keyframes_.empty() ? -1 : static_cast<int>(keyframes_.size()) - 1; }

// ==================== 私有辅助函数 ====================

/**
 * @brief 计算两个位姿之间的相对平移距离
 *
 * @param a 第一个位姿
 * @param b 第二个位姿
 * @return double 欧氏距离（米）
 *
 * 计算公式：dist = ||a.position - b.position||
 */
double KeyframeManager::relativeDistance(const localization::Pose& a, const localization::Pose& b) const {
  return (a.position - b.position).norm();
}

/**
 * @brief 计算两个位姿之间的相对旋转角度
 *
 * @param a 第一个位姿
 * @param b 第二个位姿
 * @return double 旋转角度（弧度）
 *
 * 计算原理：
 * 1. 计算相对四元数：dq = a.orientation^{-1} * b.orientation
 * 2. 将四元数转换为旋转角度：
 *    - 四元数表示旋转：q = [w, v]，其中 v 是旋转轴，θ 是旋转角
 *    - q = [cos(θ/2), sin(θ/2) * axis]
 *    - 因此：θ = 2 * atan2(||v||, |w|)
 *
 * @note 使用 normalized() 确保四元数单位化，避免数值误差
 */
double KeyframeManager::relativeRotationRad(const localization::Pose& a, const localization::Pose& b) const {
  // 计算相对四元数：从 a 到 b 的旋转
  const auto dq = a.orientation.normalized().inverse() * b.orientation.normalized();

  // 从四元数提取旋转角度：θ = 2 * atan2(||vec||, |w|)
  return 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
}

}  // namespace slam_mapping