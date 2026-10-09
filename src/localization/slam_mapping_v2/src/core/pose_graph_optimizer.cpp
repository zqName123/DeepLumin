#include "slam_mapping_v2/core/pose_graph_optimizer.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>

namespace slam_mapping_v2 {

namespace {

// 根据回环检测的 ICP 得分构建自适应噪声模型。
// 核心思想：ICP 得分越差（越不那么可信），噪声 sigma 越大，则位姿图在优化时
// 对这条回环因子的"信任度"越低，避免不可靠的回环约束把整张图带偏。
gtsam::SharedNoiseModel buildLoopNoiseModel(const LoopClosureConfig& cfg, double score) {
  // 得分下限阈值：若未配置则用 1.0 兜底
  const double fitness_th = cfg.icp_fitness_thresh > 1e-9 ? cfg.icp_fitness_thresh : 1.0;
  // 将得分夹到 [loop_score_floor, fitness_th] 区间，防止异常大/小值
  const double clamped_score = std::max(cfg.loop_score_floor, std::min(score, fitness_th));
  // 归一化得分比例 ratio ∈ [0.1, 1.0]；得分越接近阈值 ratio 越大 → 噪声越大
  const double ratio = std::max(0.1, std::min(clamped_score / fitness_th, 1.0));
  // 旋转方向噪声 sigma（弧度级别），随 ratio 与全局噪声缩放因子线性放大
  const double rot_sigma = std::max(1e-6, cfg.loop_rot_noise * ratio * cfg.loop_noise_scale);
  // 平移方向噪声 sigma（米级别）
  const double pos_sigma = std::max(1e-6, cfg.loop_pos_noise * ratio * cfg.loop_noise_scale);
  // 6 维向量：前 3 维为旋转噪声，后 3 维为平移噪声（DPI 惯性/RPY 顺序）→ 对角协方差
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma, pos_sigma, pos_sigma, pos_sigma)
          .finished());
}

}  // namespace

// 初始化 ISAM2 增量优化器并保存配置。
void PoseGraphOptimizer::configure(const LoopClosureConfig& cfg) {
  cfg_ = cfg;
  // ISAM2 参数：当线性化误差超过阈值时触发重线性化，relinearizeSkip 表示每隔几帧重线性化一次
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam2_ = std::make_shared<gtsam::ISAM2>(params);
  reset();
}

// 重置优化器状态：清空因子图、初始估计，并重建 ISAM2 实例。
void PoseGraphOptimizer::reset() {
  previous_index_ = 0;         // 里程计因子增量添加的游标归零
  graph_.resize(0);            // 清空待添加因子
  initialized_estimate_.clear();  // 清空初始线性化估计
  if (isam2_) {
    // 重建全新 ISAM2 实例，彻底丢弃旧的贝叶斯树与增量信息
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
  }
}

// 为 [previous_index_, size-1] 这段"新增"的关键帧增量添加相邻关键帧间的里程计因子。
// 使用 odom 系的位姿差作为因子观测值，保证因子是"相对量"，与全局参考系无关。
void PoseGraphOptimizer::addOdomFactors(const std::vector<Keyframe>& kfs) {
  const int latest_index = static_cast<int>(kfs.size()) - 1;
  // 只处理上次优化之后新出现的关键帧（增量思想，避免重复添加）
  for (int i = previous_index_; i < latest_index; ++i) {
    const auto& p1 = kfs[i];
    const auto& p2 = kfs[i + 1];
    if (i == 0) {
      // 第一个关键帧需要固定住参考系（否则整个图可自由刚体漂移，无法求解）：
      // 1) 把它设为初始估计的起点（odom 位姿）
      initialized_estimate_.insert(
          i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                          gtsam::Point3(p1.pose_odom.position)));
      // 2) 给它加一个"极紧的先验因子"（方差 1e-12 ≈ 0），相当于固锚
      auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
          i, gtsam::Pose3(gtsam::Rot3(p1.pose_odom.orientation.normalized().toRotationMatrix()),
                          gtsam::Point3(p1.pose_odom.position)),
          noise));
    }
    // 为第 i+1 帧提供初始估计（odom 位姿），供非线性求解迭代起步
    initialized_estimate_.insert(
        i + 1, gtsam::Pose3(gtsam::Rot3(p2.pose_odom.orientation.normalized().toRotationMatrix()),
                            gtsam::Point3(p2.pose_odom.position)));
    // 计算 i → i+1 的相对旋转：R12 = R1^T * R2（把 p2 的旋转转到 p1 坐标系）
    const Eigen::Matrix3d R12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                p2.pose_odom.orientation.normalized().toRotationMatrix();
    // 计算 i → i+1 的相对平移：t12 = R1^T * (p2 位置 - p1 位置)（p1 坐标系下的平移）
    const Eigen::Vector3d t12 = p1.pose_odom.orientation.normalized().toRotationMatrix().transpose() *
                                (p2.pose_odom.position - p1.pose_odom.position);
    // 里程计噪声（固定方差）：odom 相对量自带的精度
    auto noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << cfg_.odom_rot_noise, cfg_.odom_rot_noise, cfg_.odom_rot_noise,
         cfg_.odom_pos_noise, cfg_.odom_pos_noise, cfg_.odom_pos_noise).finished());
    // 添加相邻关键帧之间的相对位姿因子（BetweenFactor）
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1,
                                                  gtsam::Pose3(gtsam::Rot3(R12),
                                                               gtsam::Point3(t12)),
                                                  noise));
  }
  previous_index_ = latest_index;  // 推进游标，下次只处理更新的帧
}

// 添加一条回环因子：连接 history_idx 与 current_idx 两个关键帧。
void PoseGraphOptimizer::addLoopFactor(const LoopCandidate& c) {
  // 输入合法性检查：跳过无效、非有限、非正得分、含非有限位姿的回环
  if (!c.valid || !std::isfinite(c.score) || c.score <= 0.0 ||
      !c.diff_rot.array().isFinite().all() || !c.diff_pos.array().isFinite().all()) {
    return;
  }
  // 回环因子是一条"非相邻帧"之间的相对位姿约束，
  // 噪声由 ICP 得分自适应决定（buildLoopNoiseModel：得分越差 → 噪声越大）
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      c.history_idx, c.current_idx,
      gtsam::Pose3(gtsam::Rot3(c.diff_rot), gtsam::Point3(c.diff_pos)),
      buildLoopNoiseModel(cfg_, c.score)));
}

// 核心优化接口：把累积的因子喂给 ISAM2，求解全局一致的位姿，并回写 pose_map。
void PoseGraphOptimizer::optimize(std::vector<Keyframe>* kfs) {
  if (!kfs || !isam2_) return;
  // 无任何新因子（如只有 1 个关键帧）则直接跳过
  if (graph_.size() == 0 && initialized_estimate_.empty()) return;
  // 增量更新：ISAM2 只重算受影响的少部分节点，比全局 BA 高效
  isam2_->update(graph_, initialized_estimate_);
  isam2_->update();  // 第二次 update 触发重线性化/重排序，改善解的质量
  // 一次性因子已交给 ISAM2 内部维护，本地图清空等待下一批增量
  graph_.resize(0);
  initialized_estimate_.clear();
  // 取出优化后的所有位姿估计
  const auto optimized = isam2_->calculateBestEstimate();
  const int latest_index = static_cast<int>(kfs->size()) - 1;
  // 把优化结果回写到每个关键帧的 pose_map（地图系位姿）
  for (int i = 0; i <= latest_index && i < static_cast<int>(kfs->size()); ++i) {
    if (!optimized.exists(i)) continue;  // 该帧未被优化（缺初始估计）则跳过
    const auto pose = optimized.at<gtsam::Pose3>(i);
    // 将 GTSAM Pose3 转回 4x4 齐次变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = pose.rotation().matrix().cast<double>();
    T(0, 3) = pose.translation().x();
    T(1, 3) = pose.translation().y();
    T(2, 3) = pose.translation().z();
    // 更新位置、姿态（归一化四元数）与坐标系标记，并标记该帧已优化
    (*kfs)[i].pose_map.position = T.block<3, 1>(0, 3);
    (*kfs)[i].pose_map.orientation = Quatd(T.block<3, 3>(0, 0)).normalized();
    (*kfs)[i].pose_map.frame_id = (*kfs)[i].pose_odom.frame_id;
    (*kfs)[i].optimized = true;
  }
}

}  // namespace slam_mapping_v2