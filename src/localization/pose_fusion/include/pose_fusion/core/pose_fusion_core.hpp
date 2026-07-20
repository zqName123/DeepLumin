#pragma once

/**
 * @file pose_fusion_core.hpp
 * @brief 位姿融合内核（无 ROS 依赖）
 *
 * 状态分解：
 *   FusionState.odom_base  — 局部里程计位姿（odom→base），由 DR 递推，SLAM 平滑纠偏
 *   FusionState.map_odom   — 全局外参（map→odom），由 Global Matcher 平滑 / 重定位硬置
 *   输出 map_base = map_odom ∘ odom_base
 *
 * 调用约定：
 *   initialize() 一次 → 异步 feed*() 写入观测 → 定频 tick()/status() 读出
 */

#include "pose_fusion/core/fusion_state_machine.hpp"
#include "pose_fusion/core/observation_buffer.hpp"
#include "pose_fusion/core/quality_gate.hpp"

#include <deque>
#include <optional>

namespace pose_fusion {

class PoseFusionCore {
 public:
  /** 注入配置，重置内部状态为 UNINITIALIZED */
  bool initialize(const FusionConfig& config);
  /** 更新 localization_manager 下发的观测策略（开关/权重） */
  void setObserverPolicy(const ObserverPolicyData& policy);

  /** 异步喂入观测；内部先缓冲，再按策略与质量门决定是否更新状态 */
  void feedDrOdom(const PoseObservation& obs);
  void feedSlamOdom(const PoseObservation& obs);
  void feedGlobalMatch(const PoseObservation& obs);
  void feedRelocalizationReset(const ResetObservation& reset);
  /** 硬重置：map_odom ← map_base，odom_base ← I（用于 initialpose / 重定位） */
  void reset(const ResetObservation& reset);

  /**
   * 定频节拍：惰性初始化（若尚未 init）→ 刷新状态名 → 组装 FusionOutput。
   * 注意：DR/SLAM/GM 的状态更新发生在 feed* 回调中，tick 主要负责输出与状态机。
   */
  FusionOutput tick(double now);
  /** 诊断快照：各源 age、是否可用、最近 used/rejected 列表 */
  FusionStatusData status(double now) const;

 private:
  /** DR 初值：map_odom=I, odom_base=I → DR_ONLY（map≈odom 冷启动） */
  void initializeAtCurrentDrOrOrigin(double timestamp);
  /** SLAM 初值：odom_base=slam.pose, map_odom=I → LOCAL_FUSION */
  void initializeFromSlam(const PoseObservation& obs);
  /** 用相邻 DR 相对位姿 delta 递推 odom_base，并累加过程噪声 */
  void propagateWithDr(const PoseObservation& obs);
  /** 将 SLAM 对齐到当前融合局部系后做指数平滑 */
  void updateWithSlam(const PoseObservation& obs);
  /** 由 GM 的 map_base 反推 map_odom 观测，再质量加权平滑 */
  void updateWithGlobalMatcher(const PoseObservation& obs);
  void recordUsed(ObservationSource source, const std::string& detail);
  void recordRejected(ObservationSource source, const std::string& reason);
  void trimDiagnostics();
  FusionOutput buildOutput(double now) const;
  void updateStateName(double now);
  static Mat6d initialCovariance(const InitializationConfig& cfg);

  FusionConfig config_;
  ObserverPolicyData policy_;
  QualityGate gate_;
  ObservationBuffer buffer_;
  FusionStateMachine state_machine_;
  FusionState state_;
  std::string current_map_id_ = "default";
  bool initialized_config_ = false;

  std::optional<PoseObservation> last_dr_;   ///< 上一帧 DR，用于算相对 delta
  std::optional<PoseObservation> last_slam_;
  /**
   * 融合局部系相对 SLAM 局部系的固定外参：
   *   fusion_odom_from_slam_odom_ = odom_base ∘ inv(slam_pose)
   * 首次 SLAM 时锁定，之后用它把 SLAM 位姿映射到融合 odom 系再平滑。
   */
  std::optional<Pose3d> fusion_odom_from_slam_odom_;
  double last_dr_time_ = 0.0;
  double last_slam_time_ = 0.0;
  double last_global_matcher_time_ = 0.0;
  double last_relocalization_time_ = 0.0;

  mutable std::deque<std::string> used_sources_;
  mutable std::deque<std::string> rejected_sources_;
  mutable std::deque<std::string> reject_reasons_;
};

}  // namespace pose_fusion
