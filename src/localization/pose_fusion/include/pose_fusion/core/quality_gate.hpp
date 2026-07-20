#pragma once

/**
 * @file quality_gate.hpp
 * @brief 观测质量门控：在融合更新前过滤坏帧 / 跳变 / 低质量匹配
 *
 * 被 PoseFusionCore 在 feed* / update* / reset 路径上调用；
 * 失败时通过 reason 字符串记录到 FusionStatus 的 reject_reasons。
 */

#include "pose_fusion/common/types.hpp"

#include <string>

namespace pose_fusion {

class QualityGate {
 public:
  void setConfig(const FusionConfig& config);

  /** 通用位姿观测：valid、时间戳、有限值、frame/child_frame 一致性 */
  bool validatePoseObservation(const PoseObservation& obs, std::string* reason) const;
  /** Global Matcher：在通用校验之上再看 success/converged/inlier/fitness/map_id */
  bool validateGlobalMatcher(const PoseObservation& obs, std::string* reason) const;
  /** 硬重置：valid、时间戳、有限位姿、帧名 */
  bool validateReset(const ResetObservation& obs, std::string* reason) const;
  /** DR 相邻帧相对运动：dt、平移、航向变化门限 */
  bool validateDrDelta(const Pose3d& delta, double dt, std::string* reason) const;
  /** SLAM 观测相对当前预测的残差门限 */
  bool validateSlamResidual(const Pose3d& predicted, const Pose3d& observed, std::string* reason) const;
  /** 观测到的 map_odom 相对当前 map_odom 的残差门限 */
  bool validateGlobalResidual(const Pose3d& current_map_odom,
                              const Pose3d& observed_map_odom,
                              std::string* reason) const;

 private:
  FusionConfig config_;
};

}  // namespace pose_fusion
