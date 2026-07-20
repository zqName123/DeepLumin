/**
 * @file fusion_state_machine.cpp
 * @brief 状态名仅用于诊断与 FusionStatus 展示，不直接改变滤波增益
 */

#include "pose_fusion/core/fusion_state_machine.hpp"

namespace pose_fusion {

void FusionStateMachine::update(const FusionStatusData& status) {
  if (!status.initialized) {
    state_ = FusionStateName::UNINITIALIZED;
    return;
  }
  if (!status.output_valid) {
    state_ = FusionStateName::FAILURE;
    return;
  }
  // 有全局匹配 → 地图约束；否则有 SLAM → 局部融合；否则有 DR → 纯推算；都无 → 降级
  if (status.is_global_matcher_available) {
    state_ = FusionStateName::MAP_CONSTRAINED;
  } else if (status.is_slam_available) {
    state_ = FusionStateName::LOCAL_FUSION;
  } else if (status.is_dr_available) {
    state_ = FusionStateName::DR_ONLY;
  } else {
    state_ = FusionStateName::DEGRADED;
  }
}

}  // namespace pose_fusion
