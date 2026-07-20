#pragma once

/**
 * @file fusion_state_machine.hpp
 * @brief 融合运行状态机：根据各观测源新鲜度推导当前模式名
 *
 * 优先级（已初始化且输出有效时）：
 *   MAP_CONSTRAINED > LOCAL_FUSION > DR_ONLY > DEGRADED
 */

#include "pose_fusion/common/types.hpp"

namespace pose_fusion {

class FusionStateMachine {
 public:
  void setState(FusionStateName state) { state_ = state; }
  FusionStateName state() const { return state_; }
  /** 由 FusionStatusData 中的 initialized / 各源 available 标志更新 state_ */
  void update(const FusionStatusData& status);

 private:
  FusionStateName state_ = FusionStateName::UNINITIALIZED;
};

}  // namespace pose_fusion
