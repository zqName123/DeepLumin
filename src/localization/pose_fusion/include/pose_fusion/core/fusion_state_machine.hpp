#pragma once

#include "pose_fusion/common/types.hpp"

namespace pose_fusion {

class FusionStateMachine {
 public:
  void setState(FusionStateName state) { state_ = state; }
  FusionStateName state() const { return state_; }
  void update(const FusionStatusData& status);

 private:
  FusionStateName state_ = FusionStateName::UNINITIALIZED;
};

}  // namespace pose_fusion
