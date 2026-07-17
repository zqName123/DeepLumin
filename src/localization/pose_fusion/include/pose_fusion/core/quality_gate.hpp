#pragma once

#include "pose_fusion/common/types.hpp"

#include <string>

namespace pose_fusion {

class QualityGate {
 public:
  void setConfig(const FusionConfig& config);
  bool validatePoseObservation(const PoseObservation& obs, std::string* reason) const;
  bool validateGlobalMatcher(const PoseObservation& obs, std::string* reason) const;
  bool validateReset(const ResetObservation& obs, std::string* reason) const;
  bool validateDrDelta(const Pose3d& delta, double dt, std::string* reason) const;
  bool validateSlamResidual(const Pose3d& predicted, const Pose3d& observed, std::string* reason) const;
  bool validateGlobalResidual(const Pose3d& current_map_odom,
                              const Pose3d& observed_map_odom,
                              std::string* reason) const;

 private:
  FusionConfig config_;
};

}  // namespace pose_fusion
