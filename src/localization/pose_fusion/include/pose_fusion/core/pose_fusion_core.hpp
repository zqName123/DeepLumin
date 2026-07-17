#pragma once

#include "pose_fusion/core/fusion_state_machine.hpp"
#include "pose_fusion/core/observation_buffer.hpp"
#include "pose_fusion/core/quality_gate.hpp"

#include <deque>
#include <optional>

namespace pose_fusion {

class PoseFusionCore {
 public:
  bool initialize(const FusionConfig& config);
  void setObserverPolicy(const ObserverPolicyData& policy);

  void feedDrOdom(const PoseObservation& obs);
  void feedSlamOdom(const PoseObservation& obs);
  void feedGlobalMatch(const PoseObservation& obs);
  void feedRelocalizationReset(const ResetObservation& reset);
  void reset(const ResetObservation& reset);

  FusionOutput tick(double now);
  FusionStatusData status(double now) const;

 private:
  void initializeAtCurrentDrOrOrigin(double timestamp);
  void initializeFromSlam(const PoseObservation& obs);
  void propagateWithDr(const PoseObservation& obs);
  void updateWithSlam(const PoseObservation& obs);
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

  std::optional<PoseObservation> last_dr_;
  std::optional<PoseObservation> last_slam_;
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
