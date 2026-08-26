#pragma once

#include "simple_odom_fusion/core/time_sync_queue.hpp"

#include <optional>

namespace simple_odom_fusion {

class SimpleFusionCore {
 public:
  void initialize(const FusionConfig& config);
  void feedDr(const OdomSample& sample);
  void feedSlam(const OdomSample& sample);
  void resetWithMapBase(const GlobalObservation& observation);
  void correctWithGlobalMatcher(const GlobalObservation& observation);
  std::optional<FusionOutput> update();
  FusionOutput output(double stamp) const;
  bool initialized() const { return state_.initialized; }

 private:
  bool validDelta(const Pose3d& delta) const;
  void initializeLocalIfNeeded(const SyncedPair& pair);
  void propagateLocal(const SyncedPair& pair);
  void resetMapOdomFromMapBase(const Pose3d& map_base);

  FusionConfig config_;
  TimeSyncQueue sync_;
  FusionState state_;
  double last_global_correction_stamp_ = -1.0;
};

}  // namespace simple_odom_fusion
