#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"
#include "slam_mapping/core/i_loop_closure_backend.hpp"

#include <memory>

namespace slam_mapping {

class LoopClosureBackend {
 public:
  void configure(const LoopClosureConfig& cfg);
  void reset();
  LoopClosureResult update(std::vector<MappingKeyframe>* keyframes);
  localization::PointCloudPtr latestCurrentSubmap() const;
  localization::PointCloudPtr latestHistorySubmap() const;
  std::pair<int, int> latestLoopPair() const;
  bool enabled() const;
  bool supported() const;
  std::string name() const;

 private:
  LoopClosureConfig config_;
  std::shared_ptr<ILoopClosureBackend> impl_;
};

}  // namespace slam_mapping
