#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

namespace slam_mapping {

class LoopClosureBackend {
 public:
  void configure(const LoopClosureConfig& cfg);
  void reset();
  LoopClosureResult update(std::vector<MappingKeyframe>* keyframes);
  bool enabled() const;
  bool supported() const;

 private:
  LoopClosureConfig config_;
};

}  // namespace slam_mapping
