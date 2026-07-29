#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

#include <string>
#include <vector>

namespace slam_mapping {

class ILoopClosureBackend {
 public:
  virtual ~ILoopClosureBackend() = default;

  virtual std::string name() const = 0;
  virtual bool supported() const = 0;
  virtual void configure(const LoopClosureConfig& cfg) = 0;
  virtual void reset() = 0;
  virtual LoopClosureResult update(std::vector<MappingKeyframe>* keyframes) = 0;
};

}  // namespace slam_mapping
