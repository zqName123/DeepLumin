#include "slam_mapping/core/loop_closure_backend.hpp"

namespace slam_mapping {

void LoopClosureBackend::configure(const LoopClosureConfig& cfg) { config_ = cfg; }
void LoopClosureBackend::reset() {}
LoopClosureResult LoopClosureBackend::update(std::vector<MappingKeyframe>*) { return LoopClosureResult(); }
bool LoopClosureBackend::enabled() const { return config_.enable; }
bool LoopClosureBackend::supported() const { return false; }

}  // namespace slam_mapping
