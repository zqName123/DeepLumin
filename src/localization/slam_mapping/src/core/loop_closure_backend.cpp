#include "slam_mapping/core/loop_closure_backend.hpp"

#include "slam_mapping/core/loop_closure_fastlio2_backend.hpp"

namespace slam_mapping {

void LoopClosureBackend::configure(const LoopClosureConfig& cfg) {
  config_ = cfg;
  if (config_.type == "fastlio2_sam_lc" || config_.type.empty()) {
    impl_ = std::make_shared<Fastlio2SamLcLoopClosureBackend>();
  } else {
    impl_.reset();
  }
  if (impl_) impl_->configure(cfg);
}

void LoopClosureBackend::reset() {
  if (impl_) impl_->reset();
}

LoopClosureResult LoopClosureBackend::update(std::vector<MappingKeyframe>* keyframes) {
  if (!impl_ || !config_.enable) return LoopClosureResult();
  return impl_->update(keyframes);
}

bool LoopClosureBackend::enabled() const { return config_.enable; }
bool LoopClosureBackend::supported() const { return impl_ && impl_->supported(); }
std::string LoopClosureBackend::name() const { return impl_ ? impl_->name() : std::string(); }

}  // namespace slam_mapping
