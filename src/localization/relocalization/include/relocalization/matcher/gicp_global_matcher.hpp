#pragma once

#include <relocalization/core/keyframe_database.hpp>
#include <relocalization/interface/i_global_matcher.hpp>

namespace relocalization {

using GicpConfig = GlobalMatcherConfig;
using RelocalizationResult = MatchResult;

class GicpGlobalMatcher : public IGlobalMatcher {
 public:
  explicit GicpGlobalMatcher(const GicpConfig& config = GicpConfig());

  MatchResult alignCandidates(const CloudConstPtr& source,
                              const std::vector<DescriptorCandidate>& candidates,
                              const KeyframeDatabase& database) const override;
  std::string summary() const override;

 private:
  MatchResult alignOne(const CloudConstPtr& source,
                       const DescriptorCandidate& candidate,
                       const KeyframeDatabase& database) const;

  GicpConfig config_;
};

}  // namespace relocalization
