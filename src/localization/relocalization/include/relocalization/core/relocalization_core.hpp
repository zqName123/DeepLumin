#pragma once

#include <relocalization/core/keyframe_database.hpp>
#include <relocalization/interface/i_descriptor.hpp>
#include <relocalization/interface/i_global_matcher.hpp>

#include <memory>
#include <string>

namespace relocalization {

struct RelocalizationCoreConfig {
  std::string descriptor_type = "scan_context";
  std::string matcher_type = "gicp";
  std::string descriptor_db;
  int top_k = 5;
  int ring_key_candidates = 50;
  KeyframeDatabaseConfig database;
  GlobalMatcherConfig matcher;
};

struct RelocalizationQueryResult {
  std::vector<DescriptorCandidate> candidates;
  MatchResult match;
};

class RelocalizationCore {
 public:
  bool initialize(const RelocalizationCoreConfig& config);
  std::vector<DescriptorCandidate> queryCandidates(const CloudConstPtr& source) const;
  MatchResult matchCandidates(const CloudConstPtr& source,
                              const std::vector<DescriptorCandidate>& candidates) const;
  RelocalizationQueryResult relocalize(const CloudConstPtr& source) const;

  const RelocalizationCoreConfig& config() const { return config_; }
  const KeyframeDatabase& database() const { return database_; }
  const IRelocalizationDescriptor& descriptor() const { return *descriptor_; }
  const IGlobalMatcher& matcher() const { return *matcher_; }

 private:
  RelocalizationCoreConfig config_;
  KeyframeDatabase database_;
  std::unique_ptr<IRelocalizationDescriptor> descriptor_;
  std::unique_ptr<IGlobalMatcher> matcher_;
};

std::unique_ptr<IRelocalizationDescriptor> createDescriptor(const std::string& type);
std::unique_ptr<IGlobalMatcher> createGlobalMatcher(const std::string& type,
                                                    const GlobalMatcherConfig& config);

}  // namespace relocalization
