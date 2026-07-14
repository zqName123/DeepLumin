#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/descriptor/scan_context_descriptor.hpp>
#include <relocalization/matcher/gicp_global_matcher.hpp>

#include <ros/ros.h>

namespace relocalization {

std::unique_ptr<IRelocalizationDescriptor> createDescriptor(const std::string& type) {
  if (type == "scan_context" || type.empty()) {
    return std::unique_ptr<IRelocalizationDescriptor>(new ScanContextDescriptor());
  }
  return nullptr;
}

std::unique_ptr<IGlobalMatcher> createGlobalMatcher(const std::string& type,
                                                    const GlobalMatcherConfig& config) {
  if (type == "gicp" || type.empty()) {
    return std::unique_ptr<IGlobalMatcher>(new GicpGlobalMatcher(config));
  }
  return nullptr;
}

bool RelocalizationCore::initialize(const RelocalizationCoreConfig& config) {
  config_ = config;
  if (!database_.load(config_.database)) {
    ROS_ERROR("[relocalization] failed to load keyframe database");
    return false;
  }

  descriptor_ = createDescriptor(config_.descriptor_type);
  if (!descriptor_) {
    ROS_ERROR("[relocalization] unsupported descriptor_type: %s", config_.descriptor_type.c_str());
    return false;
  }
  if (!descriptor_->load(config_.descriptor_db)) {
    ROS_ERROR_STREAM("[relocalization] failed to load descriptor database: " << config_.descriptor_db);
    return false;
  }
  descriptor_->setRingKeyCandidates(config_.ring_key_candidates);

  matcher_ = createGlobalMatcher(config_.matcher_type, config_.matcher);
  if (!matcher_) {
    ROS_ERROR("[relocalization] unsupported matcher_type: %s", config_.matcher_type.c_str());
    return false;
  }

  ROS_INFO_STREAM("[relocalization] descriptor: " << descriptor_->summary());
  ROS_INFO_STREAM("[relocalization] matcher: " << matcher_->summary());
  return true;
}

std::vector<DescriptorCandidate> RelocalizationCore::queryCandidates(const CloudConstPtr& source) const {
  if (!descriptor_ || !source || source->empty()) {
    return {};
  }
  const Eigen::MatrixXd desc = descriptor_->makeDescriptor(source);
  return descriptor_->query(desc, config_.top_k);
}

MatchResult RelocalizationCore::matchCandidates(
    const CloudConstPtr& source,
    const std::vector<DescriptorCandidate>& candidates) const {
  if (!matcher_ || !source || source->empty() || candidates.empty()) {
    return {};
  }
  return matcher_->alignCandidates(source, candidates, database_);
}

RelocalizationQueryResult RelocalizationCore::relocalize(const CloudConstPtr& source) const {
  RelocalizationQueryResult out;
  out.candidates = queryCandidates(source);
  out.match = matchCandidates(source, out.candidates);
  return out;
}

}  // namespace relocalization
