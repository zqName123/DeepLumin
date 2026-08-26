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
  // 回环或 GNSS 任一启用时进入后端（GNSS 优化可独立于回环检测工作）
  if (!impl_ || (!config_.enable && !config_.gnss.enable)) return LoopClosureResult();
  return impl_->update(keyframes);
}

localization::PointCloudPtr LoopClosureBackend::latestCurrentSubmap() const {
  return impl_ ? impl_->latestCurrentSubmap() : localization::PointCloudPtr();
}

localization::PointCloudPtr LoopClosureBackend::latestHistorySubmap() const {
  return impl_ ? impl_->latestHistorySubmap() : localization::PointCloudPtr();
}

std::pair<int, int> LoopClosureBackend::latestLoopPair() const {
  return impl_ ? impl_->latestLoopPair() : std::pair<int, int>{-1, -1};
}

std::vector<Eigen::Vector3d> LoopClosureBackend::gnssFactorPositions() const {
  return impl_ ? impl_->gnssFactorPositions() : std::vector<Eigen::Vector3d>();
}

std::vector<Eigen::Vector3d> LoopClosureBackend::gnssRejectedPositions() const {
  return impl_ ? impl_->gnssRejectedPositions() : std::vector<Eigen::Vector3d>();
}

std::vector<std::pair<int, Eigen::Vector3d>> LoopClosureBackend::gnssAcceptedFactors() const {
  return impl_ ? impl_->gnssAcceptedFactors() : std::vector<std::pair<int, Eigen::Vector3d>>();
}

std::vector<std::pair<int, Eigen::Vector3d>> LoopClosureBackend::gnssRejectedFactors() const {
  return impl_ ? impl_->gnssRejectedFactors() : std::vector<std::pair<int, Eigen::Vector3d>>();
}

bool LoopClosureBackend::gnssMapEnuTransform(Eigen::Matrix3d* rot,
                                             Eigen::Vector3d* trans) const {
  return impl_ ? impl_->gnssMapEnuTransform(rot, trans) : false;
}

bool LoopClosureBackend::gnssEnuOrigin(double* lat_deg, double* lon_deg,
                                       double* alt) const {
  return impl_ ? impl_->gnssEnuOrigin(lat_deg, lon_deg, alt) : false;
}

bool LoopClosureBackend::enabled() const { return config_.enable; }
bool LoopClosureBackend::supported() const { return impl_ && impl_->supported(); }
std::string LoopClosureBackend::name() const { return impl_ ? impl_->name() : std::string(); }

}  // namespace slam_mapping
