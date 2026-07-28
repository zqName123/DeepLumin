#include "slam_mapping/core/keyframe_manager.hpp"

#include <cmath>

namespace slam_mapping {

void KeyframeManager::configure(const KeyframeConfig& cfg) { config_ = cfg; }

void KeyframeManager::reset() { keyframes_.clear(); }

bool KeyframeManager::shouldAdd(const localization::OdomResult& odom,
                                const localization::TimestampedPointCloud& scan) const {
  if (!odom.valid || !scan.cloud || static_cast<int>(scan.cloud->size()) < config_.min_points) {
    return false;
  }
  if (keyframes_.empty()) {
    return true;
  }
  const auto& last = keyframes_.back();
  const double dist = relativeDistance(last.pose_odom, localization::Pose{odom.timestamp, odom.position, odom.orientation, odom.covariance, odom.frame_id});
  const double rot = relativeRotationRad(last.pose_odom, localization::Pose{odom.timestamp, odom.position, odom.orientation, odom.covariance, odom.frame_id});
  const double dt = odom.timestamp - last.timestamp;
  return dist > config_.distance_thresh || rot > config_.rotation_thresh_rad ||
         (config_.time_thresh_sec > 0.0 && dt > config_.time_thresh_sec);
}

bool KeyframeManager::add(const localization::OdomResult& odom,
                          const localization::TimestampedPointCloud& scan) {
  if (!shouldAdd(odom, scan)) return false;
  MappingKeyframe kf;
  kf.id = static_cast<int>(keyframes_.size());
  kf.timestamp = odom.timestamp;
  kf.pose_odom.timestamp = odom.timestamp;
  kf.pose_odom.position = odom.position;
  kf.pose_odom.orientation = odom.orientation;
  kf.pose_odom.covariance = odom.covariance;
  kf.pose_odom.frame_id = odom.frame_id;
  kf.pose_map = kf.pose_odom;
  kf.cloud_body.reset(new localization::PointCloud(*scan.cloud));
  kf.point_count = scan.cloud->size();
  keyframes_.push_back(kf);
  return true;
}

const std::vector<MappingKeyframe>& KeyframeManager::keyframes() const { return keyframes_; }
std::vector<MappingKeyframe>& KeyframeManager::mutableKeyframes() { return keyframes_; }
const MappingKeyframe* KeyframeManager::latest() const { return keyframes_.empty() ? nullptr : &keyframes_.back(); }

double KeyframeManager::relativeDistance(const localization::Pose& a, const localization::Pose& b) const {
  return (a.position - b.position).norm();
}

double KeyframeManager::relativeRotationRad(const localization::Pose& a, const localization::Pose& b) const {
  const auto dq = a.orientation.normalized().inverse() * b.orientation.normalized();
  return 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
}

}  // namespace slam_mapping
