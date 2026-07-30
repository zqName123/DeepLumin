#include "slam_mapping/core/keyframe_manager.hpp"

#include <cmath>

namespace slam_mapping {

void KeyframeManager::configure(const KeyframeConfig& cfg) { config_ = cfg; }

void KeyframeManager::reset() {
  keyframes_.clear();
  next_frame_index_ = 0;
  last_keyframe_frame_index_ = 0;
}

bool KeyframeManager::shouldAdd(const localization::OdomResult& odom,
                                const localization::TimestampedPointCloud& scan) const {
  return shouldAddAtFrame(odom, scan, next_frame_index_);
}

bool KeyframeManager::shouldAddAtFrame(const localization::OdomResult& odom,
                                       const localization::TimestampedPointCloud& scan,
                                       std::uint64_t frame_index) const {
  if (!odom.valid || !scan.cloud || static_cast<int>(scan.cloud->size()) < config_.min_points) {
    return false;
  }
  if (keyframes_.empty()) {
    return true;
  }
  if (config_.min_frame_gap > 0) {
    const auto frame_gap = frame_index - last_keyframe_frame_index_;
    if (frame_gap < static_cast<std::uint64_t>(config_.min_frame_gap)) {
      return false;
    }
  }

  localization::Pose current_pose;
  current_pose.timestamp = odom.timestamp;
  current_pose.position = odom.position;
  current_pose.orientation = odom.orientation;
  current_pose.covariance = odom.covariance;
  current_pose.frame_id = odom.frame_id;

  const auto& last = keyframes_.back();
  const double dist = relativeDistance(last.pose_odom, current_pose);
  const double rot = relativeRotationRad(last.pose_odom, current_pose);
  const double dt = odom.timestamp - last.timestamp;
  return dist > config_.distance_thresh || rot > config_.rotation_thresh_rad ||
         (config_.time_thresh_sec > 0.0 && dt > config_.time_thresh_sec);
}

bool KeyframeManager::add(const localization::OdomResult& odom,
                          const localization::TimestampedPointCloud& scan) {
  const auto frame_index = next_frame_index_++;
  if (!shouldAddAtFrame(odom, scan, frame_index)) return false;
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
  kf.source_frame_index = frame_index;
  kf.optimized = false;
  kf.loop_closed = false;
  kf.loop_match_count = 0;
  keyframes_.push_back(kf);
  last_keyframe_frame_index_ = frame_index;
  return true;
}

const std::vector<MappingKeyframe>& KeyframeManager::keyframes() const { return keyframes_; }
std::vector<MappingKeyframe>& KeyframeManager::mutableKeyframes() { return keyframes_; }
const MappingKeyframe* KeyframeManager::latest() const { return keyframes_.empty() ? nullptr : &keyframes_.back(); }
MappingKeyframe* KeyframeManager::latestMutable() { return keyframes_.empty() ? nullptr : &keyframes_.back(); }
int KeyframeManager::latestIndex() const { return keyframes_.empty() ? -1 : static_cast<int>(keyframes_.size()) - 1; }

double KeyframeManager::relativeDistance(const localization::Pose& a, const localization::Pose& b) const {
  return (a.position - b.position).norm();
}

double KeyframeManager::relativeRotationRad(const localization::Pose& a, const localization::Pose& b) const {
  const auto dq = a.orientation.normalized().inverse() * b.orientation.normalized();
  return 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
}

}  // namespace slam_mapping
