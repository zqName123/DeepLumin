#include "slam_mapping_v2/core/keyframe_manager.hpp"

#include <cmath>
#include <iostream>

namespace slam_mapping_v2 {

bool KeyframeManager::shouldAdd(const Keyframe& prev, const FrameInput& f,
                                std::uint64_t frame_index) const {
  // 数据有效性
  if (!f.valid || !f.cloud || static_cast<int>(f.cloud->size()) < cfg_.min_points) {
    return false;
  }
  // 首帧强制
  if (prev.id < 0) {
    std::cout << "[KeyframeManager] First frame, force add" << std::endl;
    return true;
  }
  // 最小帧间隔
  if (cfg_.min_frame_gap > 0 &&
      frame_index - prev.source_frame < static_cast<std::uint64_t>(cfg_.min_frame_gap)) {
    return false;
  }
  // 几何变化
  const double dist = (f.position - prev.pose_odom.position).norm();
  const double rot = prev.pose_odom.orientation.angularDistance(f.orientation);
  const double dt = f.timestamp - prev.timestamp;
  const bool add_by_dist = dist > cfg_.distance_thresh;
  const bool add_by_rot = cfg_.rotation_thresh_rad > 0.0 && rot > cfg_.rotation_thresh_rad;
  const bool add_by_time = cfg_.time_thresh_sec > 0.0 && dt > cfg_.time_thresh_sec;
  if (add_by_dist || add_by_rot || add_by_time) {
    std::cout << "[KeyframeManager] Add keyframe: frame=" << frame_index
              << " dist=" << dist << "/" << cfg_.distance_thresh
              << " rot=" << rot << "/" << cfg_.rotation_thresh_rad
              << " dt=" << dt << "/" << cfg_.time_thresh_sec
              << " reason=" << (add_by_dist ? "DIST" : (add_by_rot ? "ROT" : "TIME"))
              << std::endl;
  }
  return add_by_dist || add_by_rot || add_by_time;
}

}  // namespace slam_mapping_v2
