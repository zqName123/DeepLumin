#include "slam_mapping_v2/core/keyframe_manager.hpp"

#include <cmath>

namespace slam_mapping_v2 {

bool KeyframeManager::shouldAdd(const Keyframe& prev, const FrameInput& f,
                                std::uint64_t frame_index) const {
  // 数据有效性
  if (!f.valid || !f.cloud || static_cast<int>(f.cloud->size()) < cfg_.min_points) {
    return false;
  }
  // 首帧强制
  if (prev.id < 0) {
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
  return dist > cfg_.distance_thresh || rot > cfg_.rotation_thresh_rad ||
         (cfg_.time_thresh_sec > 0.0 && dt > cfg_.time_thresh_sec);
}

}  // namespace slam_mapping_v2
