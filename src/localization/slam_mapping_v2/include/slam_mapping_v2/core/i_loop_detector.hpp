#pragma once

/**
 * @file i_loop_detector.hpp
 * @brief 回环检测接口（可插拔：默认 kdtree+GICP，未来可换 scan_context 等描述子）
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

#include <utility>
#include <vector>

namespace slam_mapping_v2 {

class ILoopDetector {
 public:
  virtual ~ILoopDetector() = default;

  virtual void configure(const LoopClosureConfig& cfg) = 0;
  virtual void reset() = 0;

  /// 对最新关键帧执行一次回环检测；无有效候选时返回 valid=false
  virtual LoopCandidate detect(const std::vector<Keyframe>& kfs) = 0;

  virtual const std::vector<std::pair<int, int>>& loopPairs() const = 0;
  virtual PointCloudPtr latestCurrentSubmap() const = 0;
  virtual PointCloudPtr latestHistorySubmap() const = 0;
};

}  // namespace slam_mapping_v2
