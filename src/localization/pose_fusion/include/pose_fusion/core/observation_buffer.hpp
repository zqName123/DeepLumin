#pragma once

/**
 * @file observation_buffer.hpp
 * @brief 各观测源“最新一帧”缓存
 *
 * 用途：
 *   1) feed* 时先写入，保证 tick() 惰性初始化能读到数据
 *   2) 保留最近一帧供调试 / 扩展（当前核心主要用 last_*_time_ 做 freshness）
 */

#include "pose_fusion/common/types.hpp"

#include <optional>

namespace pose_fusion {

class ObservationBuffer {
 public:
  void setDr(const PoseObservation& obs) { latest_dr_ = obs; }
  void setSlam(const PoseObservation& obs) { latest_slam_ = obs; }
  void setGlobalMatcher(const PoseObservation& obs) { latest_global_matcher_ = obs; }
  void setRelocalization(const ResetObservation& obs) { latest_relocalization_ = obs; }
  void clearRelocalization() { latest_relocalization_.reset(); }

  const std::optional<PoseObservation>& latestDr() const { return latest_dr_; }
  const std::optional<PoseObservation>& latestSlam() const { return latest_slam_; }
  const std::optional<PoseObservation>& latestGlobalMatcher() const { return latest_global_matcher_; }
  const std::optional<ResetObservation>& latestRelocalization() const { return latest_relocalization_; }

 private:
  std::optional<PoseObservation> latest_dr_;
  std::optional<PoseObservation> latest_slam_;
  std::optional<PoseObservation> latest_global_matcher_;
  std::optional<ResetObservation> latest_relocalization_;
};

}  // namespace pose_fusion
