#pragma once

/**
 * @file keyframe_manager.hpp
 * @brief 关键帧判定（纯函数式，无内部存储，天然线程安全）
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

namespace slam_mapping_v2 {

class KeyframeManager {
 public:
  void configure(const KeyframeConfig& cfg) { cfg_ = cfg; }

  /// 是否应将当前帧设为关键帧（prev.id < 0 表示首帧，强制为真）
  bool shouldAdd(const Keyframe& prev, const FrameInput& f, std::uint64_t frame_index) const;

 private:
  KeyframeConfig cfg_;
};

}  // namespace slam_mapping_v2
