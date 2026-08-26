#pragma once

/**
 * @file visual_feature_buffer.hpp
 * @brief 线程安全的视觉特征缓冲区
 *
 * 生产者：视觉特征线程（insert）
 * 消费者：后端线程（get → VisualLoopDetector::detect）
 */

#include "slam_mapping_v2/common/types_visual.hpp"

#include <condition_variable>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace slam_mapping_v2 {

/// 线程安全的视觉特征缓冲区
class VisualFeatureBuffer {
 public:
  VisualFeatureBuffer() = default;

  /// 插入单帧
  void insert(const VisualFrame& frame);

  /// 批量插入
  void insertBatch(const std::vector<VisualFrame>& frames);

  /// 获取指定关键帧 ID 的特征
  bool get(int keyframe_id, VisualFrame* out) const;

  /// 批量获取
  std::vector<VisualFrame> getBatch(const std::vector<int>& ids) const;

  /// 获取最近 N 帧
  std::vector<VisualFrame> getRecent(int n) const;

  /// 获取所有可用的 ID
  std::vector<int> availableIds() const;

  /// 等待新帧到达（带超时）
  bool waitForNew(int timeout_ms = 100);

  /// 清空缓冲区
  void clear();

  /// 当前大小
  std::size_t size() const;

  /// 设置缓冲区大小（FIFO 淘汰）
  void setBufferSize(std::size_t size);

 private:
  mutable std::shared_mutex mutex_;
  std::map<int, VisualFrame> frames_;  // keyframe_id → VisualFrame
  std::condition_variable_any cv_;
  int latest_id_ = -1;
  std::size_t buffer_size_ = 100;
};

}  // namespace slam_mapping_v2
