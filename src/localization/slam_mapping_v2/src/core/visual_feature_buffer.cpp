/**
 * @file visual_feature_buffer.cpp
 * @brief 视觉特征缓冲区实现
 */

#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

#include <algorithm>


namespace slam_mapping_v2 {

void VisualFeatureBuffer::insert(const VisualFrame& frame) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  frames_[frame.keyframe_id] = frame;
  // latest_id_ = std::max(latest_id_, frame.keyframe_id);

  // // FIFO 淘汰：超过 buffer_size 时移除最小 keyframe_id 的帧
  // while (frames_.size() > buffer_size_) {
  //   auto it = frames_.begin();  // map 按 key 升序，begin() 是最小的
  //   ROS_DEBUG("[VisualFeatureBuffer] Evicting frame id=%d (buffer full)", it->first);
  //   frames_.erase(it);
  // }

  // cv_.notify_all();
}

void VisualFeatureBuffer::insertBatch(const std::vector<VisualFrame>& frames) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  for (const auto& frame : frames) {
    frames_[frame.keyframe_id] = frame;
    latest_id_ = std::max(latest_id_, frame.keyframe_id);
  }

  // FIFO 淘汰
  // while (frames_.size() > buffer_size_) {
  //   frames_.erase(frames_.begin());
  // }

  // cv_.notify_all();
}

bool VisualFeatureBuffer::get(int keyframe_id, VisualFrame* out) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  auto it = frames_.find(keyframe_id);
  if (it == frames_.end()) {
    return false;
  }

  if (out) {
    *out = it->second;
  }
  return true;
}

std::vector<VisualFrame> VisualFeatureBuffer::getBatch(
    const std::vector<int>& ids) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::vector<VisualFrame> result;
  result.reserve(ids.size());

  for (int id : ids) {
    auto it = frames_.find(id);
    if (it != frames_.end()) {
      result.push_back(it->second);
    }
  }

  return result;
}

std::vector<VisualFrame> VisualFeatureBuffer::getRecent(int n) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::vector<VisualFrame> result;
  result.reserve(std::min(static_cast<size_t>(n), frames_.size()));

  // 从最新的开始取
  auto it = frames_.rbegin();
  for (int i = 0; i < n && it != frames_.rend(); ++i, ++it) {
    result.push_back(it->second);
  }

  return result;
}

std::vector<int> VisualFeatureBuffer::availableIds() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::vector<int> ids;
  ids.reserve(frames_.size());

  for (const auto& [id, _] : frames_) {
    ids.push_back(id);
  }

  return ids;
}

bool VisualFeatureBuffer::waitForNew(int timeout_ms) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  int prev_latest = latest_id_;
  return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                      [this, prev_latest]() { return latest_id_ > prev_latest; });
}

void VisualFeatureBuffer::clear() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  frames_.clear();
  latest_id_ = -1;
}

std::size_t VisualFeatureBuffer::size() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return frames_.size();
}

void VisualFeatureBuffer::setBufferSize(std::size_t size) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_size_ = size;

  // 立即淘汰多余的帧
  while (frames_.size() > buffer_size_) {
    frames_.erase(frames_.begin());
  }
}

}  // namespace slam_mapping_v2
