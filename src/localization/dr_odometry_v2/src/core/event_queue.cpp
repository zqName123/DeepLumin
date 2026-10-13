#include "dr_odometry_v2/core/event_queue.hpp"

#include <cmath>

namespace dr_odometry_v2 {

bool EventQueue::push(const Event& event) {
  if (!std::isfinite(event.stamp)) {
    ++invalid_dropped_;
    return false;
  }
  if (static_cast<int>(heap_.size()) >= capacity_) {
    // 溢出：丢最旧（堆顶是最早时刻，小顶语义下需重建——用临时弹出保序代价高，
    // 这里直接丢新事件并计数，容量按最坏 1s 数据配置，默认 512 足够）
    ++overflow_dropped_;
    return false;
  }
  heap_.push(Item{event});
  return true;
}

bool EventQueue::popNext(Event* event) {
  while (!heap_.empty()) {
    Event e = heap_.top().event;
    heap_.pop();
    if (watermark_ >= 0.0 && e.stamp < watermark_) {
      // 位姿观测带扫描戳（处理链延迟晚于水位）：容忍窗内放行，不回退水位，
      // 由 observePose 一阶外推到状态时刻再比对
      const bool pose_late_ok =
          e.type == EventType::kPose && pose_late_tolerance_ > 0.0 &&
          (watermark_ - e.stamp) <= pose_late_tolerance_;
      if (!pose_late_ok) {
        ++reordered_dropped_;  // 迟到事件：时间序已过，丢弃
        continue;
      }
    } else {
      watermark_ = e.stamp;
    }
    if (event != nullptr) {
      *event = e;
    }
    return true;
  }
  return false;
}

void EventQueue::clear() {
  while (!heap_.empty()) {
    heap_.pop();
  }
}

}  // namespace dr_odometry_v2
