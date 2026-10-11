#pragma once

/**
 * @file event_queue.hpp
 * @brief 时间序事件队列（IMU/轮速/GNSS/定位统一入队，按 stamp 升序处理）
 *
 * 时间同步策略（见设计文档 §5）：
 * - IMU 时间为滤波主轴；观测在时间序上"传播到观测时刻再更新"（天然对齐）；
 * - 乱序事件（stamp 已过）丢弃并计数；零戳由适配层兜底；
 * - 队列带容量上限，溢出丢最旧并计数。
 */

#include <cstdint>
#include <queue>
#include <vector>

#include "dr_odometry_v2/core/types.hpp"

namespace dr_odometry_v2 {

enum class EventType { kImu = 0, kWheel, kGnss, kPose };

struct Event {
  EventType type = EventType::kImu;
  double stamp = 0.0;
  ImuSample imu;
  WheelSample wheel;
  GnssSample gnss;
  PoseSample pose;

  static Event makeImu(const ImuSample& s) {
    Event e;
    e.type = EventType::kImu;
    e.stamp = s.stamp;
    e.imu = s;
    return e;
  }
  static Event makeWheel(const WheelSample& s) {
    Event e;
    e.type = EventType::kWheel;
    e.stamp = s.stamp;
    e.wheel = s;
    return e;
  }
  static Event makeGnss(const GnssSample& s) {
    Event e;
    e.type = EventType::kGnss;
    e.stamp = s.stamp;
    e.gnss = s;
    return e;
  }
  static Event makePose(const PoseSample& s) {
    Event e;
    e.type = EventType::kPose;
    e.stamp = s.stamp;
    e.pose = s;
    return e;
  }
};

class EventQueue {
 public:
  explicit EventQueue(int capacity = 512) : capacity_(capacity > 0 ? capacity : 512) {}

  /// 入队；stamp 非有限 → false（invalid++）
  bool push(const Event& event);

  /// 弹出时间最早的事件；空 → false。stamp < 已处理水位 → 丢弃并计数后跳过
  bool popNext(Event* event);

  std::size_t size() const { return heap_.size(); }
  bool empty() const { return heap_.empty(); }
  double processedWatermark() const { return watermark_; }
  std::uint64_t reorderedDropped() const { return reordered_dropped_; }
  std::uint64_t overflowDropped() const { return overflow_dropped_; }
  std::uint64_t invalidDropped() const { return invalid_dropped_; }
  void clear();

 private:
  struct Item {
    Event event;
    bool operator<(const Item& other) const {
      // std::priority_queue 是大顶堆 → 反向比较实现小顶
      return event.stamp > other.event.stamp;
    }
  };

  int capacity_;
  std::priority_queue<Item> heap_;
  double watermark_ = -1.0;
  std::uint64_t reordered_dropped_ = 0;
  std::uint64_t overflow_dropped_ = 0;
  std::uint64_t invalid_dropped_ = 0;
};

}  // namespace dr_odometry_v2
