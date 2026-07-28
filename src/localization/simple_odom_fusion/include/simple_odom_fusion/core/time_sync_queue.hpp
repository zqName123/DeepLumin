#pragma once

#include "simple_odom_fusion/core/types.hpp"

#include <deque>
#include <optional>

namespace simple_odom_fusion {

class TimeSyncQueue {
 public:
  void configure(std::size_t max_size, double tolerance) {
    max_size_ = max_size;
    tolerance_ = tolerance;
  }

  void pushDr(const OdomSample& sample) {
    push(sample, &dr_queue_);
  }

  void pushSlam(const OdomSample& sample) {
    push(sample, &slam_queue_);
  }

  std::optional<SyncedPair> popSynced() {
    while (!dr_queue_.empty() && !slam_queue_.empty()) {
      const auto& dr = dr_queue_.front();
      const auto& slam = slam_queue_.front();
      const double dt = dr.stamp - slam.stamp;
      if (std::abs(dt) <= tolerance_) {
        SyncedPair pair;
        pair.dr = dr;
        pair.slam = slam;
        pair.stamp = 0.5 * (dr.stamp + slam.stamp);
        dr_queue_.pop_front();
        slam_queue_.pop_front();
        return pair;
      }
      if (dt < 0.0) {
        dr_queue_.pop_front();
      } else {
        slam_queue_.pop_front();
      }
    }
    return std::nullopt;
  }

 private:
  void push(const OdomSample& sample, std::deque<OdomSample>* queue) {
    if (!queue->empty() && sample.stamp < queue->back().stamp) {
      queue->clear();
    }
    queue->push_back(sample);
    while (queue->size() > max_size_) {
      queue->pop_front();
    }
  }

  std::deque<OdomSample> dr_queue_;
  std::deque<OdomSample> slam_queue_;
  std::size_t max_size_ = 300;
  double tolerance_ = 0.03;
};

}  // namespace simple_odom_fusion
