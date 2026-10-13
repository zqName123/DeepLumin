#pragma once

/**
 * @file source_registry.hpp
 * @brief 源注册表：源的运行态簿记（纯 C++，零 ROS）
 *
 * 只做簿记，不做决策：
 *   - 每个源的最近样本、健康度、实测周期
 *   - 每个源的 glue（源系 → 虚拟系），由 Arbiter 在换源时写入
 *
 * 决策（选谁、何时换、glue 怎么算）在 Arbiter；运动公式在 MotionProvider。
 */

#include <string>
#include <vector>

#include "local_odom_gateway/core/source_spec.hpp"

namespace local_odom_gateway {

struct SourceSlot {
  OdomSourceSpec spec;
  Pose3d last_pose;              ///< 最近样本（源系）
  Twist last_twist;
  double last_stamp = -1.0;
  Pose3d glue;                   ///< 源系 → 虚拟系（切换时计算；虚拟系确立后恒定）
  bool established = false;      ///< 虚拟系是否已由该源确立
  int confirm_count = 0;         ///< 回切滞回计数

  /// 样本是否在 timeout 内（注意：云活性**不**参与，见设计 §1.2b）
  bool healthy(double now_sec) const {
    return last_stamp >= 0.0 && (now_sec - last_stamp) <= spec.timeout_sec;
  }
  double age(double now_sec) const {
    return last_stamp >= 0.0 ? (now_sec - last_stamp) : -1.0;
  }
};

class SourceRegistry {
 public:
  /// 注册一个源，返回 id（配置顺序）
  SourceId add(const OdomSourceSpec& spec) {
    SourceSlot slot;
    slot.spec = spec;
    slots_.push_back(slot);
    return static_cast<SourceId>(slots_.size() - 1);
  }

  /// 喂样本（源系位姿）
  void onSample(SourceId id, const Pose3d& pose, double stamp, const Twist& twist = Twist()) {
    if (!valid(id) || !isFinite(pose.position) || !isFinite(pose.orientation)) {
      return;
    }
    SourceSlot& s = slots_[id];
    s.last_pose = pose;
    s.last_stamp = stamp;
    if (twist.valid) {
      s.last_twist = twist;
    }
  }

  bool valid(SourceId id) const { return id >= 0 && id < static_cast<SourceId>(slots_.size()); }
  int count() const { return static_cast<int>(slots_.size()); }
  const SourceSlot& slot(SourceId id) const { return slots_[id]; }
  SourceSlot& slot(SourceId id) { return slots_[id]; }

  /// 健康源中优先级最高者（同级取样本更新者）；无健康源 → kInvalidSource
  SourceId bestHealthy(double now_sec) const {
    SourceId best = kInvalidSource;
    for (SourceId i = 0; i < static_cast<SourceId>(slots_.size()); ++i) {
      if (!slots_[i].healthy(now_sec)) {
        continue;
      }
      if (best == kInvalidSource || slots_[i].spec.priority > slots_[best].spec.priority ||
          (slots_[i].spec.priority == slots_[best].spec.priority &&
           slots_[i].last_stamp > slots_[best].last_stamp)) {
        best = i;
      }
    }
    return best;
  }

  /// 优先级最高者（无论健康）——用于识别"主源/增量源"角色
  SourceId highestPriority() const {
    SourceId best = kInvalidSource;
    for (SourceId i = 0; i < static_cast<SourceId>(slots_.size()); ++i) {
      if (best == kInvalidSource || slots_[i].spec.priority > slots_[best].spec.priority) {
        best = i;
      }
    }
    return best;
  }

 private:
  std::vector<SourceSlot> slots_;
};

}  // namespace local_odom_gateway
