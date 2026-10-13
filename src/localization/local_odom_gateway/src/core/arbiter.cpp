#include "local_odom_gateway/core/arbiter.hpp"

#include <algorithm>

namespace local_odom_gateway {

// ============================ 样本入口 ============================

SourceId Arbiter::addSource(const OdomSourceSpec& spec) { return registry_.add(spec); }

void Arbiter::onSample(SourceId id, const Pose3d& pose, double stamp, const Twist& twist) {
  if (!registry_.valid(id)) {
    return;
  }
  registry_.onSample(id, pose, stamp, twist);
  const SourceSlot& s = registry_.slot(id);

  if (s.spec.role == SourceRole::kRelative) {
    // 相对源只贡献增量：Δ = D(t_a)⁻¹⊕D(t) 与世界系无关，源系即可
    motion_->onIncrementSample(pose, stamp);
    return;
  }

  // 绝对源
  if (id == anchor_id_) {
    // 长间隔后到的样本 = 判死后的恢复：先悬挂，走滞回确认（不依赖 tick 时序）
    if (!anchor_suspended_ && motion_ != nullptr && motion_->hasAnchor() &&
        (stamp - motion_->anchor().stamp) > motion_->deadLimit()) {
      anchor_suspended_ = true;
      pending_id_ = kInvalidSource;
      confirm_count_ = 0;
    }
    if (!anchor_suspended_) {
      motion_->onAnchorSample(compose(s.glue, pose), stamp);
    }
    // 悬挂期：样本只进 registry，等 arbitrate() 确认后重对齐
    return;
  }
  if (anchor_id_ == kInvalidSource) {
    // 冷启动：虚拟系直接定为该源系（此时 map_odom 未建立，无连续性可保）
    adoptAnchor(id, pose, stamp, /*establish=*/true, 0.0);
  }
  // 其它绝对源：留待 arbitrate() 的滞回判定
}

// ============================ 仲裁 ============================

void Arbiter::arbitrate(double now_sec) {
  // ① 主源判死 → 悬挂锚（冻结，运动由增量续推）
  if (!anchor_suspended_ && anchor_id_ != kInvalidSource && motion_ != nullptr) {
    if (motion_->phase(now_sec) == Phase::kOnBackup) {
      anchor_suspended_ = true;
      pending_id_ = kInvalidSource;
      confirm_count_ = 0;
    }
  }

  // ② 目标锚源 = 健康的绝对源中优先级最高者
  SourceId target = kInvalidSource;
  for (SourceId i = 0; i < registry_.count(); ++i) {
    const SourceSlot& s = registry_.slot(i);
    if (s.spec.role != SourceRole::kAbsolute || !s.healthy(now_sec)) {
      continue;
    }
    if (target == kInvalidSource || s.spec.priority > registry_.slot(target).spec.priority) {
      target = i;
    }
  }
  if (target == kInvalidSource) {
    pending_id_ = kInvalidSource;
    confirm_count_ = 0;
    return;  // 无绝对源可用 → 锚保持冻结（相位 kOnBackup）
  }

  const SourceSlot& t = registry_.slot(target);

  // ③ 同一锚源：未悬挂则维持；悬挂则走恢复滞回
  if (target == anchor_id_) {
    if (!anchor_suspended_) {
      pending_id_ = kInvalidSource;
      confirm_count_ = 0;
      return;
    }
    if (pending_id_ != target) {
      pending_id_ = target;
      confirm_count_ = 0;
    }
    int need = std::max(1, t.spec.switch_back_confirm);
    if (flapGuardTripped(now_sec)) {
      need *= 2;
    }
    if (++confirm_count_ >= need) {
      // 恢复：glue 重对齐（吸收 DR 累积误差）→ 锚恢复更新
      pending_id_ = kInvalidSource;
      confirm_count_ = 0;
      anchor_suspended_ = false;
      adoptAnchor(target, t.last_pose, t.last_stamp, /*establish=*/false, now_sec);
      recordSwitch(now_sec);
    }
    return;
  }

  // ④ 另一绝对源竞争
  const bool upgrade =
      anchor_id_ != kInvalidSource && t.spec.priority > registry_.slot(anchor_id_).spec.priority;
  if (upgrade) {
    // 更高优先级源出现：立即切（更好源就绪）
    pending_id_ = kInvalidSource;
    confirm_count_ = 0;
    anchor_suspended_ = false;
    adoptAnchor(target, t.last_pose, t.last_stamp, /*establish=*/false, now_sec);
    recordSwitch(now_sec);
    return;
  }

  // 降级源接管：滞回确认，防断续流翻转
  if (pending_id_ != target) {
    pending_id_ = target;
    confirm_count_ = 0;
  }
  int need = std::max(1, t.spec.switch_back_confirm);
  if (flapGuardTripped(now_sec)) {
    need *= 2;
  }
  if (++confirm_count_ >= need) {
    pending_id_ = kInvalidSource;
    confirm_count_ = 0;
    anchor_suspended_ = false;
    adoptAnchor(target, t.last_pose, t.last_stamp, /*establish=*/false, now_sec);
    recordSwitch(now_sec);
  }
}

// ============================ 锚源易主 ============================

void Arbiter::adoptAnchor(SourceId id, const Pose3d& raw_pose, double stamp, bool establish,
                          double now_sec) {
  SourceSlot& s = registry_.slot(id);
  if (establish) {
    s.glue = identity();  // 虚拟系 = 该源系
  } else {
    // 连续性不变式：glue_new = unified(t_sw⁻) ⊕ T_new(t_sw)⁻¹
    Sample before;
    if (motion_->sample(&before, now_sec)) {
      s.glue = compose(before.motion, inverse(raw_pose));
    } else {
      s.glue = identity();
    }
  }
  s.established = true;
  anchor_id_ = id;
  motion_->onAnchorSample(compose(s.glue, raw_pose), stamp);
}

// ============================ 翻转保护 ============================

bool Arbiter::flapGuardTripped(double now_sec) const {
  if (config_.flap_guard_count <= 0) {
    return false;
  }
  int recent = 0;
  for (double t : switch_times_) {
    if (now_sec - t <= config_.flap_window_sec) {
      ++recent;
    }
  }
  return recent >= config_.flap_guard_count;
}

void Arbiter::recordSwitch(double now_sec) {
  switch_times_.push_back(now_sec);
  if (switch_times_.size() > 32) {
    switch_times_.erase(switch_times_.begin());
  }
  ++switch_count_;
}

}  // namespace local_odom_gateway
