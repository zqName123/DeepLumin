#include "localization_v3/core/local_odom_source.hpp"

#include <algorithm>
#include <cmath>

namespace localization_v3 {

// ============================ 样本输入 ============================

void LocalOdomSource::onSlam(const Pose3d& pose, double stamp) {
  if (!isFinite(pose.position) || !isFinite(pose.orientation)) {
    return;
  }
  last_slam_ = pose;
  last_slam_stamp_ = stamp;
}

void LocalOdomSource::onDr(const Pose3d& pose, double stamp) {
  if (!isFinite(pose.position) || !isFinite(pose.orientation)) {
    return;
  }
  last_dr_ = pose;
  last_dr_stamp_ = stamp;
  pushDrHistory(pose, stamp);
}

// ============================ DR 历史（接管锚定配对用） ============================

void LocalOdomSource::pushDrHistory(const Pose3d& pose, double stamp) {
  if (!dr_history_.empty() && stamp <= dr_history_.back().stamp) {
    return;  // 乱序样本：历史必须单调，否则插值区间失效
  }
  dr_history_.push_back({stamp, pose});
  while (!dr_history_.empty() &&
         (stamp - dr_history_.front().stamp) > config_.dr_history_sec) {
    dr_history_.pop_front();
  }
}

bool LocalOdomSource::drPoseAt(double stamp, Pose3d* out) const {
  if (out == nullptr || dr_history_.empty()) {
    return false;
  }
  // 找第一个 stamp ≥ target 的样本，与前一个样本构成插值区间
  std::size_t hi = 0;
  while (hi < dr_history_.size() && dr_history_[hi].stamp < stamp) {
    ++hi;
  }
  if (hi == 0) {
    *out = dr_history_.front().pose;  // 早于历史覆盖：取最早（误差=历史窗外的漂移）
    return true;
  }
  if (hi >= dr_history_.size()) {
    *out = dr_history_.back().pose;  // 晚于历史覆盖：取最新
    return true;
  }
  const TimedPose& a = dr_history_[hi - 1];
  const TimedPose& b = dr_history_[hi];
  const double dt = b.stamp - a.stamp;
  if (dt <= 1e-9) {
    *out = b.pose;
    return true;
  }
  const double ratio = (stamp - a.stamp) / dt;
  out->position = a.pose.position + ratio * (b.pose.position - a.pose.position);
  out->orientation = a.pose.orientation.slerp(ratio, b.pose.orientation).normalized();
  return true;
}

void LocalOdomSource::onSlamCloud(double stamp) {
  if (stamp >= 0.0 && std::isfinite(stamp)) {
    last_slam_cloud_stamp_ = stamp;  // 0 也是合法时间戳（会话起点）
  }
}

// ============================ 源可用性 ============================

bool LocalOdomSource::slamUsable(double now_sec) const {
  const bool odom_fresh =
      last_slam_stamp_ >= 0.0 && (now_sec - last_slam_stamp_) <= config_.slam_timeout_sec;
  if (!odom_fresh) {
    return false;
  }
  if (!config_.slam_require_cloud) {
    return true;
  }
  // 部分故障防护：里程计活着但注册云断了 → 源不可用（GM 无云可配）
  return last_slam_cloud_stamp_ >= 0.0 &&
         (now_sec - last_slam_cloud_stamp_) <= config_.slam_cloud_timeout_sec;
}

// ============================ 切换簿记 ============================

void LocalOdomSource::adoptSlam() {
  active_ = Source::kSlam;
  slam_confirm_count_ = 0;
}

void LocalOdomSource::adoptDr() {
  active_ = Source::kDr;
  slam_confirm_count_ = 0;
}

bool LocalOdomSource::flapGuardTripped(double now_sec) const {
  if (config_.flap_guard_count <= 0) {
    return false;  // 0 = 关闭翻转保护
  }
  int recent = 0;
  for (double t : switch_times_) {
    if (now_sec - t <= config_.flap_window_sec) {
      ++recent;
    }
  }
  return recent >= config_.flap_guard_count;
}

void LocalOdomSource::recordSwitch(double now_sec) {
  switch_times_.push_back(now_sec);
  if (switch_times_.size() > 32) {
    switch_times_.erase(switch_times_.begin());
  }
  ++switch_count_;
}

// ============================ 仲裁 + 取样 ============================

bool LocalOdomSource::purePose(Pose3d* out) const {
  if (out == nullptr) {
    return false;
  }
  switch (active_) {
    case Source::kSlam:
      *out = compose(glue_slam_, last_slam_);
      return true;
    case Source::kDr:
      *out = compose(glue_dr_, last_dr_);
      return true;
    case Source::kNone:
    default:
      return false;
  }
}

bool LocalOdomSource::sample(Pose3d* out, double now_sec) {
  if (out == nullptr) {
    return false;
  }
  const bool slam_ok = slamUsable(now_sec);
  const bool dr_fresh =
      last_dr_stamp_ >= 0.0 && (now_sec - last_dr_stamp_) <= config_.dr_timeout_sec;

  // ---- ① 切换前输出（单一纯源）：连续性不变式的基准 ----
  Pose3d before;
  purePose(&before);

  // ---- ② 仲裁（可能换源；glue_new = before ⊕ T_new(t_sw)⁻¹ 保证输出恒等）----
  switch (active_) {
    case Source::kNone: {
      // 首源：虚拟系即源系，无历史输出需要连续
      if (slam_ok) {
        glue_slam_ = identity();  // 虚拟系定为 LIO 系
        slam_frame_established_ = true;
        adoptSlam();
      } else if (dr_fresh) {
        glue_dr_ = identity();  // LIO 未就绪：临时以 DR 系为虚拟系（保活）
        adoptDr();
      } else {
        return false;
      }
      break;
    }

    case Source::kSlam: {
      if (slam_ok) {
        slam_confirm_count_ = 0;
      } else if (dr_fresh) {
        // 接管锚定（运动量不丢）：glue_dr = T0 ⊕ D(t0)⁻¹
        // t0/T0 = 最后 SLAM 样本（before）；D(t0) 按样本戳插值
        // ⇒ 输出 = T0 ⊕ (D(t0)⁻¹ ⊕ D(t))：检测窗 t0→t_sw 的真实运动一次性补上
        Pose3d d0 = last_dr_;
        drPoseAt(last_slam_stamp_, &d0);  // 历史不覆盖 t0 时退化为当前 D（输出恒等）
        glue_dr_ = compose(before, inverse(d0));
        adoptDr();
        recordSwitch(now_sec);
      } else {
        return false;  // 双源皆失：无输出（fusion 层看门狗判失效）
      }
      break;
    }

    case Source::kDr: {
      if (slam_ok) {
        if (!slam_frame_established_) {
          // 冷启动（DR 先起、LIO 刚就绪）：虚拟系直接定为 LIO 系。
          // 不走连续性换算、不等滞回确认——此时 map_odom 未建立，跳一下无所谓，
          // 换来此后 glue_slam = I：SLAM 云零成本透传，且不把 DR 启动误差固化进虚拟系。
          glue_slam_ = identity();
          slam_frame_established_ = true;
          adoptSlam();
          break;
        }
        // 回切滞回：连续 N 次仲裁确认 SLAM 可用才切（防断续流翻转）
        const int need = flapGuardTripped(now_sec)
                             ? config_.switch_back_confirm_frames * 2
                             : config_.switch_back_confirm_frames;
        if (++slam_confirm_count_ >= std::max(1, need) || !dr_fresh) {
          // 运行期回切：连续性不变式 glue_slam = unified(t_sw⁻) ⊕ T_slam⁻¹
          glue_slam_ = compose(before, inverse(last_slam_));
          adoptSlam();
          recordSwitch(now_sec);
        }
        // else：确认期间保持 DR 输出（无缝）
      } else {
        slam_confirm_count_ = 0;
        if (!dr_fresh) {
          return false;  // 双源皆失
        }
      }
      break;
    }
  }

  // ---- ③ 输出：单一纯源位姿（切换瞬间 ≡ before，构造恒等）----
  return purePose(out);
}

// ============================ 诊断 ============================

LocalOdomSource::Snapshot LocalOdomSource::snapshot(double now_sec) const {
  Snapshot s;
  s.active = active_;
  s.slam_age = last_slam_stamp_ >= 0.0 ? (now_sec - last_slam_stamp_) : -1.0;
  s.dr_age = last_dr_stamp_ >= 0.0 ? (now_sec - last_dr_stamp_) : -1.0;
  s.switch_count = switch_count_;
  return s;
}

}  // namespace localization_v3
