#pragma once

/**
 * @file arbiter.hpp
 * @brief 源仲裁与切换连续性（策略层，纯 C++）
 *
 * 角色（source_spec.hpp 的 SourceRole）：
 *   kAbsolute（SLAM/VIO）—— 提供**锚**，多个绝对源之间按优先级竞争
 *   kRelative（DR）      —— 只提供**增量**，从不提供锚
 *
 * **DR 接管不是一个"换源"**：主源死后锚自然冻结，运动由增量继续推
 * （相位 kOnBackup），glue 一个字都不用改。glue 只在"锚源易主"时变化
 * —— 这正是运动链公式 `T = A ⊕ Δ` 能做到切换无缝的原因。
 *
 * 连续性不变式（切换锚源时）：
 *   glue_new = unified(t_sw⁻) ⊕ T_new(t_sw)⁻¹
 */

#include <vector>

#include "local_odom_gateway/core/motion_provider.hpp"
#include "local_odom_gateway/core/source_registry.hpp"

namespace local_odom_gateway {

class Arbiter {
 public:
  struct Config {
    int flap_guard_count = 3;     ///< 翻转保护：窗口内换源 ≥ N 次 → 确认帧加倍（0=关）
    double flap_window_sec = 60.0;
  };

  explicit Arbiter(MotionProvider* motion) : motion_(motion) {}
  void setConfig(const Config& config) { config_ = config; }

  /// 注册源（配置驱动；新增源只加一条）
  SourceId addSource(const OdomSourceSpec& spec);

  /// 喂样本：相对源→增量轨迹；绝对源→（若为当前锚源）落锚
  void onSample(SourceId id, const Pose3d& pose, double stamp, const Twist& twist = Twist());

  /// 每个输出 tick 调用：推进回切滞回、必要时换锚源（含 glue 连续性）
  void arbitrate(double now_sec);

  SourceId anchorSource() const { return anchor_id_; }
  int switchCount() const { return switch_count_; }
  bool flapGuardTripped(double now_sec) const;
  const SourceRegistry& registry() const { return registry_; }
  SourceRegistry& registry() { return registry_; }
  const MotionProvider& motion() const { return *motion_; }

 private:
  void adoptAnchor(SourceId id, const Pose3d& raw_pose, double stamp, bool establish,
                   double now_sec);
  void recordSwitch(double now_sec);

  MotionProvider* motion_;
  SourceRegistry registry_;
  Config config_;

  SourceId anchor_id_ = kInvalidSource;
  SourceId pending_id_ = kInvalidSource;   ///< 回切候选（滞回中）
  bool anchor_suspended_ = false;          ///< 锚已冻结（主源判死）；恢复需确认 + glue 重对齐
  int confirm_count_ = 0;
  int switch_count_ = 0;
  std::vector<double> switch_times_;
};

}  // namespace local_odom_gateway
