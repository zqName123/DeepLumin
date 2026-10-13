#pragma once

/**
 * @file observation_gate.hpp
 * @brief 观测链门控（两链隔离不变式 I2 的落地点）
 *
 * 观测链只在 **kRunning** 相位通：
 *   - kRunning  : 运动链 = 纯锚（Δ 未应用）→ 观测 DR 是安全的
 *   - kGap/kOnBackup : 运动链已用 Δ → 观测必须关
 *                 （否则 map_base 里含 DR 自身增量 → 自回灌；
 *                  且观测台阶会污染 Δ，同一份 GM 修正被吃两次）
 *
 * 观测值 = map_odom ⊕ anchor（≡ map_base，绝对位姿）：
 *   - anchor 是纯源位姿（I4）⇒ 运动链不回流进观测链
 *   - 合成式的好处：观测由**锚事件**驱动，不需要额外活性探针
 *
 * 用法（事件驱动，不是时间窗）：
 *   主源样本到达 → arbiter 落锚 → gate.shouldPublish(phase) → 发观测
 */

#include "local_odom_gateway/core/source_spec.hpp"

namespace local_odom_gateway {

class ObservationGate {
 public:
  /// 该相位是否允许发布观测（= 运动链未用 Δ）
  static bool shouldPublish(Phase phase) { return phase == Phase::kRunning; }

  /// 观测值合成：map_odom ⊕ anchor ≡ map_base（GM 校正后的全局位姿）
  static Pose3d composeObservation(const Pose3d& map_odom, const Pose3d& anchor) {
    return compose(map_odom, anchor);
  }
};

}  // namespace local_odom_gateway
