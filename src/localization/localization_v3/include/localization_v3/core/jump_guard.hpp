#pragma once

/**
 * @file jump_guard.hpp
 * @brief 跳变门限（仅 TRACKING 常规 GM 修正使用）
 *
 * 职责：防止几何相似（重复巷道/对称结构）误匹配造成的不合理位姿跳变。
 *
 * 作用域（重要，一个阶段一个机制）：
 *   - 人工位姿写入           → 不适用（人工位姿与上帧差距大是常态）
 *   - 人工验证结果（首个 good）→ 不适用（人工位姿不精确，验证结果确立新基准）
 *   - TRACKING 常规 GM 修正   → 适用（本职）
 *   - TRACKING_LOST/FAILURE 恢复 → 不适用（恢复本来就是大跳变；
 *       防误匹配交给质量门限 + 恢复确认窗口）
 *
 * 两层判定（比较量为修正前后 fused_pose 变化量）：
 *   小跳  Δt <= soft_max_translation 且 Δyaw <= soft_max_yaw_deg   → 直接接受
 *   大跳  未超 hard_max_*                                          → 需连续 confirm_frames
 *         帧候选互相一致（帧间差 <= confirm_consistency_*）才接受
 *   超限  超过 hard_max_*                                          → 拒绝
 */

#include <deque>

#include "localization_v3/core/types.hpp"

namespace localization_v3 {

enum class JumpGuardVerdict {
  kAccept = 0,     ///< 接受修正
  kConfirmPending, ///< 大跳变：候选已入确认缓冲，等待后续一致帧
  kReject,         ///< 拒绝（超限 / 确认缓冲内不一致）
};

struct JumpGuardConfig {
  bool enabled = true;
  double soft_max_translation = 1.5;        ///< 小跳平移上限（m）
  double soft_max_yaw_deg = 5.0;            ///< 小跳偏航上限（deg）
  double hard_max_translation = 10.0;       ///< 大跳确认/拒绝分界（m）
  double hard_max_yaw_deg = 20.0;           ///< 大跳确认/拒绝分界（deg）
  int confirm_frames = 3;                   ///< 大跳需连续一致的候选帧数
  double confirm_consistency_trans = 0.5;   ///< 确认帧之间平移一致性（m）
  double confirm_consistency_yaw_deg = 2.0; ///< 确认帧之间偏航一致性（deg）
  double confirm_window_sec = 1.0;          ///< 确认缓冲时效（超时清空）
};

class JumpGuard {
 public:
  void setConfig(const JumpGuardConfig& config) {
    config_ = config;
    reset();
  }
  const JumpGuardConfig& config() const { return config_; }

  /// 清空确认缓冲（人工位姿 / 验证采纳 / 恢复成功后调用，重置判定基准）
  void reset() { candidates_.clear(); }

  /**
   * @brief 判定一次 GM 修正
   * @param current_map_odom  当前 map->odom
   * @param candidate_map_odom 候选 map->odom
   * @param odom_base          当前 odom->base（用于比较 fused 变化量；无 slam 时传单位位姿）
   * @param now_sec            单调时间（秒），用于确认缓冲时效
   */
  JumpGuardVerdict evaluate(const Pose3d& current_map_odom, const Pose3d& candidate_map_odom,
                            const Pose3d& odom_base, double now_sec);

  /// 确认缓冲中待确认的候选帧数（调试/状态用）
  std::size_t pendingCount() const { return candidates_.size(); }

 private:
  struct Candidate {
    Pose3d map_odom;
    double stamp = 0.0;
  };

  JumpGuardConfig config_;
  std::deque<Candidate> candidates_;
};

}  // namespace localization_v3
