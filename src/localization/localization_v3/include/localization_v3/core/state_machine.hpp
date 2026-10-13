#pragma once

/**
 * @file state_machine.hpp
 * @brief 定位状态机（纯 C++，不依赖 ROS）
 *
 * 五态（与 v2 一致，不新增状态）：
 *   UNINITIALIZED → RELOCALIZING → TRACKING ⇄ TRACKING_LOST → LOCALIZATION_FAILURE
 *
 * 设计原则：
 * - 人工位姿在任何状态下有效（写入位姿 + 强制回 TRACKING，绕过驻留）
 * - 失效判定快速：失败/退化计数阈值小（默认 3 帧级），出图等可直接 forceTrackingLost
 * - 恢复需确认窗口（GM 连续 N / reloc 连续 N），防抖
 * - TRACKING → TRACKING_LOST 驻留仅防瞬时抖动（默认 0.2s，不再拖慢失效发现）
 */

#include <chrono>
#include <string>

namespace localization_v3 {

/// 重定位命令
enum class RelocCommand {
  kNone = 0,
  kStart,
  kStop,
};

/// 初始化策略
struct InitConfig {
  bool require_initial_pose = false;
  bool use_relocalization_init = true;
  bool use_manual_initialpose = true;
  bool use_global_matcher_init = true;
};

/// 重定位启停策略
struct RelocalizationPolicy {
  bool enabled = true;
  bool auto_start = true;
  bool stop_on_success = true;
  bool stop_on_manual_pose = true;
  bool stop_on_global_match = true;
};

/// 失效检测阈值与恢复防抖
struct TrackingFailureConfig {
  int gm_failure_threshold = 3;         ///< GM 连续失败/坏质阈值（快：≈0.3s @10Hz）
  int gm_degraded_threshold = 20;       ///< 连续退化阈值
  int reloc_failure_threshold = 10;     ///< 重定位连续失败阈值
  int gm_recovery_threshold = 3;        ///< GM 恢复确认窗口
  int reloc_recovery_threshold = 2;     ///< 重定位恢复确认窗口
  double tracking_min_dwell_sec = 0.2;  ///< TRACKING 最小驻留（防瞬时抖动）
  double tracking_lost_min_dwell_sec = 1.0;  ///< TRACKING_LOST 最小驻留（防恢复抖动）
};

class LocalizationStateMachine {
 public:
  enum class State {
    kUninitialized = 0,
    kRelocalizing,
    kTracking,
    kTrackingLost,
    kLocalizationFailure,
  };

  void configure(const InitConfig& init, const RelocalizationPolicy& policy,
                 const TrackingFailureConfig& tracking_config);

  // ---- 状态查询 ----
  State state() const { return state_; }
  bool initialized() const {
    return state_ == State::kTracking || state_ == State::kTrackingLost ||
           state_ == State::kLocalizationFailure;
  }
  bool relocalizationRunning() const { return state_ == State::kRelocalizing; }
  bool trackingLost() const { return state_ == State::kTrackingLost; }
  bool localizationFailure() const { return state_ == State::kLocalizationFailure; }
  const std::string& stateName() const { return state_name_; }

  /// 取走本周期应下发的重定位命令（取走后清零）
  RelocCommand takeCommand();

  // ---- 事件 ----
  void onSlamOdom();                    ///< 未初始化时自动启动重定位
  void onManualPose();                  ///< 人工位姿：任意状态 → TRACKING（绕过驻留）
  void onRelocalizationResult(bool accepted);
  void onGlobalMatchSuccess();          ///< 初始化完成 / 恢复确认累积
  void onTrackingFailure();             ///< GM 失败/坏质：计数，达阈值 → TRACKING_LOST
  void onTrackingDegraded();            ///< GM 连续退化：计数，达阈值 → TRACKING_LOST
  void onRecoveryFailed();              ///< 重定位失败：计数，达阈值 → LOCALIZATION_FAILURE
  void onRecoveryAttemptFailed();       ///< 恢复尝试失败（质量差）：清恢复确认计数
  void forceTrackingLost();             ///< 直接进入 TRACKING_LOST（出图 / 人工验证失败）
  void forceLocalizationFailure();      ///< 直接判失效（恢复超时 / SLAM 断流）

  // ---- 计数器 ----
  int getGmConsecutiveFailures() const { return gm_consecutive_failures_; }
  int getGmDegradedCount() const { return gm_degraded_count_; }
  int getRelocConsecutiveFailures() const { return reloc_consecutive_failures_; }
  void resetGmCounter() { gm_consecutive_failures_ = 0; }
  void resetDegradedCounter() { gm_degraded_count_ = 0; }
  void resetRelocCounter() { reloc_consecutive_failures_ = 0; }

 private:
  void transitionTo(State state, bool bypass_dwell = false);
  void request(RelocCommand command) { pending_command_ = command; }
  void resetCounters();
  void resetRecoveryCounters();

  InitConfig init_;
  RelocalizationPolicy policy_;
  TrackingFailureConfig tracking_config_;

  State state_ = State::kUninitialized;
  std::string state_name_ = "UNINITIALIZED";
  RelocCommand pending_command_ = RelocCommand::kNone;

  int gm_consecutive_failures_ = 0;
  int gm_degraded_count_ = 0;
  int reloc_consecutive_failures_ = 0;
  int gm_recovery_consecutive_successes_ = 0;
  int reloc_recovery_consecutive_successes_ = 0;

  std::chrono::steady_clock::time_point last_transition_stamp_;
};

}  // namespace localization_v3
