#include "localization_v3/core/state_machine.hpp"

namespace localization_v3 {

void LocalizationStateMachine::configure(const InitConfig& init,
                                         const RelocalizationPolicy& policy,
                                         const TrackingFailureConfig& tracking_config) {
  init_ = init;
  policy_ = policy;
  tracking_config_ = tracking_config;
  transitionTo(State::kUninitialized, true);
  resetCounters();
}

void LocalizationStateMachine::transitionTo(State state, bool bypass_dwell) {
  const State old_state = state_;

  // 驻留检查：只防 TRACKING ⇄ TRACKING_LOST 抖动；人工/初始化/失效切换不受限
  if (old_state != state && !bypass_dwell) {
    const double dwell_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_transition_stamp_)
            .count();
    if (old_state == State::kTracking && state == State::kTrackingLost &&
        dwell_sec < tracking_config_.tracking_min_dwell_sec) {
      return;
    }
    if (old_state == State::kTrackingLost && state == State::kTracking &&
        dwell_sec < tracking_config_.tracking_lost_min_dwell_sec) {
      return;
    }
  }

  state_ = state;
  last_transition_stamp_ = std::chrono::steady_clock::now();

  switch (state) {
    case State::kUninitialized:
      state_name_ = "UNINITIALIZED";
      break;
    case State::kRelocalizing:
      state_name_ = "RELOCALIZING";
      break;
    case State::kTracking:
      state_name_ = "TRACKING";
      break;
    case State::kTrackingLost:
      state_name_ = "TRACKING_LOST";
      break;
    case State::kLocalizationFailure:
      state_name_ = "LOCALIZATION_FAILURE";
      break;
  }

  if (old_state != state) {
    if (state == State::kTracking) {
      resetCounters();  // 恢复成功，清全部失败计数
    } else if (state == State::kTrackingLost) {
      reloc_consecutive_failures_ = 0;
      gm_degraded_count_ = 0;
    }
  }
}

void LocalizationStateMachine::resetCounters() {
  gm_consecutive_failures_ = 0;
  reloc_consecutive_failures_ = 0;
  gm_degraded_count_ = 0;
  resetRecoveryCounters();
}

void LocalizationStateMachine::resetRecoveryCounters() {
  gm_recovery_consecutive_successes_ = 0;
  reloc_recovery_consecutive_successes_ = 0;
}

RelocCommand LocalizationStateMachine::takeCommand() {
  const RelocCommand cmd = pending_command_;
  pending_command_ = RelocCommand::kNone;
  return cmd;
}

// ============================== 事件 ==============================

void LocalizationStateMachine::onSlamOdom() {
  if (state_ != State::kUninitialized) {
    return;
  }
  if (!policy_.enabled || !policy_.auto_start || !init_.use_relocalization_init) {
    return;
  }
  transitionTo(State::kRelocalizing);
  request(RelocCommand::kStart);
}

void LocalizationStateMachine::onManualPose() {
  if (!init_.use_manual_initialpose) {
    return;
  }
  // 人工位姿：任何状态下停止重定位并强制回 TRACKING（绕过驻留）
  if (policy_.enabled && policy_.stop_on_manual_pose &&
      (state_ == State::kRelocalizing || state_ == State::kTrackingLost ||
       state_ == State::kLocalizationFailure)) {
    request(RelocCommand::kStop);
  }
  transitionTo(State::kTracking, true);
  resetCounters();  // 人工位姿 = 全新起点：清残留失败/退化计数（状态未变时 transitionTo 不清）
}

void LocalizationStateMachine::onRelocalizationResult(bool accepted) {
  switch (state_) {
    case State::kRelocalizing:
      // 初始化阶段：成功即完成初始化
      if (!accepted) {
        return;
      }
      transitionTo(State::kTracking, true);
      if (policy_.enabled && policy_.stop_on_success) {
        request(RelocCommand::kStop);
      }
      break;

    case State::kTrackingLost:
      // 恢复阶段：确认窗口内累积
      if (accepted) {
        if (++reloc_recovery_consecutive_successes_ >= tracking_config_.reloc_recovery_threshold) {
          transitionTo(State::kTracking);
          if (state_ == State::kTracking && policy_.enabled && policy_.stop_on_success) {
            request(RelocCommand::kStop);
          }
          resetRecoveryCounters();
        }
      } else {
        reloc_consecutive_failures_++;
        resetRecoveryCounters();
      }
      break;

    default:
      break;
  }
}

void LocalizationStateMachine::onGlobalMatchSuccess() {
  if (!init_.use_global_matcher_init) {
    return;
  }

  // 初始化：GM 自身成功即完成初始化
  if (state_ == State::kUninitialized || state_ == State::kRelocalizing) {
    transitionTo(State::kTracking);
    if (state_ == State::kTracking && policy_.enabled && policy_.stop_on_global_match) {
      request(RelocCommand::kStop);
    }
    return;
  }

  // 恢复确认窗口（TRACKING_LOST / LOCALIZATION_FAILURE）
  if (state_ == State::kLocalizationFailure || state_ == State::kTrackingLost) {
    if (++gm_recovery_consecutive_successes_ >= tracking_config_.gm_recovery_threshold) {
      transitionTo(State::kTracking);
      if (state_ == State::kTracking && policy_.enabled) {
        request(RelocCommand::kStop);
      }
      resetRecoveryCounters();
    }
    return;
  }
}

void LocalizationStateMachine::onTrackingFailure() {
  if (state_ != State::kTracking) {
    return;
  }
  gm_consecutive_failures_++;
  if (gm_consecutive_failures_ >= tracking_config_.gm_failure_threshold) {
    transitionTo(State::kTrackingLost);
    if (state_ == State::kTrackingLost) {
      request(RelocCommand::kStart);
    }
  }
}

void LocalizationStateMachine::onTrackingDegraded() {
  if (state_ != State::kTracking) {
    return;
  }
  gm_degraded_count_++;
  if (gm_degraded_count_ >= tracking_config_.gm_degraded_threshold) {
    transitionTo(State::kTrackingLost);
    if (state_ == State::kTrackingLost) {
      request(RelocCommand::kStart);
    }
  }
}

void LocalizationStateMachine::onRecoveryFailed() {
  if (state_ != State::kTrackingLost) {
    return;
  }
  reloc_consecutive_failures_++;
  if (reloc_consecutive_failures_ >= tracking_config_.reloc_failure_threshold) {
    transitionTo(State::kLocalizationFailure);
    request(RelocCommand::kStop);
  }
}

void LocalizationStateMachine::onRecoveryAttemptFailed() {
  resetRecoveryCounters();
}

void LocalizationStateMachine::forceTrackingLost() {
  // 快路径（出图 / 人工验证失败）：仅从 TRACKING 触发，不驻留
  if (state_ != State::kTracking) {
    return;
  }
  transitionTo(State::kTrackingLost, true);
  if (state_ == State::kTrackingLost) {
    request(RelocCommand::kStart);
  }
}

void LocalizationStateMachine::forceLocalizationFailure() {
  // 失效判定（恢复超时 / SLAM 断流）：从 TRACKING 或 TRACKING_LOST 都可直接判失效
  if (state_ == State::kLocalizationFailure) {
    return;
  }
  if (state_ == State::kUninitialized || state_ == State::kRelocalizing) {
    return;  // 尚未建立定位，不属于"定位失效"
  }
  transitionTo(State::kLocalizationFailure, true);
  request(RelocCommand::kStop);
}

}  // namespace localization_v3
