#include "localization_v2/core/initialization.hpp"

namespace localization_v2 {

void LocalizationStateMachine::configure(const InitConfig& init,
                                          const RelocalizationPolicy& policy,
                                          const TrackingFailureConfig& tracking_config) {
  init_ = init;
  policy_ = policy;
  tracking_config_ = tracking_config;
  transitionTo(State::kUninitialized);
  resetCounters();
}

void LocalizationStateMachine::transitionTo(State state, bool bypass_dwell) {
  const State old_state = state_;

  // 驻留时间检查：防止 TRACKING ↔ TRACKING_LOST 快速切换
  // 仅两种方向受限制，其他状态切换（如人工初值、初始化等）不受影响
  if (old_state != state && !bypass_dwell) {
    const auto now = std::chrono::steady_clock::now();
    const double dwell_sec =
        std::chrono::duration<double>(now - last_transition_stamp_).count();

    // TRACKING → TRACKING_LOST：需要 TRACKING 最小驻留时间（防止快速重入）
    if (old_state == State::kTracking && state == State::kTrackingLost) {
      if (dwell_sec < tracking_config_.tracking_min_dwell_sec) {
        return;  // 驻留时间不足，抑制切换
      }
    }

    // TRACKING_LOST → TRACKING：需要 TRACKING_LOST 最小驻留时间（防止快速恢复后又丢失）
    if (old_state == State::kTrackingLost && state == State::kTracking) {
      if (dwell_sec < tracking_config_.tracking_lost_min_dwell_sec) {
        return;  // 驻留时间不足，抑制切换
      }
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

  // 状态切换时重置相关计数器
  if (old_state != state) {
    if (state == State::kTracking) {
      // 成功恢复，重置所有计数器
      resetCounters();
    } else if (state == State::kTrackingLost) {
      // 进入跟踪丢失，重置重定位失败计数和退化计数
      reloc_consecutive_failures_ = 0;
      gm_degraded_count_ = 0;
    }
  }
}

void LocalizationStateMachine::resetCounters() {
  gm_consecutive_failures_ = 0;
  reloc_consecutive_failures_ = 0;
  gm_degraded_count_ = 0;
}

RelocCommand LocalizationStateMachine::takeCommand() {
  const RelocCommand cmd = pending_command_;
  pending_command_ = RelocCommand::kNone;
  return cmd;
}

// ========== 事件处理 ==========

void LocalizationStateMachine::onSlamOdom() {
  // 只在未初始化状态下处理
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

  // 人工定位在任何状态下都有效
  // 根据当前状态决定是否需要下发 stop 命令
  switch (state_) {
    case State::kRelocalizing:
    case State::kTrackingLost:
      // 正在重定位中，需要停止
      if (policy_.enabled && policy_.stop_on_manual_pose) {
        request(RelocCommand::kStop);
      }
      break;
    case State::kLocalizationFailure:
      // 从失败状态恢复，不自动启动重定位
      if (policy_.enabled) {
        request(RelocCommand::kNone);
      }
      break;
    default:
      break;
  }

  // 人工初值：绕过驻留时间检查，强制恢复
  transitionTo(State::kTracking, true);
}

void LocalizationStateMachine::onRelocalizationResult(bool accepted) {
  // 根据当前状态处理重定位结果
  switch (state_) {
    case State::kRelocalizing:
      // 初始化阶段的重定位结果
      if (!accepted) {
        return;
      }
      transitionTo(State::kTracking, true);  // 初始化切换：绕过驻留时间检查
      if (policy_.enabled && policy_.stop_on_success) {
        request(RelocCommand::kStop);
      }
      break;

    case State::kTrackingLost:
      // 跟踪丢失阶段的重定位结果：使用恢复确认窗口
      if (accepted) {
        reloc_recovery_consecutive_successes_++;
        if (reloc_recovery_consecutive_successes_ >= tracking_config_.reloc_recovery_threshold) {
          // 恢复成功
          transitionTo(State::kTracking);
          if (state_ == State::kTracking && policy_.enabled && policy_.stop_on_success) {
            request(RelocCommand::kStop);
          }
          resetRecoveryCounters();
        }
        // 恢复未成功（确认窗口未满），继续累积
      } else {
        // 恢复失败，增加计数
        reloc_consecutive_failures_++;
        resetRecoveryCounters();  // 失败时重置恢复确认计数器
      }
      break;

    default:
      // 其他状态忽略重定位结果
      break;
  }
}

void LocalizationStateMachine::onGlobalMatchSuccess() {
  if (!init_.use_global_matcher_init) {
    return;
  }

  // 在未初始化状态下处理
  if (state_ == State::kUninitialized || state_ == State::kRelocalizing) {
    transitionTo(State::kTracking);
    if (state_ == State::kTracking && policy_.enabled && policy_.stop_on_global_match) {
      request(RelocCommand::kStop);
    }
    return;
  }

  // LOCALIZATION_FAILURE 状态：使用恢复确认窗口
  if (state_ == State::kLocalizationFailure) {
    gm_recovery_consecutive_successes_++;
    if (gm_recovery_consecutive_successes_ >= tracking_config_.gm_recovery_threshold) {
      transitionTo(State::kTracking);
      if (state_ == State::kTracking && policy_.enabled) {
        request(RelocCommand::kStop);
      }
      resetRecoveryCounters();
    }
    return;
  }

  // TRACKING_LOST 状态：使用恢复确认窗口
  if (state_ == State::kTrackingLost) {
    gm_recovery_consecutive_successes_++;
    if (gm_recovery_consecutive_successes_ >= tracking_config_.gm_recovery_threshold) {
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
  // 只在 TRACKING 状态下处理
  if (state_ != State::kTracking) {
    return;
  }

  gm_consecutive_failures_++;

  // 判断是否达到阈值
  if (gm_consecutive_failures_ >= tracking_config_.gm_failure_threshold) {
    transitionTo(State::kTrackingLost);
    request(RelocCommand::kStart);  // 启动重定位恢复
  }
}

void LocalizationStateMachine::onTrackingDegraded() {
  // 只在 TRACKING 状态下处理
  if (state_ != State::kTracking) {
    return;
  }

  gm_degraded_count_++;

  // 判断是否达到退化阈值
  if (gm_degraded_count_ >= tracking_config_.gm_degraded_threshold) {
    transitionTo(State::kTrackingLost);
    request(RelocCommand::kStart);  // 启动重定位恢复
  }
}

void LocalizationStateMachine::onRecoveryFailed() {
  // 只在 TRACKING_LOST 状态下处理
  if (state_ != State::kTrackingLost) {
    return;
  }

  reloc_consecutive_failures_++;

  // 判断是否达到阈值
  if (reloc_consecutive_failures_ >= tracking_config_.reloc_failure_threshold) {
    transitionTo(State::kLocalizationFailure);
    request(RelocCommand::kStop);  // 停止重定位
  }
}

void LocalizationStateMachine::forceLocalizationFailure() {
  // 只在 TRACKING_LOST 状态下处理
  if (state_ != State::kTrackingLost) {
    return;
  }
  transitionTo(State::kLocalizationFailure);
  request(RelocCommand::kStop);
}

void LocalizationStateMachine::onRecoveryAttemptFailed() {
  resetRecoveryCounters();
}

void LocalizationStateMachine::resetRecoveryCounters() {
  gm_recovery_consecutive_successes_ = 0;
  reloc_recovery_consecutive_successes_ = 0;
}

}  // namespace localization_v2