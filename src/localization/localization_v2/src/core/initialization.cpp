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

void LocalizationStateMachine::transitionTo(State state) {
  const State old_state = state_;
  state_ = state;

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
      // 进入跟踪丢失，重置重定位失败计数
      reloc_consecutive_failures_ = 0;
    }
  }
}

void LocalizationStateMachine::resetCounters() {
  gm_consecutive_failures_ = 0;
  reloc_consecutive_failures_ = 0;
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

  // 统一转移到 TRACKING
  transitionTo(State::kTracking);
}

void LocalizationStateMachine::onRelocalizationResult(bool accepted) {
  // 根据当前状态处理重定位结果
  switch (state_) {
    case State::kRelocalizing:
      // 初始化阶段的重定位结果
      if (!accepted) {
        return;
      }
      transitionTo(State::kTracking);
      if (policy_.enabled && policy_.stop_on_success) {
        request(RelocCommand::kStop);
      }
      break;

    case State::kTrackingLost:
      // 跟踪丢失阶段的重定位结果
      if (accepted) {
        // 恢复成功
        transitionTo(State::kTracking);
        if (policy_.enabled && policy_.stop_on_success) {
          request(RelocCommand::kStop);
        }
      } else {
        // 恢复失败，增加计数
        reloc_consecutive_failures_++;
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
    if (policy_.enabled && policy_.stop_on_global_match) {
      request(RelocCommand::kStop);
    }
    return;
  }

  // LOCALIZATION_FAILURE 状态：GM 成功时恢复到 TRACKING
  if (state_ == State::kLocalizationFailure) {
    transitionTo(State::kTracking);
    if (policy_.enabled) {
      request(RelocCommand::kStop);
    }
    return;
  }

  // TRACKING_LOST 状态：GM 成功且质量好时恢复到 TRACKING
  if (state_ == State::kTrackingLost) {
    transitionTo(State::kTracking);
    if (policy_.enabled) {
      request(RelocCommand::kStop);
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

}  // namespace localization_v2
