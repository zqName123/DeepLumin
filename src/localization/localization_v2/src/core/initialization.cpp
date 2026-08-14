#include "localization_v2/core/initialization.hpp"

namespace localization_v2 {

void InitStateMachine::configure(const InitConfig& init, const RelocalizationPolicy& policy) {
  init_ = init;
  policy_ = policy;
  transitionTo(State::kUninitialized);
}

void InitStateMachine::transitionTo(State state) {
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
  }
}

RelocCommand InitStateMachine::takeCommand() {
  const RelocCommand cmd = pending_command_;
  pending_command_ = RelocCommand::kNone;
  return cmd;
}

void InitStateMachine::onSlamOdom() {
  if (state_ != State::kUninitialized) {
    return;
  }
  if (!policy_.enabled || !policy_.auto_start || !init_.use_relocalization_init) {
    return;
  }
  transitionTo(State::kRelocalizing);
  request(RelocCommand::kStart);
}

void InitStateMachine::onManualPose() {
  if (!init_.use_manual_initialpose) {
    return;
  }
  transitionTo(State::kTracking);
  if (policy_.enabled && policy_.stop_on_manual_pose) {
    request(RelocCommand::kStop);
  }
}

void InitStateMachine::onRelocalizationResult(bool accepted) {
  if (state_ != State::kRelocalizing || !accepted) {
    return;
  }
  transitionTo(State::kTracking);
  if (policy_.enabled && policy_.stop_on_success) {
    request(RelocCommand::kStop);
  }
}

void InitStateMachine::onGlobalMatchSuccess() {
  if (!init_.use_global_matcher_init || initialized()) {
    return;
  }
  transitionTo(State::kTracking);
  if (policy_.enabled && policy_.stop_on_global_match) {
    request(RelocCommand::kStop);
  }
}

}  // namespace localization_v2
