#include <gtest/gtest.h>

#include "localization_v3/core/state_machine.hpp"

using namespace localization_v3;

class LocalizationStateMachineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    InitConfig init;
    init.use_relocalization_init = true;
    init.use_manual_initialpose = true;
    init.use_global_matcher_init = true;

    RelocalizationPolicy policy;
    policy.enabled = true;
    policy.auto_start = true;
    policy.stop_on_success = true;
    policy.stop_on_manual_pose = true;
    policy.stop_on_global_match = true;

    TrackingFailureConfig tracking;
    tracking.gm_failure_threshold = 3;
    tracking.reloc_failure_threshold = 2;
    tracking.gm_recovery_threshold = 2;
    tracking.reloc_recovery_threshold = 2;
    tracking.tracking_min_dwell_sec = 0.0;   // 单测不等驻留
    tracking.tracking_lost_min_dwell_sec = 0.0;

    sm_.configure(init, policy, tracking);
  }

  LocalizationStateMachine sm_;
};

TEST_F(LocalizationStateMachineTest, InitialState) {
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kUninitialized);
  EXPECT_FALSE(sm_.initialized());
  EXPECT_EQ(sm_.stateName(), "UNINITIALIZED");
}

TEST_F(LocalizationStateMachineTest, SlameOdomTriggersRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, ManualPoseAlwaysWorkInUninitialized) {
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kNone);
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInRelocalizing) {
  sm_.onSlamOdom();
  sm_.takeCommand();
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInLocalizationFailure) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);

  // 连续失败 → TRACKING_LOST
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // 恢复失败 → LOCALIZATION_FAILURE
  sm_.onRecoveryFailed();
  sm_.onRecoveryFailed();
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  sm_.takeCommand();

  // 人工位姿直接拉回 TRACKING
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, FastFailureThreshold) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  sm_.takeCommand();

  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);  // 未达阈值
  sm_.onTrackingFailure();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, ForceTrackingLostOnlyFromTracking) {
  sm_.forceTrackingLost();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kUninitialized);

  sm_.onSlamOdom();
  sm_.forceTrackingLost();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);  // 初始化中不回退

  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  sm_.takeCommand();
  sm_.forceTrackingLost();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, ForceLocalizationFailureFromTracking) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  sm_.takeCommand();

  sm_.forceLocalizationFailure();  // SLAM 断流等：直接判失效
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, RecoveryConfirmWindow) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  sm_.takeCommand();

  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  sm_.takeCommand();

  // 恢复确认窗口：第 1 次成功不回 TRACKING
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  // 第 2 次成功（reloc_recovery_threshold=2）→ TRACKING
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, RecoveryFailedToLocalizationFailure) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  sm_.takeCommand();

  sm_.onRecoveryFailed();
  sm_.onRecoveryFailed();  // reloc_failure_threshold=2
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, GmRecoveryConfirmWindow) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  sm_.takeCommand();

  sm_.onGlobalMatchSuccess();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  sm_.onGlobalMatchSuccess();  // gm_recovery_threshold=2
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
}

TEST_F(LocalizationStateMachineTest, DegradedThreshold) {
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  sm_.takeCommand();

  for (int i = 0; i < 19; ++i) {
    sm_.onTrackingDegraded();
    ASSERT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  }
  sm_.onTrackingDegraded();  // 第 20 次
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
