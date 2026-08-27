#include <gtest/gtest.h>
#include "localization_v2/core/initialization.hpp"

using namespace localization_v2;

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

    TrackingFailureConfig tracking_config;
    tracking_config.gm_failure_threshold = 3;
    tracking_config.reloc_failure_threshold = 2;
    tracking_config.reloc_timeout_sec = 10.0;

    sm_.configure(init, policy, tracking_config);
  }

  LocalizationStateMachine sm_;
};

TEST_F(LocalizationStateMachineTest, InitialState) {
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kUninitialized);
  EXPECT_FALSE(sm_.initialized());
  EXPECT_FALSE(sm_.trackingLost());
  EXPECT_FALSE(sm_.localizationFailure());
  EXPECT_EQ(sm_.stateName(), "UNINITIALIZED");
}

TEST_F(LocalizationStateMachineTest, SlameOdomTriggersRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  EXPECT_EQ(sm_.stateName(), "RELOCALIZING");
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, ManualPoseAlwaysWorkInUninitialized) {
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.stateName(), "TRACKING");
  EXPECT_TRUE(sm_.initialized());
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);

  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, RelocalizationSuccessInRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);

  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, RelocalizationFailureInRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  // 取走 start 命令
  sm_.takeCommand();

  sm_.onRelocalizationResult(false);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kNone);
}

TEST_F(LocalizationStateMachineTest, GlobalMatchSuccessInRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);

  sm_.onGlobalMatchSuccess();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, TrackingToTrackingLost) {
  // 先进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);

  // 模拟 GM 连续失败
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  EXPECT_EQ(sm_.stateName(), "TRACKING_LOST");
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStart);
  EXPECT_TRUE(sm_.trackingLost());
}

TEST_F(LocalizationStateMachineTest, TrackingLostToTrackingByRelocSuccess) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // 重定位成功恢复
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
  EXPECT_FALSE(sm_.trackingLost());
}

TEST_F(LocalizationStateMachineTest, TrackingLostToTrackingByManualPose) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // 人工初值恢复
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, TrackingLostToLocalizationFailure) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // 重定位连续失败
  for (int i = 0; i < 2; ++i) {
    sm_.onRecoveryFailed();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  EXPECT_EQ(sm_.stateName(), "LOCALIZATION_FAILURE");
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kStop);
  EXPECT_TRUE(sm_.localizationFailure());
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInLocalizationFailure) {
  // 进入 LOCALIZATION_FAILURE
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  for (int i = 0; i < 2; ++i) {
    sm_.onRecoveryFailed();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);

  // 人工初值恢复
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_TRUE(sm_.initialized());
}

TEST_F(LocalizationStateMachineTest, CounterResetOnTracking) {
  // 进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.getGmConsecutiveFailures(), 0);
  EXPECT_EQ(sm_.getRelocConsecutiveFailures(), 0);

  // 模拟一些失败（需要先清除之前的命令）
  sm_.takeCommand();
  sm_.onTrackingFailure();
  EXPECT_EQ(sm_.getGmConsecutiveFailures(), 1);

  // 进入 TRACKING_LOST
  sm_.onTrackingFailure();
  sm_.onTrackingFailure();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // 重定位成功进入 TRACKING，计数器应该重置
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.getGmConsecutiveFailures(), 0);
  EXPECT_EQ(sm_.getRelocConsecutiveFailures(), 0);
}

TEST_F(LocalizationStateMachineTest, NoActionInTrackingForSlamOdom) {
  // 进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);

  // 清除之前的命令
  sm_.takeCommand();

  // TRACKING 状态下收到 SLAM 数据不应有状态变化
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kNone);
}

TEST_F(LocalizationStateMachineTest, NoActionInTrackingForGlobalMatch) {
  // 进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);

  // 清除之前的命令
  sm_.takeCommand();

  // TRACKING 状态下收到 GM 成功不应有状态变化
  sm_.onGlobalMatchSuccess();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kNone);
}

TEST_F(LocalizationStateMachineTest, RelocFailureIgnoredInTrackingLost) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);

  // TRACKING_LOST 状态下重定位失败应增加计数
  sm_.onRelocalizationResult(false);
  EXPECT_EQ(sm_.getRelocConsecutiveFailures(), 1);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
}

TEST_F(LocalizationStateMachineTest, RecoveryFailedOnlyInTrackingLost) {
  // 进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);

  // 清除之前的命令
  sm_.takeCommand();

  // TRACKING 状态下 onRecoveryFailed 不应有动作
  sm_.onRecoveryFailed();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), RelocCommand::kNone);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
