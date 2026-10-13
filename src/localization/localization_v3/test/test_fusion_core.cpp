#include <gtest/gtest.h>

#include <limits>

#include "localization_v3/core/fusion_core.hpp"

using namespace localization_v3;

namespace {

Pose3d makePose(double x, double y, double yaw_rad = 0.0, double z = 0.0) {
  Pose3d p;
  p.position << x, y, z;
  p.orientation = Quatd(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  return p;
}

FusionCore::Config defaultConfig() {
  FusionCore::Config cfg;
  cfg.manual_settle_sec = 0.3;
  cfg.jump_guard.enabled = true;
  cfg.jump_guard.soft_max_translation = 1.5;
  cfg.jump_guard.soft_max_yaw_deg = 5.0;
  cfg.jump_guard.hard_max_translation = 10.0;
  cfg.jump_guard.hard_max_yaw_deg = 20.0;
  cfg.jump_guard.confirm_frames = 3;
  cfg.jump_guard.confirm_consistency_trans = 0.5;
  cfg.jump_guard.confirm_window_sec = 1.0;
  return cfg;
}

/// 无 slam（odom_base 单位）：fused == map_odom，便于直接断言跳变
class FusionCoreTest : public ::testing::Test {
 protected:
  void SetUp() override { fusion_.setConfig(defaultConfig()); }
  FusionCore fusion_;
};

/// 有 odom 漂移（odom_base = (10,0)）：验证 map_base ↔ map_odom 换算
class FusionCoreWithOdomTest : public ::testing::Test {
 protected:
  void SetUp() override {
    fusion_.setConfig(defaultConfig());
    fusion_.setOdomBase(makePose(10.0, 0.0));
  }
  FusionCore fusion_;
};

}  // namespace

// ==================== 融合数学：map_base ↔ map_odom ====================

TEST_F(FusionCoreWithOdomTest, ManualPoseComposesMapOdomCorrectly) {
  // map_base=(5,0)，odom_base=(10,0) → map_odom=(-5,0)
  ASSERT_TRUE(fusion_.applyManualPose(makePose(5.0, 0.0), false, 0.0));
  EXPECT_NEAR(fusion_.mapOdom().position.x(), -5.0, 1e-9);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 5.0, 1e-9);  // fused 还原人工位姿
  EXPECT_NEAR(fusion_.fusedPose().position.y(), 0.0, 1e-9);
}

TEST_F(FusionCoreWithOdomTest, RelocalizationUsesMapBaseNotMapOdom) {
  // P5 回归：重定位结果是 map->base，写入后 fused 必须等于该 map_base
  ASSERT_TRUE(fusion_.applyRelocalization(makePose(3.0, 4.0), false, 0.0));
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 3.0, 1e-9);
  EXPECT_NEAR(fusion_.fusedPose().position.y(), 4.0, 1e-9);
}

// ==================== 人工位姿流程 ====================

TEST_F(FusionCoreTest, ManualPoseBypassesJumpGuard) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  // 人工位姿跳 50m（远超 hard 门限 10m）——必须无条件生效
  ASSERT_TRUE(fusion_.applyManualPose(makePose(50.0, 0.0), false, 10.0));
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 50.0, 1e-9);
  EXPECT_TRUE(fusion_.manualVerifyPending());
}

TEST_F(FusionCoreTest, StaleResultsDroppedDuringSettle) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  // settle 窗口内（0 ~ 0.3s）：GM 结果一律丢弃
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(1.0, 0.0), QualityLevel::kGood, 0.1),
            PoseUpdateResult::kRejectedStale);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);  // 人工位姿保持
}

TEST_F(FusionCoreTest, FirstGoodResultAfterSettleVerifiesWithoutDistanceCheck) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  // settle 后首个 good 结果：即使与人工位姿相差 8m（人工不精确），也直接采纳
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 8.0, 1e-9);
  EXPECT_FALSE(fusion_.manualVerifyPending());
}

TEST_F(FusionCoreTest, NonGoodResultDoesNotVerify) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.0, 0.0), QualityLevel::kDegraded, 0.5),
            PoseUpdateResult::kRejectedQuality);
  EXPECT_TRUE(fusion_.manualVerifyPending());
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);
}

TEST_F(FusionCoreTest, CancelManualVerifyKeepsPoseAndRestoresNormalRules) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  fusion_.cancelManualVerify();
  EXPECT_FALSE(fusion_.manualVerifyPending());
  // 恢复常规规则：小跳直接接受
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(0.5, 0.0), QualityLevel::kGood, 1.0),
            PoseUpdateResult::kAccepted);
}

// ==================== 常规 TRACKING：jump guard ====================

TEST_F(FusionCoreTest, SmallJumpAcceptedDirectly) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);  // 确立基准
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(1.0, 0.0), QualityLevel::kGood, 1.0),
            PoseUpdateResult::kAccepted);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 1.0, 1e-9);
}

TEST_F(FusionCoreTest, LargeJumpNeedsConfirmation) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);

  // 8m 跳变（>soft 1.5m、<hard 10m）：前 2 帧待确认
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.0, 0.0), QualityLevel::kGood, 1.0),
            PoseUpdateResult::kConfirmPending);
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.1, 0.0), QualityLevel::kGood, 1.1),
            PoseUpdateResult::kConfirmPending);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);  // 未确认前不动
  // 第 3 帧一致 → 接受
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.0, 0.0), QualityLevel::kGood, 1.2),
            PoseUpdateResult::kAccepted);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 8.0, 1e-9);
}

TEST_F(FusionCoreTest, InconsistentLargeJumpRejected) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);

  // 互相不一致的大跳候选（几何相似误匹配特征）：位姿保持不动
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(8.0, 0.0), QualityLevel::kGood, 1.0),
            PoseUpdateResult::kConfirmPending);
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(-6.0, 0.0), QualityLevel::kGood, 1.1),
            PoseUpdateResult::kConfirmPending);
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(2.0, 0.0), QualityLevel::kGood, 1.2),
            PoseUpdateResult::kConfirmPending);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);
}

TEST_F(FusionCoreTest, HugeJumpRejected) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);
  // 30m > hard 10m：直接拒绝
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(30.0, 0.0), QualityLevel::kGood, 1.0),
            PoseUpdateResult::kRejectedGuard);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);
}

TEST_F(FusionCoreTest, BadQualityNeverApplied) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);
  EXPECT_EQ(fusion_.applyGlobalMatch(makePose(0.3, 0.0), QualityLevel::kBad, 1.0),
            PoseUpdateResult::kRejectedQuality);
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 0.0, 1e-9);
}

// ==================== 恢复期：无 jump guard ====================

TEST_F(FusionCoreTest, RecoveryUpdateBypassesJumpGuard) {
  ASSERT_TRUE(fusion_.applyManualPose(makePose(0.0, 0.0), false, 0.0));
  ASSERT_EQ(fusion_.applyGlobalMatch(makePose(0.0, 0.0), QualityLevel::kGood, 0.5),
            PoseUpdateResult::kAcceptedVerified);
  // 恢复期 50m 跳变（远超 hard 门限）：必须生效（恢复本来就是大跳变）
  EXPECT_TRUE(fusion_.applyRecoveryMatch(makePose(50.0, 0.0), 2.0));
  EXPECT_NEAR(fusion_.fusedPose().position.x(), 50.0, 1e-9);
}

TEST_F(FusionCoreTest, InvalidPoseRejected) {
  Pose3d bad;
  bad.position << std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0;
  EXPECT_FALSE(fusion_.applyManualPose(bad, false, 0.0));
  EXPECT_FALSE(fusion_.applyRelocalization(bad, false, 0.0));
  EXPECT_FALSE(fusion_.applyRecoveryMatch(bad, 0.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
