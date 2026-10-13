#include <gtest/gtest.h>

#include "local_odom_gateway/core/cloud_unifier.hpp"
#include "local_odom_gateway/core/motion_provider.hpp"
#include "local_odom_gateway/core/observation_gate.hpp"

using namespace local_odom_gateway;

namespace {
Pose3d makePose(double x, double y) {
  Pose3d p;
  p.position = Vec3d(x, y, 0.0);
  return p;
}
}  // namespace

// ───────────────────────── 云统一：桥契约（云与里程计同系） ─────────────────────────

TEST(CloudUnifierTest, RegisteredCloudUsesGlue) {
  CloudUnifier u;
  const Pose3d glue = makePose(2, 3);
  EXPECT_NEAR(u.transform(CloudMode::kRegistered, glue, makePose(0, 0)).position.x(), 2.0, 1e-9);
}

TEST(CloudUnifierTest, RawRegisterUsesUnifiedPoseAndExtrinsic) {
  CloudUnifier u;
  u.setBaseToLidar(makePose(1, 0));
  const Pose3d unified = makePose(5, 5);
  const Pose3d T = u.transform(CloudMode::kRawRegister, identity(), unified);
  // T = unified ⊕ base_to_lidar = (5,5) + (1,0) = (6,5)
  EXPECT_NEAR(T.position.x(), 6.0, 1e-9);
  EXPECT_NEAR(T.position.y(), 5.0, 1e-9);
}

TEST(CloudUnifierTest, PassThroughOnlyWhenGlueIdentity) {
  EXPECT_TRUE(CloudUnifier::canPassThrough(CloudMode::kRegistered, identity()));
  EXPECT_FALSE(CloudUnifier::canPassThrough(CloudMode::kRegistered, makePose(1e-3, 0)));
  EXPECT_FALSE(CloudUnifier::canPassThrough(CloudMode::kRawRegister, identity()));
}

// ───────────────────────── 观测门控：相位互斥 ─────────────────────────

TEST(ObservationGateTest, OnlyRunningPhasePublishes) {
  EXPECT_TRUE(ObservationGate::shouldPublish(Phase::kRunning));
  EXPECT_FALSE(ObservationGate::shouldPublish(Phase::kGap));
  EXPECT_FALSE(ObservationGate::shouldPublish(Phase::kOnBackup));
  EXPECT_FALSE(ObservationGate::shouldPublish(Phase::kNone));
}

TEST(ObservationGateTest, ObservationIsAbsolutePose) {
  const Pose3d obs = ObservationGate::composeObservation(makePose(10, 20), makePose(1, 2));
  EXPECT_NEAR(obs.position.x(), 11.0, 1e-9);
  EXPECT_NEAR(obs.position.y(), 22.0, 1e-9);
}

// ───────────────────────── 两链隔离不变式（端到端） ─────────────────────────

TEST(IsolationInvariantTest, IncrementAppliedOnlyWhenGateClosed) {
  MotionProvider mp;
  MotionProvider::Config cfg;
  cfg.overdue_sec = 0.15;
  cfg.dead_sec = 0.5;
  mp.setConfig(cfg);

  mp.onIncrementSample(makePose(100, 50), 0.0);
  mp.onAnchorSample(makePose(3, 4), 0.0);
  mp.onIncrementSample(makePose(102, 50), 0.60);

  // kRunning：运动链不含 Δ，观测链开
  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.10));
  EXPECT_EQ(mp.phase(0.10), Phase::kRunning);
  EXPECT_TRUE(ObservationGate::shouldPublish(mp.phase(0.10)));
  EXPECT_NEAR(s.motion.position.x(), s.anchor.position.x(), 1e-9);  // I1

  // kGap / kOnBackup：Δ 应用，观测链关
  ASSERT_TRUE(mp.sample(&s, 0.60));
  EXPECT_EQ(mp.phase(0.60), Phase::kOnBackup);
  EXPECT_FALSE(ObservationGate::shouldPublish(mp.phase(0.60)));  // I2
  EXPECT_NEAR(s.anchor.position.x(), 3.0, 1e-9);                 // I4：锚纯
  EXPECT_NEAR(s.motion.position.x(), 5.0, 1e-9);                 // A ⊕ Δ，从 SLAM 位姿起
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
