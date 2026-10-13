#include <gtest/gtest.h>

#include "dr_odometry_v2/core/pose_gate.hpp"

using namespace dr_odometry_v2;

namespace {

Pose3d makePose(double x, double y, double yaw_deg = 0.0) {
  Pose3d p;
  p.position = Vec3d(x, y, 0.0);
  p.orientation = math::quatFromYaw(math::deg2rad(yaw_deg));
  return p;
}

PoseGate::Config defaultConfig() {
  PoseGate::Config cfg;
  cfg.soft_trans = 0.5;
  cfg.soft_yaw_deg = 2.0;
  cfg.hard_trans = 5.0;
  cfg.hard_yaw_deg = 15.0;
  cfg.confirm_frames = 3;
  cfg.consistency_trans = 0.5;
  cfg.consistency_yaw_deg = 2.0;
  return cfg;
}

}  // namespace

TEST(PoseGateTest, SmallJumpAccepted) {
  PoseGate gate;
  gate.setConfig(defaultConfig());
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(0.3, 0), 0.0), GateVerdict::kAccept);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(0, 0, 1.0), 0.1), GateVerdict::kAccept);
}

TEST(PoseGateTest, LargeJumpNeedsConfirmation) {
  PoseGate gate;
  gate.setConfig(defaultConfig());
  // 2m：>soft(0.5) <hard(5) → 连续 3 帧一致才台阶修正
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.0), GateVerdict::kConfirmPending);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.1, 0), 0.1), GateVerdict::kConfirmPending);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.2), GateVerdict::kAcceptStep);
  EXPECT_EQ(gate.pendingCount(), 0u);
}

TEST(PoseGateTest, InconsistentLargeJumpResets) {
  PoseGate gate;
  gate.setConfig(defaultConfig());
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.0), GateVerdict::kConfirmPending);
  // 来回跳（伪跳变特征）：缓冲清空重新累计
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(-1.5, 0), 0.1), GateVerdict::kConfirmPending);
  EXPECT_EQ(gate.pendingCount(), 1u);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.2), GateVerdict::kConfirmPending);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.3), GateVerdict::kConfirmPending);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(2.0, 0), 0.4), GateVerdict::kAcceptStep);
}

TEST(PoseGateTest, HugeJumpRejected) {
  PoseGate gate;
  gate.setConfig(defaultConfig());
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(20.0, 0), 0.0), GateVerdict::kReject);
  EXPECT_EQ(gate.evaluate(makePose(0, 0), makePose(0, 0, 30.0), 0.1), GateVerdict::kReject);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
