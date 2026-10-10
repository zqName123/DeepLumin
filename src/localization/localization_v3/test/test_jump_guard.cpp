#include <gtest/gtest.h>

#include "localization_v3/core/jump_guard.hpp"

using namespace localization_v3;

namespace {

Pose3d makePose(double x, double y, double yaw_deg = 0.0) {
  Pose3d p;
  p.position << x, y, 0.0;
  p.orientation = Quatd(Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  return p;
}

JumpGuardConfig defaultConfig() {
  JumpGuardConfig cfg;
  cfg.enabled = true;
  cfg.soft_max_translation = 1.5;
  cfg.soft_max_yaw_deg = 5.0;
  cfg.hard_max_translation = 10.0;
  cfg.hard_max_yaw_deg = 20.0;
  cfg.confirm_frames = 3;
  cfg.confirm_consistency_trans = 0.5;
  cfg.confirm_consistency_yaw_deg = 2.0;
  cfg.confirm_window_sec = 1.0;
  return cfg;
}

const Pose3d kIdentity;  // odom_base 单位

}  // namespace

TEST(JumpGuardTest, SmallJumpAccepted) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(1.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kAccept);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(0, 1.2), kIdentity, 0.1),
            JumpGuardVerdict::kAccept);
}

TEST(JumpGuardTest, SmallYawJumpAccepted) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(0, 0, 3.0), kIdentity, 0.0),
            JumpGuardVerdict::kAccept);
}

TEST(JumpGuardTest, LargeJumpRequiresConfirmation) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  // 5m > soft(1.5) 且 < hard(10)：需 3 帧一致
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kConfirmPending);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.1, 0), kIdentity, 0.1),
            JumpGuardVerdict::kConfirmPending);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.2),
            JumpGuardVerdict::kAccept);
  EXPECT_EQ(guard.pendingCount(), 0u);  // 接受后清缓冲
}

TEST(JumpGuardTest, InconsistentCandidatesResetBuffer) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kConfirmPending);
  // 第二帧跳到别处：不一致 → 缓冲清空重新累计
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(-4.0, 0), kIdentity, 0.1),
            JumpGuardVerdict::kConfirmPending);
  EXPECT_EQ(guard.pendingCount(), 1u);
  // 从 1 帧重新累计，需要再攒 2 帧
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(-4.1, 0), kIdentity, 0.2),
            JumpGuardVerdict::kConfirmPending);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(-4.0, 0), kIdentity, 0.3),
            JumpGuardVerdict::kAccept);
}

TEST(JumpGuardTest, ExpiredBufferPruned) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kConfirmPending);
  // 超过 confirm_window_sec(1.0)：旧候选过期，重新累计
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 2.0),
            JumpGuardVerdict::kConfirmPending);
  EXPECT_EQ(guard.pendingCount(), 1u);
}

TEST(JumpGuardTest, HugeJumpRejected) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(15.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kReject);
  // 偏航超限同样拒绝
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(0, 0, 30.0), kIdentity, 0.1),
            JumpGuardVerdict::kReject);
}

TEST(JumpGuardTest, OdomOffsetDoesNotAffectVerdict) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  // odom_base 有漂移偏移时，判定仍基于 fused 变化量（= map_odom 修正量）：
  // 0.5m 小修正直接接受，5m 大跳需确认——与 map_odom 绝对值无关
  const Pose3d odom_base = makePose(10.0, 0);
  EXPECT_EQ(guard.evaluate(makePose(-10.0, 0), makePose(-10.5, 0), odom_base, 0.0),
            JumpGuardVerdict::kAccept);
  EXPECT_EQ(guard.evaluate(makePose(1000.0, 0), makePose(1005.0, 0), odom_base, 0.1),
            JumpGuardVerdict::kConfirmPending);
}

TEST(JumpGuardTest, DisabledGuardAcceptsEverything) {
  JumpGuardConfig cfg = defaultConfig();
  cfg.enabled = false;
  JumpGuard guard;
  guard.setConfig(cfg);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(100.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kAccept);
}

TEST(JumpGuardTest, ResetClearsBuffer) {
  JumpGuard guard;
  guard.setConfig(defaultConfig());
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.0),
            JumpGuardVerdict::kConfirmPending);
  guard.reset();
  EXPECT_EQ(guard.pendingCount(), 0u);
  EXPECT_EQ(guard.evaluate(makePose(0, 0), makePose(5.0, 0), kIdentity, 0.1),
            JumpGuardVerdict::kConfirmPending);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
