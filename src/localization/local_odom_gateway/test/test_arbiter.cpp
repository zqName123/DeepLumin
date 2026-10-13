#include <gtest/gtest.h>

#include "local_odom_gateway/core/arbiter.hpp"

using namespace local_odom_gateway;

namespace {

Pose3d makePose(double x, double y, double yaw_deg = 0.0) {
  Pose3d p;
  p.position = Vec3d(x, y, 0.0);
  p.orientation = Quatd(Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Vec3d::UnitZ()));
  return p;
}

OdomSourceSpec slamSpec() {
  OdomSourceSpec s;
  s.name = "slam";
  s.role = SourceRole::kAbsolute;
  s.priority = 10;
  s.timeout_sec = 0.25;
  s.switch_back_confirm = 3;
  return s;
}

OdomSourceSpec drSpec() {
  OdomSourceSpec s;
  s.name = "dr";
  s.role = SourceRole::kRelative;
  s.priority = 5;
  s.timeout_sec = 0.5;
  return s;
}

}  // namespace

// ───────────────────────── 冷启动：虚拟系 = 主源系 ─────────────────────────

TEST(ArbiterTest, ColdStartEstablishesPrimaryFrame) {
  MotionProvider mp;
  Arbiter arb(&mp);
  const SourceId slam = arb.addSource(slamSpec());
  const SourceId dr = arb.addSource(drSpec());

  // DR 先起（相对源，只进增量）
  arb.onSample(dr, makePose(100, 50), 0.0);
  // SLAM 到达：冷启动直接确立虚拟系 = SLAM 系
  arb.onSample(slam, makePose(5, 5), 0.2);
  arb.arbitrate(0.2);

  EXPECT_EQ(arb.anchorSource(), slam);
  EXPECT_TRUE(isIdentity(arb.registry().slot(slam).glue));  // glue = I

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.2));
  EXPECT_NEAR(s.motion.position.x(), 5.0, 1e-9);   // = SLAM 原始位姿
  EXPECT_NEAR(s.anchor.position.x(), 5.0, 1e-9);
}

// ───────────────────────── 相对源从不提供锚 ─────────────────────────

TEST(ArbiterTest, RelativeSourceNeverBecomesAnchor) {
  MotionProvider mp;
  Arbiter arb(&mp);
  const SourceId slam = arb.addSource(slamSpec());
  const SourceId dr = arb.addSource(drSpec());

  arb.onSample(dr, makePose(100, 50), 0.0);
  arb.arbitrate(0.05);
  EXPECT_EQ(arb.anchorSource(), kInvalidSource);   // DR 不能当锚源

  // 只有 DR 持续来 → 锚始终没有（无绝对源）
  Sample tmp;
  for (int i = 1; i <= 5; ++i) {
    arb.onSample(dr, makePose(100 + i, 50), 0.1 * i);
    arb.arbitrate(0.1 * i);
    EXPECT_EQ(arb.anchorSource(), kInvalidSource);
    EXPECT_FALSE(mp.sample(&tmp, 0.1 * i));  // 无锚则无输出
  }
  (void)slam;
}

// ───────────────────────── DR 接管是相位变化，不是换源（glue 不动） ─────────────────────────

TEST(ArbiterTest, TakeoverIsPhaseOnlyNoGlueChange) {
  MotionProvider mp;
  mp.setConfig([] { MotionProvider::Config c; c.overdue_sec = 0.15; c.dead_sec = 0.5; return c; }());
  Arbiter arb(&mp);
  const SourceId slam = arb.addSource(slamSpec());
  const SourceId dr = arb.addSource(drSpec());

  arb.onSample(dr, makePose(100, 50), 0.0);
  arb.onSample(slam, makePose(0, 0), 0.0);
  arb.arbitrate(0.0);
  const Pose3d glue_before = arb.registry().slot(slam).glue;

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.10));
  EXPECT_EQ(mp.phase(0.10), Phase::kRunning);

  // SLAM 断流，DR 续推
  arb.onSample(dr, makePose(101.5, 50), 0.45);
  arb.arbitrate(0.45);
  ASSERT_TRUE(mp.sample(&s, 0.45));
  EXPECT_EQ(mp.phase(0.45), Phase::kGap);
  EXPECT_NEAR(s.motion.position.x(), 1.5, 1e-9);

  arb.onSample(dr, makePose(102, 50), 0.60);
  arb.arbitrate(0.60);
  ASSERT_TRUE(mp.sample(&s, 0.60));
  EXPECT_EQ(mp.phase(0.60), Phase::kOnBackup);
  EXPECT_NEAR(s.motion.position.x(), 2.0, 1e-9);   // 公式不变

  // 接管只是相位变化：锚源不变、glue 不变
  EXPECT_EQ(arb.anchorSource(), slam);
  EXPECT_TRUE(isIdentity(arb.registry().slot(slam).glue));
  EXPECT_EQ(arb.switchCount(), 0);
  (void)glue_before;
}

// ───────────────────────── 回切：连续性不变式 ─────────────────────────

TEST(ArbiterTest, SwitchBackIsContinuous) {
  MotionProvider mp;
  Arbiter arb(&mp);
  const SourceId slam = arb.addSource(slamSpec());
  const SourceId dr = arb.addSource(drSpec());

  arb.onSample(dr, makePose(100, 50), 0.0);
  arb.onSample(slam, makePose(0, 0), 0.0);
  arb.arbitrate(0.0);

  // DR 推进一段（锚冻结，运动 = 锚 ⊕ 增量）
  for (int i = 1; i <= 3; ++i) {
    arb.onSample(dr, makePose(100 + 2.0 * i, 50), 0.15 * i);
    arb.arbitrate(0.15 * i);
  }
  Sample before;
  arb.arbitrate(0.60);
  ASSERT_TRUE(mp.sample(&before, 0.60));
  EXPECT_EQ(mp.phase(0.60), Phase::kOnBackup);
  const double expected = before.motion.position.x();  // = 6.0（DR 走了 6m）
  EXPECT_NEAR(expected, 6.0, 1e-9);

  // SLAM 恢复（自身系落在 3,4）：滞回 3 帧后回切
  Sample last = before;
  for (int i = 1; i <= 3; ++i) {
    const double t = 0.6 + i * 0.05;
    arb.onSample(slam, makePose(3, 4), t);
    arb.onSample(dr, makePose(106, 50), t);
    arb.arbitrate(t);
    mp.sample(&last, t);
  }
  EXPECT_EQ(arb.anchorSource(), slam);                     // 已回切
  EXPECT_NEAR(last.motion.position.x(), expected, 1e-9);   // 切换瞬间恒等
  EXPECT_NEAR(last.motion.position.y(), before.motion.position.y(), 1e-9);
  // glue 吸收了 DR 累积误差（非 I）
  EXPECT_GT(arb.registry().slot(slam).glue.position.norm(), 1e-6);
}

// ───────────────────────── 回切滞回 ─────────────────────────

TEST(ArbiterTest, SwitchBackNeedsConfirmations) {
  MotionProvider mp;
  Arbiter arb(&mp);
  const SourceId slam = arb.addSource(slamSpec());   // switch_back_confirm = 3
  const SourceId dr = arb.addSource(drSpec());

  arb.onSample(dr, makePose(100, 50), 0.0);
  arb.onSample(slam, makePose(0, 0), 0.0);
  arb.arbitrate(0.0);

  // 断流 → 相位 kOnBackup
  arb.onSample(dr, makePose(100, 50), 0.6);
  arb.arbitrate(0.6);
  EXPECT_EQ(mp.phase(0.6), Phase::kOnBackup);

  // SLAM 恢复：前 2 帧不切，第 3 帧切
  for (int i = 1; i <= 2; ++i) {
    const double t = 0.7 + i * 0.05;
    arb.onSample(slam, makePose(1, 1), t);
    arb.arbitrate(t);
    EXPECT_EQ(arb.anchorSource(), slam);  // 仍为 slam（锚源未变），但未确认
    // 注意：锚源仍是 slam（从未易主），这里验证的是相位不跳回
    EXPECT_NE(mp.phase(t), Phase::kRunning);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
