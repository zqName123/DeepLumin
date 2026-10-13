#include <gtest/gtest.h>

#include "local_odom_gateway/core/motion_provider.hpp"

using namespace local_odom_gateway;

namespace {

Pose3d makePose(double x, double y, double yaw_deg = 0.0) {
  Pose3d p;
  p.position = Vec3d(x, y, 0.0);
  p.orientation = Quatd(Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Vec3d::UnitZ()));
  return p;
}

MotionProvider::Config defaultConfig() {
  MotionProvider::Config cfg;
  cfg.overdue_sec = 0.15;   // 显式阈值（可预测）
  cfg.dead_sec = 0.50;
  cfg.increment_history_sec = 1.0;
  return cfg;
}

MotionProvider makeProvider() {
  MotionProvider mp;
  mp.setConfig(defaultConfig());
  return mp;
}

}  // namespace

// ───────────────────────── R4：kRunning 期 DR 不进运动链 ─────────────────────────

TEST(MotionProviderTest, RunningPhaseExcludesDrIncrement) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(100, 50), 0.0);   // DR 在自有系
  mp.onAnchorSample(makePose(0, 0), 0.0);         // 主源锚（配对 D(0)=(100,50)）

  mp.onIncrementSample(makePose(103, 50), 0.05);  // DR 前进 3m

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.10));               // age 0.10 ≤ 0.15 → kRunning
  EXPECT_EQ(mp.phase(0.10), Phase::kRunning);
  EXPECT_NEAR(s.motion.position.x(), 0.0, 1e-9);  // 纯锚，DR 增量不应用
  EXPECT_NEAR(s.motion.position.x(), s.anchor.position.x(), 1e-9);
}

// ───────────────────────── I4：锚永不含增量 ─────────────────────────

TEST(MotionProviderTest, AnchorNeverContainsIncrement) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(100, 50), 0.0);
  mp.onAnchorSample(makePose(0, 0), 0.0);
  mp.onIncrementSample(makePose(101.5, 50), 0.45);

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.45));               // kGap
  EXPECT_NEAR(s.anchor.position.x(), 0.0, 1e-9);  // 锚仍是纯源位姿
  EXPECT_NEAR(s.motion.position.x(), 1.5, 1e-9);  // 运动含增量
}

// ───────────────────────── kGap：增量从锚时刻起算（不留滞后） ─────────────────────────

TEST(MotionProviderTest, GapPhaseAppliesIncrementFromAnchorStamp) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(100, 50), 0.0);
  mp.onAnchorSample(makePose(0, 0), 0.0);
  mp.onIncrementSample(makePose(101.5, 50), 0.45);

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.45));
  EXPECT_EQ(mp.phase(0.45), Phase::kGap);
  // A ⊕ (D(t_a)⁻¹ ⊕ D(t)) = 0 + (101.5-100) = 1.5
  EXPECT_NEAR(s.motion.position.x(), 1.5, 1e-9);
}

TEST(MotionProviderTest, BackupPhaseKeepsSameFormula) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(100, 50), 0.0);
  mp.onAnchorSample(makePose(0, 0), 0.0);
  mp.onIncrementSample(makePose(102, 50), 0.60);  // age 0.6 > 0.5 → kOnBackup

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.60));
  EXPECT_EQ(mp.phase(0.60), Phase::kOnBackup);
  EXPECT_NEAR(s.motion.position.x(), 2.0, 1e-9);  // 公式不变，锚冻结
  EXPECT_NEAR(s.anchor.position.x(), 0.0, 1e-9);
}

// ───────────────────────── 时间对齐：配对按扫描戳插值 ─────────────────────────

TEST(MotionProviderTest, PairingUsesSampleStampNotArrival) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(100, 50), 0.0);   // D(0.0)
  mp.onIncrementSample(makePose(102, 50), 0.2);   // D(0.2)
  mp.onAnchorSample(makePose(0, 0), 0.1);         // 样本戳 0.1 → 应配对 D(0.1)=(101,50)
  mp.onIncrementSample(makePose(103, 50), 0.3);

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.3));
  // A ⊕ (D(0.1)⁻¹ ⊕ D(0.3)) = 0 + (103-101) = 2.0
  // 若误用到达时刻的 D(0.2)=(102,50) 配对，会得到 1.0
  EXPECT_NEAR(s.motion.position.x(), 2.0, 1e-9);
}

// ───────────────────────── 退化：无配对 → 冻结在锚 ─────────────────────────

TEST(MotionProviderTest, NoPairingFallsBackToHold) {
  MotionProvider mp = makeProvider();
  mp.onAnchorSample(makePose(5, 5), 0.0);         // 增量源尚未启动 → 无配对
  mp.onIncrementSample(makePose(100, 50), 0.40);  // 后来才有

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.45));
  EXPECT_FALSE(mp.anchor().has_pairing);
  EXPECT_NEAR(s.motion.position.x(), 5.0, 1e-9);  // 退化为冻结，不乱推
}

// ───────────────────────── 相位边界 ─────────────────────────

TEST(MotionProviderTest, PhaseBoundariesFollowThresholds) {
  MotionProvider mp = makeProvider();
  mp.onIncrementSample(makePose(0, 0), 0.0);
  mp.onAnchorSample(makePose(0, 0), 0.0);

  EXPECT_EQ(mp.phase(0.150), Phase::kRunning);   // age == overdue → 仍算新鲜
  EXPECT_EQ(mp.phase(0.151), Phase::kGap);
  EXPECT_EQ(mp.phase(0.500), Phase::kGap);       // age == dead → 仍在检测窗
  EXPECT_EQ(mp.phase(0.501), Phase::kOnBackup);
  EXPECT_EQ(mp.phase(-1.0), Phase::kRunning);    // 未初始化锚 → kNone 由 hasAnchor 兜
}

TEST(MotionProviderTest, AdaptiveThresholdsFollowPeriod) {
  MotionProvider::Config cfg = defaultConfig();
  cfg.overdue_sec = 0.0;   // 自适应
  cfg.dead_sec = 0.0;
  cfg.overdue_ratio = 1.3;
  cfg.dead_ratio = 2.0;
  MotionProvider mp;
  mp.setConfig(cfg);

  // 10Hz 主源：周期 EMA → 0.1s
  for (int i = 0; i < 10; ++i) {
    mp.onAnchorSample(makePose(i * 0.2, 0), i * 0.1);
    mp.onIncrementSample(makePose(0, 0), i * 0.1);
  }
  EXPECT_NEAR(mp.periodEma(), 0.1, 1e-6);
  EXPECT_NEAR(mp.overdueLimit(), 0.13, 1e-6);   // 1.3 × 0.1
  EXPECT_NEAR(mp.deadLimit(), 0.20, 1e-6);      // 2.0 × 0.1
}

// ───────────────────────── 契约：切 DR 从 SLAM 失效位姿开始 ─────────────────────────

TEST(MotionProviderTest, DrTakeoverStartsFromSlamPoseNotDrPose) {
  MotionProvider mp = makeProvider();
  // DR 自有系在 (100,50)（绝对位姿很大），SLAM 链位姿在 (3,4)
  mp.onIncrementSample(makePose(100, 50), 0.0);
  mp.onAnchorSample(makePose(3, 4), 0.0);        // A = SLAM 失效位姿 (3,4)
  mp.onIncrementSample(makePose(102, 50), 0.60); // DR 前进 2m

  Sample s;
  ASSERT_TRUE(mp.sample(&s, 0.60));              // kOnBackup：已切 DR
  // 输出 = A ⊕ Δ = (3,4) + 2m = (5,4) —— 从 SLAM 位姿接着走
  EXPECT_NEAR(s.motion.position.x(), 5.0, 1e-9);
  EXPECT_NEAR(s.motion.position.y(), 4.0, 1e-9);
  // DR 的绝对位姿 (102,50) 绝不出现
  EXPECT_NEAR(s.motion.position.y(), 4.0, 1e-9);
  EXPECT_GT(std::abs(s.motion.position.y() - 50.0), 1.0);
}

// ───────────────────────── 无锚无输出 ─────────────────────────

TEST(MotionProviderTest, NoAnchorNoOutput) {
  MotionProvider mp = makeProvider();
  Sample s;
  EXPECT_FALSE(mp.sample(&s, 1.0));
  EXPECT_EQ(mp.phase(1.0), Phase::kNone);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
