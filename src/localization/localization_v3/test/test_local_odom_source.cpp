#include <gtest/gtest.h>

#include "localization_v3/core/local_odom_source.hpp"

using namespace localization_v3;

namespace {

Pose3d makePose(double x, double y, double yaw_deg = 0.0) {
  Pose3d p;
  p.position = Vec3d(x, y, 0.0);
  p.orientation =
      Quatd(Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Vec3d::UnitZ()));
  return p;
}

LocalOdomSource::Config defaultConfig() {
  LocalOdomSource::Config cfg;
  cfg.slam_timeout_sec = 0.5;
  cfg.dr_timeout_sec = 0.5;
  cfg.switch_back_confirm_frames = 3;
  cfg.flap_guard_count = 3;
  cfg.flap_window_sec = 60.0;
  return cfg;
}

}  // namespace

// ============================ 基础契约：虚拟系 / 切换连续性 ============================

TEST(LocalOdomSourceTest, SlamFirstAdoptsSlamFrame) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(1.0, 2.0), 0.0);

  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  // 首源：虚拟系 = SLAM 系（glue=I）
  EXPECT_NEAR(out.position.x(), 1.0, 1e-9);
  EXPECT_NEAR(out.position.y(), 2.0, 1e-9);
}

TEST(LocalOdomSourceTest, TakeoverContinuousWhenDrHistoryMissesT0) {
  // DR 历史不覆盖 t0（接管前才收到首个 DR 样本）：退化为接管时刻锚定，输出恒等
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(10.0, 0.0), 0.0);

  Pose3d before;
  ASSERT_TRUE(src.sample(&before, 0.1));

  // SLAM 断流 0.6s，DR 在完全不同的自有系（如 (100, 50)）
  src.onDr(makePose(100.0, 50.0), 0.6);
  Pose3d after;
  ASSERT_TRUE(src.sample(&after, 0.6));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);

  // 连续性：切换瞬间 unified 严格相等（粘合变换吸收两系差异）
  EXPECT_NEAR(after.position.x(), before.position.x(), 1e-9);
  EXPECT_NEAR(after.position.y(), before.position.y(), 1e-9);
}

TEST(LocalOdomSourceTest, DrMotionPropagatesAfterTakeover) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(0.0, 0.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));

  src.onDr(makePose(100.0, 50.0), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));  // 接管
  const double x0 = out.position.x();

  // DR 前进 2m（自有系 x+2）→ 统一系同向前进 2m
  src.onDr(makePose(102.0, 50.0), 0.7);
  ASSERT_TRUE(src.sample(&out, 0.7));
  EXPECT_NEAR(out.position.x() - x0, 2.0, 1e-9);
}

TEST(LocalOdomSourceTest, HandbackIsContinuousWithHysteresis) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(0.0, 0.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));

  // 切 DR
  src.onDr(makePose(100.0, 50.0), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));
  ASSERT_EQ(src.active(), LocalOdomSource::Source::kDr);

  // SLAM 恢复（乱入不同系）：滞回确认期间保持 DR
  src.onSlam(makePose(5.0, 5.0), 0.7);
  Pose3d d1;
  ASSERT_TRUE(src.sample(&d1, 0.7));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);  // 第 1 帧未确认
  src.onSlam(makePose(5.0, 5.0), 0.8);
  Pose3d d2;
  ASSERT_TRUE(src.sample(&d2, 0.8));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);  // 第 2 帧未确认

  // 第 3 帧达到确认 → 切回 SLAM，且输出连续
  src.onSlam(makePose(5.0, 5.0), 0.9);
  Pose3d d3;
  ASSERT_TRUE(src.sample(&d3, 0.9));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_NEAR(d3.position.x(), d2.position.x(), 1e-9);  // 回切瞬间连续
  EXPECT_NEAR(d3.position.y(), d2.position.y(), 1e-9);
}

TEST(LocalOdomSourceTest, HandbackIsContinuousWhileDrHasMoved) {
  // DR 接管后前进一段再回切：切换瞬间输出恒等（DR 运动已体现在切换前输出里）
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  src.onSlam(makePose(0.0, 0.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));

  src.onDr(makePose(100.0, 50.0), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));  // 接管
  src.onDr(makePose(110.0, 50.0), 0.7);  // DR 前进 10m
  Pose3d in_dr;
  ASSERT_TRUE(src.sample(&in_dr, 0.7));
  EXPECT_NEAR(in_dr.position.x(), 10.0, 1e-9);  // DR 运动已进输出

  // SLAM 恢复（自身系落在 3,4）：滞回确认 3 帧后回切
  Pose3d last = in_dr;
  for (int i = 1; i <= 3; ++i) {
    src.onSlam(makePose(3.0, 4.0), 0.8 + i * 0.1);
    src.onDr(makePose(110.0, 50.0), 0.8 + i * 0.1);
    ASSERT_TRUE(src.sample(&last, 0.8 + i * 0.1));
  }
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_NEAR(last.position.x(), in_dr.position.x(), 1e-9);
  EXPECT_NEAR(last.position.y(), in_dr.position.y(), 1e-9);
}

TEST(LocalOdomSourceTest, RoundTripSwitchingPreservesMotion) {
  // SLAM→DR→SLAM→DR 全程：回切恒等；接管补检测窗运动（运动量不变式）
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  src.onSlam(makePose(0.0, 0.0), 0.0);
  Pose3d out, prev;
  ASSERT_TRUE(src.sample(&out, 0.1));

  // ① SLAM→DR（接管）：DR 历史不覆盖 t0 → 输出恒等
  prev = out;
  src.onDr(makePose(100.0, 50.0), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));
  ASSERT_EQ(src.active(), LocalOdomSource::Source::kDr);
  EXPECT_NEAR(out.position.x(), prev.position.x(), 1e-9);

  // DR 前进 4m
  src.onDr(makePose(104.0, 50.0), 0.7);
  ASSERT_TRUE(src.sample(&out, 0.7));
  EXPECT_NEAR(out.position.x(), 4.0, 1e-9);

  // ② DR→SLAM（回切，滞回 3 帧）
  for (int i = 1; i <= 3; ++i) {
    src.onSlam(makePose(2.0, 3.0), 0.7 + i * 0.1);
    src.onDr(makePose(104.0, 50.0), 0.7 + i * 0.1);
    prev = out;
    ASSERT_TRUE(src.sample(&out, 0.7 + i * 0.1));
    EXPECT_NEAR(out.position.x(), prev.position.x(), 1e-9);  // 确认期与回切瞬间都连续
    EXPECT_NEAR(out.position.y(), prev.position.y(), 1e-9);
  }
  ASSERT_EQ(src.active(), LocalOdomSource::Source::kSlam);

  // ③ SLAM→DR（再次接管）：t0=1.0（最后 SLAM 样本）→ D(t0)=(104,50)，
  // D(1.6)=(110,50) ⇒ 检测窗真实运动 6m 补进输出：(4,0)→(10,0)
  src.onDr(makePose(110.0, 50.0), 1.6);
  ASSERT_TRUE(src.sample(&out, 1.6));
  ASSERT_EQ(src.active(), LocalOdomSource::Source::kDr);
  EXPECT_NEAR(out.position.x(), 10.0, 1e-9);

  // 之后走 DR 增量（+1m）
  src.onDr(makePose(111.0, 50.0), 1.7);
  ASSERT_TRUE(src.sample(&out, 1.7));
  EXPECT_NEAR(out.position.x(), 11.0, 1e-9);
}

// ============================ 虚拟系 / 云契约 ============================

TEST(LocalOdomSourceTest, SlamGlueIdentityWhenSlamOnly) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(3.0, 4.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_NEAR(src.slamGlue().position.norm(), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(src.slamGlue().orientation.normalized().w()), 1.0, 1e-12);
}

TEST(LocalOdomSourceTest, SlamGlueNonIdentityAfterDrExcursion) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(0, 0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  ASSERT_NEAR(src.slamGlue().position.norm(), 0.0, 1e-12);
  src.onDr(makePose(100, 50), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));
  for (int i = 1; i <= 3; ++i) {
    src.onSlam(makePose(5.0, 5.0), 0.7 + i * 0.1);
    src.onDr(makePose(100.0, 50.0), 0.7 + i * 0.1);
    ASSERT_TRUE(src.sample(&out, 0.7 + i * 0.1));
  }
  ASSERT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_GT(src.slamGlue().position.norm(), 1.0);
}

TEST(LocalOdomSourceTest, SlamCloudFreshnessRequired) {
  // SLAM 源可用 = odom 新鲜 && 注册云新鲜（部分故障防护）
  LocalOdomSource src;
  LocalOdomSource::Config cfg = defaultConfig();
  cfg.slam_require_cloud = true;
  cfg.slam_cloud_timeout_sec = 0.5;
  src.setConfig(cfg);

  src.onSlam(makePose(0, 0), 0.0);
  src.onSlamCloud(0.0);
  src.onDr(makePose(100, 50), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);  // odom+云均新鲜

  // odom 持续、注册云断 → SLAM 不可用 → 切 DR（DR 同时保持活跃）
  src.onSlam(makePose(0, 0), 0.3);
  src.onDr(makePose(100, 50), 0.3);
  ASSERT_TRUE(src.sample(&out, 0.6));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);

  // 云恢复（连同 odom）→ 回切确认帧后回到 SLAM
  for (int i = 1; i <= 3; ++i) {
    src.onSlam(makePose(0, 0), 0.7 + i * 0.1);
    src.onSlamCloud(0.7 + i * 0.1);
    src.onDr(makePose(100.0, 50.0), 0.7 + i * 0.1);
    ASSERT_TRUE(src.sample(&out, 0.7 + i * 0.1));
  }
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
}

TEST(LocalOdomSourceTest, CloudNotRequiredWhenDisabled) {
  LocalOdomSource src;
  LocalOdomSource::Config cfg = defaultConfig();
  cfg.slam_require_cloud = false;  // 默认/云处理关闭时
  src.setConfig(cfg);
  src.onSlam(makePose(0, 0), 0.0);
  src.onDr(makePose(10, 0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);  // 无云也判可用
}

// ============================ 纯 SLAM 期 DR 结构性不参与（毛刺回归） ============================

TEST(LocalOdomSourceTest, SlamOutputExcludesDrIncrement) {
  // 纯 SLAM 期（含"超龄但未判死"的断流期）输出 = 纯 SLAM 位姿：
  // DR 增量**结构性**不进输出链（无任何 gap 填充路径）→ 断开"DR→输出→观测"自回灌，
  // 正常节拍的周期抖动也不可能触发 DR 参与。
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onDr(makePose(100, 50), 0.0);
  src.onSlam(makePose(0, 0), 0.0);
  src.onDr(makePose(103, 50), 0.05);  // DR 前进 3m
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));  // 新鲜期
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_NEAR(out.position.x(), 0.0, 1e-9);  // 纯 SLAM，DR 增量不进输出

  // 超龄但未判死（age 0.45 < slam_timeout 0.5）：依旧纯 SLAM，DR 又走 5m 也不补
  src.onDr(makePose(108, 50), 0.45);
  ASSERT_TRUE(src.sample(&out, 0.45));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_NEAR(out.position.x(), 0.0, 1e-9);
}

TEST(LocalOdomSourceTest, NoGlitchWithinNormalSlamCadence) {
  // 回归（间隔性毛刺）：SLAM 正常节拍（10Hz，含周期抖动）+ DR 全程存活同速前进时，
  // 输出必须始终等于纯 SLAM 位姿——旧 gap 填充在节拍抖动下反复启停，产生台阶/尖刺。
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  const double periods[] = {0.10, 0.11, 0.14, 0.10, 0.13, 0.10, 0.12, 0.10};
  double t = 0.0;
  src.onDr(makePose(100.0, 50.0), 0.0);
  src.onSlam(makePose(0.0, 0.0), 0.0);
  for (double dt : periods) {
    // 本帧样本到来前一刻：上一样本 age = dt-1ms ≤ 0.139，仍在 SLAM 期
    Pose3d aged;
    ASSERT_TRUE(src.sample(&aged, t + dt - 1e-3));
    EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
    EXPECT_NEAR(aged.position.x(), 2.0 * t, 1e-9);  // 纯 SLAM，无台阶

    // 新样本到达：DR 前进 2m/s（自有系 x+），SLAM 同步前进（虚拟系 x+）
    t += dt;
    src.onDr(makePose(100.0 + 2.0 * t, 50.0), t);
    src.onSlam(makePose(2.0 * t, 0.0), t);
    Pose3d fresh;
    ASSERT_TRUE(src.sample(&fresh, t));
    EXPECT_NEAR(fresh.position.x(), 2.0 * t, 1e-9);  // 仍纯 SLAM
  }
}

TEST(LocalOdomSourceTest, TakeoverPreservesDetectionWindowMotion) {
  // 切换语义："锚定的位姿 T0 + 自 t0 起的 DR 增量"——
  // 纯 SLAM 期输出 = 纯 SLAM（含断流未判死期，DR 不进输出链）；
  // 接管锚定 glue_dr = T0 ⊕ D(t0)⁻¹ ⇒ 检测窗 t0→t_sw 的真实运动一次性补进输出
  // （替代旧"轨迹短一截 + GM 事后回拉"的跳变）。
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  src.onDr(makePose(100, 50), 0.0);
  src.onSlam(makePose(0, 0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_NEAR(out.position.x(), 0.0, 1e-9);

  // 断流未判死期（age 0.45 < 0.5）：输出仍纯 SLAM，DR 前进 1.5m 不进输出
  src.onDr(makePose(101.5, 50), 0.45);
  ASSERT_TRUE(src.sample(&out, 0.45));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_NEAR(out.position.x(), 0.0, 1e-9);

  // 超时接管：T0=(0,0)、D(t0)=D(0.0)=(100,50)、D(0.55)=(102,50)
  // ⇒ 输出 = (102-100) = 2.0：检测窗运动（1.5m）+ 接管前 0.5m 全部保留
  src.onDr(makePose(102.0, 50), 0.55);
  ASSERT_TRUE(src.sample(&out, 0.55));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);
  EXPECT_NEAR(out.position.x(), 2.0, 1e-9);

  // 接管后再走 0.5m：增量续推
  src.onDr(makePose(102.5, 50), 0.65);
  ASSERT_TRUE(src.sample(&out, 0.65));
  EXPECT_NEAR(out.position.x(), 2.5, 1e-9);
}

TEST(LocalOdomSourceTest, TakeoverPairsDrAtSlamSampleStamp) {
  // D(t0) 必须按【SLAM 样本时间戳】插值，不能取接管时刻/到达时刻的 DR 状态：
  // 否则检测窗运动量算错（毛刺/丢运动的根源之一）。
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  src.onDr(makePose(100, 50), 0.0);   // D(0.0)
  src.onSlam(makePose(0, 0), 0.2);    // t0=0.2 → D(t0) 应为插值 (101,50)
  src.onDr(makePose(102, 50), 0.4);   // 0.0→0.4 前进 2m
  src.onDr(makePose(103, 50), 0.6);   // 再 1m
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.25));
  EXPECT_NEAR(out.position.x(), 0.0, 1e-9);  // 纯 SLAM

  src.onDr(makePose(104, 50), 0.9);
  ASSERT_TRUE(src.sample(&out, 0.9));        // age 0.7 > 0.5 → 接管
  // T0=(0,0)，D(t0=0.2)=(101,50)，D(0.9)=(104,50) ⇒ 输出 = 3.0
  // 若误用接管时刻 D(0.9) 配对会得到 0.0
  EXPECT_NEAR(out.position.x(), 3.0, 1e-9);
}

// ============================ 失效 / 保护 / 冷启动 ============================

TEST(LocalOdomSourceTest, BothLostGivesNoOutput) {
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(0.0, 0.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));

  // SLAM 断且 DR 无数据 → 无输出
  EXPECT_FALSE(src.sample(&out, 1.0));
  // DR 来了 → 接管有输出
  src.onDr(makePose(3.0, 0.0), 1.1);
  EXPECT_TRUE(src.sample(&out, 1.1));
  // 双断 → 无输出
  EXPECT_FALSE(src.sample(&out, 2.0));
}

TEST(LocalOdomSourceTest, FlapGuardDoublesConfirmFrames) {
  LocalOdomSource src;
  LocalOdomSource::Config cfg = defaultConfig();
  cfg.flap_guard_count = 1;  // 一次切换后即进入翻转保护
  src.setConfig(cfg);

  src.onSlam(makePose(0, 0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  // 切 DR（第 1 次切换 → 触发翻转保护）
  src.onDr(makePose(10, 0), 0.6);
  ASSERT_TRUE(src.sample(&out, 0.6));
  // SLAM 恢复：确认帧加倍（3*2=6），前 5 帧保持 DR（DR 持续运行）
  for (int i = 1; i <= 5; ++i) {
    src.onSlam(makePose(1, 1), 0.7 + i * 0.1);
    src.onDr(makePose(10.0, 50.0), 0.7 + i * 0.1);
    EXPECT_TRUE(src.sample(&out, 0.7 + i * 0.1));
    EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);
  }
  src.onSlam(makePose(1, 1), 1.3);
  ASSERT_TRUE(src.sample(&out, 1.3));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);  // 第 6 帧才切回
}

TEST(LocalOdomSourceTest, ColdStartDrThenSlamAnchorsLioFrame) {
  // 冷启动：DR 先跑（虚拟系临时 = DR 系保活），LIO 就绪后虚拟系**直接定为 LIO 系**
  // （glue_slam = I，不走连续性换算、不等滞回）——启动期无连续性可保，
  // 换算反而会把 DR 启动误差固化进虚拟系。
  LocalOdomSource src;
  src.setConfig(defaultConfig());

  src.onDr(makePose(100.0, 50.0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kDr);
  EXPECT_NEAR(out.position.x(), 100.0, 1e-9);  // 临时虚拟系 = DR 系

  // LIO 到达：立即采纳（不等确认帧），glue_slam = I → 输出 = LIO 原始位姿
  src.onSlam(makePose(5.0, 5.0), 0.2);
  ASSERT_TRUE(src.sample(&out, 0.2));
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);  // 无滞回，立刻切
  EXPECT_NEAR(out.position.x(), 5.0, 1e-9);
  EXPECT_NEAR(out.position.y(), 5.0, 1e-9);
  EXPECT_NEAR(src.slamGlue().position.norm(), 0.0, 1e-12);  // 虚拟系 = LIO 系
}

TEST(LocalOdomSourceTest, PurePoseHasNoArbitrationSideEffects) {
  // purePose 是观测锚取样口（桥在数据回调里用）：不得推进仲裁状态，
  // 否则取锚动作会消耗回切确认帧 / 触发换源，观测链污染运动链。
  LocalOdomSource src;
  src.setConfig(defaultConfig());
  src.onSlam(makePose(0, 0), 0.0);
  Pose3d out;
  ASSERT_TRUE(src.sample(&out, 0.1));

  // SLAM 断流、DR 新鲜：反复取锚不得换源
  src.onDr(makePose(10, 0), 0.2);
  Pose3d anchor;
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(src.purePose(&anchor));
    EXPECT_NEAR(anchor.position.x(), 0.0, 1e-9);  // 锚 = 纯源位姿
  }
  EXPECT_EQ(src.active(), LocalOdomSource::Source::kSlam);
  EXPECT_EQ(src.snapshot(0.2).switch_count, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
