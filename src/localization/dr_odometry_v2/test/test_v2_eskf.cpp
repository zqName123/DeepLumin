#include <gtest/gtest.h>

#include "dr_odometry_v2/core/eskf.hpp"

using namespace dr_odometry_v2;

namespace {

DrConfig defaultConfig() {
  DrConfig cfg;
  cfg.imu.enabled = true;
  cfg.imu.use_accel = true;
  cfg.imu.max_dt = 0.05;
  cfg.wheel.enabled = true;
  cfg.gnss.enabled = true;
  cfg.gnss.use_attitude = true;
  cfg.gnss.heading_convention = "enu";  // 单测用 enu 约定（yaw=heading）
  cfg.gnss.attitude_mode = "absolute";  // 绝对姿态语义的用例显式声明
  cfg.gnss.use_accel = false;
  cfg.gnss.use_velocity = false;
  cfg.gnss.use_position = false;
  cfg.localization.enabled = true;
  cfg.localization.pose_noise_trans = 0.02;
  cfg.localization.pose_noise_att_deg = 0.5;
  cfg.output.max_extrapolate = 0.1;
  return cfg;
}

ImuSample imuAt(double t, const Vec3d& gyro = Vec3d::Zero(),
                const Vec3d& accel = Vec3d(0, 0, 9.80665)) {
  ImuSample s;
  s.stamp = t;
  s.gyro = gyro;
  s.accel = accel;
  return s;
}

/// 静止场景：IMU 零角速度 + 重力比力 + 轮速 0
void feedStationary(Eskf& eskf, double t0, double t1, double dt, double gyro_bias_z = 0.0) {
  for (double t = t0; t <= t1 + 1e-9; t += dt) {
    eskf.processEvent(Event::makeImu(imuAt(t, Vec3d(0, 0, gyro_bias_z))));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 0.0;
    eskf.processEvent(Event::makeWheel(w));
  }
}

}  // namespace

TEST(EskfTest, StationaryPredictKeepsPose) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 1.0, 0.005);

  const auto odom = eskf.odometry(1.0);
  ASSERT_TRUE(odom.valid);
  EXPECT_NEAR(odom.pose.position.norm(), 0.0, 1e-3);
  EXPECT_NEAR(odom.velocity.norm(), 0.0, 1e-3);
  EXPECT_NEAR(math::yawFromQuat(odom.pose.orientation), 0.0, 1e-3);
}

TEST(EskfTest, WheelObservationPullsVelocity) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  // 首帧初始化
  eskf.processEvent(Event::makeImu(imuAt(0.0)));

  // 给一个错误初速，靠轮速 (speed,0,0) 观测拉回
  DrState s = eskf.state();
  s.velocity = Vec3d(5.0, 0.0, 0.0);
  eskf.reset(s);
  eskf.processEvent(Event::makeImu(imuAt(0.01)));

  for (double t = 0.01; t <= 1.0; t += 0.02) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 1.0;  // 真值：前进 1m/s
    eskf.processEvent(Event::makeWheel(w));
  }
  const auto odom = eskf.odometry(1.0);
  EXPECT_NEAR(odom.velocity.x(), 1.0, 0.3);  // 被拉向轮速
  EXPECT_NEAR(odom.velocity.y(), 0.0, 0.3);  // 非完整约束
}

TEST(EskfTest, GnssAttitudeCorrectsYaw) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  eskf.processEvent(Event::makeImu(imuAt(0.0)));

  // 初值 yaw 错 20°
  DrState s = eskf.state();
  s.orientation = math::quatFromYaw(math::deg2rad(20.0));
  eskf.reset(s);

  for (double t = 0.01; t <= 2.0; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    GnssSample g;
    g.stamp = t;
    g.roll = 0.0;
    g.pitch = 0.0;
    g.heading = 0.0;  // 真值 yaw=0
    g.attitude_valid = true;
    g.speed_mps = 5.0;  // 高速，yaw 噪声不放大
    eskf.processEvent(Event::makeGnss(g));
  }
  const auto odom = eskf.odometry(2.0);
  EXPECT_NEAR(math::yawFromQuat(odom.pose.orientation), 0.0, 1.0);  // 20° 误差被修正
}

TEST(EskfTest, CompassHeadingConvention) {
  // 罗盘约定（北基准顺时针）：heading=0°（正北）→ 世界 yaw = 90°；heading=90°（正东）→ yaw = 0°
  DrConfig cfg = defaultConfig();
  cfg.gnss.heading_convention = "compass";
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(cfg));
  eskf.processEvent(Event::makeImu(imuAt(0.0)));

  for (double t = 0.005; t <= 1.0; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    GnssSample g;
    g.stamp = t;
    g.heading = 0.0;  // 正北
    g.attitude_valid = true;
    g.speed_mps = 5.0;
    eskf.processEvent(Event::makeGnss(g));
  }
  const double yaw = math::yawFromQuat(eskf.odometry(1.0).pose.orientation);
  EXPECT_NEAR(yaw, math::deg2rad(90.0), math::deg2rad(2.0));
}

TEST(EskfTest, GyroBiasObservableWithAttitudeAid) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  eskf.processEvent(Event::makeImu(imuAt(0.0)));

  const double true_bias = 0.01;  // rad/s 的 z 陀螺零偏（约 0.57°/s 漂移）
  for (double t = 0.005; t <= 30.0; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t, Vec3d(0, 0, true_bias))));
    if (std::fmod(t, 0.1) < 0.005) {
      GnssSample g;
      g.stamp = t;
      g.heading = 0.0;
      g.attitude_valid = true;
      g.speed_mps = 5.0;
      eskf.processEvent(Event::makeGnss(g));
    }
    if (std::fmod(t, 0.02) < 0.005) {
      WheelSample w;
      w.stamp = t;
      w.speed_mps = 0.0;
      eskf.processEvent(Event::makeWheel(w));
    }
  }
  // 姿态观测使 gyro bias 可观：估计应明显向真值收敛
  EXPECT_NEAR(eskf.state().gyro_bias.z(), true_bias, 0.006);
  EXPECT_LT(std::abs(eskf.state().gyro_bias.z() - true_bias), std::abs(true_bias));
}

TEST(EskfTest, PoseObservationUpdatesPosition) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);

  // 首帧定位观测：锚定 map 基准（"起点=map 起点"的自动推广）
  PoseSample anchor;
  anchor.stamp = 0.505;
  anchor.map_base.position = Vec3d(0.0, 0.0, 0.0);
  anchor.map_base.orientation = Quatd::Identity();
  eskf.processEvent(Event::makePose(anchor));

  // 小跳位姿观测（0.3m）：连续若干帧收敛（锚定后协方差小，单帧修正有限）
  for (int i = 0; i < 20; ++i) {
    PoseSample p;
    p.stamp = 0.51 + i * 0.005;
    p.map_base.position = Vec3d(0.3, 0.0, 0.0);
    p.map_base.orientation = Quatd::Identity();
    eskf.processEvent(Event::makePose(p));
  }

  const auto odom = eskf.odometry(0.51 + 19 * 0.005);
  EXPECT_NEAR(odom.pose.position.x(), 0.3, 0.1);
  EXPECT_GT(eskf.health(1.0).pose_update_count, 0u);
}

TEST(EskfTest, FirstPoseAnchorsMapDatum) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);

  // 首帧观测与当前估计差 100m：仍直接锚定（map 基准建立），不走跳变判断
  PoseSample anchor;
  anchor.stamp = 0.51;
  anchor.map_base.position = Vec3d(100.0, 50.0, 0.0);
  eskf.processEvent(Event::makePose(anchor));
  const auto odom = eskf.odometry(0.51);
  EXPECT_NEAR(odom.pose.position.x(), 100.0, 1e-6);
  EXPECT_EQ(eskf.health(0.51).pose_step_count, 1u);
}

TEST(EskfTest, PseudoJumpRejectedThenConfirmedStep) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);

  // 首帧锚定 map 基准
  PoseSample anchor;
  anchor.stamp = 0.505;
  anchor.map_base.position = Vec3d(0.0, 0.0, 0.0);
  eskf.processEvent(Event::makePose(anchor));

  // 超限跳变（20m）→ 拒收，位姿不动
  PoseSample p;
  p.stamp = 0.51;
  p.map_base.position = Vec3d(20.0, 0.0, 0.0);
  eskf.processEvent(Event::makePose(p));
  EXPECT_NEAR(eskf.odometry(0.51).pose.position.x(), 0.0, 1e-3);
  EXPECT_EQ(eskf.health(0.51).pose_reject_count, 1u);

  // 大跳（2m）连续 3 帧一致 → 台阶修正生效（零偏不动）
  for (int i = 0; i < 3; ++i) {
    PoseSample q;
    q.stamp = 0.6 + i * 0.05;
    q.map_base.position = Vec3d(2.0, 0.0, 0.0);
    eskf.processEvent(Event::makePose(q));
    feedStationary(eskf, q.stamp, q.stamp + 0.01, 0.005);
  }
  EXPECT_NEAR(eskf.odometry(0.75).pose.position.x(), 2.0, 0.05);
  EXPECT_EQ(eskf.health(0.75).pose_step_count, 2u);  // 锚定 1 次 + 大跳台阶 1 次
}

TEST(EskfTest, RelativeAttitudeModeUsesChangesOnly) {
  // 相对模式：世界系=初始姿态系；GNSS 姿态变化量被跟踪，绝对基准无关
  DrConfig cfg = defaultConfig();
  cfg.gnss.attitude_mode = "relative";
  cfg.gnss.heading_convention = "enu";
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(cfg));
  eskf.processEvent(Event::makeImu(imuAt(0.0)));

  // 初始 yaw=10°（人为给 DR 一个非零初始姿态基准）
  DrState s = eskf.state();
  s.orientation = math::quatFromYaw(math::deg2rad(10.0));
  eskf.reset(s);
  eskf.processEvent(Event::makeImu(imuAt(0.01)));

  // GNSS heading 渐变 0→+30（enu 约定，0.5s 转向，真实车辆不会阶跃）：
  // 世界姿态应从基准 10° 转到 40°
  for (double t = 0.01; t <= 3.0; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    GnssSample g;
    g.stamp = t;
    const double ramp = std::min(1.0, std::max(0.0, (t - 1.0) / 0.5));
    g.heading = math::deg2rad(30.0) * ramp;
    g.attitude_valid = true;
    g.speed_mps = 5.0;
    eskf.processEvent(Event::makeGnss(g));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 5.0;
    eskf.processEvent(Event::makeWheel(w));
  }
  const double yaw = math::yawFromQuat(eskf.odometry(3.0).pose.orientation);
  EXPECT_NEAR(yaw, math::deg2rad(40.0), math::deg2rad(3.0));  // 10 + 30
}

TEST(EskfTest, OutputExtrapolation) {
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 1.0, 0.005);
  const auto odom = eskf.odometry(1.05);  // 外推 0.05s
  ASSERT_TRUE(odom.valid);
  EXPECT_NEAR(odom.timestamp, 1.05, 1e-6);
}

TEST(EskfTest, LatePoseObservationExtrapolatedByVelocity) {
  // ④：位姿观测带扫描戳（滞后状态时刻）——按速度一阶外推到状态时刻再比对。
  // 构造：状态匀速 1m/s 前进，观测滞后 0.2s 且位姿是"滞后时刻真值"。
  // 无外推时残差 = -v·lag，位置被向后拽；有外推时残差≈0，位置不动。
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);

  // 首帧锚定 map 基准
  PoseSample anchor;
  anchor.stamp = 0.505;
  anchor.map_base.position = Vec3d(0.0, 0.0, 0.0);
  eskf.processEvent(Event::makePose(anchor));

  // 匀速 1m/s：IMU 无加速度（重力比力），轮速持续喂 1.0 维持速度态
  for (double t = 0.51; t <= 1.0 + 1e-9; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 1.0;
    eskf.processEvent(Event::makeWheel(w));
  }
  const double x_before = eskf.odometry(1.0).pose.position.x();
  ASSERT_GT(x_before, 0.3);  // 确认状态确实在前进（速度建立）

  // 迟到观测：扫描戳 0.8（滞后 0.2s），位姿 = 0.8 时刻真值 ≈ (x_before - 1.0*0.2)
  PoseSample late;
  late.stamp = 0.8;
  late.map_base.position = Vec3d(x_before - 1.0 * 0.2, 0.0, 0.0);
  late.map_base.orientation = Quatd::Identity();
  eskf.processEvent(Event::makePose(late));

  // 外推补上 v·lag：残差≈0 → 位置不应被向后拽（无外推会被拽 ~0.2·K）
  const double x_after = eskf.odometry(1.0).pose.position.x();
  EXPECT_NEAR(x_after, x_before, 0.05);
  EXPECT_GT(eskf.health(1.0).pose_update_count, 0u);  // 观测被正常接收（未被当迟到丢）
}






TEST(EskfTest, RelativeModeIgnoresGnssPresetBeforeImu) {
  // 回归：GNSS 先于首帧 IMU 到达时，relative 模式不得把 GNSS 绝对姿态预置成初始朝向
  DrConfig cfg = defaultConfig();
  cfg.gnss.attitude_mode = "relative";
  cfg.gnss.heading_convention = "compass";
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(cfg));

  // GNSS 先到：compass heading=232° → 绝对换算 yaw=-142°（旧 bug 会预置成这个值）
  GnssSample g0;
  g0.stamp = 0.0;
  g0.heading = math::deg2rad(232.0);
  g0.attitude_valid = true;
  g0.speed_mps = 5.0;
  eskf.processEvent(Event::makeGnss(g0));  // 滤波未初始化

  eskf.processEvent(Event::makeImu(imuAt(0.01)));  // 初始化：世界系=初始车体姿态

  for (double t = 0.015; t <= 2.0; t += 0.005) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    GnssSample g;
    g.stamp = t;
    g.heading = math::deg2rad(232.0);  // 航向恒定 → 变化量为零
    g.attitude_valid = true;
    g.speed_mps = 5.0;
    eskf.processEvent(Event::makeGnss(g));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 5.0;
    eskf.processEvent(Event::makeWheel(w));
  }
  // 世界 x 轴应保持初始车体朝向（yaw≈0），而非 GNSS 绝对朝向（-142°）
  const double yaw_deg = math::yawFromQuat(eskf.odometry(2.0).pose.orientation) * 180.0 / math::kPi;
  printf("[DBG-init] final yaw=%.2f deg\n", yaw_deg);
  EXPECT_NEAR(yaw_deg, 0.0, 2.0);
  // 轨迹应沿 +x（车头朝前行驶）
  const auto odom = eskf.odometry(2.0);
  EXPECT_GT(odom.pose.position.x(), 1.0);
  EXPECT_NEAR(odom.pose.position.y(), 0.0, 0.5);
}

TEST(EskfTest, BiasFrozenKeepsValueWhenNoCalibrationSignal) {
  // 零偏冻结语义（2026-10-13）：无标定信号期（无被接受的定位/GNSS 姿态/加速度观测）→
  // bg/ba【数值】保持冻结前水平（只置零增量，不是置 0），即使轮速残差经协方差交叉项
  // 会尝试归因到零偏；速度/姿态修正不受影响。
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);

  // "冻结前已校准"的零偏 + 错误速度（制造轮速残差，交叉项会尝试拽 ba/bg）
  DrState s = eskf.state();
  s.gyro_bias = Vec3d(0.0, 0.0, 0.01);
  s.velocity = Vec3d(5.0, 0.0, 0.0);
  eskf.reset(s);
  eskf.processEvent(Event::makeImu(imuAt(0.51)));

  for (double t = 0.515; t <= 2.0; t += 0.02) {
    eskf.processEvent(Event::makeImu(imuAt(t)));
    WheelSample w;
    w.stamp = t;
    w.speed_mps = 0.0;
    eskf.processEvent(Event::makeWheel(w));
  }
  EXPECT_TRUE(eskf.health(2.0).bias_frozen);
  EXPECT_NEAR(eskf.state().gyro_bias.z(), 0.01, 1e-9);     // 数值保持冻结前，不是置 0
  EXPECT_NEAR(eskf.state().accel_bias.norm(), 0.0, 1e-9);  // 增量置零（原本就是 0）
  EXPECT_LT(eskf.state().velocity.x(), 5.0);               // 速度修正照常（只冻零偏增量）
}

TEST(EskfTest, AcceptedPoseUnfreezesBiasAndRefillsCovariance) {
  // 解冻路径：被接受的定位观测 = 标定信号 → 解冻 + 零偏协方差回充到先验
  //（冻结期 (I-KH) 经交叉项假收缩，不回充会对未观测过的零偏过度自信）
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(defaultConfig()));
  feedStationary(eskf, 0.0, 0.5, 0.005);
  EXPECT_TRUE(eskf.health(0.5).bias_frozen);  // 从未有标定信号 → 冻结

  PoseSample anchor;
  anchor.stamp = 0.505;
  anchor.map_base.position = Vec3d(0.0, 0.0, 0.0);
  anchor.map_base.orientation = Quatd::Identity();
  eskf.processEvent(Event::makePose(anchor));  // 首帧锚定 = 被接受

  EXPECT_FALSE(eskf.health(0.505).bias_frozen);
  const Mat15d P = eskf.state().covariance;
  EXPECT_GE(P(9, 9), 0.009);   // P_bg 回充 ≈ 0.01
  EXPECT_GE(P(12, 12), 0.04);  // P_ba 回充 ≈ 0.05
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
