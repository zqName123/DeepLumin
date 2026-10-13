#include <gtest/gtest.h>

#include "dr_odometry_v2/core/event_queue.hpp"

using namespace dr_odometry_v2;

namespace {
Event imuAt(double t) {
  ImuSample s;
  s.stamp = t;
  s.gyro = Vec3d(0, 0, 0);
  s.accel = Vec3d(0, 0, 9.8);
  return Event::makeImu(s);
}
}  // namespace

TEST(EventQueueTest, PopInTimeOrder) {
  EventQueue q(16);
  ASSERT_TRUE(q.push(imuAt(3.0)));
  ASSERT_TRUE(q.push(imuAt(1.0)));
  ASSERT_TRUE(q.push(imuAt(2.0)));

  Event e;
  ASSERT_TRUE(q.popNext(&e));
  EXPECT_DOUBLE_EQ(e.stamp, 1.0);
  ASSERT_TRUE(q.popNext(&e));
  EXPECT_DOUBLE_EQ(e.stamp, 2.0);
  ASSERT_TRUE(q.popNext(&e));
  EXPECT_DOUBLE_EQ(e.stamp, 3.0);
  EXPECT_FALSE(q.popNext(&e));
}

TEST(EventQueueTest, LateEventDropped) {
  EventQueue q(16);
  ASSERT_TRUE(q.push(imuAt(1.0)));
  Event e;
  ASSERT_TRUE(q.popNext(&e));
  // 迟到事件（stamp 已过水位）→ 丢弃计数
  ASSERT_TRUE(q.push(imuAt(0.5)));
  EXPECT_FALSE(q.popNext(&e));
  EXPECT_EQ(q.reorderedDropped(), 1u);
}

namespace {
Event poseAt(double t) {
  PoseSample p;
  p.stamp = t;
  return Event::makePose(p);
}
}  // namespace

TEST(EventQueueTest, LatePoseWithinToleranceAccepted) {
  // ④：位姿观测带扫描戳（晚于水位到达）——容忍窗内放行、水位不回退
  EventQueue q(16);
  q.setPoseLateTolerance(0.3);
  ASSERT_TRUE(q.push(imuAt(1.0)));
  Event e;
  ASSERT_TRUE(q.popNext(&e));  // 水位 → 1.0

  ASSERT_TRUE(q.push(poseAt(0.85)));  // 迟到 0.15s ≤ 容忍 0.3s
  ASSERT_TRUE(q.popNext(&e));
  EXPECT_EQ(e.type, EventType::kPose);
  EXPECT_DOUBLE_EQ(e.stamp, 0.85);
  EXPECT_DOUBLE_EQ(q.processedWatermark(), 1.0);  // 水位不回退
  EXPECT_EQ(q.reorderedDropped(), 0u);
}

TEST(EventQueueTest, LatePoseBeyondToleranceDropped) {
  EventQueue q(16);
  q.setPoseLateTolerance(0.3);
  ASSERT_TRUE(q.push(imuAt(1.0)));
  Event e;
  ASSERT_TRUE(q.popNext(&e));

  ASSERT_TRUE(q.push(poseAt(0.5)));  // 迟到 0.5s > 容忍 0.3s
  EXPECT_FALSE(q.popNext(&e));
  EXPECT_EQ(q.reorderedDropped(), 1u);
}

TEST(EventQueueTest, LateImuNeverTolerated) {
  // 迟到容忍只对位姿观测开放：IMU 迟到仍丢（时间主轴不可乱）
  EventQueue q(16);
  q.setPoseLateTolerance(0.3);
  ASSERT_TRUE(q.push(imuAt(1.0)));
  Event e;
  ASSERT_TRUE(q.popNext(&e));

  ASSERT_TRUE(q.push(imuAt(0.9)));  // 迟到 0.1s 在容忍窗内，但类型非位姿 → 丢
  EXPECT_FALSE(q.popNext(&e));
  EXPECT_EQ(q.reorderedDropped(), 1u);
}

TEST(EventQueueTest, InvalidStampRejected) {
  EventQueue q(16);
  Event bad = imuAt(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(q.push(bad));
  EXPECT_EQ(q.invalidDropped(), 1u);
}

TEST(EventQueueTest, OverflowRejected) {
  EventQueue q(2);
  EXPECT_TRUE(q.push(imuAt(1.0)));
  EXPECT_TRUE(q.push(imuAt(2.0)));
  EXPECT_FALSE(q.push(imuAt(3.0)));
  EXPECT_EQ(q.overflowDropped(), 1u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
