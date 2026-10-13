#include <gtest/gtest.h>

#include "localization_v3/core/failure_detector.hpp"

using namespace localization_v3;

namespace {

FailureDetectorConfig defaultConfig() {
  FailureDetectorConfig cfg;
  cfg.map_miss_count = 3;
  cfg.no_correction_timeout_sec = 3.0;
  cfg.lost_timeout_sec = 3.0;
  cfg.slam_timeout_sec = 0.5;
  return cfg;
}

}  // namespace

TEST(FailureDetectorTest, ClassifyRejectReasons) {
  EXPECT_EQ(FailureDetector::classifyRejectReason("cropped target empty"),
            RejectClass::kMapMiss);
  EXPECT_EQ(FailureDetector::classifyRejectReason("map chunks not ready"), RejectClass::kMapMiss);
  EXPECT_EQ(FailureDetector::classifyRejectReason("target map not ready"), RejectClass::kMapMiss);
  EXPECT_EQ(FailureDetector::classifyRejectReason("inlier ratio below threshold"),
            RejectClass::kQuality);
  EXPECT_EQ(FailureDetector::classifyRejectReason("inlier rmse above threshold"),
            RejectClass::kQuality);
  EXPECT_EQ(FailureDetector::classifyRejectReason("waiting source cloud"), RejectClass::kTransient);
  EXPECT_EQ(FailureDetector::classifyRejectReason("source cloud timeout"), RejectClass::kTransient);
  EXPECT_EQ(FailureDetector::classifyRejectReason(""), RejectClass::kNone);
}

TEST(FailureDetectorTest, MapMissStreakTriggers) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onSlamUpdate(0.0);
  fd.onCorrectionAccepted(0.0);

  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.1);
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.2);
  EXPECT_FALSE(fd.mapMissExceeded());
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.3);
  EXPECT_TRUE(fd.mapMissExceeded());
  EXPECT_NE(fd.failureReason(0.3).find("out of map"), std::string::npos);
}

TEST(FailureDetectorTest, QualityFailureResetsMapMissStreak) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.1);
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.2);
  fd.onGmResult(false, QualityLevel::kBad, "inlier ratio below threshold", 0.3);
  EXPECT_FALSE(fd.mapMissExceeded());
  EXPECT_EQ(fd.mapMissStreak(), 0);
}

TEST(FailureDetectorTest, GoodResultResetsMapMissStreak) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.1);
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.2);
  fd.onGmResult(true, QualityLevel::kGood, "", 0.3);
  EXPECT_FALSE(fd.mapMissExceeded());
}

TEST(FailureDetectorTest, TransientNotCountedAsMapMiss) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  for (int i = 0; i < 10; ++i) {
    fd.onGmResult(false, QualityLevel::kBad, "source cloud timeout", 0.1 * i);
  }
  EXPECT_FALSE(fd.mapMissExceeded());
  EXPECT_EQ(fd.lastRejectClass(), RejectClass::kTransient);
}

TEST(FailureDetectorTest, NoCorrectionWatchdog) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onCorrectionAccepted(0.0);
  EXPECT_FALSE(fd.noCorrectionTimeout(2.0));
  EXPECT_TRUE(fd.noCorrectionTimeout(3.5));
  // 有新修正后复位
  fd.onCorrectionAccepted(3.5);
  EXPECT_FALSE(fd.noCorrectionTimeout(4.0));
}

TEST(FailureDetectorTest, LostTimeout) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onTrackingLost(1.0);
  EXPECT_FALSE(fd.lostTimeout(3.0));
  EXPECT_TRUE(fd.lostTimeout(4.5));
  fd.onTrackingRecovered(4.5);
  EXPECT_FALSE(fd.lostTimeout(10.0));
}

TEST(FailureDetectorTest, SlamTimeout) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onSlamUpdate(0.0);
  EXPECT_FALSE(fd.slamTimeout(0.3));
  EXPECT_TRUE(fd.slamTimeout(0.6));
  EXPECT_NE(fd.failureReason(0.6).find("slam odometry timeout"), std::string::npos);
}

TEST(FailureDetectorTest, ResetClearsEverything) {
  FailureDetector fd;
  fd.configure(defaultConfig());
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.1);
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.2);
  fd.onGmResult(false, QualityLevel::kBad, "cropped target empty", 0.3);
  ASSERT_TRUE(fd.mapMissExceeded());
  fd.reset();
  EXPECT_FALSE(fd.mapMissExceeded());
  EXPECT_FALSE(fd.noCorrectionTimeout(100.0));
  EXPECT_FALSE(fd.lostTimeout(100.0));
  EXPECT_FALSE(fd.slamTimeout(100.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
