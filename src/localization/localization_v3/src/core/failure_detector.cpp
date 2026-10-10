#include "localization_v3/core/failure_detector.hpp"

namespace localization_v3 {

RejectClass FailureDetector::classifyRejectReason(const std::string& reason) {
  if (reason.empty()) {
    return RejectClass::kNone;
  }
  // 出图 / 地图不可用
  if (reason.find("cropped target empty") != std::string::npos ||
      reason.find("map chunks not ready") != std::string::npos ||
      reason.find("target map not ready") != std::string::npos ||
      reason.find("target map points below") != std::string::npos) {
    return RejectClass::kMapMiss;
  }
  // 瞬态（数据未就绪）
  if (reason.find("waiting source cloud") != std::string::npos ||
      reason.find("source cloud timeout") != std::string::npos ||
      reason.find("source points below") != std::string::npos ||
      reason.find("manual pose reset during match") != std::string::npos) {
    return RejectClass::kTransient;
  }
  // 其余视为质量问题（inlier/rmse/fitness/delta 不达标等）
  return RejectClass::kQuality;
}

void FailureDetector::reset() {
  map_miss_streak_ = 0;
  last_reject_class_ = RejectClass::kNone;
  last_reject_reason_.clear();
  last_correction_stamp_ = -1.0;
  last_slam_stamp_ = -1.0;
  tracking_lost_since_ = -1.0;
}

void FailureDetector::onGmResult(bool success, QualityLevel quality,
                                 const std::string& reject_reason, double now_sec) {
  (void)now_sec;
  if (success && quality != QualityLevel::kBad) {
    map_miss_streak_ = 0;
    last_reject_class_ = RejectClass::kNone;
    last_reject_reason_.clear();
    return;
  }
  // 失败 / 坏质：按拒因分类
  const RejectClass cls =
      success ? RejectClass::kQuality : classifyRejectReason(reject_reason);
  last_reject_class_ = cls;
  last_reject_reason_ = reject_reason;
  if (cls == RejectClass::kMapMiss) {
    ++map_miss_streak_;
  } else {
    map_miss_streak_ = 0;
  }
}

void FailureDetector::onCorrectionAccepted(double now_sec) {
  last_correction_stamp_ = now_sec;
}

void FailureDetector::onTrackingLost(double now_sec) {
  tracking_lost_since_ = now_sec;
}

void FailureDetector::onTrackingRecovered(double now_sec) {
  (void)now_sec;
  tracking_lost_since_ = -1.0;
}

void FailureDetector::onSlamUpdate(double now_sec) {
  last_slam_stamp_ = now_sec;
}

bool FailureDetector::noCorrectionTimeout(double now_sec) const {
  return last_correction_stamp_ >= 0.0 &&
         (now_sec - last_correction_stamp_) > config_.no_correction_timeout_sec;
}

bool FailureDetector::lostTimeout(double now_sec) const {
  return tracking_lost_since_ >= 0.0 &&
         (now_sec - tracking_lost_since_) > config_.lost_timeout_sec;
}

bool FailureDetector::slamTimeout(double now_sec) const {
  return last_slam_stamp_ >= 0.0 && (now_sec - last_slam_stamp_) > config_.slam_timeout_sec;
}

std::string FailureDetector::failureReason(double now_sec) const {
  if (slamTimeout(now_sec)) {
    return "slam odometry timeout";
  }
  if (mapMissExceeded()) {
    return "out of map (" + last_reject_reason_ + ")";
  }
  if (lostTimeout(now_sec)) {
    return "recovery timeout, no confirmed relocalization";
  }
  if (noCorrectionTimeout(now_sec)) {
    return "no accepted pose correction within timeout";
  }
  return "";
}

}  // namespace localization_v3
