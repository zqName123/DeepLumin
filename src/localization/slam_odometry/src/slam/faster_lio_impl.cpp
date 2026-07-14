/**
 * @file faster_lio_impl.cpp
 * @brief Faster-LIO SLAM（ISlamOdometry 接口 → LaserMappingCore）
 */
#include "localization/slam/faster_lio_impl.hpp"

#include <glog/logging.h>

#include <cstdint>

namespace localization {

/**
    初始化：创建laserMappingCore实例，并初始化glog日志

*/
bool FasterLIOImpl::initialize(const SlamConfig& cfg) {
    config_ = cfg;
    core_ = std::make_unique<LaserMappingCore>();
    static bool glog_inited = false;  // 初始化glog
    if (!glog_inited) {
        google::InitGoogleLogging("faster_lio");
        FLAGS_logtostderr = 1; // 输出到标准错误
        FLAGS_minloglevel = google::INFO; // 最小日志级别为INFO
        glog_inited = true;
    }
    LOG(INFO) << "[SLAM][Impl] initialize start, type=" << cfg.type; // 打印日志
    const bool ok = core_->initialize(cfg); // 初始化核心
    LOG(INFO) << "[SLAM][Impl] initialize " << (ok ? "OK" : "FAILED"); // 打印日志
    return ok;
}

void FasterLIOImpl::feedImu(const ImuData& imu) {
    if (!core_) {
        LOG(WARNING) << "[SLAM][Impl][FeedImu] skip: core null";
        return;
    }
    core_->feedImu(imu);
}

void FasterLIOImpl::feedLidar(const TimestampedPointCloud& pc) {
    if (!core_) {
        LOG(WARNING) << "[SLAM][Impl][FeedLidar] skip: core null";
        return;
    }

    core_->feedLidar(pc);
}

void FasterLIOImpl::feedLidarCloud(double timestamp, PointCloudPtr cloud) {
    if (!core_) {
        LOG(WARNING) << "[SLAM][Impl][FeedLidar] skip: core null";
        return;
    }
    core_->feedLidarCloud(timestamp, cloud);
}

void FasterLIOImpl::feedDynamicMask(const TimestampedPointCloud& mask) {
    (void)mask;
}

bool FasterLIOImpl::processOnce() {
    if (!core_) {
        LOG(WARNING) << "[SLAM][Impl][Process] skip: core null";
        return false;
    }
    const bool ok = core_->processOnce();
    if (ok) {
        const auto odom = core_->getOdometry();
        LOG(INFO) << "[SLAM][Impl][Process] OK ts=" << odom.timestamp << " valid=" << odom.valid
                  << " pos=(" << odom.position.transpose() << ")";
    }
    return ok;
}

OdomResult FasterLIOImpl::getOdometry() const {
    return core_ ? core_->getOdometry() : OdomResult();
}

TimestampedPointCloud FasterLIOImpl::getLocalMap() const {
    return core_ ? core_->getLocalMap() : TimestampedPointCloud();
}

TimestampedPointCloud FasterLIOImpl::getCurrentScan() const {
    return core_ ? core_->getCurrentScan() : TimestampedPointCloud();
}

bool FasterLIOImpl::isDegenerate() const {
    return core_ && core_->isDegenerate();
}

double FasterLIOImpl::getDegeneracyFactor() const {
    return core_ ? core_->getDegeneracyFactor() : 0.0;
}

void FasterLIOImpl::reset() {
    LOG(INFO) << "[SLAM][Impl] reset";
    if (core_) {
        core_->reset();
    }
}

}  // namespace localization
