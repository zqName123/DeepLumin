#pragma once

#include "localization/interface/i_slam_odometry.hpp"
#include "localization/slam/laser_mapping_core.hpp"

#include <memory>

namespace localization {

/**
    FasterLio的实现类：薄封装，只负责接口转换和日志打印
*/
class FasterLIOImpl : public ISlamOdometry {
public:
    bool initialize(const SlamConfig& cfg) override;
    void feedImu(const ImuData& imu) override;
    void feedLidar(const TimestampedPointCloud& pc) override;
    // 对齐 ysw StandardPCLCallBack 入队逻辑（预处理在 ROS 层完成） 
    void feedLidarCloud(double timestamp, PointCloudPtr cloud);
    void feedDynamicMask(const TimestampedPointCloud& mask) override;
    bool processOnce() override;
    OdomResult getOdometry() const override;
    TimestampedPointCloud getLocalMap() const override;
    TimestampedPointCloud getCurrentScan() const override;
    bool isDegenerate() const override;
    double getDegeneracyFactor() const override;
    void reset() override;

private:
    std::unique_ptr<LaserMappingCore> core_;
    SlamConfig config_;
};

}  // namespace localization
