#pragma once

#include "localization/common/config.hpp"
#include "localization/common/types.hpp"

#include <memory>
#include <mutex>

namespace localization {

/**
 * @brief Faster-LIO 核心（无 ROS）：IMU 去畸变 + iVox 地图 + IEKF 状态估计
 */
class LaserMappingCore {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LaserMappingCore();
    ~LaserMappingCore();

    bool initialize(const SlamConfig& cfg);
    void feedImu(const ImuData& imu);
    void feedLidar(const TimestampedPointCloud& pc);
    void feedCanVelocity(const CanVelocityData& can);
    void feedGnssAttitude(const GnssAttitudeData& gnss);
    /** 对齐 ysw StandardPCLCallBack：持锁入队，不拷贝点云 */
    void feedLidarCloud(double timestamp, PointCloudPtr cloud);
    bool processOnce();
    void reset();

    OdomResult getOdometry() const;
    TimestampedPointCloud getLocalMap() const;
    TimestampedPointCloud getCurrentScan() const;
    /** 有效对应点（effect points，body 系）：仅包含里程计匹配中通过有效平面对应筛选的点 */
    TimestampedPointCloud getEffectScan() const;
    bool isDegenerate() const;
    double getDegeneracyFactor() const;

private:
    struct Impl; // 核心实现的结构体
    std::unique_ptr<Impl> impl_; // 核心实现的指针
    mutable std::mutex mutex_; // 互斥锁

    SlamConfig config_; // 配置
    OdomResult odom_;
    TimestampedPointCloud local_map_; // 局部地图
    TimestampedPointCloud current_scan_; // 当前扫描
    TimestampedPointCloud effect_scan_; // 有效对应点（body 系）
    bool initialized_ = false;
    bool is_degenerate_ = false; // 是否退化
    double degeneracy_factor_ = 0.0;

    bool applyYswVelocityAid();

    bool applyCanVelocityUpdate(); // 速度采用观测方式融合
};

}  // namespace localization
