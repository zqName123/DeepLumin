#pragma once

#include "localization/common/config.hpp"
#include "localization/common/types.hpp"
#include "localization/interface/i_slam_odometry.hpp"

#include <memory>

namespace localization {

/**
 * @brief FAST-LIO2（faster_lio_ws）里程计适配实现
 *
 * 通过 ISlamOdometry 接口解耦接入 faster_lio_ws 的 LaserMapping 核心，
 * 作为 slam/type=fastlio2 的新工厂类型使用。faster-lio 已在 Release(-O3)
 * 下稳定运行，规避原 FasterLIOImpl 在 Release 下的矩阵计算问题。
 * 使用 PIMPL 隔离 faster_lio 头文件，避免与 vendored thirdparty/faster_lio
 * 的同名头文件冲突。
 */
class FastLio2WsImpl : public ISlamOdometry {
 public:
  bool initialize(const SlamConfig& cfg) override;
  void feedImu(const ImuData& imu) override;
  void feedLidar(const TimestampedPointCloud& pc) override;
  void feedCanVelocity(const CanVelocityData& can) override;
  void feedGnssAttitude(const GnssAttitudeData& gnss) override;
  void feedDynamicMask(const TimestampedPointCloud& mask) override;
  bool processOnce() override;
  OdomResult getOdometry() const override;
  TimestampedPointCloud getLocalMap() const override;
  TimestampedPointCloud getCurrentScan() const override;
  TimestampedPointCloud getEffectScan() const override;
  bool isDegenerate() const override;
  double getDegeneracyFactor() const override;
  void reset() override;

  // 原始消息直通：与 faster_lio_ws 独立运行保持一致
  bool preferRawFeed() const override { return true; }
  void feedImuRaw(const sensor_msgs::Imu& msg) override;
  void feedLidarRaw(const sensor_msgs::PointCloud2& msg) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/// 注册 fastlio2 工厂类型（由 slam_odometry_node 启动时显式调用，避免 --as-needed 丢弃注册）
void registerFastLio2Plugin();

}  // namespace localization
