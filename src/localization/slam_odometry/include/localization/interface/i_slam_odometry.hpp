#pragma once

#include "localization/common/config.hpp"
#include "localization/common/types.hpp"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <string>

namespace localization {
/**
    SLAM里程计统一的抽象接口：纯虚基类
*/
class ISlamOdometry {
public:
    virtual ~ISlamOdometry() = default; // 虚析构函数，保证基类指针可以指向派生类对象

    virtual bool initialize(const SlamConfig& cfg) = 0; // 初始化
    // 数据输出
    virtual void feedImu(const ImuData& imu) = 0;
    virtual void feedLidar(const TimestampedPointCloud& pc) = 0;
    virtual void feedCanVelocity(const CanVelocityData& can) = 0;
    virtual void feedGnssAttitude(const GnssAttitudeData& gnss) = 0;
    virtual void feedDynamicMask(const TimestampedPointCloud& mask) = 0;
    // 计算主逻辑
    virtual bool processOnce() = 0;
    virtual OdomResult getOdometry() const = 0; // 获取里程计结果
    virtual TimestampedPointCloud getLocalMap() const = 0; // 获取局部地图
    virtual TimestampedPointCloud getCurrentScan() const = 0; // 获取当前扫描
    virtual TimestampedPointCloud getEffectScan() const = 0; // 获取有效对应点（body 系）
    virtual bool isDegenerate() const = 0; // 退化判断
    virtual double getDegeneracyFactor() const = 0; // 获取退化因子
    virtual void reset() = 0; // 重置

    // 原始消息直通（可选能力）：
    // fastlio2 等实现直接消费原始 sensor 消息，由算法自身预处理配置处理，
    // 避免节点侧二次预处理（坐标旋转/字段丢失）造成不一致。默认关闭。
    virtual bool preferRawFeed() const { return false; }
    virtual void feedImuRaw(const sensor_msgs::Imu& msg) { (void)msg; }
    virtual void feedLidarRaw(const sensor_msgs::PointCloud2& msg) { (void)msg; }
};

class ISlamOdometryFactory {
public:
    virtual ~ISlamOdometryFactory() = default;
    virtual std::shared_ptr<ISlamOdometry> create(const std::string& type) = 0;
};

}  // namespace localization
