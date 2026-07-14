#pragma once

#include "localization/common/config.hpp"
#include "localization/common/types.hpp"
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
    virtual void feedDynamicMask(const TimestampedPointCloud& mask) = 0;
    // 计算主逻辑
    virtual bool processOnce() = 0;
    virtual OdomResult getOdometry() const = 0; // 获取里程计结果
    virtual TimestampedPointCloud getLocalMap() const = 0; // 获取局部地图
    virtual TimestampedPointCloud getCurrentScan() const = 0; // 获取当前扫描
    virtual bool isDegenerate() const = 0; // 退化判断
    virtual double getDegeneracyFactor() const = 0; // 获取退化因子
    virtual void reset() = 0; // 重置
};

class ISlamOdometryFactory {
public:
    virtual ~ISlamOdometryFactory() = default;
    virtual std::shared_ptr<ISlamOdometry> create(const std::string& type) = 0;
};

}  // namespace localization
