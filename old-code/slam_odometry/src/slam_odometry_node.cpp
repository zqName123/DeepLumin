/**
 * @file slam_odometry_node.cpp
 * @brief SLAM 里程计 ROS 节点实现
 * 
 * 该节点实现了基于 Faster-LIO 的激光雷达惯性里程计功能，主要功能包括：
 * - 订阅 IMU、激光雷达、轮速计、GNSS 等传感器数据
 * - 运行 Faster-LIO 算法进行实时位姿估计
 * - 发布里程计、局部地图、配准点云等输出
 * - 支持 IMU 预热、点云时间戳自动检测、退化场景处理等特性
 * 
 * @author DeepLumin Team
 * @date 2024
 */

// =============================================================================
// 头文件包含
// =============================================================================

// DeepLumin 本地化模块头文件
#include <localization/factory/factory_registry.hpp>    // SLAM 工厂注册
#include <localization/ros/ros_converters.hpp>          // ROS 消息转换工具
#include <localization/ros/global_map_viz.hpp>          // 全局地图可视化
#include <localization/ros/sensor_topics.hpp>           // 传感器话题配置
#include <localization/slam/faster_lio_impl.hpp>        // Faster-LIO 实现
#include <localization/slam/fastlio2_ws_impl.hpp>     // FAST-LIO2 (faster_lio_ws) 实现

// ROS 标准消息头文件
#include <nav_msgs/Odometry.h>                          // 里程计消息
#include <ros/ros.h>                                    // ROS 核心
#include <sensor_msgs/Imu.h>                            // IMU 消息
#include <sensor_msgs/PointCloud2.h>                    // 点云消息
#include <tf/transform_broadcaster.h>                   // TF 变换广播

// DeepLumin 自定义消息
#include <deeplumin_msgs/CanReceiveInfo.h>              // CAN 轮速计消息
#include <deeplumin_msgs/Gpchc.h>                       // GNSS 姿态消息

// 标准库头文件
#include <algorithm>                                    // 算法工具
#include <cstdint>                                      // 整数类型定义
#include <string>                                       // 字符串处理


// =============================================================================
// 匿名命名空间：辅助函数
// =============================================================================
namespace {

/**
 * @brief 将传感器外参转换为 4x4 齐次变换矩阵
 * 
 * @param ext 传感器外参结构体，包含旋转矩阵和平移向量
 * @return Eigen::Matrix4d 4x4 齐次变换矩阵
 * 
 * 变换矩阵格式：
 * [ R  t ]
 * [ 0  1 ]
 * 其中 R 是 3x3 旋转矩阵，t 是 3x1 平移向量
 */
Eigen::Matrix4d sensorExtrinsicToMatrix(const localization::SensorExtrinsic& ext) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();           // 初始化为单位矩阵
    t.block<3, 3>(0, 0) = ext.rotation;                        // 填充旋转矩阵 (左上 3x3)
    t.block<3, 1>(0, 3) = ext.translation;                     // 填充平移向量 (右上 3x1)
    return t;
}

/**
 * @brief 将里程计结果转换为 4x4 齐次变换矩阵
 * 
 * @param odom 里程计结果，包含位置和姿态
 * @return Eigen::Matrix4d 4x4 齐次变换矩阵 (odom -> base_link)
 */
Eigen::Matrix4d odomToMatrix(const localization::OdomResult& odom) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    // 将四元数转换为旋转矩阵，并确保归一化
    t.block<3, 3>(0, 0) = odom.orientation.normalized().toRotationMatrix();
    t.block<3, 1>(0, 3) = odom.position;                       // 填充位置向量
    return t;
}

/**
 * @brief 将源里程计结果通过变换矩阵进行坐标变换
 * 
 * @param src 源里程计结果
 * @param transform 4x4 齐次变换矩阵
 * @return localization::OdomResult 变换后的里程计结果
 */
localization::OdomResult matrixToOdom(const localization::OdomResult& src,
                                      const Eigen::Matrix4d& transform) {
    localization::OdomResult out = src;                        // 复制源数据
    out.position = transform.block<3, 1>(0, 3);                // 提取新的位置
    // 提取新的旋转矩阵并转换为四元数，确保归一化
    out.orientation = localization::Quatd(transform.block<3, 3>(0, 0)).normalized();
    return out;
}

/**
 * @brief 将传感器坐标系下的里程计结果转换到 base_link 坐标系
 * 
 * 坐标变换链：odom -> sensor -> base_link
 * 即：T_odom_base = T_odom_sensor * T_sensor_base
 * 其中 T_sensor_base = T_base_sensor^{-1}
 * 
 * @param sensor_odom 传感器坐标系下的里程计结果
 * @param base_to_sensor base_link 到传感器的变换外参
 * @return localization::OdomResult base_link 坐标系下的里程计结果
 */
localization::OdomResult transformSensorOdomToBase(
    const localization::OdomResult& sensor_odom,
    const localization::SensorExtrinsic& base_to_sensor) {
    // 如果里程计无效，直接返回（避免无效计算）
    if (!sensor_odom.valid) {
        return sensor_odom;
    }
    // 计算 odom -> sensor 的变换矩阵
    const Eigen::Matrix4d t_odom_sensor = odomToMatrix(sensor_odom);
    // 计算 base -> sensor 的变换矩阵
    const Eigen::Matrix4d t_base_sensor = sensorExtrinsicToMatrix(base_to_sensor);
    // 应用变换：odom -> base = odom -> sensor -> base
    // 即：T_odom_base = T_odom_sensor * T_base_sensor^{-1}
    return matrixToOdom(sensor_odom, t_odom_sensor * t_base_sensor.inverse());
}

}  // namespace


// =============================================================================
// SlamOdometryNode 类定义
// =============================================================================
/**
 * @class SlamOdometryNode
 * @brief SLAM 里程计 ROS 节点主类
 * 
 * 该类封装了 SLAM 里程计节点的所有功能，包括：
 * - 参数加载与初始化
 * - 传感器数据订阅与回调处理
 * - SLAM 算法运行与结果发布
 * - TF 变换广播
 * - 特殊模式处理（IMU 预热、无点时间戳等）
 */
class SlamOdometryNode {
public:
    /**
     * @brief 构造函数：初始化节点、加载参数、创建 SLAM 实例
     * 
     * @param nh ROS 节点句柄（公有命名空间）
     * 
     * 初始化流程：
     * 1. 注册默认插件
     * 2. 从 ROS 参数服务器加载配置
     * 3. 创建并初始化 SLAM 实例
     * 4. 创建传感器订阅者和结果发布者
     * 5. 输出初始化日志
     */
    explicit SlamOdometryNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        // -------------------------------------------------------------------------
        // 步骤 1: 注册 SLAM 插件（工厂模式）
        // -------------------------------------------------------------------------
        localization::registerDefaultPlugins();

        // -------------------------------------------------------------------------
        // 步骤 2: 从 ROS 参数服务器加载配置
        // -------------------------------------------------------------------------
        // 加载坐标系配置（map, odom, base_link 等）
        frames_ = localization_ros::loadFrameConfig(pnh_);
        // 加载传感器外参（IMU、LiDAR 相对于 base_link 的变换）
        extrinsics_ = localization_ros::loadExtrinsicConfig(pnh_);
        // 加载传感器输入话题配置
        sensor_topics_ = localization_ros::loadSensorTopicsConfig(pnh_);
        // 加载输出话题配置
        output_topics_ = localization_ros::loadOutputTopicsConfig(pnh_);
        // 加载 SLAM 算法配置
        slam_config_ = localization_ros::loadSlamConfig(pnh_);

        // -------------------------------------------------------------------------
        // 步骤 3: 加载额外的运行时参数（不在 SlamConfig 结构体中）
        // -------------------------------------------------------------------------
        // TF 发布开关
        pnh_.param<bool>("slam/publish_tf", publish_tf_, true);
        
        // IMU 预热相关参数
        pnh_.param<bool>("slam/startup/enable_imu_warmup", enable_imu_warmup_, true);
        pnh_.param<double>("slam/startup/imu_warmup_duration", imu_warmup_duration_, 1.0);
        pnh_.param<int>("slam/startup/min_imu_samples", min_imu_samples_, 20);
        
        // 点云时间戳模式参数
        pnh_.param<std::string>("slam/point_time_mode", point_time_mode_, "auto");
        
        // 无点时间戳模式下的处理频率参数
        pnh_.param<double>("slam/no_point_time/startup_process_rate", no_point_time_startup_process_rate_, 2.0);
        pnh_.param<double>("slam/no_point_time/normal_process_rate", no_point_time_normal_process_rate_, 10.0);
        pnh_.param<int>("slam/no_point_time/startup_valid_frames", no_point_time_startup_valid_frames_, 20);

        // -------------------------------------------------------------------------
        // 步骤 4: 创建并初始化 SLAM 实例
        // -------------------------------------------------------------------------
        // 检查 SLAM 类型，当前支持 faster_lio / fastlio2
        if (slam_config_.type != "faster_lio" && slam_config_.type != "fastlio2") {
            ROS_WARN("[slam_odometry] unsupported slam/type '%s', using faster_lio",
                     slam_config_.type.c_str());
        }

        // 通过工厂模式创建 SLAM 实例
        slam_ = localization::SlamFactory::instance().create(slam_config_.type);
        if (!slam_) {
            ROS_ERROR("[slam_odometry] failed to create SLAM implementation for type: %s",
                      slam_config_.type.c_str());
            ros::shutdown();
            return;
        }
        
        // 初始化 SLAM 算法
        if (!slam_->initialize(slam_config_)) {
            ROS_ERROR("[slam_odometry] failed to initialize Faster-LIO");
            ros::shutdown();
            return;
        }

        // -------------------------------------------------------------------------
        // 步骤 5: 创建传感器数据订阅者
        // -------------------------------------------------------------------------
        // IMU 数据订阅（大缓冲区，防止数据丢失）
        imu_sub_ = nh_.subscribe(sensor_topics_.imu, 200000, &SlamOdometryNode::onImu, this);
        // 激光雷达点云订阅（大缓冲区）
        lidar_sub_ = nh_.subscribe(sensor_topics_.lidar, 200000, &SlamOdometryNode::onLidar, this);
        
        // 条件订阅：仅在启用轮速计时订阅 CAN 数据
        if (slam_config_.use_vel) {
            can_sub_ = nh_.subscribe(sensor_topics_.wheel, 1000, &SlamOdometryNode::onCan, this);
            // 条件订阅：仅在启用 GNSS 姿态时订阅 GNSS 数据
            if (slam_config_.use_gnss_attitude) {
                gnss_sub_ = nh_.subscribe(sensor_topics_.gnss, 100, &SlamOdometryNode::onGnss, this);
            }
        }

        // -------------------------------------------------------------------------
        // 步骤 6: 创建结果发布者
        // -------------------------------------------------------------------------
        // 里程计结果发布
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topics_.slam, 100);
        // 局部地图发布（降采样后的地图）
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topics_.local_map, 10);
        // 配准后的扫描点云发布
        registered_scan_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(output_topics_.registered_scan, 10);
        // 有效对应点（effect points）发布
        effect_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topics_.effect_scan, 10);
        // 全局地图可视化（从 PCD 文件加载）
        global_map_viz_.setup(nh_, pnh_, output_topics_.global_map, frames_.map, "slam_odometry");

        // -------------------------------------------------------------------------
        // 步骤 7: 输出初始化完成日志
        // -------------------------------------------------------------------------
        ROS_INFO("[slam_odometry] imu=%s lidar=%s can=%s gnss=%s odom=%s local_map=%s registered_scan=%s effect_scan=%s",
                 sensor_topics_.imu.c_str(), sensor_topics_.lidar.c_str(),
                 sensor_topics_.wheel.c_str(), sensor_topics_.gnss.c_str(),
                 output_topics_.slam.c_str(), output_topics_.local_map.c_str(),
                 output_topics_.registered_scan.c_str(), output_topics_.effect_scan.c_str());
        ROS_INFO("[slam_odometry] use_vel=%s use_gnss_attitude=%s can_diff_thresh=%.3fkm/h can_timeout=%.3fs",
                 slam_config_.use_vel ? "true" : "false",
                 slam_config_.use_gnss_attitude ? "true" : "false",
                 slam_config_.can_vel_diff_thresh_kmh, slam_config_.can_timeout_sec);
        ROS_INFO("[slam_odometry] startup_imu_warmup=%s duration=%.3fs min_samples=%d publish_tf=%s",
                 enable_imu_warmup_ ? "true" : "false", imu_warmup_duration_, min_imu_samples_,
                 publish_tf_ ? "true" : "false");
        ROS_INFO("[slam_odometry] point_time_mode=%s no_point_time_startup_rate=%.3fHz normal_rate=%.3fHz startup_valid_frames=%d",
                 point_time_mode_.c_str(), no_point_time_startup_process_rate_,
                 no_point_time_normal_process_rate_, no_point_time_startup_valid_frames_);
    }

    /**
     * @brief 主循环 tick 函数，每轮 ROS spin 调用一次
     * 
     * 执行流程：
     * 1. 检查是否应该运行 processOnce（处理频率控制）
     * 2. 调用 SLAM 算法处理一帧数据
     * 3. 获取并转换里程计结果到 base_link 坐标系
     * 4. 更新无点时间戳模式的启动状态
     * 5. 发布里程计、局部地图、配准点云等结果
     */
    void tick() {
        // 检查 SLAM 实例是否有效，以及是否应该运行处理
        if (!slam_ || !shouldRunProcessOnce()) {
            return;
        }
        
        // 调用 SLAM 算法处理一帧数据
        if (!slam_->processOnce()) {
            return;
        }

        // 获取传感器坐标系下的里程计结果
        const auto odom_sensor = slam_->getOdometry();
        // 转换到 base_link 坐标系（使用 IMU 外参，因为 Faster-LIO 输出在 IMU 坐标系）
        const auto odom = transformSensorOdomToBase(odom_sensor, extrinsics_.base_to_imu);
        // 更新无点时间戳模式的启动状态
        updateNoPointTimeStartupState(odom);
        
        // 获取局部地图和当前扫描
        const auto local_map = slam_->getLocalMap();
        const auto current_scan = slam_->getCurrentScan();
        const auto effect_scan = slam_->getEffectScan();

        // -------------------------------------------------------------------------
        // 发布里程计结果
        // -------------------------------------------------------------------------
        if (odom.valid) {
            // 发布 Odometry 消息
            odom_pub_.publish(localization_ros::toRosOdom(odom, frames_.odom, frames_.base_link));
            // 如果启用，广播 TF 变换
            if (publish_tf_) {
                tf_broadcaster_.sendTransform(
                    localization_ros::toRosTransform(odom, frames_.odom, frames_.base_link));
            }
        }

        // -------------------------------------------------------------------------
        // 发布局部地图
        // -------------------------------------------------------------------------
        if (local_map.cloud && !local_map.cloud->empty()) {
            auto map_msg = local_map;
            map_msg.frame_id = frames_.odom;  // 设置坐标系为 odom
            local_map_pub_.publish(localization_ros::toRosPointCloud(map_msg));
        }

        // -------------------------------------------------------------------------
        // 发布配准后的扫描点云 / 有效对应点云（转换到 odom 坐标系）
        // -------------------------------------------------------------------------
        if (odom.valid) {
            if (current_scan.cloud && !current_scan.cloud->empty()) {
                publishScanInOdom(odom, current_scan, registered_scan_pub_);
            }
            if (effect_scan.cloud && !effect_scan.cloud->empty()) {
                publishScanInOdom(odom, effect_scan, effect_scan_pub_);
            }
        }
    }

private:
    /**
     * @brief 将 body/lidar 系点云变换到 odom 系并发布
     *
     * 与 /localization/cloud_registered 使用同一变换链：
     * T_odom_lidar = T_odom_base * T_base_lidar
     */
    void publishScanInOdom(const localization::OdomResult& odom,
                           const localization::TimestampedPointCloud& scan,
                           const ros::Publisher& pub) {
        if (!scan.cloud || scan.cloud->empty()) {
            return;
        }
        localization::TimestampedPointCloud out;
        out.timestamp = scan.timestamp;
        out.frame_id = frames_.odom;
        out.cloud.reset(new localization::PointCloud());
        out.cloud->reserve(scan.cloud->size());

        // 计算 odom -> lidar 完整变换
        const Eigen::Matrix4d t_odom_lidar =
            odomToMatrix(odom) * sensorExtrinsicToMatrix(extrinsics_.base_to_lidar);
        const Eigen::Matrix3d R = t_odom_lidar.block<3, 3>(0, 0);
        const Eigen::Vector3d t = t_odom_lidar.block<3, 1>(0, 3);

        // 将点云从 lidar 坐标系转换到 odom 坐标系
        for (const auto& pt : scan.cloud->points) {
            const Eigen::Vector3d pw = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
            localization::PointType out_pt = pt;
            out_pt.x = static_cast<float>(pw.x());
            out_pt.y = static_cast<float>(pw.y());
            out_pt.z = static_cast<float>(pw.z());
            out.cloud->push_back(out_pt);
        }
        pub.publish(localization_ros::toRosPointCloud(out));
    }

    /**
     * @brief 更新 IMU 预热状态
     * 
     * @param stamp 当前 IMU 消息的时间戳（秒）
     * 
     * 功能：
     * - 记录第一帧和最后一帧 IMU 时间戳
     * - 统计 IMU 样本数
     * - 判断是否满足预热条件（时间和样本数双重条件）
     */
    void updateImuWarmup(double stamp) {
        // 记录第一帧 IMU 时间戳
        if (first_imu_stamp_ <= 0.0) {
            first_imu_stamp_ = stamp;
        }
        // 更新最后一帧 IMU 时间戳
        last_imu_stamp_ = stamp;
        // 增加样本计数
        ++imu_warmup_samples_;

        // 如果禁用预热或已完成预热，直接返回
        if (!enable_imu_warmup_ || imu_warmup_ready_) {
            return;
        }

        // 检查预热条件：持续时间足够且样本数足够
        const double duration = last_imu_stamp_ - first_imu_stamp_;
        if (duration >= imu_warmup_duration_ &&
            imu_warmup_samples_ >= static_cast<std::uint64_t>(std::max(1, min_imu_samples_))) {
            imu_warmup_ready_ = true;
            ROS_INFO("[slam_odometry] IMU warmup ready: samples=%lu duration=%.3fs dropped_lidar=%lu",
                     static_cast<unsigned long>(imu_warmup_samples_), duration,
                     static_cast<unsigned long>(dropped_lidar_before_warmup_));
        }
    }

    /**
     * @brief 检查是否可以向 SLAM 输入激光雷达数据
     * 
     * @return true 可以输入（禁用预热或已完成预热）
     * @return false 不可以输入（正在预热中）
     */
    bool canFeedLidar() const {
        return !enable_imu_warmup_ || imu_warmup_ready_;
    }

    /**
     * @brief 检查点云消息是否包含时间戳字段
     * 
     * @param msg ROS PointCloud2 消息
     * @return true 包含时间戳字段（字段名为 "t", "time" 或 "timestamp"）
     * @return false 不包含时间戳字段
     * 
     * 说明：不同激光雷达厂商使用不同的字段名表示点时间戳
     * - Ouster: "t"
     * - Velodyne: "time"
     * - 其他: "timestamp"
     */
    static bool hasPointTimeField(const sensor_msgs::PointCloud2& msg) {
        return std::any_of(msg.fields.begin(), msg.fields.end(), [](const auto& field) {
            return field.name == "t" || field.name == "time" || field.name == "timestamp";
        });
    }

    /**
     * @brief 判断当前是否处于无点时间戳模式
     * 
     * @param msg ROS PointCloud2 消息（用于自动检测）
     * @return true 处于无点时间戳模式
     * @return false 处于正常模式（点包含时间戳）
     * 
     * 模式说明：
     * - "none": 强制无点时间戳模式
     * - "present"/"curvature": 强制正常模式
     * - "auto": 自动检测第一帧点云，后续保持一致
     */
    bool noPointTimeMode(const sensor_msgs::PointCloud2& msg) {
        // 强制无点时间戳模式
        if (point_time_mode_ == "none") {
            point_time_known_ = true;
            point_time_available_ = false;
            return true;
        }
        
        // 强制正常模式（点时间戳存在）
        if (point_time_mode_ == "present" || point_time_mode_ == "curvature") {
            point_time_known_ = true;
            point_time_available_ = true;
            return false;
        }
        
        // 自动检测模式：仅在首次检测时执行
        if (!point_time_known_) {
            point_time_available_ = hasPointTimeField(msg);
            point_time_known_ = true;
            if (point_time_available_) {
                ROS_INFO("[slam_odometry] point time field detected, use normal Faster-LIO lidar feed");
            } else {
                ROS_WARN("[slam_odometry] point time field is missing, enable no-point-time compatible feed");
            }
        }
        return !point_time_available_;
    }

    /**
     * @brief 检查当前是否处于无点时间戳激活状态
     * 
     * @return true 已确定无点时间戳且处于该模式
     * @return false 正常模式或尚未确定
     */
    bool noPointTimeActive() const {
        return point_time_known_ && !point_time_available_;
    }

    /**
     * @brief 判断是否应该运行 processOnce（处理频率控制）
     * 
     * @return true 应该运行
     * @return false 不应该运行（受频率限制）
     * 
     * 说明：在无点时间戳模式下，需要控制处理频率以避免过度消耗 CPU
     * 启动阶段使用较低频率，稳定后恢复正常频率
     */
    bool shouldRunProcessOnce() {
        // 非无点时间戳模式，正常处理
        if (!noPointTimeActive()) {
            return true;
        }

        // 根据启动状态选择处理频率
        const double rate = no_point_time_startup_done_ ? no_point_time_normal_process_rate_
                                                        : no_point_time_startup_process_rate_;
        // 如果频率设置无效，允许处理
        if (rate <= 0.0) {
            return true;
        }

        // 检查是否满足时间间隔要求
        const ros::WallTime now = ros::WallTime::now();
        if (!last_no_point_time_process_wall_.isZero() &&
            (now - last_no_point_time_process_wall_).toSec() < 1.0 / rate) {
            return false;  // 时间间隔不足，跳过本次处理
        }
        last_no_point_time_process_wall_ = now;
        return true;
    }

    /**
     * @brief 更新无点时间戳模式的启动状态
     * 
     * @param odom 当前里程计结果
     * 
     * 功能：统计有效里程计帧数，达到阈值后切换到正常处理频率
     */
    void updateNoPointTimeStartupState(const localization::OdomResult& odom) {
        // 仅在无点时间戳模式、启动未完成且里程计有效时执行
        if (!noPointTimeActive() || no_point_time_startup_done_ || !odom.valid) {
            return;
        }
        
        ++no_point_time_valid_odom_count_;
        // 检查是否达到启动完成阈值
        if (no_point_time_valid_odom_count_ >=
            static_cast<std::uint64_t>(std::max(1, no_point_time_startup_valid_frames_))) {
            no_point_time_startup_done_ = true;
            ROS_INFO("[slam_odometry] no-point-time startup done: valid_odom_frames=%lu, restore process rate to %.3fHz",
                     static_cast<unsigned long>(no_point_time_valid_odom_count_),
                     no_point_time_normal_process_rate_);
        }
    }

    /**
     * @brief IMU 数据回调函数
     * 
     * @param msg ROS IMU 消息指针
     * 
     * 处理流程：
     * 1. 将 ROS IMU 消息转换为内部格式
     * 2. 输入到 SLAM 算法
     * 3. 更新 IMU 预热状态
     */
    void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!slam_) {
            return;
        }
        if (slam_->preferRawFeed()) {
            // 原始消息直通：由 SLAM 实现自身的预处理配置处理（如 faster-lio ouster64.yaml）
            slam_->feedImuRaw(*msg);
        } else {
            slam_->feedImu(localization_ros::fromRos(*msg));
        }
        updateImuWarmup(msg->header.stamp.toSec());
    }

    /**
     * @brief 激光雷达点云回调函数
     * 
     * @param msg ROS PointCloud2 消息指针
     * 
     * 处理流程：
     * 1. 检查 IMU 预热状态（未完成则丢弃点云）
     * 2. 检测点云时间戳模式
     * 3. 将点云输入到 SLAM 算法
     */
    void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!slam_) {
            return;
        }
        
        // 检查 IMU 预热状态
        if (!canFeedLidar()) {
            ++dropped_lidar_before_warmup_;
            // 定期输出丢弃日志（首次和每 20 帧）
            if (dropped_lidar_before_warmup_ == 1 || dropped_lidar_before_warmup_ % 20 == 0) {
                const double duration = first_imu_stamp_ > 0.0 ? last_imu_stamp_ - first_imu_stamp_ : 0.0;
                ROS_INFO("[slam_odometry] drop lidar before IMU warmup: dropped=%lu imu_samples=%lu duration=%.3fs lidar_t=%.9f",
                         static_cast<unsigned long>(dropped_lidar_before_warmup_),
                         static_cast<unsigned long>(imu_warmup_samples_), duration,
                         msg->header.stamp.toSec());
            }
            return;
        }
        
        if (slam_->preferRawFeed()) {
            // 原始消息直通：与 faster_lio_ws 独立运行一致，
            // 由 ouster64.yaml 中的预处理配置处理，跳过节点侧二次预处理。
            slam_->feedLidarRaw(*msg);
            return;
        }

        // 检测点云时间戳模式
        noPointTimeMode(*msg);
        // 输入点云到 SLAM 算法
        slam_->feedLidar(localization_ros::fromRos(*msg, slam_config_));
    }

    /**
     * @brief CAN 轮速计数据回调函数
     * 
     * @param msg DeepLumin CAN 接收信息消息指针
     * 
     * 处理流程：
     * 1. 提取时间戳（如果消息时间戳无效则使用当前时间）
     * 2. 根据档位计算带符号速度（倒车为负）
     * 3. 输入到 SLAM 算法
     */
    void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
        if (!slam_ || !slam_config_.use_vel) {
            return;
        }
        
        localization::CanVelocityData can;
        // 提取时间戳，如果消息时间戳无效则使用当前 ROS 时间
        can.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                            ? msg->header.stamp.toSec()
                            : ros::Time::now().toSec();
        // 根据档位计算带符号速度：gear==2.0 表示倒车，速度取负
        can.speed_kmh = (msg->gear == 2.0) ? -msg->speed : msg->speed;
        can.valid = msg->valid;
        
        slam_->feedCanVelocity(can);
    }

    /**
     * @brief GNSS 姿态数据回调函数
     * 
     * @param msg DeepLumin GNSS CHC 消息指针
     * 
     * 处理流程：
     * 1. 提取时间戳（如果消息时间戳无效则使用当前时间）
     * 2. 提取航向角、俯仰角、横滚角
     * 3. 输入到 SLAM 算法用于姿态校正
     */
    void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
        if (!slam_ || !slam_config_.use_vel || !slam_config_.use_gnss_attitude) {
            return;
        }
        
        localization::GnssAttitudeData gnss;
        // 提取时间戳，如果消息时间戳无效则使用当前 ROS 时间
        gnss.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                             ? msg->header.stamp.toSec()
                             : ros::Time::now().toSec();
        // 提取姿态角（度）
        gnss.heading_deg = msg->heading;   // 航向角
        gnss.pitch_deg = msg->pitch;       // 俯仰角
        gnss.roll_deg = msg->roll;         // 横滚角
        gnss.valid = true;
        
        slam_->feedGnssAttitude(gnss);
    }

    // =========================================================================
    // 成员变量
    // =========================================================================
    
    // ROS 节点句柄
    ros::NodeHandle nh_;                    // 公有命名空间节点句柄
    ros::NodeHandle pnh_;                   // 私有命名空间节点句柄 (~)
    
    // 配置结构体
    localization::FrameConfig frames_;                      // 坐标系配置
    localization::ExtrinsicConfig extrinsics_;              // 传感器外参配置
    localization::SlamConfig slam_config_;                  // SLAM 算法配置
    
    // 运行时参数
    bool publish_tf_ = true;                                // 是否发布 TF 变换
    bool enable_imu_warmup_ = true;                         // 是否启用 IMU 预热
    double imu_warmup_duration_ = 1.0;                      // IMU 预热持续时间（秒）
    int min_imu_samples_ = 20;                              // 最小 IMU 样本数
    std::string point_time_mode_ = "auto";                  // 点云时间戳模式
    double no_point_time_startup_process_rate_ = 2.0;       // 无点时间戳启动处理频率（Hz）
    double no_point_time_normal_process_rate_ = 10.0;       // 无点时间戳正常处理频率（Hz）
    int no_point_time_startup_valid_frames_ = 20;           // 无点时间戳启动有效帧数阈值
    
    // IMU 预热状态变量
    double first_imu_stamp_ = 0.0;                          // 第一帧 IMU 时间戳
    double last_imu_stamp_ = 0.0;                           // 最后一帧 IMU 时间戳
    std::uint64_t imu_warmup_samples_ = 0;                  // IMU 样本计数
    std::uint64_t dropped_lidar_before_warmup_ = 0;         // 预热前丢弃的激光雷达帧数
    bool imu_warmup_ready_ = false;                         // IMU 预热完成标志
    
    // 点云时间戳模式状态变量
    bool point_time_known_ = false;                         // 是否已确定点时间戳模式
    bool point_time_available_ = false;                     // 点云是否包含时间戳
    bool no_point_time_startup_done_ = false;               // 无点时间戳启动是否完成
    ros::WallTime last_no_point_time_process_wall_;         // 上次处理时间（墙钟时间）
    std::uint64_t no_point_time_valid_odom_count_ = 0;      // 无点时间戳模式有效里程计数
    
    // 话题配置
    localization_ros::SensorTopicsConfig sensor_topics_;    // 传感器输入话题配置
    localization_ros::OutputTopicsConfig output_topics_;    // 输出话题配置
    
    // SLAM 实例
    std::shared_ptr<localization::ISlamOdometry> slam_;     // SLAM 算法接口指针
    
    // 可视化
    localization_ros::GlobalMapVizPublisher global_map_viz_; // 全局地图可视化发布器
    
    // ROS 订阅者和发布者
    ros::Subscriber imu_sub_;               // IMU 数据订阅者
    ros::Subscriber lidar_sub_;             // 激光雷达点云订阅者
    ros::Subscriber can_sub_;               // CAN 轮速计订阅者
    ros::Subscriber gnss_sub_;              // GNSS 姿态订阅者
    ros::Publisher odom_pub_;               // 里程计结果发布者
    ros::Publisher local_map_pub_;          // 局部地图发布者
    ros::Publisher registered_scan_pub_;    // 配准扫描点云发布者
    ros::Publisher effect_scan_pub_;        // 有效对应点云发布者
    tf::TransformBroadcaster tf_broadcaster_; // TF 变换广播器
};


// =============================================================================
// 主函数
// =============================================================================
/**
 * @brief 程序入口点
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码（0 表示正常退出）
 * 
 * 主循环逻辑：
 * 1. 初始化 ROS 节点
 * 2. 创建 SlamOdometryNode 实例
 * 3. 以 5000Hz 频率循环执行：spinOnce -> tick -> sleep
 * 
 * 注意：5000Hz 是主循环频率，实际 SLAM 处理频率由 shouldRunProcessOnce 控制
 */
int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "slam_odometry");
    ros::NodeHandle nh;

    // 注册 fastlio2（FAST-LIO2 / faster_lio_ws）工厂类型
    localization::registerFastLio2Plugin();

    // 创建 SLAM 里程计节点
    SlamOdometryNode node(nh);
    
    // 设置主循环频率（5000Hz，即每 0.2ms 一次）
    ros::Rate rate(5000);
    
    // 主循环
    while (ros::ok()) {
        ros::spinOnce();    // 处理回调队列中的消息
        node.tick();        // 执行 SLAM 处理
        rate.sleep();       // 休眠以维持频率
    }
    
    return 0;
}
