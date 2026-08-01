/**
 * @file pointcloud_preprocess.cpp
 * @brief 点云预处理模块实现（从 ysw_loc faster-lio pointcloud_preprocess.cc 移植）
 *
 * 该模块负责将不同厂商（Ouster、Velodyne、Robosense）的原始 ROS PointCloud2 消息
 * 转换为统一的 localization::PointCloud 格式，供 Faster-LIO 后续处理使用。
 *
 * 核心功能：
 * - 支持多种 LiDAR 型号：Ouster-64、Velodyne-32、Robosense-16、Livox-AVIA
 * - 盲区过滤：剔除距离传感器过近的点（如车体反射点）
 * - 点云降采样：按固定间隔取点，减少计算量
 * - 时间偏移计算：为每个点计算相对于扫描起始时刻的时间偏移（存储在 curvature 字段）
 * - 坐标系适配：根据实际安装方式调整坐标轴（如 Ouster 的 90° 旋转）
 *
 * 时间偏移计算原理：
 * - 对于带有 time/ring 字段的点云（如 Velodyne），可直接使用或基于 ring 计算
 * - 对于无时间字段的有组织点云（如 Robosense-16），基于扫描角速度估算
 * - 时间偏移单位：毫秒（ms），用于后续 IMU 运动补偿（去畸变）
 *
 * @see localization::ros::PointCloudPreprocess
 * @see localization::ImuProcess（使用 curvature 进行去畸变）
 */

// 模块头文件
#include <localization/ros/pointcloud_preprocess.hpp>

// ROS PCL 转换
#include <pcl_conversions/pcl_conversions.h>

// 标准库
#include <algorithm>   // 算法工具（std::min, std::max）
#include <cmath>       // 数学函数（std::atan2, std::isfinite）
#include <limits>      // 数值极限

namespace localization_ros {
namespace {

/**
 * @brief 将扫描频率转换为角速度（度/毫秒）
 *
 * @param scan_rate 扫描频率（Hz），即每秒扫描圈数
 * @return double 角速度（度/毫秒）
 *
 * 计算公式：omega = 360° / (1000ms / scan_rate) = 360° * scan_rate / 1000
 *
 * 示例：
 * - scan_rate = 10 Hz → omega = 3.6 deg/ms
 * - scan_rate = 20 Hz → omega = 7.2 deg/ms
 *
 * 该角速度用于估算无时间戳点云中每个点的时间偏移。
 * 若 scan_rate 无效（<= 0），返回默认值 3.61 deg/ms（对应约 10 Hz）。
 *
 * @note [NEW][parking_bag_adaptation] 新增函数，用于 Robosense-16 等无 time 字段的 LiDAR
 */
double scanRateToOmegaDegPerMs(const double scan_rate) {
    if (scan_rate <= 1e-6) {
        return 3.61;  // 默认值：约 10 Hz 的角速度
    }
    return 360.0 / (1000.0 / scan_rate);  // 360° / 每圈时间(ms)
}

}  // namespace

/**
 * @brief 设置点云预处理配置参数
 *
 * @param cfg SLAM 配置结构体，包含 LiDAR 型号、盲区距离、扫描线数等参数
 *
 * 配置内容包括：
 * - lidar_type: LiDAR 型号（1=AVIA, 2=VELO32, 3=OUST64, 4=ROBOSENSE16）
 * - blind: 盲区距离（米），距离小于此值的点将被过滤
 * - point_filter_num: 点云滤波间隔，每 N 个点取 1 个
 * - scan_line: 扫描线数（如 64、32、16）
 * - time_scale: 时间尺度因子（用于 Velodyne time 字段转换）
 * - scan_rate: 扫描频率（Hz），用于 Robosense-16 时间偏移估算
 *
 * @note 若 lidar_type 配置值无效，默认使用 OUST64
 */
void PointCloudPreprocess::set(const localization::SlamConfig& cfg) {
    // 根据配置值设置 LiDAR 类型枚举
    switch (cfg.lidar_type) {
        case 1:
            lidar_type_ = LidarType::AVIA;  // Livox AVIA
            break;
        case 2:
            lidar_type_ = LidarType::VELO32;  // Velodyne 32 线
            break;
        case 3:
            lidar_type_ = LidarType::OUST64;  // Ouster 64 线（默认）
            break;
        case 4:  // [NEW][parking_bag_adaptation] 新增 Robosense-16 支持
            lidar_type_ = LidarType::ROBOSENSE16;
            break;
        default:
            lidar_type_ = LidarType::OUST64;  // 默认回退到 Ouster-64
            break;
    }

    // 拷贝其他配置参数
    blind_ = cfg.blind;                          // 盲区距离（米）
    point_filter_num_ = cfg.point_filter_num;    // 点云滤波间隔
    num_scans_ = cfg.scan_line;                  // 扫描线数
    time_scale_ = static_cast<float>(cfg.time_scale);  // 时间尺度因子
    scan_rate_ = cfg.scan_rate;                  // [NEW][parking_bag_adaptation] 扫描频率
}

/**
 * @brief 处理 ROS PointCloud2 消息，输出统一格式的点云
 *
 * @param msg 输入的 ROS PointCloud2 消息（原始点云数据）
 * @param pcl_out 输出的统一格式点云指针（localization::PointCloud）
 *
 * 处理流程：
 * 1. 根据 lidar_type_ 选择对应的处理函数
 * 2. 调用具体型号的处理函数进行格式转换和时间偏移计算
 * 3. 将结果拷贝到输出指针
 *
 * 支持的 LiDAR 型号：
 * - OUST64: Ouster 64 线 LiDAR
 * - VELO32: Velodyne 32 线 LiDAR
 * - ROBOSENSE16: Robosense 16 线 LiDAR（新增）
 * - AVIA: Livox AVIA（当前不支持 PointCloud2 输入）
 */
void PointCloudPreprocess::process(const sensor_msgs::PointCloud2& msg, localization::PointCloudPtr& pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            oust64Handler(msg);  // Ouster-64 处理
            break;
        case LidarType::VELO32:
            velodyneHandler(msg);  // Velodyne-32 处理
            break;
        case LidarType::ROBOSENSE16:  // [NEW][parking_bag_adaptation]
            robosense16Handler(msg);  // Robosense-16 处理
            break;
        case LidarType::AVIA:
        default:
            // Livox AVIA 使用自定义消息格式，不支持标准 PointCloud2
            ROS_ERROR("[PointCloudPreprocess] unsupported lidar type for PointCloud2 input");
            cloud_out_.clear();
            break;
    }
    // 将处理后的点云拷贝到输出指针
    *pcl_out = cloud_out_;
}

/**
 * @brief Ouster-64 点云处理函数
 *
 * @param msg 输入的 ROS PointCloud2 消息
 *
 * 处理流程：
 * 1. 将 ROS 消息转换为 PCL 点云（ouster_ros::Point 格式）
 * 2. 按 point_filter_num_ 间隔降采样
 * 3. 盲区过滤：剔除距离小于 blind_ 的点
 * 4. 坐标系旋转：将 Ouster 坐标系旋转 90° 以适配 ysw 安装方式
 * 5. 时间偏移提取：将 Ouster 的 t 字段（单位 ns）转换为 ms
 * 6. 构建输出点云
 *
 * 坐标系旋转说明：
 * - 原始 Ouster 坐标系：X 朝前，Y 朝左，Z 朝上
 * - ysw 安装方式：LiDAR 绕 Z 轴旋转 +90°（即 X 朝左，Y 朝后）
 * - 转换公式：x' = y, y' = -x, z' = z
 * - 注意：若实际安装无旋转，应改回 x' = x, y' = y
 * - 同时 extrinsic_R（LiDAR→IMU 外参旋转）必须与此坐标系定义保持一致
 *
 * 时间偏移说明：
 * - Ouster 的 t 字段单位为纳秒（ns）
 * - 转换为毫秒：t_ms = t / 1e6
 * - 该值与 imu_processing 中的 curvature/1000 对应
 */
void PointCloudPreprocess::oust64Handler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();  // 清空输出点云

    // 将 ROS 消息转换为 Ouster 专用点云格式
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.size());
    cloud_out_.reserve(plsize);  // 预分配内存

    // 遍历所有点
    for (int i = 0; i < plsize; ++i) {
        // 降采样：只保留每 point_filter_num_ 个点中的第 1 个
        if (i % point_filter_num_ != 0) {
            continue;
        }

        const auto& src = pl_orig.points[i];

        // 盲区过滤：计算点距传感器距离的平方，与盲区距离平方比较
        const double range = src.x * src.x + src.y * src.y + src.z * src.z;
        if (range < blind_ * blind_) {
            continue;  // 距离过近，跳过（可能是车体反射点）
        }

        // 构建输出点
        localization::PointType added_pt;

        // 坐标系旋转（适配 ysw 安装方式，绕 Z 轴 +90°）
        // 注意：若实际传感器安装无旋转，应改回 added_pt.x = src.x; added_pt.y = src.y;
        added_pt.x = src.y;
        added_pt.y = -src.x;
        added_pt.z = src.z;

        // 拷贝强度值
        added_pt.intensity = src.intensity;

        // 法向量字段（未使用，置零）
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;

        // 时间偏移：Ouster 的 t 字段为纳秒，转换为毫秒
        // 与 imu_processing 中的 curvature/1000 对应
        added_pt.curvature = src.t / 1e6f;

        cloud_out_.points.push_back(added_pt);
    }

    // 设置输出点云元数据
    cloud_out_.width = cloud_out_.points.size();  // 点云宽度 = 总点数
    cloud_out_.height = 1;                        // 高度 = 1（无序点云）
    cloud_out_.is_dense = false;                  // 可能包含无效点（NaN/Inf）
}

/**
 * @brief Robosense-16 点云处理函数（新增）
 *
 * @param msg 输入的 ROS PointCloud2 消息
 *
 * [NEW][parking_bag_adaptation]
 * 针对 parking bag 数据适配的 Robosense-16 有组织点云处理。
 * Robosense-16 点云特点：
 * - 有组织点云（organized cloud）：height > 1, width > 1
 * - 无 ring 字段：需要通过索引计算所在线数
 * - 无 time 字段：需要基于扫描角速度估算时间偏移
 *
 * 处理流程：
 * 1. 将 ROS 消息转换为标准 XYZI 点云
 * 2. 验证点云是否为有组织格式
 * 3. 计算扫描角速度（基于 scan_rate_）
 * 4. 逐点处理：
 *    - 有效性检查（排除 NaN/Inf）
 *    - 盲区过滤
 *    - 线数计算：ring = idx / cols
 *    - 时间偏移估算：基于 yaw 角变化和角速度
 *    - 降采样
 * 5. 构建输出点云
 *
 * 时间偏移估算原理：
 * - 假设 LiDAR 以恒定角速度 omega_l 旋转
 * - 计算当前点与首点的 yaw 角差：delta_yaw = yaw_first - yaw_current
 * - 时间偏移：offset_ms = delta_yaw / omega_l
 * - 处理跨圈情况（yaw 角从 360° 跳变到 0°）
 * - 处理时间累积（多圈扫描时累加周期时间）
 */
void PointCloudPreprocess::robosense16Handler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();  // 清空输出点云

    // 将 ROS 消息转换为标准 XYZI 点云格式
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);

    // 获取点云尺寸
    const int height = static_cast<int>(pl_orig.height);  // 扫描线数（应为 16）
    const int width = static_cast<int>(pl_orig.width);    // 每线点数
    const int plsize = static_cast<int>(pl_orig.points.size());
    cloud_out_.reserve(plsize);  // 预分配内存

    // 检查是否为有组织点云
    if (height <= 1 || width <= 0) {
        ROS_WARN_THROTTLE(1.0,
            "[PointCloudPreprocess][parking_bag_adaptation] /rslidar_points is not organized, "
            "fallback to XYZI-only ordering");
    }

    // 计算扫描参数
    const int rings = std::max(1, height);           // 扫描线数（至少 1）
    const int cols = std::max(1, width);             // 每线点数（至少 1）
    const double omega_l = scanRateToOmegaDegPerMs(scan_rate_);  // 角速度（度/ms）

    // 每线首点标志：用于标记每线的第一个点
    std::vector<bool> is_first(rings, true);
    // 每线首点的 yaw 角（度）
    std::vector<double> yaw_fp(rings, 0.0);
    // 每线最后一个点的时间偏移（ms）
    std::vector<float> time_last(rings, 0.0f);

    // 逐点处理
    for (int idx = 0; idx < plsize; ++idx) {
        const auto& src = pl_orig.points[idx];

        // 有效性检查：排除 NaN 或 Inf 坐标
        if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z)) {
            continue;
        }

        // 构建输出点
        localization::PointType added_pt;
        added_pt.normal_x = 0.0f;
        added_pt.normal_y = 0.0f;
        added_pt.normal_z = 0.0f;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;

        // 盲区过滤
        const double range = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
        if (range <= blind_ * blind_) {
            continue;
        }

        // 计算点所在线数（ring）
        // 有组织点云中，索引 idx = ring * cols + col
        // 因此 ring = idx / cols
        const int ring = std::min(rings - 1, idx / cols);

        // 计算当前点的 yaw 角（度）
        // atan2(y, x) 返回弧度，乘以 57.29577951308232（即 180/π）转换为度
        const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.29577951308232;

        // 时间偏移估算
        float offset_ms = 0.0f;
        if (is_first[ring]) {
            // 每线的第一个点：记录首点 yaw 角，时间偏移为 0
            yaw_fp[ring] = yaw_angle;
            is_first[ring] = false;
        } else {
            // 非首点：基于与首点的 yaw 角差估算时间偏移
            if (yaw_angle <= yaw_fp[ring]) {
                // 正常情况：当前 yaw <= 首点 yaw
                offset_ms = static_cast<float>((yaw_fp[ring] - yaw_angle) / omega_l);
            } else {
                // 跨圈情况：当前 yaw > 首点 yaw（从 360° 跳变到 0°）
                offset_ms = static_cast<float>((yaw_fp[ring] - yaw_angle + 360.0) / omega_l);
            }

            // 处理多圈扫描的时间累积
            // 若当前时间偏移小于上一时刻，说明已多扫描了一圈，需加上一个周期时间
            if (offset_ms < time_last[ring]) {
                offset_ms += static_cast<float>(360.0 / omega_l);  // 加上一圈的时间
            }
        }
        // 更新该线最后一个点的时间偏移
        time_last[ring] = offset_ms;
        added_pt.curvature = offset_ms;  // 存储时间偏移到 curvature 字段

        // 降采样：只保留每 point_filter_num_ 个点中的第 1 个
        const int col = idx % cols;
        const int linear_index = ring * cols + col;
        if (linear_index % point_filter_num_ == 0) {
            cloud_out_.points.push_back(added_pt);
        }
    }

    // 设置输出点云元数据
    cloud_out_.width = cloud_out_.points.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

/**
 * @brief Velodyne 点云处理函数
 *
 * @param msg 输入的 ROS PointCloud2 消息
 *
 * Velodyne 点云特点：
 * - 包含 ring 字段：标识点所在的扫描线
 * - 可能包含 time 字段：标识点相对于扫描起始时刻的时间偏移
 * - 若 time 字段存在，可直接使用；否则基于 ring 和 yaw 角计算
 *
 * 处理流程：
 * 1. 将 ROS 消息转换为 Velodyne 专用点云格式
 * 2. 检查是否存在 time 字段（given_offset_time_）
 * 3. 逐点处理：
 *    - 提取坐标和强度
 *    - 若存在 time 字段，直接使用并应用 time_scale_ 转换
 *    - 若无 time 字段，基于 ring 和 yaw 角估算时间偏移
 * 4. 降采样和盲区过滤
 * 5. 构建输出点云
 *
 * 时间偏移计算（无 time 字段时）：
 * - 默认角速度：omega_l = 3.61 deg/ms（对应 10 Hz）
 * - 每线首点：time = 0，记录首点 yaw 角
 * - 非首点：基于 yaw 角差计算时间偏移
 * - 处理跨圈和多圈情况（与 Robosense-16 类似）
 */
void PointCloudPreprocess::velodyneHandler(const sensor_msgs::PointCloud2& msg) {
    cloud_out_.clear();  // 清空输出点云

    // 将 ROS 消息转换为 Velodyne 专用点云格式
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.points.size());
    cloud_out_.reserve(plsize);  // 预分配内存

    // 默认角速度（度/ms），对应 Velodyne 典型扫描频率（约 10 Hz）
    const double omega_l = 3.61;

    // 每线首点标志
    std::vector<bool> is_first(num_scans_, true);
    // 每线首点 yaw 角
    std::vector<double> yaw_fp(num_scans_, 0.0);
    // 每线上一个点的 yaw 角
    std::vector<float> yaw_last(num_scans_, 0.0);
    // 每线上一个点的时间偏移
    std::vector<float> time_last(num_scans_, 0.0);

    // 检查点云是否包含 time 字段
    // 通过检查最后一个点的 time 值是否大于 0 来判断
    if (plsize > 0 && pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;   // 点云包含 time 字段
    } else {
        given_offset_time_ = false;  // 点云不包含 time 字段，需要计算
    }

    // 逐点处理
    for (int i = 0; i < plsize; ++i) {
        // 构建输出点
        localization::PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        // 若点云包含 time 字段，直接使用并应用时间尺度因子
        added_pt.curvature = pl_orig.points[i].time * time_scale_;

        // 若点云不包含 time 字段，基于 ring 和 yaw 角计算时间偏移
        if (!given_offset_time_) {
            // 获取点所在的扫描线（ring）
            const int layer = pl_orig.points[i].ring;

            // 计算当前点的 yaw 角（度）
            const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                // 该线的第一个点：记录首点 yaw 角，时间偏移为 0
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0f;
                yaw_last[layer] = static_cast<float>(yaw_angle);
                time_last[layer] = added_pt.curvature;
                continue;  // 跳过降采样和盲区检查（首点通常用于时间基准）
            }

            // 非首点：基于与首点的 yaw 角差计算时间偏移
            if (yaw_angle <= yaw_fp[layer]) {
                // 正常情况
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle) / omega_l);
            } else {
                // 跨圈情况
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle + 360.0) / omega_l);
            }

            // 处理多圈扫描的时间累积
            if (added_pt.curvature < time_last[layer]) {
                added_pt.curvature += static_cast<float>(360.0 / omega_l);
            }

            // 更新该线状态
            yaw_last[layer] = static_cast<float>(yaw_angle);
            time_last[layer] = added_pt.curvature;
        }

        // 降采样：只保留每 point_filter_num_ 个点中的第 1 个
        if (i % point_filter_num_ == 0) {
            // 盲区过滤
            const double range = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range > blind_ * blind_) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    // 设置输出点云元数据
    cloud_out_.width = cloud_out_.points.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace localization_ros