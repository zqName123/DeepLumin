/**
 * @file laser_mapping_core.cpp
 * @brief Faster-LIO 激光雷达建图核心实现
 *
 * 该文件实现了基于 Faster-LIO 的激光雷达惯性里程计核心算法，主要功能包括：
 * - IMU 数据预处理与状态预测
 * - 激光雷达点云数据同步与去畸变
 * - 迭代扩展卡尔曼滤波 (IEKF) 状态估计
 * - iVox 增量式体素地图构建与更新
 * - 轮速计和 GNSS 姿态辅助校正
 * - 退化场景检测与处理
 *
 * 算法流程：
 * 1. 数据同步 (syncPackages): 对齐 IMU 和 LiDAR 数据时间戳
 * 2. IMU 处理 (ImuProcess): 利用 IMU 数据进行状态预测和点云去畸变
 * 3. 速度辅助 (applyYswVelocityAid): 轮速计和 GNSS 姿态校正
 * 4. 首帧初始化: 构建初始地图
 * 5. 点云降采样: 减少计算量
 * 6. IEKF 更新: 迭代卡尔曼滤波优化位姿
 * 7. 地图增量更新: 将新点云加入 iVox 地图
 *
 * @author DeepLumin Team
 * @date 2024
 */

// =============================================================================
// 头文件包含
// =============================================================================

// 模块头文件
#include "localization/slam/laser_mapping_core.hpp"

// 第三方库
#include <glog/logging.h>            // Google 日志库
#include <pcl/filters/voxel_grid.h>  // PCL 体素滤波

// 标准库
#include <algorithm>     // 算法工具
#include <cstdint>       // 整数类型
#include <cmath>         // 数学函数
#include <execution>     // 并行执行策略
#include <limits>        // 数值极限
#include <iomanip>       // 格式化输出
#include <memory>        // 智能指针
#include <mutex>         // 互斥锁

// Faster-LIO 第三方库头文件
#include "common_lib.h"      // Faster-LIO 通用工具
#include "imu_processing.hpp" // IMU 处理
#include "ivox3d/ivox3d.h"   // 增量体素地图
#include "options.h"         // 算法选项
#include "use-ikfom.hpp"     // IKFoM 滤波器
#include "utils.h"           // 工具函数

namespace localization {

// =============================================================================
// 匿名命名空间：内部辅助函数和类型
// =============================================================================
namespace {

using namespace faster_lio;  // 使用 Faster-LIO 命名空间

// 根据编译选项选择 iVox 节点类型
#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

/**
 * @brief 日志频率限制器
 *
 * 用于控制日志输出频率，避免日志过多影响性能
 * 例如 LogEveryN(1000) 表示每 1000 次调用才输出一次日志
 */
struct LogEveryN {
    explicit LogEveryN(int n) : interval(n) {}
    bool hit() { return interval > 0 && (++count % interval) == 0; }

    int interval;  // 间隔次数
    int count = 0; // 计数器
};

/**
 * @brief 将内部 IMU 数据转换为 Faster-LIO 的 ImuSample 格式
 *
 * @param imu 内部 ImuData 结构
 * @return ImuSamplePtr Faster-LIO 的 IMU 样本指针
 *
 * 转换内容包括：
 * - 时间戳
 * - 角速度 (gyro)
 * - 线加速度 (accel)
 */
ImuSamplePtr toImuSample(const ImuData& imu) {
    auto sample = std::make_shared<ImuSample>();
    sample->timestamp = imu.timestamp;
    sample->angular_velocity.x = imu.gyro.x();
    sample->angular_velocity.y = imu.gyro.y();
    sample->angular_velocity.z = imu.gyro.z();
    sample->linear_acceleration.x = imu.accel.x();
    sample->linear_acceleration.y = imu.accel.y();
    sample->linear_acceleration.z = imu.accel.z();
    return sample;
}

/**
 * @brief 记录 IKFoM 滤波器状态日志
 *
 * @param stage 当前处理阶段名称
 * @param sync_id 同步 ID
 * @param scan_cb_id 扫描回调 ID
 * @param s 当前状态
 *
 * 输出内容包括：位置、速度、重力向量、陀螺仪零偏
 */
void logIkfState(const char* stage, std::uint64_t sync_id, std::uint64_t scan_cb_id,
                   const state_ikfom& s) {
    LOG(INFO) << "[SLAM][Align][State][" << stage << "] sync=#" << sync_id << " feed_cb=#" << scan_cb_id
              << " pos=(" << s.pos.transpose() << ")"
              << " vel=(" << s.vel.transpose() << ")"
              << " grav=(" << s.grav[0] << " " << s.grav[1] << " " << s.grav[2] << ")"
              << " bg=(" << s.bg.transpose() << ")";
}

/**
 * @brief 记录地图初始化前几个点的日志
 *
 * @param tag 日志标签
 * @param undistort 去畸变后的点云（LiDAR 坐标系）
 * @param world 世界坐标系下的点云
 *
 * 用于调试首帧地图初始化，输出前 5 个点的坐标对比
 */
void logMapInitFirstPoints(const char* tag, const PointCloudType& undistort, const PointCloudType& world) {
    const int n = std::min(5, static_cast<int>(undistort.size()));
    LOG(INFO) << "[SLAM][" << tag << "][MapInitPts] count=" << n << " total=" << undistort.size();
    for (int i = 0; i < n; ++i) {
        const auto& u = undistort.points[i];
        const auto& w = world.points[i];
        LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][MapInitPts] i=" << i
                  << " undistort=(" << u.x << " " << u.y << " " << u.z << ") curv_ms=" << u.curvature
                  << " world=(" << w.x << " " << w.y << " " << w.z << ")";
    }
}

/**
 * @brief 记录观测模型诊断日志
 *
 * @param tag 日志标签
 * @param frame_num 当前帧号
 * @param sync_id 同步 ID
 * @param max_frames 最大记录帧数
 * @param cnt_pts 点云总数
 * @param effect_feat_num 有效特征点数
 * @param scan_down_body 降采样后的本体坐标系点云
 * @param scan_down_world 降采样后的世界坐标系点云
 * @param nearest_points 每个点的近邻点
 * @param residuals 残差
 * @param point_selected_surf 点选择标志
 * @param plane_coef 平面系数
 * @param ekfom_data EKFoM 数据结构
 * @param logged_this_iekf 是否已记录标志（引用）
 *
 * 功能：
 * - 计算残差统计信息（均值、RMS、最大值）
 * - 记录前 5 个有效点的详细信息
 * - 记录近邻点和平面系数
 * - 记录观测雅可比矩阵
 */
void logObsModelDiag(const char* tag, int frame_num, std::uint64_t sync_id, int max_frames, int cnt_pts,
                     int effect_feat_num, const CloudPtr& scan_down_body, const CloudPtr& scan_down_world,
                     const std::vector<PointVector>& nearest_points, const std::vector<float>& residuals,
                     const std::vector<char>& point_selected_surf, const common::VV4F& plane_coef,
                     const esekfom::dyn_share_datastruct<double>& ekfom_data, bool& logged_this_iekf) {
    // 超过最大记录帧数、已记录或无效特征数时跳过
    if (frame_num >= max_frames || logged_this_iekf || effect_feat_num < 1) {
        return;
    }
    logged_this_iekf = true;

    // 计算残差统计
    double abs_sum = 0.0;
    double sq_sum = 0.0;
    double max_abs = 0.0;
    int valid_cnt = 0;
    for (int i = 0; i < cnt_pts; ++i) {
        if (!point_selected_surf[i]) {
            continue;
        }
        const double r = residuals[i];
        abs_sum += std::abs(r);
        sq_sum += r * r;
        max_abs = std::max(max_abs, static_cast<double>(std::abs(r)));
        ++valid_cnt;
    }
    const double abs_mean = valid_cnt > 0 ? abs_sum / valid_cnt : 0.0;
    const double rms = valid_cnt > 0 ? std::sqrt(sq_sum / valid_cnt) : 0.0;

    LOG(INFO) << "[SLAM][" << tag << "][ObsDiag] sync=#" << sync_id << " iekf_frame=" << frame_num
              << " effect_feat_num=" << effect_feat_num << " valid_cnt=" << valid_cnt
              << " residual abs_mean=" << abs_mean << " rms=" << rms << " max=" << max_abs;

    // 记录前 5 个有效点的详细信息
    int logged_pts = 0;
    int effect_idx = 0;
    for (int i = 0; i < cnt_pts && logged_pts < 5; ++i) {
        if (!point_selected_surf[i]) {
            continue;
        }
        const auto& pb = scan_down_body->points[i];
        const auto& pw = scan_down_world->points[i];
        const auto& pc = plane_coef[i];
        LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                  << " scan_idx=" << i << " effect_idx=" << effect_idx << " body=(" << pb.x << " " << pb.y << " "
                  << pb.z << ") world=(" << pw.x << " " << pw.y << " " << pw.z << ") residual=" << residuals[i]
                  << " plane_coef=(" << pc[0] << " " << pc[1] << " " << pc[2] << " " << pc[3] << ")";
        // 记录近邻点
        const auto& near = nearest_points[i];
        const int near_n = std::min(5, static_cast<int>(near.size()));
        for (int k = 0; k < near_n; ++k) {
            LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                      << " near[" << k << "]=(" << near[k].x << " " << near[k].y << " " << near[k].z << ")";
        }
        // 记录观测雅可比矩阵
        if (effect_idx < ekfom_data.h_x.rows()) {
            const auto hx = ekfom_data.h_x.row(effect_idx);
            LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                      << " h_x[0:5]=(" << hx(0) << " " << hx(1) << " " << hx(2) << " " << hx(3) << " " << hx(4)
                      << " " << hx(5) << ") h=" << ekfom_data.h(effect_idx);
        }
        ++logged_pts;
        ++effect_idx;
    }
}

/**
 * @brief 记录协方差矩阵对角线元素日志
 *
 * @param tag 日志标签
 * @param stage 处理阶段
 * @param sync_id 同步 ID
 * @param iekf_frame IEKF 帧号
 * @param kf EKFoM 滤波器
 *
 * 输出 23 维状态向量的协方差对角线元素：
 * - pos[0:3]: 位置
 * - rot[3:6]: 旋转
 * - off_R[6:9]: LiDAR-IMU 外参旋转
 * - off_T[9:12]: LiDAR-IMU 外参平移
 * - vel[12:15]: 速度
 * - bg[15:18]: 陀螺仪零偏
 * - ba[18:21]: 加速度计零偏
 * - grav[21:23]: 重力向量
 */
void logPDiagonal(const char* tag, const char* stage, std::uint64_t sync_id, int iekf_frame,
                const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf) {
    const auto P = kf.get_P();
    const Eigen::Matrix<double, 23, 1> d = P.diagonal();
    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][" << tag << "][P][" << stage << "] sync=#" << sync_id
              << " iekf_frame=" << iekf_frame << " pos=(" << d[0] << " " << d[1] << " " << d[2] << ") rot=(" << d[3]
              << " " << d[4] << " " << d[5] << ") off_R=(" << d[6] << " " << d[7] << " " << d[8] << ") off_T=("
              << d[9] << " " << d[10] << " " << d[11] << ") vel=(" << d[12] << " " << d[13] << " " << d[14] << ") bg=("
              << d[15] << " " << d[16] << " " << d[17] << ") ba=(" << d[18] << " " << d[19] << " " << d[20] << ") grav=("
              << d[21] << " " << d[22] << ")";
    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][" << tag << "][P][" << stage << "] P_diag_all=("
              << d.transpose() << ")";
}

}  // namespace

// =============================================================================
// 全局辅助函数
// =============================================================================

/**
 * @brief 将角度归一化到 [-180, 180] 度范围
 *
 * @param angle_deg 输入角度（度）
 * @return float 归一化后的角度
 */
float normalizePitchRoll(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/**
 * @brief 将欧拉角（航向-俯仰-横滚）转换为四元数
 *
 * @param heading_deg 航向角（度，绕 Z 轴）
 * @param pitch_deg 俯仰角（度，绕 Y 轴）
 * @param roll_deg 横滚角（度，绕 X 轴）
 * @return Eigen::Quaterniond 归一化四元数
 *
 * 旋转顺序：Z-Y-X（航向-俯仰-横滚）
 */
Eigen::Quaterniond anglesToQuaternion(double heading_deg, double pitch_deg, double roll_deg) {
    const double heading_rad = heading_deg * M_PI / 180.0;
    const double pitch_rad = pitch_deg * M_PI / 180.0;
    const double roll_rad = roll_deg * M_PI / 180.0;
    const Eigen::Quaterniond q_z(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_y(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond q_x(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
    return (q_z * q_y * q_x).normalized();
}

// =============================================================================
// LaserMappingCore::Impl 结构体定义
// =============================================================================

/**
 * @struct LaserMappingCore::Impl
 * @brief LaserMappingCore 的 PIMPL 实现结构体
 *
 * 包含所有内部状态和数据，避免暴露实现细节
 */
struct LaserMappingCore::Impl {
    // -------------------------------------------------------------------------
    // iVox 地图相关
    // -------------------------------------------------------------------------
    IVoxType::Options ivox_options;              // iVox 配置选项
    std::shared_ptr<IVoxType> ivox;              // iVox 增量体素地图

    // -------------------------------------------------------------------------
    // IMU 处理
    // -------------------------------------------------------------------------
    std::shared_ptr<ImuProcess> imu_process;     // IMU 处理器

    // -------------------------------------------------------------------------
    // 状态估计
    // -------------------------------------------------------------------------
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  // IKFoM 滤波器
    state_ikfom state_point;                          // 当前状态点

    // -------------------------------------------------------------------------
    // 地图参数
    // -------------------------------------------------------------------------
    float det_range = 300.0f;                    // 检测范围（米）
    double cube_len = 2000.0;                    // 地图立方体边长
    double filter_size_map_min = 0.2;            // 地图最小滤波尺寸
    bool localmap_initialized = false;           // 局部地图是否已初始化
    bool extrinsic_est_en = false;               // 是否在线估计外参

    // -------------------------------------------------------------------------
    // 辅助传感器
    // -------------------------------------------------------------------------
    bool use_vel = false;                        // 是否使用轮速计
    bool use_gnss_attitude = true;               // 是否使用 GNSS 姿态
    double can_vel_diff_thresh_kmh = 2.0;        // 轮速差异阈值（km/h）
    double can_timeout_sec = 0.2;                // 轮速超时时间（秒）
    double gnss_attitude_timeout_sec = 1.0;      // GNSS 姿态超时时间（秒）
    double attitude_diff_thresh_deg = 2.0;       // 姿态差异阈值（度）
    double attitude_correction_gain = 0.7;       // 姿态校正增益
    CanVelocityData last_can;                    // 最新轮速数据
    GnssAttitudeData last_gnss_attitude;         // 最新 GNSS 姿态数据
    bool has_can = false;                        // 是否有轮速数据
    bool has_gnss_attitude = false;              // 是否有 GNSS 姿态数据
    std::uint64_t vel_reset_count = 0;           // 速度重置计数
    std::uint64_t attitude_reset_count = 0;      // 姿态重置计数

    // -------------------------------------------------------------------------
    // LiDAR-IMU 外参
    // -------------------------------------------------------------------------
    Vec3d extrinT = Vec3d::Zero();               // 外参平移
    Mat3d extrinR = Mat3d::Identity();           // 外参旋转

    // -------------------------------------------------------------------------
    // 点云数据
    // -------------------------------------------------------------------------
    CloudPtr scan_undistort{new PointCloudType()};   // 去畸变后的点云
    CloudPtr scan_down_body{new PointCloudType()};   // 降采样后的本体坐标系点云
    CloudPtr scan_down_world{new PointCloudType()};  // 降采样后的世界坐标系点云

    // -------------------------------------------------------------------------
    // 观测模型相关
    // -------------------------------------------------------------------------
    std::vector<PointVector> nearest_points;     // 每个点的近邻点
    common::VV4F corr_pts;                       // 对应点
    common::VV4F corr_norm;                      // 对应法向量
    pcl::VoxelGrid<PointType> voxel_scan;        // 扫描降采样滤波器
    std::vector<float> residuals;                // 残差
    // 注意：必须使用 char 而非 bool，因为 par_unseq 并行执行时
    // std::vector<bool> 的位压缩存储会导致同字节并发写入的数据竞争
    std::vector<char> point_selected_surf;       // 点选择标志
    common::VV4F plane_coef;                     // 平面系数

    // -------------------------------------------------------------------------
    // 数据缓冲区
    // -------------------------------------------------------------------------
    std::mutex buffer_mutex;                     // 缓冲区互斥锁
    std::deque<double> time_buffer;              // 时间戳缓冲区
    std::deque<PointCloudType::Ptr> lidar_buffer; // 激光雷达数据缓冲区
    std::deque<ImuSamplePtr> imu_buffer;         // IMU 数据缓冲区

    common::MeasureGroup measures;               // 同步后的测量数据组

    // -------------------------------------------------------------------------
    // 时间同步相关
    // -------------------------------------------------------------------------
    double last_timestamp_lidar = 0;             // 上一帧激光雷达时间戳
    double lidar_end_time = 0;                   // 当前激光雷达扫描结束时间
    double last_timestamp_imu = -1.0;            // 上一帧 IMU 时间戳
    double first_lidar_time = 0.0;               // 第一帧激光雷达时间
    bool lidar_pushed = false;                   // 激光雷达是否已推入

    // -------------------------------------------------------------------------
    // 状态标志
    // -------------------------------------------------------------------------
    int scan_count = 0;                          // 扫描计数
    bool flg_first_scan = true;                  // 是否为首帧
    bool flg_ekf_inited = false;                 // EKF 是否已初始化
    double lidar_mean_scantime = 0.0;            // 激光雷达平均扫描时间
    int scan_num = 0;                            // 扫描次数
    int effect_feat_num = 0;                     // 有效特征点数
    int frame_num = 0;                           // 帧号

    // -------------------------------------------------------------------------
    // 退化检测（当前禁用）
    // -------------------------------------------------------------------------
    // 验证阶段关闭：obs_hessian 并行累加存在 data race，且非 ysw 原版逻辑
    static constexpr bool kEnableDegeneracyDiag = false;
    static constexpr int kObsDiagMaxIekfFrames = 3;  // 最大观测诊断帧数
    bool obs_diag_logged_this_iekf = false;          // 当前 IEKF 是否已记录诊断

    // -------------------------------------------------------------------------
    // 缓冲区大小限制
    // -------------------------------------------------------------------------
    // 对齐 ysw StandardPCLCallBack：当 lidar_buffer_.size() <= 2 时入队（最多缓存 3 帧）
    static constexpr std::size_t kMaxLidarBufferSize = 3;

    // -------------------------------------------------------------------------
    // 统计计数
    // -------------------------------------------------------------------------
    std::uint64_t imu_feed_count = 0;            // IMU 输入计数
    std::uint64_t lidar_feed_count = 0;          // 激光雷达输入计数
    std::uint64_t lidar_drop_count = 0;          // 激光雷达丢弃计数
    std::uint64_t process_tick_count = 0;        // 处理 tick 计数
    std::uint64_t sync_ok_count = 0;             // 同步成功计数

    // -------------------------------------------------------------------------
    // 当前状态输出
    // -------------------------------------------------------------------------
    common::V3D euler_cur = common::V3D::Zero(); // 当前欧拉角
    common::V3D pos_lidar = common::V3D::Zero(); // LiDAR 位置

    // -------------------------------------------------------------------------
    // 退化检测相关
    // -------------------------------------------------------------------------
    Eigen::Matrix<double, 6, 6> obs_hessian = Eigen::Matrix<double, 6, 6>::Zero();  // 观测海森矩阵
    double degeneracy_threshold = 100.0;         // 退化阈值

    /**
     * @brief 同步 IMU 和激光雷达数据包
     *
     * @return true 同步成功
     * @return false 同步失败（数据不足）
     *
     * 同步逻辑：
     * 1. 检查缓冲区是否有数据
     * 2. 推入激光雷达数据，计算扫描结束时间
     * 3. 等待 IMU 数据覆盖激光雷达扫描时间段
     * 4. 提取同步时间窗口内的 IMU 数据
     * 5. 清空已使用的缓冲区数据
     */
    bool syncPackages() {
        // 检查缓冲区是否有数据
        if (lidar_buffer.empty() || imu_buffer.empty()) {
            static LogEveryN throttle(3000);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] wait buffer: lidar_buf=" << lidar_buffer.size()
                          << " imu_buf=" << imu_buffer.size()
                          << " last_imu_ts=" << last_timestamp_imu;
            }
            return false;
        }

        // 推入激光雷达数据（如果尚未推入）
        if (!lidar_pushed) {
            measures.lidar_ = lidar_buffer.front();
            measures.lidar_bag_time_ = time_buffer.front();

            // 计算扫描结束时间
            // 使用点云最后一个点的时间戳（curvature 字段存储毫秒时间）
            const double back_curv_s = measures.lidar_->points.empty()
                                           ? 0.0
                                           : measures.lidar_->points.back().curvature / double(1000);
            const char* sync_branch = "unknown";

            if (measures.lidar_->points.size() <= 1) {
                // 点太少，使用平均扫描时间
                LOG(WARNING) << "[SLAM][Core][Sync] too few input points: "
                             << measures.lidar_->points.size();
                lidar_end_time = measures.lidar_bag_time_ + lidar_mean_scantime;
                sync_branch = "few_pts_use_mean";
            } else if (back_curv_s < 0.5 * lidar_mean_scantime) {
                // 最后一个点时间戳异常，使用平均扫描时间
                lidar_end_time = measures.lidar_bag_time_ + lidar_mean_scantime;
                sync_branch = "use_mean_scantime";
            } else {
                // 使用最后一个点的时间戳计算扫描结束时间
                scan_num++;
                lidar_end_time = measures.lidar_bag_time_ + back_curv_s;
                // 在线更新平均扫描时间
                lidar_mean_scantime += (back_curv_s - lidar_mean_scantime) / scan_num;
                sync_branch = "use_back_curvature";
            }

            measures.lidar_end_time_ = lidar_end_time;
            lidar_pushed = true;

            // 定期输出同步日志
            static LogEveryN throttle(50);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] branch=" << sync_branch << " mean_scantime=" << lidar_mean_scantime
                          << " back_curv_ms=" << (back_curv_s * 1000.0)
                          << " scan_dt=" << (lidar_end_time - measures.lidar_bag_time_);
            }
        }

        // 检查 IMU 数据是否覆盖到激光雷达扫描结束时间
        if (last_timestamp_imu < lidar_end_time) {
            static LogEveryN throttle(3000);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] wait IMU: last_imu=" << last_timestamp_imu
                          << " lidar_end=" << lidar_end_time
                          << " gap=" << (lidar_end_time - last_timestamp_imu) << "s";
            }
            return false;
        }

        // 提取同步时间窗口内的 IMU 数据
        double imu_time = imu_buffer.front()->timestamp;
        measures.imu_.clear();
        while (!imu_buffer.empty() && imu_time < lidar_end_time) {
            imu_time = imu_buffer.front()->timestamp;
            if (imu_time > lidar_end_time) {
                break;
            }
            measures.imu_.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        // 清空已使用的激光雷达数据
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;

        ++sync_ok_count;
        // 输出同步成功日志
        const double imu_first_ts =
            measures.imu_.empty() ? -1.0 : imuTimestamp(measures.imu_.front());
        const double imu_last_ts = measures.imu_.empty() ? -1.0 : imuTimestamp(measures.imu_.back());
        LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][Align][Sync] OK #" << sync_ok_count
                  << " feed_cb=#" << scan_count << " lidar_t=" << measures.lidar_bag_time_
                  << " lidar_end=" << lidar_end_time << " scan_dt=" << (lidar_end_time - measures.lidar_bag_time_)
                  << " mean_scantime=" << lidar_mean_scantime << " lidar_pts=" << measures.lidar_->points.size()
                  << " imu_used=" << measures.imu_.size() << " imu_first_ts=" << imu_first_ts
                  << " imu_last_ts=" << imu_last_ts << " imu_buf_left=" << imu_buffer.size()
                  << " lidar_buf_left=" << lidar_buffer.size();
        return true;
    }

    /**
     * @brief 将点从 LiDAR 本体坐标系转换到世界坐标系
     *
     * @param pi 输入点（本体坐标系）
     * @param po 输出点（世界坐标系）
     *
     * 变换链：
     * p_global = R_world_imu * (R_imu_lidar * p_body + t_imu_lidar) + t_world_imu
     *          = state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I)
     *            + state_point.pos
     */
    void pointBodyToWorld(const PointType* pi, PointType* po) const {
        common::V3D p_body(pi->x, pi->y, pi->z);
        common::V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                             state_point.pos);

        po->x = static_cast<float>(p_global(0));
        po->y = static_cast<float>(p_global(1));
        po->z = static_cast<float>(p_global(2));
        po->intensity = pi->intensity;
    }

    /**
     * @brief 增量式地图更新
     *
     * 将当前帧点云加入 iVox 地图，支持并行处理
     *
     * 处理逻辑：
     * 1. 将点从本体坐标系转换到世界坐标系
     * 2. 检查点附近是否已有地图点
     * 3. 如果距离体素中心较远，直接添加（无需降采样）
     * 4. 如果距离较近，根据距离判断是否需要添加（距离更近的优先）
     */
    void mapIncremental() {
        PointVector points_to_add;
        PointVector point_no_need_downsample;

        const int cur_pts = static_cast<int>(scan_down_body->size());
        points_to_add.reserve(cur_pts);
        point_no_need_downsample.reserve(cur_pts);

        // 创建索引数组用于并行处理
        std::vector<size_t> index(cur_pts);
        for (size_t i = 0; i < static_cast<size_t>(cur_pts); ++i) {
            index[i] = i;
        }

        // 并行处理每个点
        std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t i) {
            // 转换到世界坐标系
            pointBodyToWorld(&(scan_down_body->points[i]), &(scan_down_world->points[i]));

            PointType& point_world = scan_down_world->points[i];
            if (!nearest_points[i].empty() && flg_ekf_inited) {
                // 获取近邻点
                const PointVector& points_near = nearest_points[i];

                // 计算体素中心
                Eigen::Vector3f center =
                    ((point_world.getVector3fMap() / static_cast<float>(filter_size_map_min)).array().floor() + 0.5f) *
                    static_cast<float>(filter_size_map_min);

                // 计算最近邻点到体素中心的距离
                Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                // 如果最近邻点距离体素中心较远，直接添加（无需降采样）
                if (fabs(dis_2_center.x()) > 0.5f * static_cast<float>(filter_size_map_min) &&
                    fabs(dis_2_center.y()) > 0.5f * static_cast<float>(filter_size_map_min) &&
                    fabs(dis_2_center.z()) > 0.5f * static_cast<float>(filter_size_map_min)) {
                    point_no_need_downsample.emplace_back(point_world);
                    return;
                }

                // 判断是否需要添加该点（距离体素中心更近则添加）
                bool need_add = true;
                const float dist = common::calc_dist(point_world.getVector3fMap(), center);
                if (points_near.size() >= static_cast<size_t>(options::NUM_MATCH_POINTS)) {
                    for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                        if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6f) {
                            need_add = false;
                            break;
                        }
                    }
                }
                if (need_add) {
                    points_to_add.emplace_back(point_world);
                }
            } else {
                // 没有近邻点或 EKF 未初始化，直接添加
                points_to_add.emplace_back(point_world);
            }
        });

        // 批量添加点到 iVox 地图
        ivox->AddPoints(points_to_add);
        ivox->AddPoints(point_no_need_downsample);
    }

    /**
     * @brief 观测模型：计算点云到地图的配准残差
     *
     * @param s 当前状态
     * @param ekfom_data EKFoM 数据结构
     *
     * 处理流程：
     * 1. 将点云从本体坐标系转换到世界坐标系
     * 2. 在 iVox 地图中查找每个点的近邻点
     * 3. 拟合局部平面，计算点到平面的距离（残差）
     * 4. 筛选有效特征点
     * 5. 构建观测雅可比矩阵
     */
    void obsModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
        const int cnt_pts = static_cast<int>(scan_down_body->size());

        // 创建索引数组
        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        // 重置观测海森矩阵
        obs_hessian.setZero();

        // 步骤 1: 点云配准和近邻搜索（并行执行）
        Timer::Evaluate(
            [&, this]() {
                // 预计算变换矩阵
                const auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
                const auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

                std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t i) {
                    PointType& point_body = scan_down_body->points[i];
                    PointType& point_world = scan_down_world->points[i];

                    // 将点从本体坐标系转换到世界坐标系
                    common::V3F p_body = point_body.getVector3fMap();
                    point_world.getVector3fMap() = R_wl * p_body + t_wl;
                    point_world.intensity = point_body.intensity;

                    // 在 iVox 地图中查找近邻点
                    auto& points_near = nearest_points[i];
                    if (ekfom_data.converge) {
                        ivox->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                        point_selected_surf[i] =
                            (points_near.size() >= static_cast<size_t>(options::MIN_NUM_MATCH_POINTS)) ? 1 : 0;
                        // 拟合局部平面
                        if (point_selected_surf[i]) {
                            point_selected_surf[i] =
                                common::esti_plane(plane_coef[i], points_near, options::ESTI_PLANE_THRESHOLD) ? 1 : 0;
                        }
                    }

                    // 计算点到平面的距离（残差）
                    if (point_selected_surf[i]) {
                        auto temp = point_world.getVector4fMap();
                        temp[3] = 1.0f;
                        const float pd2 = plane_coef[i].dot(temp);

                        // 检查残差质量（距离越远，允许的残差越大）
                        const bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                        if (valid_corr) {
                            point_selected_surf[i] = 1;
                            residuals[i] = pd2;
                        } else {
                            // 残差不符合质量要求，清除选择标志
                            // 注意：原始 Faster-LIO 此处缺少 else 分支，会导致旧残差被带入下一轮 IEKF
                            point_selected_surf[i] = 0;
                        }
                    }
                });
            },
            "    ObsModel (Lidar Match)");

        // 步骤 2: 统计有效特征点
        effect_feat_num = 0;
        corr_pts.resize(cnt_pts);
        corr_norm.resize(cnt_pts);
        for (int i = 0; i < cnt_pts; i++) {
            if (point_selected_surf[i]) {
                corr_norm[effect_feat_num] = plane_coef[i];
                corr_pts[effect_feat_num] = scan_down_body->points[i].getVector4fMap();
                corr_pts[effect_feat_num][3] = residuals[i];
                effect_feat_num++;
            }
        }
        corr_pts.resize(effect_feat_num);
        corr_norm.resize(effect_feat_num);

        // 检查是否有有效特征点
        if (effect_feat_num < 1) {
            ekfom_data.valid = false;
            LOG(WARNING) << "[SLAM][Core][ObsModel] No Effective Points!";
            return;
        }

        // 步骤 3: 构建观测雅可比矩阵（并行执行）
        Timer::Evaluate(
            [&, this]() {
                ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);
                ekfom_data.h.resize(effect_feat_num);

                index.resize(effect_feat_num);
                // 预计算常用矩阵
                const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
                const common::V3F off_t = s.offset_T_L_I.cast<float>();
                const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

                std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t i) {
                    // 计算点在本体坐标系下的位置
                    common::V3F point_this_be = corr_pts[i].head<3>();
                    common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                    // 转换到 IMU 坐标系
                    common::V3F point_this = off_R * point_this_be + off_t;
                    common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                    // 平面法向量
                    common::V3F norm_vec = corr_norm[i].head<3>();

                    // 计算雅可比矩阵的各个分量
                    common::V3F C(Rt * norm_vec);
                    common::V3F A(point_crossmat * C);

                    // 根据是否估计外参，构建完整的雅可比矩阵
                    if (extrinsic_est_en) {
                        // 估计外参：包含外参旋转和平移的导数
                        common::V3F B(point_be_crossmat * off_R.transpose() * C);
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            B[0], B[1], B[2], C[0], C[1], C[2];
                    } else {
                        // 不估计外参：外参部分为零
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                    }

                    // 观测残差
                    ekfom_data.h(i) = -corr_pts[i][3];

                    // 退化检测（当前禁用）
                    if constexpr (kEnableDegeneracyDiag) {
                        Eigen::Matrix<double, 6, 1> A_i;
                        A_i << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];
                        obs_hessian += A_i * A_i.transpose();
                    }
                });
            },
            "    ObsModel (IEKF Build Jacobian)");

        // 输出观测模型诊断日志
        logObsModelDiag("Align", frame_num, sync_ok_count, kObsDiagMaxIekfFrames, cnt_pts, effect_feat_num,
                        scan_down_body, scan_down_world, nearest_points, residuals, point_selected_surf,
                        plane_coef, ekfom_data, obs_diag_logged_this_iekf);
    }

    /**
     * @brief 构建里程计结果
     *
     * @return OdomResult 当前状态对应的里程计结果
     *
     * 注意：协方差矩阵的旋转部分取 P(3:6,3:6)，而非 P(6:9,6:9)
     * 原实现误取外参旋转的协方差，导致融合层低估姿态不确定性
     */
    OdomResult buildOdomResult() const {
        OdomResult odom;
        odom.timestamp = lidar_end_time;
        odom.position = state_point.pos;
        odom.orientation = Quatd(state_point.rot.coeffs()[3], state_point.rot.coeffs()[0],
                                 state_point.rot.coeffs()[1], state_point.rot.coeffs()[2]);
        odom.velocity = state_point.vel;
        odom.valid = flg_ekf_inited;
        odom.frame_id = "odom";

        // 状态布局：pos[0:3], rot[3:6], offset_R_L_I[6:9], offset_T_L_I[9:12], vel[12:15]...
        const auto P = kf.get_P();
        odom.covariance.block<3, 3>(0, 0) = P.block<3, 3>(0, 0);  // 位置协方差
        odom.covariance.block<3, 3>(3, 3) = P.block<3, 3>(3, 3);  // 旋转协方差

        return odom;
    }

    /**
     * @brief 计算退化程度
     *
     * @return std::pair<bool, double> 是否退化、退化因子
     *
     * 通过观测海森矩阵的特征值分析判断退化：
     * - 如果最小特征值 < 阈值 或 条件数 > 20，认为退化
     * - 退化因子用于调整状态更新步长
     */
    std::pair<bool, double> computeDegeneracy() const {
        if (obs_hessian.isZero(1e-9)) {
            return {false, 0.0};
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(obs_hessian);
        const auto eigenvalues = solver.eigenvalues();
        const double min_ev = eigenvalues.minCoeff();
        const double max_ev = eigenvalues.maxCoeff();
        if (max_ev < 1e-9) {
            return {true, 1.0};
        }
        const double ratio = min_ev / max_ev;
        const bool degenerate = min_ev < degeneracy_threshold || ratio < 0.05;
        const double factor = degenerate ? std::min(1.0, std::max(0.0, 1.0 - ratio * 10.0)) : 0.0;
        return {degenerate, factor};
    }
};

// =============================================================================
// LaserMappingCore 公共接口实现
// =============================================================================

LaserMappingCore::LaserMappingCore() : impl_(std::make_unique<Impl>()) {}

LaserMappingCore::~LaserMappingCore() = default;

/**
 * @brief 初始化 LaserMappingCore
 *
 * @param cfg SLAM 配置
 * @return true 初始化成功
 * @return false 初始化失败
 *
 * 初始化流程：
 * 1. 创建 Impl 实例
 * 2. 设置算法参数（迭代次数、平面阈值、退化阈值等）
 * 3. 设置地图参数（检测范围、体素分辨率等）
 * 4. 设置外参和传感器参数
 * 5. 创建 iVox 地图和 IMU 处理器
 * 6. 初始化 IKFoM 滤波器
 */
bool LaserMappingCore::initialize(const SlamConfig& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = cfg;
    impl_ = std::make_unique<Impl>();

    // 设置算法选项
    options::NUM_MAX_ITERATIONS = cfg.max_iteration;
    options::ESTI_PLANE_THRESHOLD = static_cast<float>(cfg.esti_plane_threshold);
    impl_->degeneracy_threshold = cfg.degeneracy_threshold;
    impl_->det_range = static_cast<float>(cfg.det_range);
    impl_->cube_len = cfg.cube_side_length;
    impl_->filter_size_map_min = cfg.filter_size_map;
    impl_->extrinsic_est_en = cfg.estimate_extrinsic;
    impl_->extrinT = cfg.extrinsic_T;
    impl_->extrinR = cfg.extrinsic_R;

    // 设置 iVox 选项
    impl_->ivox_options.resolution_ = static_cast<float>(cfg.ivox_grid_resolution);
    switch (cfg.ivox_nearby_type) {
        case 0:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::CENTER;
            break;
        case 6:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY6;
            break;
        case 26:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            break;
        case 18:
        default:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
    }

    // 创建 iVox 地图和 IMU 处理器
    impl_->ivox = std::make_shared<IVoxType>(impl_->ivox_options);
    impl_->imu_process = std::make_shared<ImuProcess>();
    // 与 ysw_loc faster-lio 一致：平均扫描时间初值为 0.0，由 SyncPackages 在线估计
    impl_->lidar_mean_scantime = 0.0;

    // 设置降采样滤波器
    impl_->voxel_scan.setLeafSize(static_cast<float>(cfg.filter_size_surf),
                                  static_cast<float>(cfg.filter_size_surf),
                                  static_cast<float>(cfg.filter_size_surf));

    // 设置辅助传感器参数
    impl_->use_vel = cfg.use_vel;
    impl_->use_gnss_attitude = cfg.use_gnss_attitude;
    impl_->can_vel_diff_thresh_kmh = cfg.can_vel_diff_thresh_kmh;
    impl_->can_timeout_sec = cfg.can_timeout_sec;
    impl_->gnss_attitude_timeout_sec = cfg.gnss_attitude_timeout_sec;
    impl_->attitude_diff_thresh_deg = cfg.attitude_diff_thresh_deg;
    impl_->attitude_correction_gain = cfg.attitude_correction_gain;

    // 设置 IMU 处理器参数
    impl_->imu_process->SetExtrinsic(cfg.extrinsic_T, cfg.extrinsic_R);
    impl_->imu_process->SetGyrCov(common::V3D(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov));
    impl_->imu_process->SetAccCov(common::V3D(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov));
    impl_->imu_process->SetGyrBiasCov(common::V3D(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov));
    impl_->imu_process->SetAccBiasCov(common::V3D(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov));

    // 初始化 IKFoM 滤波器
    std::vector<double> epsi(23, 0.001);
    impl_->kf.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
            impl_->obsModel(s, ekfom_data);
        },
        options::NUM_MAX_ITERATIONS, epsi.data());

    // 初始化输出
    odom_ = OdomResult();
    odom_.frame_id = "odom";
    local_map_.cloud.reset(new PointCloud());
    current_scan_.cloud.reset(new PointCloud());
    initialized_ = true;
    is_degenerate_ = false;
    degeneracy_factor_ = 0.0;

    LOG(INFO) << "[SLAM][Core][Init] OK type=" << cfg.type << " lidar_type=" << cfg.lidar_type
              << " scan_rate=" << cfg.scan_rate << " blind=" << cfg.blind
              << " point_filter=" << cfg.point_filter_num << " ivox_res=" << cfg.ivox_grid_resolution
              << " max_iter=" << cfg.max_iteration << " filter_size_map=" << cfg.filter_size_map
              << " extrinsic_est=" << cfg.estimate_extrinsic << " scan_period=" << impl_->lidar_mean_scantime
              << " use_vel=" << cfg.use_vel << " use_gnss_attitude=" << cfg.use_gnss_attitude
              << " LASER_POINT_COV=" << options::LASER_POINT_COV
              << " NUM_MAX_ITERATIONS=" << options::NUM_MAX_ITERATIONS;

    return true;
}

/**
 * @brief 输入 IMU 数据
 *
 * @param imu IMU 数据
 *
 * 将 IMU 数据转换为 Faster-LIO 格式并加入缓冲区
 */
void LaserMappingCore::feedImu(const ImuData& imu) {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Core][FeedImu] skip: not initialized";
        return;
    }
    // 转换为 Faster-LIO 的 ImuSample 格式
    const auto sample = toImuSample(imu);
    // 持锁加入缓冲区
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    if (sample->timestamp < impl_->last_timestamp_imu) {
        // 时间戳回退，清空缓冲区
        LOG(WARNING) << "[SLAM][Core][FeedImu] loop back, clear buffer";
        impl_->imu_buffer.clear();
    }
    impl_->last_timestamp_imu = sample->timestamp;
    impl_->imu_buffer.emplace_back(sample);
    ++impl_->imu_feed_count;
    if (impl_->imu_feed_count == 1 || impl_->imu_feed_count % 1000 == 0) {
        LOG(INFO) << "[SLAM][Core][FeedImu] #" << impl_->imu_feed_count << " ts=" << sample->timestamp
                  << " buf=" << impl_->imu_buffer.size();
    }
}

/**
 * @brief 输入激光雷达点云数据
 *
 * @param pc 时间戳点云数据
 */
void LaserMappingCore::feedLidar(const TimestampedPointCloud& pc) {
    if (!pc.cloud || pc.cloud->empty()) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: empty cloud ts=" << pc.timestamp;
        return;
    }
    feedLidarCloud(pc.timestamp, pc.cloud);
}

/**
 * @brief 输入轮速计数据
 *
 * @param can 轮速计数据
 */
void LaserMappingCore::feedCanVelocity(const CanVelocityData& can) {
    if (!initialized_) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    impl_->last_can = can;
    impl_->has_can = true;
}

/**
 * @brief 输入 GNSS 姿态数据
 *
 * @param gnss GNSS 姿态数据
 */
void LaserMappingCore::feedGnssAttitude(const GnssAttitudeData& gnss) {
    if (!initialized_) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    impl_->last_gnss_attitude = gnss;
    impl_->has_gnss_attitude = true;
}

/**
 * @brief 输入激光雷达点云（内部实现）
 *
 * @param timestamp 时间戳
 * @param cloud 点云数据
 *
 * 处理逻辑：
 * 1. 持锁进行预处理
 * 2. 检查时间戳回退
 * 3. 缓冲区未满时入队，否则丢弃
 */
void LaserMappingCore::feedLidarCloud(double timestamp, PointCloudPtr cloud) {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: not initialized";
        return;
    }
    if (!cloud || cloud->empty()) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: empty cloud ts=" << timestamp;
        return;
    }

    // 对齐 ysw StandardPCLCallBack：持锁 preprocess 后入队
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    ++impl_->scan_count;

    // 检查时间戳回退
    if (timestamp < impl_->last_timestamp_lidar) {
        LOG(ERROR) << "[SLAM][Align][FeedLidar] loop back, clear buffer";
        impl_->lidar_buffer.clear();
        impl_->time_buffer.clear();
    }

    // 缓冲区未满时入队
    if (impl_->lidar_buffer.size() <= Impl::kMaxLidarBufferSize - 1) {
        impl_->lidar_buffer.push_back(cloud);
        impl_->time_buffer.push_back(timestamp);
        impl_->last_timestamp_lidar = timestamp;
        ++impl_->lidar_feed_count;
        const float curv_max = cloud->points.back().curvature;
        LOG(INFO) << "[SLAM][Align][FeedLidar] #" << impl_->scan_count << " ts=" << timestamp
                  << " pts=" << cloud->size() << " curv_max_ms=" << curv_max
                  << " buf=" << impl_->lidar_buffer.size();
    } else {
        // 缓冲区满，丢弃当前帧
        ++impl_->lidar_drop_count;
        LOG(WARNING) << "[SLAM][Align][FeedLidar] buffer full, drop cb=#" << impl_->scan_count
                       << " buf=" << impl_->lidar_buffer.size();
    }
}



/**
 * @brief 应用轮速计和 GNSS 姿态辅助校正
 *
 * @return true 进行了校正
 * @return false 未进行校正
 *
 * 校正逻辑：
 * 1. GNSS 姿态校正：当俯仰/横滚角与 GNSS 差异超过阈值时，按比例校正
 * 2. 轮速计校正：当 LiDAR 估计速度与轮速差异超过阈值时，直接重置速度
 */
bool LaserMappingCore::applyYswVelocityAid() {
    auto& impl = *impl_;
    if (!impl.use_vel || !impl.has_can) {
        return false;
    }

    // 获取当前状态
    state_ikfom state = impl.kf.get_x();
    const Eigen::Quaterniond q(state.rot.coeffs()[3], state.rot.coeffs()[0],
                               state.rot.coeffs()[1], state.rot.coeffs()[2]);
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    // 提取当前姿态角
    const float pitch_rad = static_cast<float>(std::asin(R(0, 2)));
    const float roll_rad = static_cast<float>(std::atan2(-R(1, 2), R(2, 2)));
    const float heading_rad = static_cast<float>(std::atan2(R(1, 0), R(0, 0)));
    const float pitch_deg = normalizePitchRoll(pitch_rad * 180.0f / static_cast<float>(M_PI));
    const float roll_deg = normalizePitchRoll(roll_rad * 180.0f / static_cast<float>(M_PI));
    const float final_heading = heading_rad * 180.0f / static_cast<float>(M_PI);

    bool changed = false;

    // GNSS 姿态校正
    if (impl.use_gnss_attitude && impl.has_gnss_attitude) {
        const double gnss_age_sec = std::abs(impl.measures.lidar_bag_time_ - impl.last_gnss_attitude.timestamp);
        // 当俯仰或横滚角差异超过阈值时进行校正
        if (std::abs(pitch_deg + impl.last_gnss_attitude.pitch_deg) > impl.attitude_diff_thresh_deg ||
            std::abs(roll_deg - impl.last_gnss_attitude.roll_deg) > impl.attitude_diff_thresh_deg) {
            // 按比例校正姿态（增益控制）
            const double modified_pitch_deg = pitch_deg + impl.attitude_correction_gain *
                                                          (-impl.last_gnss_attitude.pitch_deg - pitch_deg);
            const double modified_roll_deg = roll_deg + impl.attitude_correction_gain *
                                                        (impl.last_gnss_attitude.roll_deg - roll_deg);
            const Eigen::Quaterniond q_modified = anglesToQuaternion(final_heading, modified_pitch_deg,
                                                                     modified_roll_deg);
            state.rot = SO3(q_modified.w(), q_modified.x(), q_modified.y(), q_modified.z());
            impl.kf.change_x(state);
            ++impl.attitude_reset_count;
            changed = true;
            LOG(INFO) << "[SLAM][YswVel] Heading: " << final_heading << " , Pitch: " << pitch_deg
                      << " , Roll: " << roll_deg;
            LOG(INFO) << "[SLAM][YswVel] reset pitch error = "
                      << (-impl.last_gnss_attitude.pitch_deg - pitch_deg);
            LOG(INFO) << "[SLAM][YswVel] modified pitch = " << modified_pitch_deg
                      << " modified roll = " << modified_roll_deg
                      << " gnss_age=" << gnss_age_sec
                      << " attitude_reset_count=" << impl.attitude_reset_count;
        }
    }

    // 轮速计速度校正
    const double lidar_speed_kmh = std::sqrt(state.vel(0) * state.vel(0) + state.vel(1) * state.vel(1)) * 3.6;
    const double can_speed_correct = impl.last_can.speed_kmh;
    if (std::abs(lidar_speed_kmh - can_speed_correct) > impl.can_vel_diff_thresh_kmh) {
        // 将轮速转换到世界坐标系
        const Eigen::Matrix3d rot_world_body = state.rot.toRotationMatrix();
        const Eigen::Vector3d can_body_vel_kmh(can_speed_correct, 0.0001, 0.0001);
        const Eigen::Vector3d can_world_vel_kmh = rot_world_body * can_body_vel_kmh;
        state.vel(0) = can_world_vel_kmh(0) / 3.6;
        state.vel(1) = can_world_vel_kmh(1) / 3.6;
        state.vel(2) = can_world_vel_kmh(2) / 3.6;
        impl.kf.change_x(state);
        ++impl.vel_reset_count;
        changed = true;
        LOG(INFO) << "[SLAM][YswVel] CAN velocity correction: LiDAR=" << lidar_speed_kmh
                  << " km/h, CAN=" << can_speed_correct << " km/h,"
                  << " reset_count=" << impl.vel_reset_count
                  << " vel=(" << state.vel.transpose() << ")";
    }
    return changed;
}

/**
 * @brief 单次 SLAM 处理流程
 *
 * @return true 处理成功
 * @return false 处理失败或跳过
 *
 * 处理流程（对齐 ysw LaserMapping::Run()）：
 * 1. 同步 IMU 和激光雷达数据包
 * 2. IMU 处理（状态预测 + 点云去畸变）
 * 3. 轮速计/GNSS 辅助校正
 * 4. 首帧初始化（构建初始地图）
 * 5. 点云降采样
 * 6. IEKF 迭代更新
 * 7. 地图增量更新
 * 8. 构建里程计结果
 */
bool LaserMappingCore::processOnce() {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Core][Process] skip: not initialized";
        return false;
    }

    ++impl_->process_tick_count;

    // 步骤 1: 同步数据包
    {
        std::lock_guard<std::mutex> buf_lock(impl_->buffer_mutex);
        if (!impl_->syncPackages()) {
            return false;
        }
    }

    LOG(INFO) << "[SLAM][Core][Process] tick=" << impl_->process_tick_count
              << " -> ImuProcess (imu_n=" << impl_->measures.imu_.size() << ")";

    logIkfState("before_ImuProcess", impl_->sync_ok_count, impl_->scan_count,
                impl_->kf.get_x());

    // 步骤 2: IMU 处理
    // 注意：必须先清空 scan_undistort，否则 IMU 初始化阶段会复用上一帧点云
    impl_->scan_undistort->clear();
    impl_->imu_process->Process(impl_->measures, impl_->kf, impl_->scan_undistort);
    if (impl_->scan_undistort->empty()) {
        LOG(WARNING) << "[SLAM][Core][Process] ImuProcess done but scan empty (likely IMU init), imu_n="
                     << impl_->measures.imu_.size();
        return false;
    }

    logIkfState("after_ImuProcess", impl_->sync_ok_count, impl_->scan_count, impl_->kf.get_x());

    // 步骤 3: 轮速计/GNSS 辅助校正
    applyYswVelocityAid();
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        logPDiagonal("Align", "after_ImuProcess", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
    }

    // 步骤 4: 首帧初始化
    if (impl_->flg_first_scan) {
        impl_->state_point = impl_->kf.get_x();
        impl_->scan_down_world->resize(impl_->scan_undistort->size());
        for (size_t i = 0; i < impl_->scan_undistort->size(); i++) {
            impl_->pointBodyToWorld(&impl_->scan_undistort->points[i], &impl_->scan_down_world->points[i]);
        }
        // 将首帧点云加入地图
        impl_->ivox->AddPoints(impl_->scan_down_world->points);
        impl_->first_lidar_time = impl_->measures.lidar_bag_time_;
        impl_->flg_first_scan = false;
        LOG(INFO) << "[SLAM][Align][MapInit] first_scan sync=#" << impl_->sync_ok_count
                  << " feed_cb=#" << impl_->scan_count
                  << " lidar_t=" << impl_->measures.lidar_bag_time_
                  << " pts=" << impl_->scan_undistort->size() << " t0=" << impl_->first_lidar_time;
        logIkfState("first_scan_map", impl_->sync_ok_count, impl_->scan_count, impl_->state_point);
        logMapInitFirstPoints("Align", *impl_->scan_undistort, *impl_->scan_down_world);
        // 首帧只建图，不执行 IEKF、不发布结果、不更新 odom_
        return false;
    }

    // 步骤 5: 点云降采样
    impl_->obs_diag_logged_this_iekf = false;
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        LOG(INFO) << "[SLAM][Align][ObsDiag] >>> IEKF start sync=#" << impl_->sync_ok_count
                  << " iekf_frame=" << impl_->frame_num << " feed_cb=#" << impl_->scan_count;
    }

    // 检查 EKF 初始化状态
    const double lidar_dt = impl_->measures.lidar_bag_time_ - impl_->first_lidar_time;
    impl_->flg_ekf_inited = lidar_dt >= options::INIT_TIME;

    Timer::Evaluate(
        [this]() {
            impl_->voxel_scan.setInputCloud(impl_->scan_undistort);
            impl_->voxel_scan.filter(*impl_->scan_down_body);
        },
        "Downsample PointCloud");

    const int cur_pts = static_cast<int>(impl_->scan_down_body->size());
    if (cur_pts < 5) {
        LOG(WARNING) << "[SLAM][Core][Process] Too few downsampled points: undistort="
                     << impl_->scan_undistort->size() << " down=" << cur_pts;
        return false;
    }

    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][Align][Process] downsample "
              << impl_->scan_undistort->size() << " -> " << cur_pts << " ekf_inited=" << impl_->flg_ekf_inited
              << " lidar_t=" << impl_->measures.lidar_bag_time_ << " first_lidar_t=" << impl_->first_lidar_time
              << " delta_t=" << lidar_dt << " INIT_TIME=" << options::INIT_TIME;

    impl_->scan_down_world->resize(cur_pts);
    impl_->nearest_points.resize(cur_pts);
    impl_->residuals.resize(cur_pts, 0);
    impl_->point_selected_surf.resize(cur_pts, 1);
    impl_->plane_coef.resize(cur_pts, common::V4F::Zero());

    Timer::Evaluate(
        [this]() {
            double solve_H_time = 0;
            if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
                logPDiagonal("Align", "before_IEKF", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
                impl_->kf.set_iteration_diag(true);
            }
            impl_->kf.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            impl_->kf.set_iteration_diag(false);
            impl_->state_point = impl_->kf.get_x();
            impl_->euler_cur = SO3ToEuler(impl_->state_point.rot);
            impl_->pos_lidar =
                impl_->state_point.pos + impl_->state_point.rot * impl_->state_point.offset_T_L_I;
        },
        "IEKF Solve and Update");

    logIkfState("after_IEKF", impl_->sync_ok_count, impl_->scan_count, impl_->state_point);
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        logPDiagonal("Align", "after_IEKF", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
    }
    if (impl_->frame_num < 10) {
        LOG(INFO) << "[SLAM][Align][Process] early_pos frame #" << impl_->frame_num << " sync=#"
                  << impl_->sync_ok_count << " feed_cb=#" << impl_->scan_count
                  << " lidar_t=" << impl_->measures.lidar_bag_time_ << " pts="
                  << impl_->measures.lidar_->size() << " pos=(" << impl_->state_point.pos.transpose()
                  << ")";
    }

    Timer::Evaluate([this]() { impl_->mapIncremental(); }, "    Incremental Mapping");

    if constexpr (Impl::kEnableDegeneracyDiag) {
        const auto degeneracy = impl_->computeDegeneracy();
        is_degenerate_ = degeneracy.first;
        degeneracy_factor_ = degeneracy.second;
    } else {
        is_degenerate_ = false;
        degeneracy_factor_ = 0.0;
    }

    odom_ = impl_->buildOdomResult();
    odom_.is_degenerate = is_degenerate_;
    odom_.degeneracy_factor = degeneracy_factor_;

    // 返回 LiDAR body 系去畸变点云供关键帧存储（insertKeyFrame 期望 cloud_body）。
    // 注意：不转换到世界系，下游使用时以 odom_.pose 做外部变换；
    // 如需发布世界系，由调用方（如 slam_debug_node）自行用 odom 变换。
    current_scan_.timestamp = impl_->lidar_end_time;
    // 进行下采样
    pcl::VoxelGrid<PointI> voxel_grid;
    impl_->voxel_scan.setInputCloud(impl_->scan_undistort);
    impl_->voxel_scan.filter(*current_scan_.cloud);
    // current_scan_.cloud.reset(new PointCloud(*impl_->scan_undistort));

    local_map_.timestamp = impl_->lidar_end_time;
    local_map_.cloud.reset(new PointCloud(*impl_->scan_down_world));

    impl_->frame_num++;
    LOG(INFO) << "[SLAM][Core][Process] frame #" << impl_->frame_num << " OK odom_valid=" << odom_.valid
              << " [mapping] In=" << impl_->scan_undistort->points.size() << " downsamp=" << cur_pts
              << " MapGrid=" << impl_->ivox->NumValidGrids() << " effect=" << impl_->effect_feat_num
              << " degenerate=" << (Impl::kEnableDegeneracyDiag ? (is_degenerate_ ? 1 : 0) : -1);

    return true;
}

void LaserMappingCore::reset() {
    SlamConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }
        cfg = config_;
    }
    initialize(cfg);
}

OdomResult LaserMappingCore::getOdometry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_;
}

TimestampedPointCloud LaserMappingCore::getLocalMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_map_;
}

TimestampedPointCloud LaserMappingCore::getCurrentScan() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_scan_;
}

bool LaserMappingCore::isDegenerate() const {
    return is_degenerate_;
}

double LaserMappingCore::getDegeneracyFactor() const {
    return degeneracy_factor_;
}

}  // namespace localization