#ifndef FASTER_LIO_LASER_ODOMETRY_H
#define FASTER_LIO_LASER_ODOMETRY_H

/**
 * @file laser_odometry.h
 * @brief Faster-LIO 纯里程计核心（从 faster_lio_ws 的 laser_mapping 裁剪）
 *
 * 与 faster_lio_ws 的 LaserMapping 保持同一算法实现，仅移除建图/发布部分：
 * - 保留：IMU/LiDAR 缓冲与同步、IMU 去畸变、IEKF 状态估计、iVox 局部地图配准、
 *          轮速/GNSS 姿态辅助校正
 * - 移除：全局地图累积/PCD 保存、path/cloud 发布、ROS 订阅发布（由节点负责）
 *
 * 该文件直接移植自 faster_lio_ws/src/faster-lio/include/laser_mapping.h，
 * 命名空间、数据结构、数值逻辑与原始版本一致，保证 -O3 优化编译下行为相同。
 */

#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <condition_variable>
#include <cstdint>
#include <thread>

#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"
#include "dcreg_wrapper.h"

namespace faster_lio {

/// 传感器健康状态
struct SensorHealth {
    // 轮速
    bool can_healthy = false;
    double can_age = 1e9;
    bool has_can_data = false;
    double can_last_update = 0.0;

    // GNSS
    bool gnss_healthy = false;
    double gnss_age = 1e9;
    bool has_gnss_data = false;
    int gnss_status = 0;
    int gnss_nsv = 0;
    double gnss_last_update = 0.0;

    // LiDAR
    bool lidar_healthy = false;
    size_t lidar_points = 0;

    /// 转为字符串（用于日志）
    std::string toString() const {
        std::ostringstream oss;
        oss << "can=" << (can_healthy ? "ok" : "fail") << "(age=" << can_age << ") "
            << "gnss=" << (gnss_healthy ? "ok" : "fail") << "(age=" << gnss_age
            << " status=" << gnss_status << " nsv=" << gnss_nsv << ") "
            << "lidar=" << (lidar_healthy ? "ok" : "fail") << "(pts=" << lidar_points << ")";
        return oss.str();
    }
};

/// 退化检测信息
struct DegeneracyInfo {
    bool is_degenerate = false;              ///< 是否退化
    double degeneracy_ratio = 1.0;           ///< λ_min / λ_max，越小越退化
    Eigen::Vector3d degenerate_axis = Eigen::Vector3d::Zero();  ///< 物理空间中的退化轴方向
    bool is_rot_degenerate[3] = {false, false, false};  ///< roll/pitch/yaw 是否退化
};

/// 纯里程计核心：与 faster_lio_ws LaserMapping 的算法部分一致
class LaserOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// 轮速计数据（与 slam_odometry 的 CanVelocityData 对应）
    struct CanVelocityData {
        double timestamp = 0.0;
        double speed_kmh = 0.0;  ///< 纵向速度 (km/h)，倒车为负
        bool valid = true;
    };

    /// GNSS 姿态数据（与 slam_odometry 的 GnssAttitudeData 对应）
    struct GnssAttitudeData {
        double timestamp = 0.0;
        double heading_deg = 0.0;
        double pitch_deg = 0.0;
        double roll_deg = 0.0;
        int status = 0;      ///< GNSS 状态（如 RTK 固定解=33）
        int nsv = 0;         ///< 可见卫星数量
        bool valid = true;
    };

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    LaserOdometry();
    ~LaserOdometry() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser odometry deconstruct";
    }

    /// 从 yaml 配置初始化（与 faster_lio_ws InitWithoutROS 一致）
    bool InitWithoutROS(const std::string &config_yaml);

    /// 单帧处理：IMU 预测 + 去畸变 + IEKF 更新 + 局部地图增量
    bool Run();

    // callbacks of lidar and imu（与 faster_lio_ws 一致，原始消息直通）
    void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    /// 轮速/GNSS 数据输入（由节点订阅并转发）
    void FeedCanVelocity(const CanVelocityData &can);
    void FeedGnssAttitude(const GnssAttitudeData &gnss);

    /// 设置轮速/GNSS 辅助参数（覆盖 yaml 配置）
    void SetVelocityGnssParams(bool use_vel, bool use_gnss_attitude,
                               double can_vel_diff_thresh_kmh = 2.0,
                               double can_timeout_sec = 0.2,
                               double gnss_attitude_timeout_sec = 1.0,
                               double attitude_diff_thresh_deg = 2.0,
                               double attitude_correction_gain = 0.7);

    /// 设置观测参数（轮速噪声、非完整性约束噪声、GNSS 噪声、最小状态等）
    void SetObservationParams(double wheel_velocity_noise,
                              double non_holonomic_noise,
                              double gnss_attitude_noise_rad2,
                              int min_gnss_status, int min_gnss_nsv, int min_lidar_points);

    /// 获取传感器健康状态
    SensorHealth GetSensorHealth() const;

    // sync lidar with imu
    bool SyncPackages();

    /// interface of mtk, customized observation model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// 停止/清理（无 PCD 保存；建图部分由 slam_mapping_v2 负责）
    void Finish();

    /// 解耦的只读接口（与 faster_lio_ws 一致）
    bool GetOdometry(nav_msgs::Odometry &out) const;
    bool GetScanBody(PointCloudType::Ptr &out) const;
    bool GetScanInImu(PointCloudType::Ptr &out) const;
    bool GetEffectInImu(PointCloudType::Ptr &out) const;
    bool GetScanWorld(PointCloudType::Ptr &out) const;
    bool GetLocalMap(PointCloudType::Ptr &out) const;

    /// 统计信息
    int frameCount() const { return frame_num_; }
    double lastLidarTime() const { return lidar_end_time_; }
    int effectFeatureNum() const { return effect_feat_num_; }

   private:
    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    void PrintState(const state_ikfom &s);

    /// 轮速/GNSS 辅助校正（与 faster_lio_ws ApplyVelocityGnssAid 一致）
    bool ApplyVelocityGnssAid();

    /// 更新传感器健康状态（检查超时）
    void UpdateSensorHealth();

    /// 轮速观测更新（单次更新，使用完整 23 维 H 矩阵）
    void ApplyWheelSpeedObservation();

    /// GNSS 姿态观测更新（单次更新，只观测 pitch 和 roll）
    void ApplyGnssAttitudeObservation();

    /// IMU 加速度计重力方向观测：利用加速度计测量重力方向，约束 roll 和 pitch
    /// 只有当车辆不加速时（|acc_norm - g| < threshold）才应用
    void ApplyImuGravityObservation();

    /// 检测当前是否处于低加速状态（适合进行重力方向观测）
    bool isLowAcceleration() const;

    /// 退化检测：基于法向量协方差分析
    DegeneracyInfo DetectDegeneracy() const;

    /// P 矩阵膨胀：在退化方向上恢复 P 的不确定性，让后续观测有话语权
    void InflateDegenerateP(const DegeneracyInfo& info);

    /// DCReg 包装观测模型：调用原 ObsModel + DCReg 诊断 + 鲁棒加权
    void ObsModelWithDCReg(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// DCReg 后处理：对 ekfom_data 做退化诊断 + Huber + 退化降权
    void ApplyDCRegAndWeights(
        esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// DCReg P 矩阵膨胀：在 DCReg 检测到的退化方向上进行定向协方差膨胀
    void InflateDegeneratePFromDCReg(const DCRegResult& dc_result);

    /// 检查 GNSS 姿态数据是否有效
    bool isGnssAttitudeValid() const;

   private:
    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    double filter_size_map_min_ = 0;
    float map_trim_radius_ = 0.0f;        ///< iVox 局部地图剔除半径（米，0=不剔除）
    int map_trim_interval_frames_ = 10;   ///< 每 N 帧执行一次剔除

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    common::VV4F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<char> point_selected_surf_;           // selected points
    common::VV4F plane_coef_;                         // plane coeffs

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;

    /// options
    bool time_sync_en_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    /// statistics and flags
    int scan_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    bool timediff_set_flg_ = false;
    int effect_feat_num_ = 0, frame_num_ = 0;

    ///////////////////////// vel / gnss auxiliary ///////////////////////////////////////////////////////
    bool use_vel_ = false;                        ///< 是否使用轮速计速度约束
    bool use_gnss_attitude_ = false;              ///< 是否使用 GNSS 姿态校正
    double can_vel_diff_thresh_kmh_ = 2.0;        ///< 轮速差异阈值 (km/h)
    double can_timeout_sec_ = 0.2;                ///< 轮速超时时间 (秒)
    double gnss_attitude_timeout_sec_ = 1.0;      ///< GNSS 姿态超时时间 (秒)
    double attitude_diff_thresh_deg_ = 2.0;       ///< 姿态差异阈值 (度)
    double attitude_correction_gain_ = 0.7;       ///< 姿态校正增益 (0.0-1.0)
    CanVelocityData last_can_;                    ///< 最新轮速数据
    GnssAttitudeData last_gnss_attitude_;         ///< 最新 GNSS 姿态数据
    bool has_can_ = false;                        ///< 是否已有轮速数据
    bool has_gnss_attitude_ = false;              ///< 是否已有 GNSS 姿态数据
    std::uint64_t vel_reset_count_ = 0;           ///< 速度重置计数
    std::uint64_t attitude_reset_count_ = 0;      ///< 姿态重置计数

    ///////////////////////// 观测更新参数 ///////////////////////////////////////////////////////
    double wheel_velocity_noise_ = 0.01;          ///< 轮速观测噪声方差 (m²/s²)
    double non_holonomic_noise_ = 0.01;           ///< 非完整性约束噪声方差 (m²/s²)，横向/垂向速度约束
    double gnss_attitude_noise_rad2_ = 0.03;      ///< GNSS 姿态观测噪声方差 (rad²)，与 lm_slam 对齐
    int min_gnss_status_ = 33;                    ///< 最小 GNSS 状态（RTK 固定解）
    int min_gnss_nsv_ = 8;                        ///< 最小 GNSS 卫星数量
    int min_lidar_points_ = 100;                  ///< 最小点云数量

    ///////////////////////// IMU 重力方向观测参数 ///////////////////////////////////////////////////////
    bool use_imu_gravity_ = true;                 ///< 是否使用IMU重力方向观测约束roll/pitch
    double imu_gravity_noise_ = 0.02;             ///< IMU重力观测噪声方差 (m²/s²)²，对应~0.01rad roll/pitch误差
    double imu_gravity_acc_threshold_ = 0.5;      ///< 低加速检测阈值 (m/s²)，|acc_norm - 9.81| < threshold
    Eigen::Vector3d last_acc_ = Eigen::Vector3d::Zero();  ///< 最新IMU加速度计读数
    bool has_imu_data_ = false;                   ///< 是否已收到IMU数据

    ///////////////////////// 传感器健康状态 ///////////////////////////////////////////////////////
    mutable std::mutex health_mutex_;             ///< 健康状态互斥锁
    SensorHealth sensor_health_;                  ///< 传感器健康状态

    ///////////////////////// DCReg 退化检测 & 鲁棒估计 /////////////////////////////////////////
    bool use_dcreg_degeneracy_ = true;            ///< 是否启用 DCReg 退化检测（含权重和 P 膨胀）
    bool use_huber_weight_ = true;                ///< 是否启用 Huber 鲁棒损失
    bool use_degeneracy_weight_ = true;           ///< 是否启用退化方向降权
    double dcreg_condition_threshold_ = 10.0;     ///< DCReg 退化条件数阈值
    double huber_k_ = 0.5;                        ///< Huber 损失阈值 (m)
    double degeneracy_weight_alpha_ = 0.5;        ///< 退化方向降权强度
    DCRegResult dcreg_result_;                    ///< 最新 DCReg 退化检测结果

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    common::MeasureGroup measures_;                    // sync IMU and lidar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    vect3 pos_lidar_;                                  // lidar position after eskf update
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool extrinsic_est_en_ = true;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_LASER_ODOMETRY_H