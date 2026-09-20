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
#include <deque>
#include <queue>
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

    /// 设置观测参数（轮速噪声、GNSS 姿态噪声）
    void SetObservationParams(double wheel_velocity_noise,
                              double gnss_attitude_noise);

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

    /// 轮速观测更新（3D 世界系速度伪观测，lm_slam 同构）
    void ApplyWheelSpeedObservation();

    /// GNSS 姿态观测更新（lm_slam 式相对姿态：首帧基准 + 3D 含 yaw）
    void ApplyGnssAttitudeObservation();

    /// IMU 加速度计重力方向观测：利用加速度计测量重力方向，约束 roll 和 pitch
    /// 只有当车辆不加速时（|acc_norm - g| < threshold）才应用
    void ApplyImuGravityObservation();

    /// 检测当前是否处于低加速状态（适合进行重力方向观测）
    bool isLowAcceleration() const;

    /// DCReg 包装观测模型：调用原 ObsModel + DCReg 诊断 + 鲁棒加权
    /// （三期方案：退化方向 H 列非线性软缩放在此后处理中完成，无任何 P 手术）
    void ObsModelWithDCReg(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// DCReg 后处理：对 ekfom_data 做退化诊断 + Huber + 退化方向 H 列非线性软缩放
    void ApplyDCRegAndWeights(
        esekfom::dyn_share_datastruct<double> &ekfom_data);

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

    ///////////////////////// GNSS 相对姿态观测（lm_slam 同构）/////////////////////////////////////
    /// 观测"姿态相对首帧的变化"而非绝对值，安装角/坐标系/绝对误差被基准吸收
    Eigen::Matrix3d R_gnss_first_ = Eigen::Matrix3d::Identity();  ///< 首帧 GNSS 旋转基准
    Eigen::Matrix3d R_est_first_ = Eigen::Matrix3d::Identity();   ///< 首帧 LIO 旋转基准
    bool gnss_baseline_captured_ = false;         ///< 是否已捕获首帧基准
    double last_used_gnss_ts_ = -1.0;             ///< 上次实际使用的 GNSS 消息时间戳
    bool gnss_update_every_frame_ = false;        ///< 每帧复用最新缓存值（lm_slam 行为，默认关）
    int gnss_baseline_wait_frames_ = 50;          ///< 基准捕获前等待帧数（等 LIO 收敛）
    int gnss_min_nsv_ = 6;                        ///< GNSS 最低可见卫星数门控：遮挡/隧道内 nsv→0 时
                                                  ///< 姿态解算漂移（实测相对 pitch 残差达 12°），必须跳过
    // GNSS pitch 滑动窗平均（对齐 lm_slam filter()/window_size=50，只对 pitch 滤波）：
    // 滤掉 GNSS pitch 高频抖动，提供稳定 pitch 参考压 z 漂移；heading/roll 直接相对、不滤波
    std::deque<double> gnss_pitch_window_;
    double gnss_pitch_window_sum_ = 0.0;
    size_t gnss_pitch_window_size_ = 50;
    double last_gnss_raw_pitch_deg_ = 0.0;        ///< 平滑前的 GNSS 原始 pitch（诊断：定位 2° 偏差归属）

    ///////////////////////// 观测更新参数 ///////////////////////////////////////////////////////
    double wheel_velocity_noise_ = 0.1089;        ///< 轮速 3D 伪观测噪声方差 (m²/s²)，σ=0.33 对齐 lm_slam
    double gnss_attitude_noise_ = 0.03;           ///< GNSS 相对姿态噪声方差 (rad²)，σ≈9.9° 对齐 lm_slam 实际生效值

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
    /// 退化分数的帧间平滑（指数衰减）：DCReg 单帧特征比对点云噪声敏感，分数在阈值附近
    /// 剧烈抖动（实测 rot_y: 0.152→0→0），导致膨胀"脉冲式"——只活一帧，下一帧 LiDAR 又把
    /// P 压回 1e-7。平滑让膨胀在退化段持续 3~5 帧，轮速/GNSS 才有持续话语权。
    std::array<double, 6> smooth_degeneracy_scores_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool degeneracy_score_initialized_ = false;   ///< 是否已初始化平滑分数
    double dcreg_score_smooth_alpha_ = 0.7;       ///< 平滑：当前帧权重（0.7*当前 + 0.3*上一帧）
    double dcreg_col_score_gate_ = 0.05;          ///< H 列缩放门控：平滑分数 > gate 的方向列乘 c_min
                                                  ///< （0.05 ⟺ cond>26.3 @T=25；比例映射已被实车+定量双重否决）
    double dcreg_col_min_scale_ = 1e-3;           ///< 退化方向列缩放下限：保留的弱约束比例，保 GNSS 话语权
    bool dcreg_constrain_enabled_ = false;        ///< 退化干预总开关：false=仅检测+日志（lm_slam 一致版），
                                                  ///< true=启用滞回状态机+列缩放+地图冻结（v4 实验路径，保留备用）

    // —— 退化滞回状态机（v4：切断门控闪断→P 摆动→增益振荡正反馈，仅 dcreg_constrain_enabled_=true 生效）——
    // 实车证据（0622）：score 在 gate 两侧逐帧摆动时，P_pitch 在 1e-7↔4e-3 间跳变 4 个量级，
    // k_pitch 1e-5↔0.13 跳变，lio_pitch 被 +6.6°↔−4.9° 振荡拖拽（effect num 曾暴跌至 152）。
    // 任一方向 s>gate 即进入退化态并锁定该方向；进入后强制保持 hold_frames 帧，期间无论
    // 当帧 is_degenerate/score 如何一律缩放锁定列，且 MapIncremental 不插点（状态供 Run() 读取）。
    bool degenerate_active_ = false;              ///< 滞回退化态是否激活
    int degenerate_hold_counter_ = 0;             ///< 剩余强制保持帧数
    int dcreg_hold_frames_ = 10;                  ///< 触发后保持帧数（10Hz 下约 1 秒）
    bool held_degenerate_dirs_[6] = {false};      ///< 本次退化 episode 锁定的方向（scores 顺序）
    double last_sm_frame_time_ = -1.0;            ///< 状态机已处理的帧时间（IEKF 多次迭代去重）
    bool map_freeze_this_frame_ = false;          ///< 本帧冻结地图增量（Run() 门控 MapIncremental）

    // 卡方分布阈值（99.9% 置信度）
    static constexpr double CHI2_THRESHOLD_3DOF = 16.27;

    ///////////////////////// 工具函数：马氏距离异常值检测 ////////////////////////////////////////
    template <int dim>
    std::pair<bool, double> MahalanobisOutlierRejection(
        const Eigen::Matrix<double, dim, 1>& residual,
        const Eigen::Matrix<double, dim, 23>& H,
        const Eigen::Matrix<double, 23, 23>& P,
        const Eigen::Matrix<double, dim, dim>& R,
        double threshold) const {
        Eigen::Matrix<double, dim, dim> S = H * P * H.transpose() + R;
        auto llt = S.llt();
        if (llt.info() != Eigen::Success) {
            return {true, std::numeric_limits<double>::max()};
        }
        double mahalanobis = residual.transpose() * llt.solve(residual);
        return {mahalanobis > threshold, mahalanobis};
    }

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