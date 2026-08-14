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

namespace faster_lio {

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
