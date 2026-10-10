/**
 * @file laser_odometry.cc
 * @brief Faster-LIO 纯里程计核心实现
 *
 * 直接移植自 faster_lio_ws/src/faster-lio/src/laser_mapping.cc，
 * 仅移除建图/发布/PCD 保存部分，算法逻辑与原始版本逐行一致，
 * 保证在 -O3 优化编译下与 faster_lio_ws 运行效果一致。
 */

#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>
#include <cmath>
#include <Eigen/Geometry>

#include "laser_odometry.h"
#include "utils.h"
#include "syslog/logger.h"
#include <chrono>

namespace faster_lio {

/// 每行号独立的节流器（替代 ROS_INFO_THROTTLE）
template<int Line>
struct ThrottleByLine {
  static bool shouldLog(double rate) {
    static std::chrono::steady_clock::time_point last_log_time;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_log_time).count();
    if (elapsed >= rate) {
      last_log_time = now;
      return true;
    }
    return false;
  }
};

#define SHOULD_LOG(rate) ThrottleByLine<__LINE__>::shouldLog(rate)


/// 将角度归一化到 [-180, 180] 度范围
static float NormalizePitchRoll(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/// 将欧拉角（航向-俯仰-横滚，Z-Y-X）转换为四元数
static Eigen::Quaterniond AnglesToQuaternion(double heading_deg, double pitch_deg, double roll_deg) {
    const double heading_rad = heading_deg * M_PI / 180.0;
    const double pitch_rad = pitch_deg * M_PI / 180.0;
    const double roll_rad = roll_deg * M_PI / 180.0;
    const Eigen::Quaterniond q_z(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_y(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond q_x(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
    return (q_z * q_y * q_x).normalized();
}

bool LaserOdometry::InitWithoutROS(const std::string &config_yaml) {
    // LOG(INFO) << "init laser odometry from " << config_yaml;
    SYS_LOG(syslog::LogSeverity::INFO, true) << "init laser odometry from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
            if (use_dcreg_degeneracy_) {
                ObsModelWithDCReg(s, ekfom_data);
            } else {
                ObsModel(s, ekfom_data);
            }
        },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        // LOG(INFO) << "using phc ivox";
        SYS_LOG(syslog::LogSeverity::INFO, true) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        // LOG(INFO) << "using default ivox";
        SYS_LOG(syslog::LogSeverity::INFO, true) << "using default ivox";
    }

    return true;
}

bool LaserOdometry::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
        map_trim_radius_ = yaml["mapping"]["map_trim_radius"].as<float>(0.0f);
        map_trim_interval_frames_ = yaml["mapping"]["map_trim_interval_frames"].as<int>(10);

        // 缓冲区治理（Stage 2）：不写 buffer_policy 段时用结构体默认值（=启用）
        if (yaml["buffer_policy"]) {
            auto bp = yaml["buffer_policy"];
            buffer_policy_.enable = bp["enable"].as<bool>(buffer_policy_.enable);
            buffer_policy_.lidar_keep_sec = bp["lidar_keep_sec"].as<double>(buffer_policy_.lidar_keep_sec);
            buffer_policy_.lidar_max_frames = bp["lidar_max_frames"].as<int>(buffer_policy_.lidar_max_frames);
            buffer_policy_.lidar_catchup_frames =
                bp["lidar_catchup_frames"].as<int>(buffer_policy_.lidar_catchup_frames);
            buffer_policy_.imu_keep_margin_sec =
                bp["imu_keep_margin_sec"].as<double>(buffer_policy_.imu_keep_margin_sec);
            buffer_policy_.imu_max_samples = bp["imu_max_samples"].as<int>(buffer_policy_.imu_max_samples);
            buffer_policy_.imu_orphan_keep_sec =
                bp["imu_orphan_keep_sec"].as<double>(buffer_policy_.imu_orphan_keep_sec);
        }

        // 轮速/GNSS 辅助
        use_vel_ = yaml["vel_gnss"]["use_vel"].as<bool>(false);
        use_gnss_attitude_ = yaml["vel_gnss"]["use_gnss_attitude"].as<bool>(false);
        can_vel_diff_thresh_kmh_ = yaml["vel_gnss"]["can_vel_diff_thresh_kmh"].as<double>(2.0);
        can_timeout_sec_ = yaml["vel_gnss"]["can_timeout_sec"].as<double>(0.5);
        gnss_attitude_timeout_sec_ = yaml["vel_gnss"]["gnss_attitude_timeout_sec"].as<double>(1.0);
        attitude_diff_thresh_deg_ = yaml["vel_gnss"]["attitude_diff_thresh_deg"].as<double>(2.0);
        attitude_correction_gain_ = yaml["vel_gnss"]["attitude_correction_gain"].as<double>(0.7);

        // 观测噪声（固定 R，对齐 lm_slam：轮速 σ=0.33 m/s，GNSS 姿态 σ≈1.8°）
        wheel_velocity_noise_ = yaml["vel_gnss"]["wheel_velocity_noise"].as<double>(wheel_velocity_noise_);
        gnss_attitude_noise_ = yaml["vel_gnss"]["gnss_attitude_noise"].as<double>(gnss_attitude_noise_);
        // GNSS 相对姿态观测行为
        gnss_update_every_frame_ = yaml["vel_gnss"]["gnss_update_every_frame"].as<bool>(gnss_update_every_frame_);
        gnss_baseline_wait_frames_ = yaml["vel_gnss"]["gnss_baseline_wait_frames"].as<int>(gnss_baseline_wait_frames_);
        gnss_min_nsv_ = yaml["vel_gnss"]["gnss_min_nsv"].as<int>(gnss_min_nsv_);

        // IMU 重力方向观测参数
        use_imu_gravity_ = yaml["vel_gnss"]["use_imu_gravity"].as<bool>(false);
        imu_gravity_noise_ = yaml["vel_gnss"]["imu_gravity_noise"].as<double>(0.05);
        imu_gravity_acc_threshold_ = yaml["vel_gnss"]["imu_gravity_acc_threshold"].as<double>(0.5);

        // DCReg 退化检测 + 鲁棒估计参数
        if (YAML::Node dcreg = yaml["dcreg"]) {
            use_dcreg_degeneracy_ = dcreg["use_dcreg_degeneracy"].as<bool>(use_dcreg_degeneracy_);
            use_huber_weight_ = dcreg["use_huber_weight"].as<bool>(use_huber_weight_);
            use_degeneracy_weight_ = dcreg["use_degeneracy_weight"].as<bool>(use_degeneracy_weight_);
            dcreg_condition_threshold_ = dcreg["dcreg_condition_threshold"].as<double>(dcreg_condition_threshold_);
            huber_k_ = dcreg["huber_k"].as<double>(huber_k_);
            degeneracy_weight_alpha_ = dcreg["degeneracy_weight_alpha"].as<double>(degeneracy_weight_alpha_);
            dcreg_score_smooth_alpha_ = dcreg["score_smooth_alpha"].as<double>(dcreg_score_smooth_alpha_);
            dcreg_col_score_gate_ = dcreg["col_scale_score_gate"].as<double>(dcreg_col_score_gate_);
            dcreg_col_min_scale_ = dcreg["col_scale_min"].as<double>(dcreg_col_min_scale_);
            dcreg_hold_frames_ = dcreg["hold_frames"].as<int>(dcreg_hold_frames_);
            dcreg_constrain_enabled_ = dcreg["constrain_enabled"].as<bool>(dcreg_constrain_enabled_);
        }

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        // LOG(ERROR) << "bad conversion";
        SYS_LOG(syslog::LogSeverity::ERROR, true) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        // LOG(INFO) << "Using AVIA Lidar (livox_ros_driver::CustomMsg)";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using AVIA Lidar (livox_ros_driver::CustomMsg)";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        // LOG(INFO) << "Using Velodyne 32 Lidar";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        // LOG(INFO) << "Using OUST 64 Lidar";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::HESAIxt32);
        // LOG(INFO) << "Using Hesai Pandar 32 Lidar";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using Hesai Pandar 32 Lidar";
    } else if (lidar_type == 5) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        // LOG(INFO) << "Using Robosense Lidar";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using Robosense Lidar";
    } else if (lidar_type == 6) {
        preprocess_->SetLidarType(LidarType::LIVOX);
        // LOG(INFO) << "Using Livox Lidar (sensor_msgs::PointCloud2)";
        SYS_LOG(syslog::LogSeverity::INFO, false) << "Using Livox Lidar (sensor_msgs::PointCloud2)";
    } else {
        // LOG(WARNING) << "unknown lidar_type";
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        // LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    // 编译时间戳自报：启动第一行即可确认二进制新旧（避免 strings /proc/exe 误判——
    // 诊断字符串在 libslam_odometry_v2_core.so 中，节点 exe 本身查不到）
    // LOG(INFO) << "[Build] laser_odometry compiled at " << __DATE__ << " " << __TIME__
    //           << " | use_vel=" << use_vel_ << " use_gnss_attitude=" << use_gnss_attitude_
    //           << " gnss_every_frame=" << gnss_update_every_frame_
    //           << " wheel_noise=" << wheel_velocity_noise_
    //           << " gnss_noise=" << gnss_attitude_noise_;

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

LaserOdometry::LaserOdometry() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserOdometry::SetVelocityGnssParams(bool use_vel, bool use_gnss_attitude,
                                          double can_vel_diff_thresh_kmh,
                                          double can_timeout_sec,
                                          double gnss_attitude_timeout_sec,
                                          double attitude_diff_thresh_deg,
                                          double attitude_correction_gain) {
    use_vel_ = use_vel;
    use_gnss_attitude_ = use_gnss_attitude;
    can_vel_diff_thresh_kmh_ = can_vel_diff_thresh_kmh;
    can_timeout_sec_ = can_timeout_sec;
    gnss_attitude_timeout_sec_ = gnss_attitude_timeout_sec;
    attitude_diff_thresh_deg_ = attitude_diff_thresh_deg;
    attitude_correction_gain_ = attitude_correction_gain;
}

void LaserOdometry::SetObservationParams(double wheel_velocity_noise,
                                         double gnss_attitude_noise) {
    wheel_velocity_noise_ = wheel_velocity_noise;
    gnss_attitude_noise_ = gnss_attitude_noise;
    // LOG(INFO) << "[Obs] params: wheel_noise=" << wheel_velocity_noise_
    //           << " gnss_attitude_noise=" << gnss_attitude_noise_;
    SYS_LOG(syslog::LogSeverity::INFO, false) << "[Obs] params: wheel_noise=" << wheel_velocity_noise_
                                             << " gnss_attitude_noise=" << gnss_attitude_noise_;    
}
    
SensorHealth LaserOdometry::GetSensorHealth() const {
    std::lock_guard<std::mutex> lock(health_mutex_);
    return sensor_health_;
}

void LaserOdometry::UpdateSensorHealth() {
    std::lock_guard<std::mutex> lock(health_mutex_);
    double current_time = measures_.lidar_bag_time_;

    // 轮速超时检查
    if (sensor_health_.has_can_data) {
        sensor_health_.can_age = current_time - sensor_health_.can_last_update;
        if (sensor_health_.can_age > can_timeout_sec_) {
            if (sensor_health_.can_healthy) {
                // LOG_FIRST_N(WARNING, 1) << "[Health] wheel speed timeout: age="
                //                         << sensor_health_.can_age;
                SYS_LOG(syslog::LogSeverity::WARNING, true) << "[Health] wheel speed timeout: age=" << sensor_health_.can_age;
            }
            sensor_health_.can_healthy = false;
        }
    }

    // GNSS 超时检查
    if (sensor_health_.has_gnss_data) {
        sensor_health_.gnss_age = current_time - sensor_health_.gnss_last_update;
        if (sensor_health_.gnss_age > gnss_attitude_timeout_sec_) {
            if (sensor_health_.gnss_healthy) {
                // LOG_FIRST_N(WARNING, 1) << "[Health] GNSS timeout: age="
                //                         << sensor_health_.gnss_age;
                SYS_LOG(syslog::LogSeverity::WARNING, true) << "[Health] GNSS timeout: age=" << sensor_health_.gnss_age;
            }
            sensor_health_.gnss_healthy = false;
        }
    }

    // LiDAR 健康检查（最小点数固定 100，与原 min_lidar_points 默认值一致）
    sensor_health_.lidar_points = scan_undistort_ ? scan_undistort_->size() : 0;
    sensor_health_.lidar_healthy = static_cast<int>(sensor_health_.lidar_points) >= 100;
}

// ====================================================================
// DCReg 退化检测 + Huber 鲁棒估计
// ====================================================================

void LaserOdometry::ObsModelWithDCReg(
    state_ikfom &s,
    esekfom::dyn_share_datastruct<double> &ekfom_data) {

    // ① 调用原观测模型
    ObsModel(s, ekfom_data);

    // ② 后处理：DCReg 诊断 + 鲁棒加权
    if (ekfom_data.valid) {
        ApplyDCRegAndWeights(ekfom_data);
    }
}

void LaserOdometry::ApplyDCRegAndWeights(
    esekfom::dyn_share_datastruct<double> &ekfom_data) {

    if (effect_feat_num_ < 1) return;

    // 1. 提取 6-DOF Jacobian (N×6) 并计算 Hessian (6×6)
    //    h_x 原列: [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, ...]
    //    DCReg 期望: [rot_x, rot_y, rot_z, pos_x, pos_y, pos_z]
    Eigen::MatrixXd J_6_raw = ekfom_data.h_x.leftCols(6);
    Eigen::MatrixXd J_6_dcreg(J_6_raw.rows(), 6);
    J_6_dcreg.leftCols(3) = J_6_raw.rightCols(3);   // rot
    J_6_dcreg.rightCols(3) = J_6_raw.leftCols(3);   // pos
    Eigen::Matrix<double, 6, 6> H_6 = J_6_dcreg.transpose() * J_6_dcreg;

    // 2. DCReg 退化诊断
    if (use_dcreg_degeneracy_) {
        dcreg_result_ = ComputeDCReg(H_6, dcreg_condition_threshold_);

        // 分数帧间平滑（指数衰减）：抑制单帧抖动，让 H 列置零持续而非脉冲。
        //   当前帧抖动: rot_y 0.152→0→0 导致置零只活一帧、P 下一帧被 LiDAR 压回。
        //   smooth = α*当前 + (1-α)*上一帧，退化段持续 3~5 帧。
        if (!degeneracy_score_initialized_) {
            for (int d = 0; d < 6; ++d) {
                smooth_degeneracy_scores_[d] = dcreg_result_.degeneracy_scores[d];
            }
            degeneracy_score_initialized_ = true;
        } else {
            const double a = dcreg_score_smooth_alpha_;
            for (int d = 0; d < 6; ++d) {
                smooth_degeneracy_scores_[d] =
                    a * dcreg_result_.degeneracy_scores[d] +
                    (1.0 - a) * smooth_degeneracy_scores_[d];
            }
        }
    }

    // 2.5 退化处理两条路径（由 dcreg.constrain_enabled 选择）：
    //     · false（默认，lm_slam 一致版）：DCReg 只检测+打日志，H/P/地图完全不干预——
    //       lm_slam 无任何退化处理即靠 0.5m 降采样的天然低信息量 + 轮速/GNSS 顺序更新工作。
    //     · true（v4 实验路径保留备用）：滞回状态机 + H 列二值缩放 + 退化帧冻结地图。
    //     本函数每 IEKF 迭代调用一次，状态机按 lidar_end_time_ 去重，每帧只走一次。
    if (use_dcreg_degeneracy_) {
        if (dcreg_constrain_enabled_ && last_sm_frame_time_ != lidar_end_time_) {
            last_sm_frame_time_ = lidar_end_time_;
            bool any_over_gate = false;
            for (int d = 0; d < 6; ++d) {
                if (smooth_degeneracy_scores_[d] > dcreg_col_score_gate_) {
                    any_over_gate = true;
                    held_degenerate_dirs_[d] = true;  // 锁定：激活期内该方向不再放行
                }
            }
            if (any_over_gate) {
                if (!degenerate_active_) {
                    LOG(WARNING) << "[DCReg] degeneracy episode START: score=["
                                 << smooth_degeneracy_scores_[0] << ","
                                 << smooth_degeneracy_scores_[1] << ","
                                 << smooth_degeneracy_scores_[2] << ","
                                 << smooth_degeneracy_scores_[3] << ","
                                 << smooth_degeneracy_scores_[4] << ","
                                 << smooth_degeneracy_scores_[5] << "] hold=" << dcreg_hold_frames_;
                }
                degenerate_active_ = true;
                degenerate_hold_counter_ = dcreg_hold_frames_;  // 持续触发则持续装填
            } else if (degenerate_active_) {
                if (--degenerate_hold_counter_ <= 0) {
                    degenerate_active_ = false;
                    for (int d = 0; d < 6; ++d) held_degenerate_dirs_[d] = false;
                    LOG(WARNING) << "[DCReg] degeneracy episode END: map unfrozen";
                }
            }
            map_freeze_this_frame_ = degenerate_active_;
        }

        if (dcreg_constrain_enabled_ && degenerate_active_) {
            static constexpr int kScoreToCol[6] = {3, 4, 5, 0, 1, 2};
            double col_scales[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
            for (int d = 0; d < 6; ++d) {
                if (held_degenerate_dirs_[d]) {
                    col_scales[d] = dcreg_col_min_scale_;
                    ekfom_data.h_x.col(kScoreToCol[d]) *= col_scales[d];
                }
            }
            LOG_EVERY_N(WARNING, 6) << "[DCReg] H cols scaled(hold="
                                    << degenerate_hold_counter_ << "): score=["
                                    << smooth_degeneracy_scores_[0] << ","
                                    << smooth_degeneracy_scores_[1] << ","
                                    << smooth_degeneracy_scores_[2] << ","
                                    << smooth_degeneracy_scores_[3] << ","
                                    << smooth_degeneracy_scores_[4] << ","
                                    << smooth_degeneracy_scores_[5] << "]"
                                    << " col_scale=["
                                    << col_scales[0] << ","
                                    << col_scales[1] << ","
                                    << col_scales[2] << ","
                                    << col_scales[3] << ","
                                    << col_scales[4] << ","
                                    << col_scales[5] << "]"
                                    << " cond_rot=" << dcreg_result_.cond_schur_rot
                                    << " cond_trans=" << dcreg_result_.cond_schur_trans;
        } else if (!dcreg_constrain_enabled_ && dcreg_result_.is_degenerate) {
            // 仅检测模式：DCReg 指标照常输出，供与 lm_slam 基线对比观察，不触碰滤波器
            LOG_EVERY_N(WARNING, 6) << "[DCReg] degenerate detected (monitor only, no constrain): score=["
                                    << smooth_degeneracy_scores_[0] << ","
                                    << smooth_degeneracy_scores_[1] << ","
                                    << smooth_degeneracy_scores_[2] << ","
                                    << smooth_degeneracy_scores_[3] << ","
                                    << smooth_degeneracy_scores_[4] << ","
                                    << smooth_degeneracy_scores_[5] << "]"
                                    << " cond_rot=" << dcreg_result_.cond_schur_rot
                                    << " cond_trans=" << dcreg_result_.cond_schur_trans;
        }
    }

    // 3. 逐点计算权重并应用到 h_x 和 h
    for (int i = 0; i < effect_feat_num_; i++) {
        double w = 1.0;

        // Huber 鲁棒权重
        if (use_huber_weight_) {
            w *= HuberWeight(ekfom_data.h(i), huber_k_);
        }

        // 退化方向降权
        if (use_degeneracy_weight_ && dcreg_result_.is_degenerate) {
            Eigen::Matrix<double, 1, 6> J_row = J_6_dcreg.row(i);
            w *= DegeneracyWeight(J_row, dcreg_result_.degeneracy_scores,
                                  degeneracy_weight_alpha_);
        }

        // 应用权重 (IRLS: w → √w)
        // 框架内部组装 HTH = h_x^T * h_x，乘以 √w 等价于对每行加权重 w
        double sqrt_w = std::sqrt(w);
        ekfom_data.h_x.row(i) *= sqrt_w;
        ekfom_data.h(i) *= sqrt_w;
    }
}

void LaserOdometry::ApplyWheelSpeedObservation() {
    // lm_slam 同构：单一 3D 世界系速度伪观测（前向轮速 + 隐式 NHC 统一噪声 σ），
    // 不拆分 NHC、不做自适应 R；退化时由 H 列置零让 P 自然增长、轮速话语权自发出现
    state_ikfom state = kf_.get_x();

    // CAN 新鲜度检查：断流后陈旧速度持续注入会拖住车辆加减速
    const double can_age = std::abs(measures_.lidar_bag_time_ - last_can_.timestamp);
    if (can_age > can_timeout_sec_) {
        LOG_EVERY_N(WARNING, 50) << "[Obs] wheel speed stale: age=" << can_age
                                 << " > " << can_timeout_sec_ << "s, skip";
        return;
    }

    Eigen::Matrix3d R_wl = state.rot.toRotationMatrix();
    double v_x = last_can_.speed_kmh / 3.6;  // km/h -> m/s
    Eigen::Vector3d can_body_vel(v_x, 0.0, 0.0);

    // 伪观测: h(x) = R * [v_x, 0, 0]^T - state.vel = 0
    // residual = 0 - h(x) = state.vel - R * [v_x, 0, 0]^T
    Eigen::Vector3d residual = state.vel - R_wl * can_body_vel;

    // H = ∂h/∂x（框架约定 residual = z - h(x)，H 为观测模型雅可比）
    // 对齐 lm_slam：轮速只观测速度（∂h/∂vel = -I），旋转完全交给 GNSS 姿态观测。
    // 刻意去掉 rot 耦合项 -R·hat(v_body)：有该项时轮速残差同时受 pitch 误差与速度误差
    // 影响，EKF 无法区分该修哪个，与 GNSS 在 pitch 通道"打架"（lm_slam 无此项，职责分离）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);
    H.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();          // ∂h/∂vel = -I

    // 固定噪声（σ=0.33 m/s，对齐 lm_slam）：轮速只兜底不主导，
    // 标定误差/颠簸/轻微打滑由大噪声吸收
    Eigen::Matrix3d R_obs = wheel_velocity_noise_ * Eigen::Matrix3d::Identity();

    // 马氏距离异常值检测（CAN 野值保险，无退化放宽）
    auto P = kf_.get_P();
    auto [is_outlier, md] = MahalanobisOutlierRejection<3>(
        residual, H, P, R_obs, CHI2_THRESHOLD_3DOF);
    if (is_outlier) {
        LOG(WARNING) << "[Obs] wheel speed Mahalanobis outlier:"
                     << " md=" << md << " threshold=" << CHI2_THRESHOLD_3DOF
                     << " residual_norm=" << residual.norm()
                     << " speed_ms=" << v_x;
        return;
    }

    // 单次更新（标准 EKF：退化方向话语权由 H 列置零保证，无任何 P 手术）
    kf_.update_single_observation(H, residual, R_obs);

    LOG_EVERY_N(INFO, 1) << "[Obs] wheel speed update: residual_norm=" << residual.norm()
                          << " speed_ms=" << v_x;
}

void LaserOdometry::ApplyGnssAttitudeObservation() {
    // lm_slam 同构：相对姿态观测。
    // 不直接用 GNSS 绝对姿态（含安装角/坐标系/绝对误差），
    // 而是约束"姿态相对首帧基准的变化"与 GNSS 一致：R_target = R_gnss·R_gnss_f⁻¹·R_est_f
    if (!has_gnss_attitude_) {
        return;
    }

    // 质量门控：无卫星（nsv=0，遮挡/隧道内）时接收机姿态解算漂移，
    // 实测相对 pitch 残差达 12°；无卫星时 heading/pitch/roll 物理上不可能有效，
    // 必须跳过（含基准捕获阶段——不能用垃圾数据固化首帧基准）
    if (last_gnss_attitude_.nsv < gnss_min_nsv_) {
        LOG_EVERY_N(WARNING, 50) << "[Obs] GNSS quality gate: nsv="
                                 << last_gnss_attitude_.nsv << " < " << gnss_min_nsv_
                                 << ", skip attitude update";
        return;
    }

    // 新消息才更新：GNSS 1-5Hz vs LiDAR 帧 10Hz，每帧复用同一消息会
    // 虚假压缩协方差（等效放大权重）。可经 gnss_update_every_frame 切回 lm_slam 行为
    if (!gnss_update_every_frame_ &&
        last_gnss_attitude_.timestamp == last_used_gnss_ts_) {
        return;
    }

    state_ikfom state = kf_.get_x();

    // 1. GNSS 欧拉角（度）→ 旋转矩阵（ZYX：R = Rz(yaw)·Ry(pitch)·Rx(roll)）
    //    ⚠️ 华测 Gpchc 为 NED 约定（北为 0、顺时针为正、z 轴向下），LIO 为 ENU/ZYX。
    //    正确换算：yaw_enu = 90°-heading、pitch_enu = -pitch、roll 不变
    //    （dr_odometry 的 (90-heading) 与 lm_slam 的 -heading/-pitch 等价，
    //     90° 常数偏移由首帧基准吸收）。漏掉取负时相对 yaw 符号反向，
    //     残差 ≈ 2×转弯角，随转弯不断增大——即 "residual too large" 的根因。
    const double heading_rad = -last_gnss_attitude_.heading_deg * M_PI / 180.0;
    const double pitch_rad = -last_gnss_attitude_.pitch_deg * M_PI / 180.0;
    const double roll_rad = last_gnss_attitude_.roll_deg * M_PI / 180.0;
    const double cy = std::cos(heading_rad), sy = std::sin(heading_rad);
    const double cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);
    const double cr = std::cos(roll_rad), sr = std::sin(roll_rad);
    Eigen::Matrix3d R_gnss;
    R_gnss << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
              sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
              -sp,     cp * sr,                cp * cr;

    const Eigen::Matrix3d R_est = state.rot.toRotationMatrix();

    // 2. 首帧基准捕获（延迟 gnss_baseline_wait_frames_ 帧，等 LIO 收敛后再固化基准，
    //    避免未收敛姿态污染基准导致 GNSS 观测持续带偏）
    if (!gnss_baseline_captured_) {
        if (frame_num_ < gnss_baseline_wait_frames_) {
            return;
        }
        R_gnss_first_ = R_gnss;
        R_est_first_ = R_est;
        gnss_baseline_captured_ = true;
        last_used_gnss_ts_ = last_gnss_attitude_.timestamp;
        LOG(INFO) << "[Obs] GNSS baseline captured at frame " << frame_num_
                  << ": heading=" << last_gnss_attitude_.heading_deg
                  << " pitch=" << last_gnss_attitude_.pitch_deg
                  << " roll=" << last_gnss_attitude_.roll_deg;
        return;
    }

    // 3. 目标姿态：GNSS 相对变化作用在 LIO 初始姿态上（3D 全维，含 yaw）
    const Eigen::Matrix3d R_target = R_gnss * R_gnss_first_.transpose() * R_est_first_;

    // 4. 残差 = log(R_est^T · R_target)（SO3 切空间，3 维）
    const Eigen::Matrix3d R_diff = R_est.transpose() * R_target;
    Eigen::AngleAxisd aa(R_diff);
    Eigen::Vector3d residual = aa.angle() * aa.axis();

    // 5. 残差合理性上限（1.0 rad ≈ 57°）：仅拦截 GNSS 野值/姿态翻转类彻底错误。
    //    不设紧门限（如 0.3）：LIO 真实漂移（隧道段）随时间可超过任何紧门限，
    //    紧门限会形成"漂移→拒绝更新→继续漂移"的自锁死循环。
    //    lm_slam 无任何门控，大残差由 EKF 增益 K = P/(P+R) 自然软化，逐帧渐进拉回。
    //    诊断：对比 GNSS / LIO 各自相对首帧基准的 yaw 变化量——
    //    若 gnss_rel_yaw ≈ -lio_rel_yaw 说明符号约定错误；
    //    若差值为常数说明基准/安装角问题；若缓慢发散说明 LIO 真实漂移（GNSS 正在拉回）。
    auto wrapPi = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    const double yaw_now = std::atan2(R_est(1, 0), R_est(0, 0));
    const double yaw_first = std::atan2(R_est_first_(1, 0), R_est_first_(0, 0));
    const double gnss_yaw_now = std::atan2(R_gnss(1, 0), R_gnss(0, 0));
    const double gnss_yaw_first = std::atan2(R_gnss_first_(1, 0), R_gnss_first_(0, 0));
    const double lio_rel_yaw_deg = wrapPi(yaw_now - yaw_first) * 180.0 / M_PI;
    const double gnss_rel_yaw_deg = wrapPi(gnss_yaw_now - gnss_yaw_first) * 180.0 / M_PI;

    if (residual.norm() > 1.0) {
        LOG(WARNING) << "[Obs] GNSS attitude residual insane: " << residual.norm()
                     << " rad, vec=[" << residual.transpose() << "]"
                     << " gnss_rel_yaw=" << gnss_rel_yaw_deg << "deg"
                     << " lio_rel_yaw=" << lio_rel_yaw_deg << "deg"
                     << " heading=" << last_gnss_attitude_.heading_deg << "deg, skip";
        return;
    }
    {
        // 增益诊断：读取实际参与 K 计算的全局 P（无任何瞬态膨胀，P_ 即真实）。
        // 逐轴打印 k_rot = p/(p+r)（顺序 roll/pitch/yaw），可直接看出哪个旋转轴在被 GNSS 拉
        const Eigen::Vector3d p_rot = kf_.get_P().diagonal().template segment<3>(3);
        const Eigen::Vector3d k_rot =
            p_rot.array() / (p_rot.array() + gnss_attitude_noise_);
        // pitch 三值对比（定位持续 ~2° 偏差归属：LIO 漂移 or GNSS 解算/平滑问题）：
        // lio_pitch 用与 ApplyVelocityGnssAid 相同的状态提取口径；
        // GNSS pitch 为 NED 值，参与观测时取负（NED→ENU，见 apply_gnss 处 -pitch_rad），
        // 故同步打印 gnss_enu=-smooth 便于直接与 lio_pitch 比大小。
        const state_ikfom s_diag = kf_.get_x();
        const Eigen::Quaterniond q_diag(s_diag.rot.coeffs()[3], s_diag.rot.coeffs()[0],
                                        s_diag.rot.coeffs()[1], s_diag.rot.coeffs()[2]);
        const Eigen::Matrix3d R_diag = q_diag.normalized().toRotationMatrix();
        const double lio_pitch_deg = std::asin(
            std::max(-1.0, std::min(1.0, static_cast<double>(R_diag(0, 2))))) * 180.0 / M_PI;
        LOG_EVERY_N(INFO, 1) << "[Obs][GNSSdiag] gnss_rel_yaw=" << gnss_rel_yaw_deg
                             << "deg lio_rel_yaw=" << lio_rel_yaw_deg
                             << "deg residual=[" << residual.transpose() << "]"
                             << " norm=" << residual.norm()
                             << " P_rot=[" << p_rot.transpose() << "]"
                             << " k_rot(r/p/y)=[" << k_rot.transpose() << "]"
                             << " lio_pitch=" << lio_pitch_deg
                             << "deg gnss_enu=" << -last_gnss_attitude_.pitch_deg
                             << "deg gnss_raw=" << last_gnss_raw_pitch_deg_ << "deg";
    }

    // 6. H 矩阵（3×23，旋转在索引 3-5；小角度近似下 ∂h/∂δθ ≈ I）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // 7. 固定噪声（默认 0.03，σ≈9.9°，对齐 lm_slam ObsModelGPS 内部实际生效值）
    Eigen::Matrix3d R_obs = gnss_attitude_noise_ * Eigen::Matrix3d::Identity();

    // 8. 无马氏门控（lm_slam 同构）：门控会在 LIO 漂移时拒收修正造成自锁，
    //    GNSS 单帧野值由 1.0 rad 上限拦截，小野值由 EKF 增益自然软化

    // 9. 单次更新
    kf_.update_single_observation(H, residual, R_obs);

    last_used_gnss_ts_ = last_gnss_attitude_.timestamp;

    LOG_EVERY_N(INFO, 1) << "[Obs] GNSS attitude update: residual_norm=" << residual.norm()
                          << " heading=" << last_gnss_attitude_.heading_deg
                          << " pitch=" << last_gnss_attitude_.pitch_deg
                          << " roll=" << last_gnss_attitude_.roll_deg
                          << " nsv=" << last_gnss_attitude_.nsv;
}

bool LaserOdometry::isLowAcceleration() const {
    if (!has_imu_data_) return false;

    // 方法 1: 加速度模值检测（传统方法）
    double acc_norm = last_acc_.norm();
    double diff = std::abs(acc_norm - common::G_m_s2);
    if (diff > imu_gravity_acc_threshold_) return false;

    return true;
}

void LaserOdometry::ApplyImuGravityObservation() {
    static int call_count = 0;
    call_count++;
    if (call_count < 100) {
        LOG(INFO) << "[Obs] IMU gravity called: use_imu_gravity=" << use_imu_gravity_
                  << " has_imu_data=" << has_imu_data_
                  << " acc_norm=" << (has_imu_data_ ? last_acc_.norm() : -1);
    }
    if (!use_imu_gravity_ || !has_imu_data_) {
        return;
    }

    // 低加速检测：只有当车辆不加速时才应用重力方向观测
    // 因为加速度计测量值 = 重力 + 加速度，加速时会污染重力方向观测
    double acc_norm = last_acc_.norm();
    double diff_from_g = std::abs(acc_norm - common::G_m_s2);
    if (!isLowAcceleration()) {
        // 每100帧打印一次调试信息，了解加速度计状态
        static int skip_count = 0;
        skip_count++;
        if (skip_count % 100 == 0 && skip_count < 1000) {
            LOG(INFO) << "[Obs] IMU gravity SKIP: acc_norm=" << acc_norm
                      << " diff_from_g=" << diff_from_g
                      << " threshold=" << imu_gravity_acc_threshold_;
        }
        return;
    }

    // 获取当前状态
    state_ikfom state = kf_.get_x();

    // 提取重力向量（3D），grav 是 S2 流形，内部存储为 3D 向量
    Eigen::Vector3d grav_vec;
    for (int i = 0; i < 3; i++) grav_vec(i) = state.grav[i];

    // 提取旋转矩阵 R_wb (body -> world)
    Eigen::Matrix3d R = state.rot.toRotationMatrix();

    // 提取加速度计偏置
    Eigen::Vector3d ba;
    for (int i = 0; i < 3; i++) ba(i) = state.ba[i];

    // ===== 残差: r = (z_acc - ba) + R^T * grav =====
    // 加速度计测量模型: z_acc = -R^T * g + ba + noise （a_true ≈ 0 时）
    // 其中 g 是重力向量在世界坐标系下（指向地心，[0,0,-9.81]）
    // 当车辆不加速时: z_acc - ba ≈ -R^T * g
    // 因此残差: r = (z_acc - ba) + R^T * grav = 0 （当状态正确时）
    // 残差反映重力方向与实测方向的差异 → 约束 roll 和 pitch
    Eigen::Vector3d Rt_grav = R.transpose() * grav_vec;
    Eigen::Vector3d residual = (last_acc_ - ba) + Rt_grav;

    // 残差门限检查：如果残差太大（> 10 m/s²），可能是车辆在加速
    if (residual.norm() > 2.0) {
        LOG_FIRST_N(WARNING, 5) << "[Obs] IMU gravity residual too large: "
                                << residual.norm() << " m/s², skip update (vehicle may be accelerating)";
        return;
    }

    // ===== 打印更新前的 P 矩阵 =====
    {
        auto P = kf_.get_P();
        LOG(INFO) << "[Obs] IMU grav pre-update P diag: pos="
                  << P.diagonal().segment<3>(0).transpose()
                  << " rot=" << P.diagonal().segment<3>(3).transpose()
                  << " vel=" << P.diagonal().segment<3>(12).transpose()
                  << " bg=" << P.diagonal().segment<3>(15).transpose()
                  << " ba=" << P.diagonal().segment<3>(18).transpose()
                  << " grav=" << P.diagonal().segment<2>(21).transpose();
    }

    // ===== 构建 H 矩阵 (3×23) =====
    // 重要：只更新 rot (3-5) 和 ba (18-20)，不更新 grav (21-22)
    // grav 是世界坐标系下的重力向量，在初始化时已经确定，不应频繁更新
    // 更新 grav 会导致 Z 轴位置漂移（因为 grav 用于 IMU 传播中的速度导数）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);

    // ∂h/∂rot (3-5): -hat(R^T * grav)
    // 框架约定: residual = z - h(x)，H = ∂h/∂x（不是 ∂residual/∂x，两者差一个负号！）
    // 观测模型: h(x) = -R^T * grav + ba（预测的加速度计读数）
    // 推导: R_new = R * Exp(δθ) → R_new^T = Exp(-δθ) * R^T ≈ (I - hat(δθ)) * R^T
    //       h(δθ) = -(I - hat(δθ)) * R^T * grav + ba
    //              = -R^T * grav + hat(δθ) * R^T * grav + ba
    //              = h(0) - hat(R^T * grav) * δθ     （hat(δθ)*v = -hat(v)*δθ）
    //       ∴ ∂h/∂δθ = -hat(R^T * grav)
    // 该雅可比使 roll/pitch 可观测（第三列≈0，yaw 不可观测）
    H.block<3, 3>(0, 3) = -SKEW_SYM_MATRIX(Rt_grav);

    // ∂h/∂ba (18-20): +I
    // h(x) = -R^T * grav + ba 对 ba 的偏导数为 +I
    H.block<3, 3>(0, 18) = Eigen::Matrix3d::Identity();

    // ⚠️ 不更新 grav (21-22) 状态！
    // 更新 grav 会导致 Z 轴位置通过 IMU 传播产生漂移
    // 因为 vel_dot = R * (in.acc - ba) + grav

    // ===== 噪声矩阵 (3×3)：固定 R =====
    Eigen::Matrix3d R_mat = imu_gravity_noise_ * Eigen::Matrix3d::Identity();

    // ===== 马氏距离异常值检测 =====
    {
        auto P = kf_.get_P();
        auto [is_outlier, md] = MahalanobisOutlierRejection<3>(
            residual, H, P, R_mat, CHI2_THRESHOLD_3DOF);
        if (is_outlier) {
            LOG_FIRST_N(WARNING, 10) << "[Obs] IMU gravity Mahalanobis outlier:"
                                     << " md=" << md << " threshold=" << CHI2_THRESHOLD_3DOF
                                     << " residual_norm=" << residual.norm();
            return;
        }
    }

    // ===== 单次更新 =====
    kf_.update_single_observation(H, residual, R_mat);

    LOG_EVERY_N(INFO, 50) << "[Obs] IMU gravity update: residual_norm=" << residual.norm()
                          << " m/s², acc_norm=" << last_acc_.norm()
                          << " diff_from_g=" << std::abs(last_acc_.norm() - common::G_m_s2);
}

bool LaserOdometry::ApplyVelocityGnssAid() {
    if (!use_vel_ || !has_can_) {
        return false;
    }

    // 获取当前状态
    state_ikfom state = kf_.get_x();
    const Eigen::Quaterniond q(state.rot.coeffs()[3], state.rot.coeffs()[0],
                               state.rot.coeffs()[1], state.rot.coeffs()[2]);
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    // 提取当前姿态角
    const float pitch_rad = static_cast<float>(std::asin(R(0, 2)));
    const float roll_rad = static_cast<float>(std::atan2(-R(1, 2), R(2, 2)));
    const float heading_rad = static_cast<float>(std::atan2(R(1, 0), R(0, 0)));
    const float pitch_deg = NormalizePitchRoll(pitch_rad * 180.0f / static_cast<float>(M_PI));
    const float roll_deg = NormalizePitchRoll(roll_rad * 180.0f / static_cast<float>(M_PI));
    const float final_heading = heading_rad * 180.0f / static_cast<float>(M_PI);

    bool changed = false;

    // GNSS 姿态校正：俯仰/横滚与 GNSS 差异超过阈值时按增益比例修正
    if (use_gnss_attitude_ && has_gnss_attitude_) {
        const double gnss_age_sec =
            std::abs(measures_.lidar_bag_time_ - last_gnss_attitude_.timestamp);
        if (std::abs(pitch_deg + last_gnss_attitude_.pitch_deg) > attitude_diff_thresh_deg_ ||
            std::abs(roll_deg - last_gnss_attitude_.roll_deg) > attitude_diff_thresh_deg_) {
            const double modified_pitch_deg =
                pitch_deg + attitude_correction_gain_ * (-last_gnss_attitude_.pitch_deg - pitch_deg);
            const double modified_roll_deg =
                roll_deg + attitude_correction_gain_ * (last_gnss_attitude_.roll_deg - roll_deg);
            const Eigen::Quaterniond q_modified =
                AnglesToQuaternion(final_heading, modified_pitch_deg, modified_roll_deg);
            state.rot = SO3(q_modified.w(), q_modified.x(), q_modified.y(), q_modified.z());
            kf_.change_x(state);
            ++attitude_reset_count_;
            changed = true;
            // LOG(INFO) << "[VelAid] GNSS attitude correction: pitch=" << pitch_deg
            //           << " -> " << modified_pitch_deg << " roll=" << roll_deg
            //           << " -> " << modified_roll_deg << " gnss_age=" << gnss_age_sec
            //           << " reset_count=" << attitude_reset_count_;
            SYS_LOG(syslog::LogSeverity::INFO, true) << "[VelAid] GNSS attitude correction: pitch=" << pitch_deg
                << " -> " << modified_pitch_deg << " roll=" << roll_deg
                << " -> " << modified_roll_deg << " gnss_age=" << gnss_age_sec
                << " reset_count=" << attitude_reset_count_;
        }
    }

    // 轮速计速度校正：LiDAR 速度与轮速差异超过阈值时直接重置速度
    const double lidar_speed_kmh =
        std::sqrt(state.vel(0) * state.vel(0) + state.vel(1) * state.vel(1)) * 3.6;
    const double can_speed_correct = last_can_.speed_kmh;
    if (std::abs(lidar_speed_kmh - can_speed_correct) > can_vel_diff_thresh_kmh_) {
        // 将轮速（车体系 x 向）转换到世界坐标系
        const Eigen::Matrix3d rot_world_body = state.rot.toRotationMatrix();
        const Eigen::Vector3d can_body_vel_kmh(can_speed_correct, 0.0001, 0.0001);
        const Eigen::Vector3d can_world_vel_kmh = rot_world_body * can_body_vel_kmh;
        state.vel(0) = can_world_vel_kmh(0) / 3.6;
        state.vel(1) = can_world_vel_kmh(1) / 3.6;
        state.vel(2) = can_world_vel_kmh(2) / 3.6;
        kf_.change_x(state);
        ++vel_reset_count_;
        changed = true;
        // LOG(INFO) << "[VelAid] CAN velocity correction: LiDAR=" << lidar_speed_kmh
        //           << " km/h, CAN=" << can_speed_correct << " km/h"
        //           << " reset_count=" << vel_reset_count_
        //           << " vel=(" << state.vel(0) << ", " << state.vel(1) << ", " << state.vel(2) << ")";
        SYS_LOG(syslog::LogSeverity::INFO, true) << "[VelAid] CAN velocity correction: LiDAR=" << lidar_speed_kmh
            << " km/h, CAN=" << can_speed_correct << " km/h"
            << " reset_count=" << vel_reset_count_
            << " vel=(" << state.vel(0) << ", " << state.vel(1) << ", " << state.vel(2) << ")";
    }
    return changed;
}

bool LaserOdometry::Run() {
    if (!SyncPackages()) {
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        // LOG(WARNING) << "No point, skip this scan!";
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "No point, skip this scan!";
        return false;
    }

    /// 轮速/GNSS 辅助校正
    ApplyVelocityGnssAid();

    /// the first scan
    if (flg_first_scan_) {
        state_point_ = kf_.get_x();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(&scan_undistort_->points[i], &scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return true;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        // LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);

            // lm_slam 一致版：DCReg 仅检测不干预（constrain_enabled=false），LiDAR 标准更新，
            // 轮速/GNSS 串行独立更新，P 全程自然演化，无任何 H/P/地图手术

            /// 更新传感器健康状态（检查超时）
            // UpdateSensorHealth();

            // ===== 外部观测（lm_slam 同构：轮速 → GNSS，串行独立更新）=====
            /// 轮速观测更新（3D 世界系速度伪观测）
            // if (use_vel_ && sensor_health_.can_healthy) {
            //     ApplyWheelSpeedObservation();
            // }

            // GNSS 姿态观测更新（相对姿态：首帧基准 + 3D 含 yaw）
            // if (use_gnss_attitude_) {
            //     ApplyGnssAttitudeObservation();
            // }

            // IMU 加速度计重力方向观测（兜底开关，默认关闭，约束 roll 和 pitch）
            // if (use_imu_gravity_) {
            //     ApplyImuGravityObservation();
            // }

            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    // 退化 episode 期间冻结增量：错配帧（实测 effect num 可暴跌至 152）的点不插入 iVox，
    // 阻止"错配状态→地图污染→后续全量更新焊死误差"正反馈。冻结不做剔除，地图保持冻结前规模。
    if (dcreg_constrain_enabled_ && map_freeze_this_frame_) {
        // LOG_EVERY_N(WARNING, 4) << "[DCReg] map incremental FROZEN (hold="
        //                         << degenerate_hold_counter_ << "), effect num=" << effect_feat_num_;
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "[DCReg] map incremental FROZEN (hold="
            << degenerate_hold_counter_ << "), effect num=" << effect_feat_num_;
    } else {
        Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");
    }
    // 局部地图范围剔除：控制 iVox 规模，避免地图无限增长导致每帧处理/发布变慢
    if (map_trim_radius_ > 0.0f && (map_trim_interval_frames_ <= 1 || frame_num_ % map_trim_interval_frames_ == 0)) {
        PointType center;
        center.x = static_cast<float>(pos_lidar_.x());
        center.y = static_cast<float>(pos_lidar_.y());
        center.z = static_cast<float>(pos_lidar_.z());
        ivox_->RemoveFarPoints(center, map_trim_radius_);
    }

    // LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
    //           << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    if (SHOULD_LOG(2.0)) {
        SYS_LOG(syslog::LogSeverity::INFO, true) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
        << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;
    }
    



    // Debug variables
    frame_num_++;
    return true;
}

bool LaserOdometry::GetOdometry(nav_msgs::Odometry &out) const {
    if (!flg_EKF_inited_) {
        return false;
    }
    out.pose.pose.position.x = state_point_.pos(0);
    out.pose.pose.position.y = state_point_.pos(1);
    out.pose.pose.position.z = state_point_.pos(2);
    out.pose.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.pose.orientation.w = state_point_.rot.coeffs()[3];
    out.header.stamp = ros::Time(measures_.lidar_bag_time_);
    out.header.frame_id = "camera_init";
    out.child_frame_id = "base_link";
    return true;
}

bool LaserOdometry::GetScanBody(PointCloudType::Ptr &out) const {
    if (!scan_down_body_ || scan_down_body_->empty()) {
        return false;
    }
    out = scan_down_body_;
    return true;
}

bool LaserOdometry::GetScanInImu(PointCloudType::Ptr &out) const {
    if (!scan_down_body_ || scan_down_body_->empty()) {
        return false;
    }
    // scan_down_body_ 在 lidar 系；用 lidar->IMU 外参（config/slam_odometry_v2.yaml mapping/extrinsic_T,R）转到
    // IMU/body 系，使外部节点按 identity base_to_imu/base_to_lidar 输出时坐标系自洽。
    const common::V3D T = common::VecFromArray<double>(extrinT_);
    const common::M3D R = common::MatFromArray<double>(extrinR_);
    out.reset(new PointCloudType());
    out->resize(scan_down_body_->size());
    for (size_t i = 0; i < scan_down_body_->size(); ++i) {
        const auto &p = scan_down_body_->points[i];
        const common::V3D p_imu = R * common::V3D(p.x, p.y, p.z) + T;
        PointType &o = out->points[i];
        o.x = static_cast<float>(p_imu(0));
        o.y = static_cast<float>(p_imu(1));
        o.z = static_cast<float>(p_imu(2));
        o.intensity = p.intensity;
        o.normal_x = 0;
        o.normal_y = 0;
        o.normal_z = 0;
        o.curvature = p.curvature;
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = scan_down_body_->is_dense;
    return true;
}

bool LaserOdometry::GetEffectInImu(PointCloudType::Ptr &out) const {
    if (corr_pts_.empty()) {
        return false;
    }
    // corr_pts_ 是 scan_down_body_（lidar 系）中通过有效平面对应筛选的点；
    // 与 GetScanInImu 一致，用 lidar->IMU 外参转到 IMU/body 系。
    const common::V3D T = common::VecFromArray<double>(extrinT_);
    const common::M3D R = common::MatFromArray<double>(extrinR_);
    out.reset(new PointCloudType());
    out->resize(corr_pts_.size());
    for (size_t i = 0; i < corr_pts_.size(); ++i) {
        const common::V3D p_imu = R * corr_pts_[i].head<3>().cast<double>() + T;
        PointType &o = out->points[i];
        o.x = static_cast<float>(p_imu(0));
        o.y = static_cast<float>(p_imu(1));
        o.z = static_cast<float>(p_imu(2));
        o.intensity = corr_pts_[i][3];  // 点到平面对应的残差（米）
        o.normal_x = 0;
        o.normal_y = 0;
        o.normal_z = 0;
        o.curvature = 0;
    }
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = true;
    return true;
}

bool LaserOdometry::GetScanWorld(PointCloudType::Ptr &out) const {
    if (!scan_down_world_ || scan_down_world_->empty()) {
        return false;
    }
    out = scan_down_world_;
    return true;
}

bool LaserOdometry::GetLocalMap(PointCloudType::Ptr &out) const {
    if (!ivox_ || ivox_->NumPoints() == 0) {
        return false;
    }
    PointVector pts;
    ivox_->GetAllPoints(pts);
    if (pts.empty()) {
        return false;
    }
    out.reset(new PointCloudType());
    out->points.resize(pts.size());
    std::copy(pts.begin(), pts.end(), out->points.begin());
    out->width = static_cast<uint32_t>(pts.size());
    out->height = 1;
    return true;
}

void LaserOdometry::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                // LOG(ERROR) << "lidar loop back, clear buffer";
                SYS_LOG(syslog::LogSeverity::ERROR, true) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void LaserOdometry::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        // LOG(WARNING) << "imu loop back, clear buffer";
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);

    // 保存最新的加速度计读数（用于重力方向观测）
    last_acc_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
    has_imu_data_ = true;

    mtx_buffer_.unlock();
}

void LaserOdometry::FeedCanVelocity(const CanVelocityData &can) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    last_can_ = can;
    has_can_ = true;

    // 立即更新健康状态
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        sensor_health_.has_can_data = true;
        sensor_health_.can_last_update = can.timestamp;
        sensor_health_.can_healthy = true;  // 收到数据即认为健康
    }
}

void LaserOdometry::FeedGnssAttitude(const GnssAttitudeData &gnss) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    last_gnss_attitude_ = gnss;
    has_gnss_attitude_ = true;

    // pitch 滑动窗平均（对齐 lm_slam filter()/window_size=50，只对 pitch 滤波）：
    // 在 GNSS 消息入口处滤波（而非帧循环——帧循环每帧复用同一消息会重复入窗失真），
    // heading/roll 保持原始值直接参与相对姿态
    gnss_pitch_window_.push_back(gnss.pitch_deg);
    gnss_pitch_window_sum_ += gnss.pitch_deg;
    if (gnss_pitch_window_.size() > gnss_pitch_window_size_) {
        gnss_pitch_window_sum_ -= gnss_pitch_window_.front();
        gnss_pitch_window_.pop_front();
    }
    last_gnss_raw_pitch_deg_ = gnss.pitch_deg;  // 诊断：保留平滑前原始值，定位 pitch 偏差归属
    last_gnss_attitude_.pitch_deg = gnss_pitch_window_sum_ / gnss_pitch_window_.size();

    // 立即更新健康状态
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        sensor_health_.has_gnss_data = true;
        sensor_health_.gnss_last_update = gnss.timestamp;
        sensor_health_.gnss_status = gnss.status;
        sensor_health_.gnss_nsv = gnss.nsv;
        // 根据 status 判断健康状态（33 为 RTK 固定解）
        sensor_health_.gnss_healthy = (gnss.status >= 33);
    }
}

/// 缓冲区治理（Stage 2）：时间裁剪 + 条数兜底 + drop-to-latest 追帧。
/// 必须在持有 mtx_buffer_ 时调用（SyncPackages 开头、空判之前）。
///
/// 为什么 LiDAR 可以丢而 IMU 不能乱丢：LiDAR 丢帧由 IMU 跨空档积分
/// （现有 imu_gap_limit 已支持）；IMU 丢在窗口内会破坏去畸变与预测。
/// 因此 IMU 的裁剪下界跟随"最旧保留的 LiDAR 帧"，而不是 IMU 自身的时间窗。
BufferDropCount LaserOdometry::PruneBuffers() {
    BufferDropCount drop;
    if (!buffer_policy_.enable) {
        return drop;
    }

    // ---- LiDAR 侧 ----
    // ① 时间窗：丢掉 t_newest_lidar - lidar_keep_sec 之前的帧
    const double t_newest = time_buffer_.empty() ? 0.0 : time_buffer_.back();
    while (!time_buffer_.empty() &&
           (t_newest - time_buffer_.front()) > buffer_policy_.lidar_keep_sec) {
        time_buffer_.pop_front();
        lidar_buffer_.pop_front();
        ++drop.lidar;
    }
    // ② 条数硬上限：时间戳冻结/卡顿时兜底
    while (static_cast<int>(lidar_buffer_.size()) > buffer_policy_.lidar_max_frames) {
        time_buffer_.pop_front();
        lidar_buffer_.pop_front();
        ++drop.lidar;
    }
    // ③ 追帧：已掉出实时则只留最新一帧，避免永久播旧账
    if (static_cast<int>(lidar_buffer_.size()) > buffer_policy_.lidar_catchup_frames) {
        drop.lidar += static_cast<int>(lidar_buffer_.size()) - 1;
        time_buffer_.erase(time_buffer_.begin(), time_buffer_.end() - 1);
        lidar_buffer_.erase(lidar_buffer_.begin(), lidar_buffer_.end() - 1);
    }
    // ④ 丢过帧就必须作废半状态：lidar_pushed_==true 时 measures_.lidar_ 指向的是
    //    旧的 front()，lidar_end_time_ 也随之过期；不作废会让下一帧用错误时间窗消费 IMU。
    if (drop.lidar > 0) {
        lidar_pushed_ = false;
        measures_.lidar_.reset();
    }

    // ---- IMU 侧（下界跟随最旧保留的 LiDAR）----
    if (imu_buffer_.empty()) {
        return drop;
    }
    double t_floor = 0.0;
    if (!time_buffer_.empty()) {
        t_floor = time_buffer_.front() - buffer_policy_.imu_keep_margin_sec;
    } else {
        // LiDAR 断流：只保留最近 imu_orphan_keep_sec 的 IMU，否则 SyncPackages 的
        // "lidar_buffer_ 为空直接 return" 会让 IMU 一枚都不消费、无限增长。
        t_floor = imu_buffer_.back()->header.stamp.toSec() - buffer_policy_.imu_orphan_keep_sec;
    }
    while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp.toSec() < t_floor) {
        imu_buffer_.pop_front();
        ++drop.imu;
    }
    // 条数兜底
    while (static_cast<int>(imu_buffer_.size()) > buffer_policy_.imu_max_samples) {
        imu_buffer_.pop_front();
        ++drop.imu;
    }
    return drop;
}

bool LaserOdometry::SyncPackages() {
    // 与 lio_odometry 对齐：SyncPackages 读写 lidar_buffer_/imu_buffer_/time_buffer_，
    // 必须持锁。当前单线程 spinOnce+tick 下行为中性，为后续改动消除隐患。
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    // 缓冲区治理（Stage 2）：必须在空判之前执行——LiDAR 断流时也要裁剪 IMU。
    const BufferDropCount dropped = PruneBuffers();
    if (dropped.lidar > 0 || dropped.imu > 0) {
        dropped_lidar_frames_ += static_cast<std::uint64_t>(dropped.lidar);
        dropped_imu_samples_ += static_cast<std::uint64_t>(dropped.imu);
        if (SHOULD_LOG(1.0)) {
            SYS_LOG(syslog::LogSeverity::WARNING, true)
                << "[BUFFER] drop lidar=" << dropped.lidar
                << " (total=" << dropped_lidar_frames_ << ")"
                << " drop imu=" << dropped.imu
                << " (total=" << dropped_imu_samples_ << ")"
                << " buf_lidar=" << lidar_buffer_.size()
                << " buf_imu=" << imu_buffer_.size();
        }
    }

    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            // LOG(WARNING) << "Too few input point cloud!";
            SYS_LOG(syslog::LogSeverity::WARNING, true) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserOdometry::PrintState(const state_ikfom &s) {
    // LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
    //           << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
    SYS_LOG(syslog::LogSeverity::INFO, true) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
        << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserOdometry::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
// ====================================================================
// ObsModel: LiDAR 点云观测模型
//
// 功能: 将当前帧体坐标系点云转换到世界系，与地图进行点-面匹配，
//       计算残差（点到平面距离）和雅可比矩阵 h_x（残差对状态变量的偏导）。
//
// 输入:
//   - s:          当前名义状态 (state_ikfom), 含 pos, rot, offset_R_L_I, offset_T_L_I
//   - ekfom_data: 输出结构, 填充 h_x (N×12 Jacobian) 和 h (N×1 残差向量)
//
// 核心原理:
//   点-面残差: r_i = n_i^T · (R * (R_off * p_i + t_off) + t) + d_i
//   其中:
//     - p_i:       体坐标系下的 LiDAR 点
//     - R_off,t_off: LiDAR→IMU 外参
//     - R,t:        IMU→世界 位姿 (待估计状态)
//     - n_i,d_i:    地图中最近平面的法向量和偏移 (n^T·x + d = 0)
//   残差 r_i 的物理含义: 点 p_i 变换到世界系后到地图平面的**有符号距离**
//
// Jacobian 推导 (对应 h_x 的 12 列):
//   状态变量: [pos(3), rot(3), offset_R(3), offset_T(3)]
//   1. ∂r/∂pos = n^T                           (位置: 单位法向量)
//   2. ∂r/∂θ   = n^T · R · hat(p_off)         (旋转: 叉乘矩阵, 右扰动)
//      其中 p_off = R_off · p_body + t_off 是 IMU 系下的点坐标
//   3. ∂r/∂θ_off = n^T · R · R_off · hat(p_body)  (LiDAR外参旋转)
//   4. ∂r/∂t_off = n^T · R                     (LiDAR外参平移)
// ====================================================================
void LaserOdometry::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    // 索引数组: 用于并行加速 (std::execution::par_unseq)
    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // ============================
    // 阶段一: 点-面匹配与残差计算
    // ============================
    Timer::Evaluate(
        [&, this]() {
            // 预计算: 当前名义状态下的位姿变换矩阵 (LiDAR→世界)
            // R_wl = R_IMUtoWorld * R_LiDartoIMU, t_wl = R * t_off + pos
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* 将体坐标系点变换到世界坐标系 */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** 在地图中搜索最近邻平面点集 **/
                    points_near.clear();
                    // iVox 高效近邻搜索: 获取世界坐标系下最近的 NUM_MATCH_POINTS 个点
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    // 检查近邻点数量是否满足最小要求
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        // 平面拟合: 通过最小二乘拟合平面方程 n^T·x + d = 0
                        // 输出 plane_coef_ = [nx, ny, nz, d]
                        // 返回 false 表示拟合的平面残差过大（非平面结构）
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    // 计算点到平面的有符号距离
                    // temp = [p_world, 1.0]^T, plane_coef_ = [n, d]^T
                    // pd2 = n·p_world + d = 点到平面的有符号距离
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    // 异常值剔除: 点距离传感器太近时, 距离/残差比应满足物理约束
                    // p_body.norm() 是点到 LiDAR 的距离;
                    // 81 * pd2 * pd2 = (9 * |pd2|)^2, 即点到平面距离的 9 倍平方
                    // 条件: 距离 > 9 * |pd2|, 即残差不应超过距离的 1/9
                    // 防止"退化"匹配: 当点非常接近传感器时, 微小角度误差会产生大残差
                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;  // 存储有符号距离作为残差
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    // ============================
    // 阶段二: 压缩有效点集
    // ============================
    effect_feat_num_ = 0;  // 有效匹配点计数

    // 预分配最大容量, 再压缩到实际有效数量
    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            // corr_norm_: 存储平面法向量
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            // corr_pts_: 存储体坐标系下的点坐标 + 残差 (前3维=点坐标, 第4维=残差)
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    // 压缩到实际有效数量
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    // 无有效匹配点 → 标记观测无效
    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        // LOG(WARNING) << "No Effective Points!";
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "No Effective Points!";
        return;
    }

    // ============================
    // 阶段三: Jacobian 矩阵构建
    // ============================
    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            // h_x: N×12 Jacobian, 对应状态 [pos, rot, offset_R, offset_T]
            // 注: FAST-LIO 框架要求 h_x 列数 = 12 (前6个状态+外参6个),
            //     实际上完整的 state_ikfom 有23维, 但框架只取前12维参与更新
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            // 预计算: 外参旋转矩阵和平移, 旋转矩阵的转置
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            // 并行计算每个点的 Jacobian
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                // ---- 坐标变换 ----
                // point_this_be: 体坐标系下的 LiDAR 点 (p_body)
                common::V3F point_this_be = corr_pts_[i].head<3>();
                // point_be_crossmat = hat(p_body): 体坐标系点的叉乘矩阵
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                // point_this = R_off * p_body + t_off: 变换到 IMU 坐标系下 (p_off)
                common::V3F point_this = off_R * point_this_be + off_t;
                // point_crossmat = hat(p_off): IMU 坐标系点的叉乘矩阵
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** 获取地图平面的法向量 ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** 计算 Jacobian 矩阵 ***/
                // C = R^T * n: 将世界系法向量旋转到 IMU 坐标系
                // 作用: 将后续的叉乘结果统一到 IMU 系, 便于链式求导
                common::V3F C(Rt * norm_vec);
                // A = hat(p_off) * R^T * n: 旋转 Jacobian
                // 推导: ∂r/∂θ = n^T · R · hat(p_off) (右扰动约定)
                //       代码中 A 是列向量, h_x 中存储 A^T 作为行向量
                // 物理含义: 点 p 在 IMU 系下的位置与法向量叉乘, 度量旋转对残差的影响
                common::V3F A(point_crossmat * C);

                // 填充 h_x 的一行 (12 列):
                // 列 0-2:   ∂r/∂pos     = n^T (法向量, 位置对残差的直接影响)
                // 列 3-5:   ∂r/∂rot     = A^T (旋转对残差的影响)
                // 列 6-8:   ∂r/∂off_R   = B^T (外参旋转对残差的影响)
                // 列 9-11:  ∂r/∂off_T   = C^T (外参平移对残差的影响)
                if (extrinsic_est_en_) {
                    // B = hat(p_body) * R_off^T * R^T * n: 外参旋转 Jacobian
                    // 推导: ∂r/∂θ_off = n^T · R · R_off · hat(p_body)
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    // 外参固定 (不优化): 外参相关列置零
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** 残差: 点到平面的有符号距离 (取负号符合 IEKF 框架约定) ***/
                // corr_pts_[i][3] 存储了阶段一计算的 pd2 = n^T·p_world + d
                // 框架约定: z - h(x) = 0, 其中 z=0 (理想点到平面距离为0)
                // 所以 ekfom_data.h(i) = 0 - pd2 = -pd2
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

void LaserOdometry::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserOdometry::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserOdometry::Finish() {
    // 纯里程计：无 PCD/轨迹保存；建图与保存由 slam_mapping_v2 负责。
    scan_down_body_ = nullptr;
    scan_undistort_ = nullptr;
    scan_down_world_ = nullptr;
    // LOG(INFO) << "laser odometry finish done";
    SYS_LOG(syslog::LogSeverity::INFO, true) << "laser odometry finish done";
}

}  // namespace faster_lio