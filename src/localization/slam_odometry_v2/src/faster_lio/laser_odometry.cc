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

namespace faster_lio {

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
    LOG(INFO) << "init laser odometry from " << config_yaml;
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
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
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

        // 轮速/GNSS 辅助
        use_vel_ = yaml["vel_gnss"]["use_vel"].as<bool>(false);
        use_gnss_attitude_ = yaml["vel_gnss"]["use_gnss_attitude"].as<bool>(false);
        can_vel_diff_thresh_kmh_ = yaml["vel_gnss"]["can_vel_diff_thresh_kmh"].as<double>(2.0);
        can_timeout_sec_ = yaml["vel_gnss"]["can_timeout_sec"].as<double>(0.2);
        gnss_attitude_timeout_sec_ = yaml["vel_gnss"]["gnss_attitude_timeout_sec"].as<double>(1.0);
        attitude_diff_thresh_deg_ = yaml["vel_gnss"]["attitude_diff_thresh_deg"].as<double>(2.0);
        attitude_correction_gain_ = yaml["vel_gnss"]["attitude_correction_gain"].as<double>(0.7);

        // IMU 重力方向观测参数
        use_imu_gravity_ = yaml["vel_gnss"]["use_imu_gravity"].as<bool>(true);
        imu_gravity_noise_ = yaml["vel_gnss"]["imu_gravity_noise"].as<double>(0.02);
        imu_gravity_acc_threshold_ = yaml["vel_gnss"]["imu_gravity_acc_threshold"].as<double>(0.5);

        // DCReg 退化检测 + 鲁棒估计参数
        if (YAML::Node dcreg = yaml["dcreg"]) {
            use_dcreg_degeneracy_ = dcreg["use_dcreg_degeneracy"].as<bool>(use_dcreg_degeneracy_);
            use_huber_weight_ = dcreg["use_huber_weight"].as<bool>(use_huber_weight_);
            use_degeneracy_weight_ = dcreg["use_degeneracy_weight"].as<bool>(use_degeneracy_weight_);
            dcreg_condition_threshold_ = dcreg["dcreg_condition_threshold"].as<double>(dcreg_condition_threshold_);
            huber_k_ = dcreg["huber_k"].as<double>(huber_k_);
            degeneracy_weight_alpha_ = dcreg["degeneracy_weight_alpha"].as<double>(degeneracy_weight_alpha_);
        }

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar (livox_ros_driver::CustomMsg)";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::HESAIxt32);
        LOG(INFO) << "Using Hesai Pandar 32 Lidar";
    } else if (lidar_type == 5) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using Robosense Lidar";
    } else if (lidar_type == 6) {
        preprocess_->SetLidarType(LidarType::LIVOX);
        LOG(INFO) << "Using Livox Lidar (sensor_msgs::PointCloud2)";
    } else {
        LOG(WARNING) << "unknown lidar_type";
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
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

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

void LaserOdometry::SetObservationParams(double wheel_velocity_noise, double non_holonomic_noise,
                                         double gnss_attitude_noise_rad2,
                                         int min_gnss_status, int min_gnss_nsv, int min_lidar_points) {
    wheel_velocity_noise_ = wheel_velocity_noise;
    non_holonomic_noise_ = non_holonomic_noise;
    gnss_attitude_noise_rad2_ = gnss_attitude_noise_rad2;
    min_gnss_status_ = min_gnss_status;
    min_gnss_nsv_ = min_gnss_nsv;
    min_lidar_points_ = min_lidar_points;
    LOG(INFO) << "[Obs] params: wheel_noise=" << wheel_velocity_noise_
              << " nhc_noise=" << non_holonomic_noise_
              << " gnss_noise_rad2=" << gnss_attitude_noise_rad2_
              << " min_gnss_status=" << min_gnss_status_
              << " min_gnss_nsv=" << min_gnss_nsv_
              << " min_lidar_points=" << min_lidar_points_;
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
                LOG_FIRST_N(WARNING, 1) << "[Health] wheel speed timeout: age="
                                        << sensor_health_.can_age;
            }
            sensor_health_.can_healthy = false;
        }
    }

    // GNSS 超时检查
    if (sensor_health_.has_gnss_data) {
        sensor_health_.gnss_age = current_time - sensor_health_.gnss_last_update;
        if (sensor_health_.gnss_age > gnss_attitude_timeout_sec_) {
            if (sensor_health_.gnss_healthy) {
                LOG_FIRST_N(WARNING, 1) << "[Health] GNSS timeout: age="
                                        << sensor_health_.gnss_age;
            }
            sensor_health_.gnss_healthy = false;
        }
    }

    // LiDAR 健康检查
    sensor_health_.lidar_points = scan_undistort_ ? scan_undistort_->size() : 0;
    sensor_health_.lidar_healthy = static_cast<int>(sensor_health_.lidar_points) >= min_lidar_points_;
}

DegeneracyInfo LaserOdometry::DetectDegeneracy() const {
    DegeneracyInfo info;
    int num_points = effect_feat_num_;
    Eigen::Matrix<double, 23, 23> P = kf_.get_P();

    if (num_points < 10) {
        info.is_degenerate = true;
        info.degeneracy_ratio = 0.0;
        LOG_FIRST_N(WARNING, 5) << "[Degeneracy] too few points: " << num_points;
        return info;
    }
    // if (num_points <1000) {
    //     double scale = std::max(1.0, 1000.0 / num_points);
    //     P.block<3, 3>(3,3) *= scale;
    //     P.block<3, 3>(12,12) *= scale;
    //     kf_.change_P(P);
    //     LOG(INFO) << "[Degeneracy] scale P: " << scale;
    // }

    // 1. 收集所有有效点的法向量
    Eigen::Matrix<double, Eigen::Dynamic, 3> normals(num_points, 3);
    for (int i = 0; i < num_points; i++) {
        normals.row(i) = corr_norm_[i].head<3>().cast<double>();
    }

    // 2. 计算法向量协方差矩阵 C = (1/N) * sum((n_i - mean) * (n_i - mean)^T)
    Eigen::Vector3d mean = normals.colwise().mean();
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (int i = 0; i < num_points; i++) {
        Eigen::Vector3d diff = normals.row(i).transpose() - mean;
        C += diff * diff.transpose();
    }
    C /= num_points;

    // 3. 特征值分解（升序：λ₀ ≤ λ₁ ≤ λ₂）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    Eigen::Vector3d lambda = es.eigenvalues();
    Eigen::Matrix3d V = es.eigenvectors();

    double ratio = lambda(0) / lambda(2);
    info.is_degenerate = ratio < 0.1;
    info.degeneracy_ratio = ratio;
    info.degenerate_axis = V.col(0);  // 最小特征值对应的方向 = 退化轴

    // 4. 将退化轴映射到姿态空间
    //   退化轴沿哪个坐标轴，对应的旋转自由度退化
    info.is_rot_degenerate[0] = std::abs(info.degenerate_axis(0)) > 0.7;  // roll
    info.is_rot_degenerate[1] = std::abs(info.degenerate_axis(1)) > 0.7;  // pitch
    info.is_rot_degenerate[2] = std::abs(info.degenerate_axis(2)) > 0.7;  // yaw

    if (info.is_degenerate) {
        LOG(WARNING) << "[Degeneracy] DETECTED! ratio=" << info.degeneracy_ratio
                     << " axis=(" << info.degenerate_axis.transpose() << ")"
                     << " rot_degen=(" << info.is_rot_degenerate[0] << ","
                     << info.is_rot_degenerate[1] << "," << info.is_rot_degenerate[2] << ")"
                     << " lambda=(" << lambda.transpose() << ")";
    }

    return info;
}

void LaserOdometry::InflateDegenerateP(const DegeneracyInfo& info) {
    // 1. 获取当前 P 矩阵 (23×23)
    Eigen::Matrix<double, 23, 23> P = kf_.get_P();

    if (effect_feat_num_ <1500) {
        double scale = std::max(1.0, 2.0 * 1500.0 / effect_feat_num_);
        P.block<3, 3>(3,3) *= scale;
        P.block<3, 3>(12,12) *= scale;
        kf_.change_P(P);
        LOG(INFO) << "[Degeneracy] scale P: " << scale;
    }

    if (!info.is_degenerate) return;

    

    // 2. 提取旋转块 (3×3, 索引 3-5)
    Eigen::Matrix3d P_rot = P.block<3,3>(3,3);

    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_rot);
    Eigen::Vector3d lambda = es.eigenvalues();  // 升序
    Eigen::Matrix3d V = es.eigenvectors();

    // 4. 膨胀策略
    //   - 正常方向：保持 LiDAR 更新后的 P
    //   - 退化方向：膨胀到 σ_min = 2.0° 的不确定性
    constexpr double TARGET_SIGMA = 2.0 * M_PI / 180.0;  // 2.0°
    constexpr double TARGET_VAR = TARGET_SIGMA * TARGET_SIGMA;  // ~1.22e-3

    // ★ 速度协方差膨胀目标：允许轮速观测有效修正
    //   速度标准差膨胀到 1.0 m/s，与轮速噪声量级匹配
    constexpr double VEL_TARGET_SIGMA = 1.0;  // 1.0 m/s
    constexpr double VEL_TARGET_VAR = VEL_TARGET_SIGMA * VEL_TARGET_SIGMA;  // 1.0

    bool inflated = false;
    for (int i = 0; i < 3; i++) {
        // 检查该方向是否退化：通过特征向量与退化轴的对齐程度
        double alignment = std::abs(V.col(i).dot(info.degenerate_axis));
        bool is_this_degenerate = alignment > 0.5;

        if (is_this_degenerate) {
            LOG(WARNING) << "[Degeneracy] inflating P_rot direction " << i
                         << " from " << lambda(i) << " to " << TARGET_VAR
                         << " (sigma=" << TARGET_SIGMA * 180.0 / M_PI << " deg)"
                         << " alignment=" << alignment;
            lambda(i) = std::max(lambda(i), TARGET_VAR);
            inflated = true;
        }
    }

    if (inflated) {
        // 5. 重建 P_rot 并写回
        P.block<3,3>(3,3) = V * lambda.asDiagonal() * V.transpose();

        // 6. ★ 同时膨胀速度协方差 (索引 12-14)
        //    IMU 传播中，速度与旋转强耦合 (vel_dot = R*(acc-ba) + grav)，
        //    当旋转退化时，旋转误差会导致速度积分误差。
        //    只膨胀旋转协方差而不膨胀速度协方差，会导致：
        //      - 滤波器对错误的速度过于自信 (协方差小)
        //      - 轮速观测的卡尔曼增益太小，无法有效修正速度
        //      - 速度误差持续累积，最终导致 LiDAR 匹配失败 (No Effective Points)
        //    因此，将速度协方差也膨胀到 1.0 m/s 的方差，使轮速观测能够有效修正。
        Eigen::Matrix3d P_vel = P.block<3,3>(12,12);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_vel(P_vel);
        Eigen::Vector3d lambda_vel = es_vel.eigenvalues();
        Eigen::Matrix3d V_vel = es_vel.eigenvectors();
        for (int i = 0; i < 3; i++) {
            if (lambda_vel(i) < VEL_TARGET_VAR) {
                LOG(WARNING) << "[Degeneracy] inflating P_vel direction " << i
                             << " from " << lambda_vel(i) << " to " << VEL_TARGET_VAR
                             << " (sigma=" << VEL_TARGET_SIGMA << " m/s)";
                lambda_vel(i) = VEL_TARGET_VAR;
            }
        }
        P.block<3,3>(12,12) = V_vel * lambda_vel.asDiagonal() * V_vel.transpose();

        kf_.change_P(P);
    }
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
    //    h_x 的列: [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, off_R, off_T]
    Eigen::MatrixXd J_6 = ekfom_data.h_x.leftCols(6);
    Eigen::Matrix<double, 6, 6> H_6 = J_6.transpose() * J_6;

    // 2. DCReg 退化诊断
    if (use_dcreg_degeneracy_) {
        dcreg_result_ = ComputeDCReg(H_6, dcreg_condition_threshold_);
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
            Eigen::Matrix<double, 1, 6> J_row = J_6.row(i);
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

void LaserOdometry::InflateDegeneratePFromDCReg(const DCRegResult& dc_result) {
    Eigen::Matrix<double, 23, 23> P = kf_.get_P();

    constexpr double ROT_TARGET_SIGMA = 2.0 * M_PI / 180.0;  // 2.0°
    constexpr double ROT_TARGET_VAR = ROT_TARGET_SIGMA * ROT_TARGET_SIGMA;
    constexpr double POS_TARGET_SIGMA = 0.5;                  // 0.5m
    constexpr double POS_TARGET_VAR = POS_TARGET_SIGMA * POS_TARGET_SIGMA;
    constexpr double VEL_TARGET_SIGMA = 1.0;                  // 1.0 m/s
    constexpr double VEL_TARGET_VAR = VEL_TARGET_SIGMA * VEL_TARGET_SIGMA;

    bool any_rot_degenerate = false;

    // 旋转膨胀 (索引 3-5)
    // if (dc_result.degenerate_mask[0] || dc_result.degenerate_mask[1] ||
    //     dc_result.degenerate_mask[2]) {
    //     any_rot_degenerate = true;
    //     Eigen::Matrix3d P_rot = P.block<3,3>(3,3);
    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_rot);
    //     Eigen::Vector3d lambda = es.eigenvalues();
    //     Eigen::Matrix3d V = es.eigenvectors();

    //     // for (int i = 0; i < 3; i++) {
    //     //     if (lambda(i) < ROT_TARGET_VAR) {
    //     //         lambda(i) = ROT_TARGET_VAR;
    //     //     }
    //     // }

    //     for (int i = 0; i < 3; i++) {
    //         double score = dc_result.degeneracy_scores[i];
    //         if (score > 0.01 || lambda[i] < ROT_TARGET_VAR) {
    //             // 动态
    //             double dynamic_target = ROT_TARGET_VAR * (1.0 + score * 9.0);
    //             double alpha = 0.5 + score * 0.5;
    //             lambda(i) = std::pow(lambda(i), 1.0 - alpha) * std::pow(dynamic_target, alpha);
    //         }
    //     }
        


    //     P.block<3,3>(3,3) = V * lambda.asDiagonal() * V.transpose();
    // }
    // 直接膨胀物理轴
    if (dc_result.degenerate_mask[0]) {  // roll degenerate
        double score = dc_result.degeneracy_scores[0];
        double target_sigma = 2.0 * M_PI / 180.0 + (5.0 * M_PI / 180.0 - 2.0 * M_PI / 180.0) * score;
        P(3, 3) = std::max(P(3, 3), target_sigma * target_sigma);
    }
    if (dc_result.degenerate_mask[1]) {  // pitch degenerate
        double score = dc_result.degeneracy_scores[1];
        double target_sigma = 2.0 * M_PI / 180.0 + (5.0 * M_PI / 180.0 - 2.0 * M_PI / 180.0) * score;
        P(4, 4) = std::max(P(4, 4), target_sigma * target_sigma);
    }
    if (dc_result.degenerate_mask[2]) {  // yaw degenerate
        double score = dc_result.degeneracy_scores[2];
        double target_sigma = 2.0 * M_PI / 180.0 + (5.0 * M_PI / 180.0 - 2.0 * M_PI / 180.0) * score;
        P(5, 5) = std::max(P(5, 5), target_sigma * target_sigma);
    }

    // 平移膨胀 (索引 0-2)
    // if (dc_result.degenerate_mask[3] || dc_result.degenerate_mask[4] ||
    //     dc_result.degenerate_mask[5]) {
    //     Eigen::Matrix3d P_pos = P.block<3,3>(0,0);
    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_pos);
    //     Eigen::Vector3d lambda = es.eigenvalues();
    //     Eigen::Matrix3d V = es.eigenvectors();

    //     for (int i = 0; i < 3; i++) {
    //         if (lambda(i) < POS_TARGET_VAR) {
    //             lambda(i) = POS_TARGET_VAR;
    //         }
    //     }
    //     P.block<3,3>(0,0) = V * lambda.asDiagonal() * V.transpose();
    // }
    // 在 InflateDegeneratePFromDCReg 中，平移膨胀块之后新增：
    // ===== yaw 退化 → heading 漂移 → 通过 IMU 传播影响 Z 位置 =====
    if (dc_result.degenerate_mask[1]) {  // pitch degenerate
        // 单独膨胀 Z 位置 P(2,2)，让 NHC 和轮速观测能修正 Z
        // P(2,2) = std::max(P(2,2), POS_TARGET_VAR);
        P(14, 14) = std::max(P(14, 14), VEL_TARGET_VAR);
    }
    

    // 速度膨胀 (索引 12-14)：旋转退化时连带膨胀
    // IMU 传播中速度与旋转强耦合，旋转退化会导致速度积分误差累积
    if (any_rot_degenerate) {
        Eigen::Matrix3d P_vel = P.block<3,3>(12,12);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_vel);
        Eigen::Vector3d lambda = es.eigenvalues();
        Eigen::Matrix3d V = es.eigenvectors();

        for (int i = 0; i < 3; i++) {
            if (lambda(i) < VEL_TARGET_VAR) {
                lambda(i) = VEL_TARGET_VAR;
            }
        }
        P.block<3,3>(12,12) = V * lambda.asDiagonal() * V.transpose();
    }

    kf_.change_P(P);

    LOG(WARNING) << "[DCReg] P inflated: mask=["
                 << dc_result.degenerate_mask[0] << ","
                 << dc_result.degenerate_mask[1] << ","
                 << dc_result.degenerate_mask[2] << ","
                 << dc_result.degenerate_mask[3] << ","
                 << dc_result.degenerate_mask[4] << ","
                 << dc_result.degenerate_mask[5] << "]"
                 << " cond_rot=" << dc_result.cond_schur_rot
                 << " cond_trans=" << dc_result.cond_schur_trans
                 << " scores=["
                 << dc_result.degeneracy_scores[0] << ","
                 << dc_result.degeneracy_scores[1] << ","
                 << dc_result.degeneracy_scores[2] << ","
                 << dc_result.degeneracy_scores[3] << ","
                 << dc_result.degeneracy_scores[4] << ","
                 << dc_result.degeneracy_scores[5] << "]";
}

void LaserOdometry::ApplyWheelSpeedObservation() {
    // 获取当前状态
    state_ikfom state = kf_.get_x();

    Eigen::Matrix3d R_wl = state.rot.toRotationMatrix();
    double v_x = last_can_.speed_kmh / 3.6;  // km/h -> m/s
    Eigen::Vector3d can_body_vel(v_x, 0.0, 0.0);

    // ===== 观测 1: 前向速度约束 (3D) =====
    // 虚拟测量: h(x) = R * [v_x, 0, 0]^T - state.vel = 0
    // residual = 0 - h(x) = state.vel - R * [v_x, 0, 0]^T
    // H = ∂h/∂x: H_rot = R * hat([v_x, 0, 0]^T), H_vel = -I
    Eigen::Vector3d residual_vel = state.vel - R_wl * can_body_vel;

    Eigen::MatrixXd H1 = Eigen::MatrixXd::Zero(3, 23);
    H1.block<3, 3>(0, 3) = -R_wl * SKEW_SYM_MATRIX(can_body_vel);  // ∂h/∂rot = -R * hat(v_body)
    H1.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();          // ∂h/∂vel = -I

    // ===== 观测 2: 非完整性约束 (2D) =====
    // 车体系下横向和垂向速度应为零
    Eigen::Vector3d v_body_actual = R_wl.transpose() * state.vel;
    Eigen::Vector2d residual_nhc;
    residual_nhc(0) = 0.0 - v_body_actual(1);  // 横向速度应为 0
    residual_nhc(1) = 0.0 - v_body_actual(2);  // 垂向速度应为 0

    Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(2, 23);
    // ★ 非完整性约束观测模型：h(x) = R^T[1:2, :] * state.vel (车体系横向/垂向速度)
    //    EKF 残差: residual = z - h(x), H = ∂h/∂x (不是残差的雅可比!)
    //    ∂h/∂vel = +R^T[1:2, :]  (正号!)
    H2.block<2, 3>(0, 12) = R_wl.transpose().block<2, 3>(1, 0);
    //    ∂h/∂rot = hat(R^T * vel) = hat(v_body)
    //    推导: R → R * exp(hat(δθ)), R_new^T ≈ (I - hat(δθ)) * R^T
    //          h_new = R_new^T * vel = R^T * vel - hat(δθ) * R^T * vel
    //          hat(δθ) * (R^T * vel) = -hat(R^T * vel) * δθ
    //          h_new = R^T * vel + hat(R^T * vel) * δθ
    //          ∂h/∂rot = hat(R^T * vel) = hat(v_body)  (正号!)
    H2.block<2, 3>(0, 3) = SKEW_SYM_MATRIX(v_body_actual).block<2, 3>(1, 0);

    // ===== ★ 残差合理性检查 =====
    // 当 LiDAR 退化导致旋转/速度估计严重错误时，轮速残差会异常增大。
    // 此时直接进行 EKF 更新会因残差过大而破坏状态（特别是旋转估计），
    // 进而导致下一帧 LiDAR 匹配失败（No Effective Points）。
    // 策略：跳过本次更新，等待下一帧 LiDAR 观测恢复或退化消失。
    const double VEL_RESIDUAL_THRESHOLD = 2.0;   // 5 m/s：正常轮速残差应 < 2 m/s
    const double NHC_RESIDUAL_THRESHOLD = 1.0;   // 5 m/s：正常 NHC 残差应 < 1 m/s
    if (residual_vel.norm() > VEL_RESIDUAL_THRESHOLD || residual_nhc.norm() > NHC_RESIDUAL_THRESHOLD) {
        LOG(WARNING) << "[Obs] Wheel speed residual too large, skip update:"
                     << " vel_norm=" << residual_vel.norm()
                     << " nhc_norm=" << residual_nhc.norm();
        return;
    }

    // ===== 合并观测 (5D) =====
    Eigen::MatrixXd H(5, 23);
    H << H1, H2;

    Eigen::VectorXd residual(5);
    residual << residual_vel, residual_nhc;

    // ===== 噪声矩阵 (5×5) =====
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(5, 5);
    R.block<3, 3>(0, 0) = wheel_velocity_noise_ * Eigen::Matrix3d::Identity();  // 速度方差
    R.block<2, 2>(3, 3) = non_holonomic_noise_ * Eigen::Matrix2d::Identity();   // 非完整性约束方差

    // 打印轮速更新前的 P 矩阵（用于调试）
    {
        auto P = kf_.get_P();
        LOG(INFO) << "[Obs] Wheel pre-update P diag: pos=" << P.diagonal().segment<3>(0).transpose()
                  << " rot=" << P.diagonal().segment<3>(3).transpose()
                  << " vel=" << P.diagonal().segment<3>(12).transpose()
                  << " bg=" << P.diagonal().segment<3>(15).transpose()
                  << " ba=" << P.diagonal().segment<3>(18).transpose()
                  << " grav=" << P.diagonal().segment<2>(21).transpose();
    }

    // 单次更新
    kf_.update_single_observation(H, residual, R);

    LOG(INFO) << "[Obs] wheel speed update: residual_norm=" << residual.norm()
              << " vel_norm=" << residual_vel.norm()
              << " nhc_norm=" << residual_nhc.norm()
              << " speed_ms=" << v_x;
}

void LaserOdometry::ApplyGnssAttitudeObservation() {
    // if (!isGnssAttitudeValid()) {
    //     LOG(INFO) << "[Obs] GNSS attitude not valid, skip update";
    //     return;
    // }
    // 获取当前状态
    state_ikfom state = kf_.get_x();

    // 1. 将 GNSS 欧拉角（度）转为弧度
    double heading_rad = last_gnss_attitude_.heading_deg * M_PI / 180.0;
    double pitch_rad = last_gnss_attitude_.pitch_deg * M_PI / 180.0;
    double roll_rad = last_gnss_attitude_.roll_deg * M_PI / 180.0;

    // 2. 构建 GNSS 旋转矩阵（ZYX 欧拉角：R = Rz(yaw) * Ry(pitch) * Rx(roll)）
    double cy = std::cos(heading_rad);
    double sy = std::sin(heading_rad);
    double cp = std::cos(pitch_rad);
    double sp = std::sin(pitch_rad);
    double cr = std::cos(roll_rad);
    double sr = std::sin(roll_rad);

    Eigen::Matrix3d R_gnss;
    R_gnss << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
              sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
              -sp,     cp * sr,                cp * cr;

    // 3. 计算 Lie 代数残差（3 维，在 SO3 切空间上）
    //    residual = log(R_est^T * R_gnss) = log(R_est.inverse() * R_gnss)
    Eigen::Matrix3d R_est = state.rot.toRotationMatrix();
    Eigen::Matrix3d R_diff = R_est.transpose() * R_gnss;
    Eigen::AngleAxisd aa(R_diff);
    Eigen::Vector3d residual = aa.angle() * aa.axis();

    // 4. 零化 heading（yaw）分量，只使用 roll 和 pitch 更新姿态
    //    单天线 GNSS 的 heading 来自速度方向（course over ground），不可靠
    residual[2] = 0.0;

    // 5. 残差门限检查（只检查 roll-pitch 分量，0.5 弧度 ≈ 28.6 度）
    if (residual.head<2>().norm() > 0.8) {
        LOG(WARNING) << "[Obs] GNSS attitude residual too large: " << residual.head<2>().norm()
                     << " rad (roll-pitch), skip update";
        return;
    }

    // 6. 打印 GNSS 更新前的 P 矩阵（用于调试）
    {
        auto P = kf_.get_P();
        LOG(INFO) << "[Obs] GNSS pre-update P diag: pos=" << P.diagonal().segment<3>(0).transpose()
                  << " rot=" << P.diagonal().segment<3>(3).transpose()
                  << " vel=" << P.diagonal().segment<3>(12).transpose()
                  << " bg=" << P.diagonal().segment<3>(15).transpose()
                  << " ba=" << P.diagonal().segment<3>(18).transpose()
                  << " grav=" << P.diagonal().segment<2>(21).transpose();
    }

    // 7. 构建 H 矩阵（3x23，旋转在索引 3-5）
    //    只更新 roll (index 3) 和 pitch (index 4)，不更新 yaw (index 5)
    //    零化 yaw 对应的 H 行，避免 EKF 虚假降低 yaw 协方差
    //    因为 residual[2] = 0，但 H.row(2) 非零会导致 EKF 在 yaw 维度上
    //    进行"零残差更新"，人为压缩 yaw 不确定度，造成虚假置信
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);
    H.block<2, 3>(0, 3) = Eigen::Matrix3d::Identity().topRows<2>();
    H.row(2).setZero();

    // 8. 噪声矩阵（与 lm_slam 对齐：方差 = 0.03 rad²）
    double variance = gnss_attitude_noise_rad2_;
    Eigen::MatrixXd R = variance * Eigen::Matrix3d::Identity();

    // 8. 单次更新
    kf_.update_single_observation(H, residual, R);

    LOG(INFO) << "[Obs] GNSS attitude update: residual_norm=" << residual.norm()
              << " rad, (roll-pitch)=" << residual.head<2>().norm() << " rad, heading=0 (ignored)"
              << " heading=" << last_gnss_attitude_.heading_deg
              << " pitch=" << last_gnss_attitude_.pitch_deg
              << " roll=" << last_gnss_attitude_.roll_deg
              << " nsv=" << last_gnss_attitude_.nsv;
}

bool LaserOdometry::isGnssAttitudeValid() const {
    std::lock_guard<std::mutex> lock(health_mutex_);

    // 1. 数据可用性
    if (!sensor_health_.has_gnss_data || !sensor_health_.gnss_healthy) {
        LOG_FIRST_N(WARNING, 5) << "[Obs] GNSS not available: has_data="
                                << sensor_health_.has_gnss_data
                                << " healthy=" << sensor_health_.gnss_healthy;
        return false;
    }

    // 2. nsv 检查
    if (sensor_health_.gnss_nsv < min_gnss_nsv_) {
        LOG_FIRST_N(WARNING, 5) << "[Obs] GNSS nsv too low: " << sensor_health_.gnss_nsv
                                << " < " << min_gnss_nsv_;
        return false;
    }

    // 3. 数据新鲜度
    if (sensor_health_.gnss_age > gnss_attitude_timeout_sec_) {
        LOG_FIRST_N(WARNING, 5) << "[Obs] GNSS data too old: " << sensor_health_.gnss_age
                                << " > " << gnss_attitude_timeout_sec_;
        return false;
    }

    return true;
}

bool LaserOdometry::isLowAcceleration() const {
    if (!has_imu_data_) return false;
    double acc_norm = last_acc_.norm();
    double diff = std::abs(acc_norm - common::G_m_s2);
    return diff < imu_gravity_acc_threshold_;
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

    // ∂r/∂rot (3-5): hat(R^T * grav)
    // 推导: R_new = R * Exp(δθ), R_new^T ≈ (I - hat(δθ)) * R^T
    //       R_new^T * grav ≈ R^T * grav - hat(δθ) * R^T * grav
    //       = R^T * grav + hat(R^T * grav) * δθ
    //       ∂(R^T * grav)/∂δθ = hat(R^T * grav)
    //       r = (z_acc - ba) + R^T * grav
    //       ∂r/∂δθ = hat(R^T * grav)
    // 这个雅可比矩阵将 3D 残差映射到旋转切空间，使得 roll/pitch 可观测
    // 注意：hat(R^T * grav) 的第三列为零向量，因此 heading (yaw) 不可观测
    H.block<3, 3>(0, 3) = SKEW_SYM_MATRIX(Rt_grav);

    // ∂r/∂ba (18-20): -I
    // 加速度计偏置也更新，因为偏置会影响 IMU 传播的精度
    // 注意：雅可比矩阵应为 -I（残差 r = (z_acc - ba) + R^T * grav 对 ba 的导数），
    // 之前使用 -0.1*I 导致 ba 欠校正，长期累积使 IMU 传播速度积分产生漂移。
    H.block<3, 3>(0, 18) = -Eigen::Matrix3d::Identity();

    // ⚠️ 不更新 grav (21-22) 状态！
    // 更新 grav 会导致 Z 轴位置通过 IMU 传播产生漂移
    // 因为 vel_dot = R * (in.acc - ba) + grav

    // ===== 噪声矩阵 (3×3) =====
    // 噪声在 m/s² 单位，imu_gravity_noise_ 是方差
    Eigen::Matrix3d R_mat = imu_gravity_noise_ * Eigen::Matrix3d::Identity();

    // ===== 单次更新 =====
    kf_.update_single_observation(H, residual, R_mat);

    LOG(INFO) << "[Obs] IMU gravity update: residual_norm=" << residual.norm()
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
            LOG(INFO) << "[VelAid] GNSS attitude correction: pitch=" << pitch_deg
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
        LOG(INFO) << "[VelAid] CAN velocity correction: LiDAR=" << lidar_speed_kmh
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
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// 轮速/GNSS 辅助校正
    // ApplyVelocityGnssAid();

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
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
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
            
            // {
            //     auto P = kf_.get_P();
            //     LOG(INFO) << "[Obs] lidar pre-update P diag: pos=" << P.diagonal().segment<3>(0).transpose()
            //       << " rot=" << P.diagonal().segment<3>(3).transpose()
            //       << " vel=" << P.diagonal().segment<3>(12).transpose()
            //       << " bg=" << P.diagonal().segment<3>(15).transpose()
            //       << " ba=" << P.diagonal().segment<3>(18).transpose()
            //       << " grav=" << P.diagonal().segment<2>(21).transpose();
            // }



            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);

            // ===== DCReg P 矩阵定向膨胀 =====
            // DCReg 检测到退化时，在退化方向膨胀 P 矩阵，
            // 让后续轮速/GNSS/IMU 重力观测有足够话语权修正退化方向的状态
            if (use_dcreg_degeneracy_ && dcreg_result_.is_degenerate) {
                InflateDegeneratePFromDCReg(dcreg_result_);
            }

            // ===== 退化检测 & P 矩阵膨胀（原法向量方案，已注释） =====
            // 检测 LiDAR 观测是否退化（如隧道场景），在退化方向上膨胀 P 矩阵，
            // 让后续的轮速（非完整性约束）和 GNSS 观测有足够的话语权
            // {
            //     DegeneracyInfo degeneracy = DetectDegeneracy();
            //     if (degeneracy.is_degenerate) {
            //         InflateDegenerateP(degeneracy);
            //     }
            // }

            /// 更新传感器健康状态（检查超时）
            // UpdateSensorHealth();

            /// 轮速观测更新（5D：前向速度 + 非完整性约束）
            if (use_vel_ && sensor_health_.can_healthy) {
                ApplyWheelSpeedObservation();
            }

            // GNSS 姿态观测更新（单次更新，只观测 roll 和 pitch）
            // if (use_gnss_attitude_ && sensor_health_.gnss_healthy) {
            //     ApplyGnssAttitudeObservation();
            // }

            if (use_gnss_attitude_) {
                ApplyGnssAttitudeObservation();
            }

            

            // IMU 加速度计重力方向观测（单次更新，约束 roll 和 pitch）
            // ApplyImuGravityObservation();

            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");
    // 局部地图范围剔除：控制 iVox 规模，避免地图无限增长导致每帧处理/发布变慢
    if (map_trim_radius_ > 0.0f && (map_trim_interval_frames_ <= 1 || frame_num_ % map_trim_interval_frames_ == 0)) {
        PointType center;
        center.x = static_cast<float>(pos_lidar_.x());
        center.y = static_cast<float>(pos_lidar_.y());
        center.z = static_cast<float>(pos_lidar_.z());
        ivox_->RemoveFarPoints(center, map_trim_radius_);
    }

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

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
                LOG(ERROR) << "lidar loop back, clear buffer";
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
        LOG(WARNING) << "imu loop back, clear buffer";
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

    // 立即更新健康状态
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        sensor_health_.has_gnss_data = true;
        sensor_health_.gnss_last_update = gnss.timestamp;
        sensor_health_.gnss_status = gnss.status;
        sensor_health_.gnss_nsv = gnss.nsv;
        // 根据 status 判断健康状态（默认 33 为 RTK 固定解）
        sensor_health_.gnss_healthy = (gnss.status >= min_gnss_status_);
    }
}

bool LaserOdometry::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
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
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
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
void LaserOdometry::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    points_near.clear();
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
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
    LOG(INFO) << "laser odometry finish done";
}

}  // namespace faster_lio