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
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
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
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
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
    mtx_buffer_.unlock();
}

void LaserOdometry::FeedCanVelocity(const CanVelocityData &can) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    last_can_ = can;
    has_can_ = true;
}

void LaserOdometry::FeedGnssAttitude(const GnssAttitudeData &gnss) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    last_gnss_attitude_ = gnss;
    has_gnss_attitude_ = true;
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
