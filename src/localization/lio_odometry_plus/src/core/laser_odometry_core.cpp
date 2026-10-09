#include "lio_odometry/core/laser_odometry_core.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <execution>
#include <fstream>
#include <Eigen/Eigenvalues>

#include "lio_odometry/core/eskf.h"
#include <chrono>

namespace slam_odometry {

namespace fasterlio {
int NUM_MAX_ITERATIONS = 4;
float ESTI_PLANE_THRESHOLD = 0.1f;
float lidar_time_interval = 0.1;
}  // namespace fasterlio

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


bool esti_plane(Vec4f& pca_result, const PointVector& point, const float& threshold) {
    if (point.size() < fasterlio::MIN_NUM_MATCH_POINTS) {
        return false;
    }

    // Original QR A·n = -1 plane fitting, identical to lightning-lm-ros1.
    // Solves the overdetermined system [x,y,z]·n = -1 via column-pivoting QR.
    // No intermediate eigenvalue checks — only the final point-to-plane distance
    // threshold is enforced. This tolerates mild motion blur during turns and
    // retains more valid correspondences in degenerate scenes.
    Vec3f normvec;

    if (point.size() == fasterlio::NUM_MATCH_POINTS) {
        Eigen::Matrix<float, fasterlio::NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<float, fasterlio::NUM_MATCH_POINTS, 1> b;

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < fasterlio::NUM_MATCH_POINTS; j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        normvec = A.colPivHouseholderQr().solve(b);
    } else {
        Eigen::MatrixXf A(point.size(), 3);
        Eigen::VectorXf b(point.size());

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (size_t j = 0; j < point.size(); j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        Eigen::VectorXf n = A.colPivHouseholderQr().solve(b);
        normvec(0) = n(0);
        normvec(1) = n(1);
        normvec(2) = n(2);
    }

    const float n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0f / n;

    for (const auto& p : point) {
        Vec4f temp = p.getVector4fMap();
        temp[3] = 1.0f;
        if (fabs(pca_result.dot(temp)) > threshold) {
            return false;
        }
    }
    return true;
}

Vec3d SO3ToEulerZYX_Robust(const SO3& rotation) {
    Mat3d R = rotation.matrix();
    double pitch = asin(-R(2, 0));
    if (pitch > M_PI / 2) pitch = M_PI / 2;
    if (pitch < -M_PI / 2) pitch = -M_PI / 2;
    double cos_pitch = cos(pitch);
    double yaw, roll;
    if (fabs(cos_pitch) > 1e-6) {
        yaw = atan2(R(1, 0) / cos_pitch, R(0, 0) / cos_pitch);
        roll = atan2(R(2, 1) / cos_pitch, R(2, 2) / cos_pitch);
    } else {
        yaw = 0;
        roll = atan2(R(0, 1), R(1, 1));
    }
    return Vec3d(yaw, pitch, roll);
}

SO3 EulerZYXToSO3(const Vec3d& euler_zyx) {
    double yaw = euler_zyx[0];
    double pitch = euler_zyx[1];
    double roll = euler_zyx[2];

    double cy = cos(yaw);
    double sy = sin(yaw);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cr = cos(roll);
    double sr = sin(roll);

    Mat3d R;
    R(0, 0) = cy * cp;
    R(0, 1) = cy * sp * sr - sy * cr;
    R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp;
    R(1, 1) = sy * sp * sr + cy * cr;
    R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cp * cr;

    return SO3(R);
}

LaserOdometryCore::LaserOdometryCore(const LaserOdometryOptions& options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
    ivox_.reset(new IVoxType(ivox_options_));
}

LaserOdometryCore::~LaserOdometryCore() {}

bool LaserOdometryCore::Init(const std::string& config_yaml) {
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, 23, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModel(s, obs); };
    eskf_options.wheelspeed_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) {
        ObsModelWheelspeed(s, obs);
    };
    eskf_options.gps_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModelGPS(s, obs); };
    // >>>>> [ZUPT] BEGIN 注册重力对齐观测 <<<<<
    eskf_options.acc_as_gravity_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) {
        ObsModelGravity(s, obs);
    };
    // >>>>> [ZUPT] END <<<<<
    // >>>>> [NHC] BEGIN 注册非完整约束观测 <<<<<
    eskf_options.nhc_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModelNHC(s, obs); };
    // >>>>> [NHC] END <<<<<
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserOdometryCore::LoadParamsFromYAML(const std::string& yaml_file) {
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;
    Vec3d lidar_T_wrt_IMU;
    Mat3d lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        extrinsic_est_en_ = yaml["fasterlio"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_options_.capacity_ = 1000000;
        if (ivox_nearby_type == 0) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        } else if (ivox_nearby_type == 6) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        } else if (ivox_nearby_type == 18) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        } else if (ivox_nearby_type == 26) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        } else {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }
        ivox_.reset(new IVoxType(ivox_options_));

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        use_wheel_speed_ = yaml["vel_gnss"]["use_wheel_speed"].as<bool>(false);
        use_gnss_ = yaml["vel_gnss"]["use_gnss"].as<bool>(false);
        wheel_velocity_noise_ = yaml["vel_gnss"]["wheel_velocity_noise"].as<double>(0.33);
        gnss_attitude_noise_ = yaml["vel_gnss"]["gnss_attitude_noise"].as<double>(0.03);
        wheel_sigma_ = yaml["vel_gnss"]["wheel_velocity_noise"].as<double>(0.25);
        wheel_huber_delta_ = yaml["vel_gnss"]["wheel_huber_delta"].as<double>(0.8);
        wheel_min_speed_ = yaml["vel_gnss"]["wheel_min_speed"].as<double>(0.5);
        wheel_max_age_ = yaml["vel_gnss"]["wheel_max_age"].as<double>(0.15);
        use_wheel_1d_ = yaml["vel_gnss"]["use_wheel_1d"].as<bool>(true);
        
        window_size_ = yaml["vel_gnss"]["gnss_pitch_window_size"].as<int>(50);
        // >>>>> [GNSS-REZERO] BEGIN —— 停车期 GNSS 基准重零（回退：删本行或配置 gnss_rezero: false）<<<<<
        gnss_rezero_enable_ = yaml["vel_gnss"]["gnss_rezero"].as<bool>(true);
        // >>>>> [GNSS-REZERO] END <<<<<

        // >>>>> [ZUPT/NHC] BEGIN 参数读取（回退：删除本块）<<<<<
        enable_zupt_ = yaml["zupt"]["enable"].as<bool>(true);
        zupt_wheel_thresh_ = yaml["zupt"]["wheel_thresh"].as<double>(0.15);
        zupt_gyr_thresh_ = yaml["zupt"]["gyr_thresh_deg"].as<double>(0.5) * M_PI / 180.0;
        zupt_acc_thresh_ = yaml["zupt"]["acc_thresh"].as<double>(0.15);
        stationary_frames_ = yaml["zupt"]["stationary_frames"].as<int>(20);
        release_factor_ = yaml["zupt"]["release_factor"].as<double>(2.0);
        zupt_var_ = yaml["zupt"]["zupt_var"].as<double>(1e-4);
        bg_calib_frames_ = yaml["zupt"]["bg_calib_frames"].as<int>(20);
        bg_p_target_ = yaml["zupt"]["bg_p_target"].as<double>(1e-6);
        gravity_align_frames_ = yaml["zupt"]["gravity_align_frames"].as<int>(100);
        gravity_noise_deg_ = yaml["zupt"]["gravity_noise_deg"].as<double>(1.0);
        use_nhc_ = yaml["zupt"]["use_nhc"].as<bool>(true);
        nhc_var_ = yaml["zupt"]["nhc_var"].as<double>(0.04);
        nhc_min_speed_ = yaml["zupt"]["nhc_min_speed"].as<double>(1.0);
        nhc_pitchrate_thresh_ = yaml["zupt"]["nhc_pitchrate_thresh_deg"].as<double>(3.0) * M_PI / 180.0;
        nhc_yawrate_thresh_ = yaml["zupt"]["nhc_yawrate_thresh_deg"].as<double>(5.0) * M_PI / 180.0;
        {
            const std::vector<double> lever = yaml["zupt"]["nhc_lever_arm"].as<std::vector<double>>(
                std::vector<double>{0.0, 0.0, 0.0});
            if (lever.size() >= 3) nhc_lever_ = Vec3d(lever[0], lever[1], lever[2]);
        }
        // >>>>> [ZUPT/NHC] END <<<<<
        lidar_update_r_ = yaml["fasterlio"]["lidar_update_r"].as<double>(0.001);
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>(false);
        map_trim_radius_ = yaml["fasterlio"]["map_trim_radius"].as<float>(0.0f);
        map_trim_interval_frames_ = yaml["fasterlio"]["map_trim_interval_frames"].as<int>(10);

    } catch (...) {
        return false;
    }

    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
    } else {
        return false;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    lidar_T_wrt_IMU = Vec3d(extrinT_[0], extrinT_[1], extrinT_[2]);
    lidar_R_wrt_IMU << extrinR_[0], extrinR_[1], extrinR_[2], extrinR_[3], extrinR_[4], extrinR_[5], extrinR_[6],
        extrinR_[7], extrinR_[8];

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu_->SetGapLimit(yaml["fasterlio"]["imu_gap_limit"].as<double>(0.15));

    return true;
}

void LaserOdometryCore::ResyncStreams() {
    // 时钟回退/跳变（PTP 重同步、驱动重启等）只影响数据时序，车的真实位姿是连续的。
    // 清输入缓冲与 IMU 跨帧衔接即可；滤波器状态、P、地图、IMU 初始化全部保留。
    std::cerr << "[lio] sensor time moved backwards; resync streams (filter state and map kept)" << std::endl;
    imu_buffer_.clear();
    lidar_buffer_.clear();
    time_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
    p_imu_->RestartStreaming();
}

void LaserOdometryCore::ResetTracking() {
    std::cerr << "[lio] sensor time moved backwards; resetting odometry and local map" << std::endl;
    imu_buffer_.clear();
    lidar_buffer_.clear();
    time_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
    last_lidar_time_ = first_lidar_time_ = lidar_end_time_ = 0.0;
    lidar_mean_scantime_ = 0.0;
    scan_num_ = frame_num_ = skip_lidar_cnt_ = effect_feat_num_ = 0;
    flg_first_scan_ = true;
    flg_EKF_inited_ = false;
    // have_new_gnss_ = have_new_wheel_speed_ = false;
    have_new_gnss_ = false;
    first_gnss_ = true;
    window_.clear();
    sum_ = 0.0;
    p_imu_->Reset();
    kf_.ChangeX(NavState());
    kf_.ChangeP(ESKF::CovType::Identity());
    state_point_ = kf_.GetX();
    ivox_.reset(new IVoxType(ivox_options_));
    nearest_points_.clear();
    scan_undistort_.reset(new PointCloudType());
    scan_down_body_->clear();
    scan_down_world_->clear();
}

void LaserOdometryCore::ProcessIMU(const IMUPtr& imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (!std::isfinite(timestamp) || !imu->angular_velocity.allFinite() ||
        !imu->linear_acceleration.allFinite()) return;
    if (timestamp < last_timestamp_imu_) ResyncStreams();
    if (timestamp == last_timestamp_imu_) return;

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(imu);

    // 无界缓冲保护：处理卡顿/长时间缺口时丢弃最老样本（约 50s @100Hz），
    // 避免内存持续上涨；正常工况永远触不到。
    if (imu_buffer_.size() > 5000) {
        std::cout << "imu buffer size > 5000 " << std::endl;
        imu_buffer_.pop_front();
    }
}

void LaserOdometryCore::ProcessPointCloud(const CloudPtr& cloud, double timestamp) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    scan_count_++;

    if (!cloud || !std::isfinite(timestamp)) return;
    if (timestamp < last_timestamp_lidar_) ResyncStreams();
    // 重复 header stamp 的帧直接丢弃，避免同一帧点云被重复更新/入图。
    if (timestamp == last_timestamp_lidar_) return;
    // Validate time/coordinates before synchronization; never let invalid points set scan end.
    cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(), [](const PointType& p) {
        return !p.getVector3fMap().allFinite() || !std::isfinite(p.time) || p.time < 0.0;
    }), cloud->points.end());
    if (cloud->empty()) return;
    cloud->width = cloud->size();
    cloud->height = 1;
    // Match Lightning's input contract: zero point times are valid. Keep input order;
    // SyncPackages uses the last retained point and the historical scan duration.
    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

void LaserOdometryCore::ProcessPointCloud(const pcl::PointCloud<OusterPoint>::Ptr& cloud, double timestamp) {
    CloudPtr out(new PointCloudType());
    preprocess_->Process(cloud, out);
    ProcessPointCloud(out, timestamp);
}

void LaserOdometryCore::ProcessPointCloud(const pcl::PointCloud<VelodynePoint>::Ptr& cloud, double timestamp) {
    CloudPtr out(new PointCloudType());
    preprocess_->Process(cloud, out);
    ProcessPointCloud(out, timestamp);
}

bool LaserOdometryCore::SyncPackages() {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        // Same fallback as Lightning LaserMapping::SyncPackages. When all times
        // have always been zero, the mean is zero and the state is aligned to header.stamp.
        if (measures_.scan_->size() <= 1 ||
            measures_.scan_->points.back().time / 1000.0 < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            const double duration = measures_.scan_->points.back().time / 1000.0;
            ++scan_num_;
            lidar_end_time_ = measures_.lidar_begin_time_ + duration;
            lidar_mean_scantime_ += (duration - lidar_mean_scantime_) / scan_num_;
        }

        fasterlio::lidar_time_interval = lidar_mean_scantime_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    // 与原版 lightning-lm / FAST-LIO 的 SyncPackages 严格一致：
    // 逐枚弹出 IMU，并且【包含】第一枚时间戳 > lidar_end_time_ 的 IMU 后停止。
    // 不能用 "while(front <= end)"——IMU 突发到样时，缓冲中可能恰好只剩晚于
    // lidar_end 的样本，严格 <= 会一枚都弹不到（meas.imu_ 为空），帧却被消费，
    // 下一帧把跨窗间隔算成 ~100ms 的假 gap。多弹的这枚"未来 IMU"由
    // UndistortPcl 的帧尾负 dt 外推（note=-1）正确处理，且循环内 tail 早于
    // last_lidar_end_time_ 的区间会被 continue 跳过，不会重复积分。
    measures_.imu_.clear();
    while (!imu_buffer_.empty()) {
        const double imu_time = imu_buffer_.front()->timestamp;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
        if (imu_time > lidar_end_time_) {
            break;
        }
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    return true;
}

bool LaserOdometryCore::Run() {
    if (!SyncPackages()) {
        // std::cout << "SyncPackages failed " << std::endl;
        return false;
    }

    // 每帧处理前重置输出点云：IMU 空包/去畸变跳过时 Process 不会写 scan_undistort_，
    // 必须保证下方空检查看到的是"本帧的空"而不是上一帧残留的点云，否则会用旧点云
    // 重复更新并把同一批点再次插入地图。
    scan_undistort_.reset(new PointCloudType());
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (!scan_undistort_ || scan_undistort_->empty()) {
        return false;
    }
    // 如果使用轮速，先消费轮速
    // if (use_wheel_speed_) {
    //     ConsumeWheelSpeedForFrame(measures_.lidar_begin_time_, measures_.lidar_end_time_);
    // }

    if (flg_first_scan_) {
        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (size_t i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;
        if (skip_lidar_cnt_ != 0) {
            return false;
        }
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        return false;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    auto old_state = kf_.GetX();
    const ESKF::CovType P_propagated = kf_.GetP(); // 获取更新前的协方差矩阵
    const bool lidar_accepted = kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_);
    // [LIDAR] 匹配质量：iters=IEKF迭代数, res=残差收敛比(1=无改善)
    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[LIDAR] ok=" << lidar_accepted
            << " iters=" << kf_.GetIterations()
            << " res=" << kf_.GetFinalRes()
            << " dpos=" << (kf_.GetX().pos_ - old_state.pos_).norm()
            << " dyaw=" << (old_state.rot_.inverse() * kf_.GetX().rot_).log().norm() * 180.0 / M_PI;
    // if (!lidar_accepted) {
    //     // Update rejected — restore propagated state and skip map insertion.
    //     // This prevents pure-prediction (often misaligned) points from polluting
    //     // the local map, matching the original lightning-lm philosophy of "reject
    //     // the frame, don't insert wrong correspondences".
    //     kf_.ChangeX(old_state);
    //     state_point_ = old_state;
    //     state_point_.timestamp_ = measures_.lidar_end_time_;
    //     return true;
    // }
    // if (use_wheel_speed_ && have_new_wheel_speed_) {
    //     kf_.Update(ESKF::ObsType::WHEEL_SPEED, wheel_velocity_noise_ * wheel_velocity_noise_);
    //     have_new_wheel_speed_ = false;
    //     // std::cout << "use_wheel_speed_  done " << std::endl;
    //     SYS_LOG(syslog::LogSeverity::INFO, false) 
    //             << "use_wheel_speed_  done ";
    // }
    
    // >>>>> [ZUPT/NHC] BEGIN 静止检测：轮速+IMU 联合判据+滞回（设计文档 §2）<<<<<
    const double speed_for_map = use_wheel_speed_ ? std::fabs(wheel_speed_obs_)
                                                 : state_point_.vel_.norm();
    Vec3d gyr_mean = Vec3d::Zero(), acc_mean = Vec3d::Zero();
    double gyr_peak = 0.0, acc_dev_peak = 0.0;
    const int n_imu = static_cast<int>(measures_.imu_.size());
    for (const auto& imu : measures_.imu_) {
        gyr_mean += imu->angular_velocity;
        acc_mean += imu->linear_acceleration;
        gyr_peak = std::max(gyr_peak, imu->angular_velocity.norm());
        acc_dev_peak = std::max(acc_dev_peak,
                                std::fabs(imu->linear_acceleration.norm() - G_m_s2));
    }
    if (n_imu > 0) {
        gyr_mean /= n_imu;
        acc_mean /= n_imu;
    }
    const bool wheel_low = speed_for_map < zupt_wheel_thresh_;
    const bool imu_static = (gyr_peak < zupt_gyr_thresh_) && (acc_dev_peak < zupt_acc_thresh_);
    // >>>>> [ZUPT] FIX-4 误触发防护：轮速瞬时归零（CAN 抖动/换挡）而状态速度仍大时禁止 ZUPT，
    //       否则行驶中速度被钉 0 → 位姿冻结/跳变 <<<<<
    const bool vel_sane = state_point_.vel_.norm() < zupt_wheel_thresh_ * 4.0;
    if (wheel_low && imu_static && vel_sane) {
        static_hold_cnt_++;
    } else if (speed_for_map > zupt_wheel_thresh_ * release_factor_) {
        static_hold_cnt_ = 0;  // 滞回：速度超阈值×release_factor 才解除
    }
    const bool stationary_now = static_hold_cnt_ >= stationary_frames_;
    if (stationary_now) {
        stationary_cnt_++;
    } else {
        stationary_cnt_ = 0;
    }
    const bool gnss_freeze = stationary_cnt_ >= gnss_freeze_frames_;
    zupt_active_ = enable_zupt_ && stationary_now;
    gyro_last_ = gyr_mean;  // [NHC] 杆臂补偿用（ω×r）
    // >>>>> [ZUPT/NHC] END <<<<<

    // >>>>> [ZUPT] BEGIN 更新序列：静止=零速约束，行驶=轮速（设计文档 §3.1-3.3）<<<<<
    if (zupt_active_) {
        const NavState st0 = kf_.GetX();
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, zupt_var_);  // 复用速度块分支，residual=-v
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[ZUPT] v " << st0.vel_.norm() << " -> " << kf_.GetX().vel_.norm()
                << " m/s (hold=" << static_hold_cnt_ << ")";
    } else if (use_wheel_speed_ && have_new_wheel_speed_) {
        const NavState st_w0 = kf_.GetX();
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, wheel_sigma_ * wheel_sigma_);
        have_new_wheel_speed_ = false;
        const NavState st_w1 = kf_.GetX();
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[WHEEL] obs=" << wheel_speed_obs_ << " m/s"
                << " vel_before=" << st_w0.vel_.norm()
                << " vel_after=" << st_w1.vel_.norm()
                << " dvel=" << (st_w1.vel_ - st_w0.vel_).norm();
    }
    // >>>>> [ZUPT] END <<<<<

    // >>>>> [ZUPT] BEGIN bg 陀螺零偏标定 + 重力对齐（设计文档 §3.4/§3.5）<<<<<
    if (zupt_active_ && n_imu > 0) {
        gyr_acc_sum_ += gyr_mean * n_imu;
        acc_acc_sum_ += acc_mean * n_imu;
        bg_acc_n_ += n_imu;
    } else if (!zupt_active_) {
        gyr_acc_sum_.setZero();
        acc_acc_sum_.setZero();
        bg_acc_n_ = 0;
        bg_calibrated_this_stop_ = false;
    }
    if (zupt_active_ && !bg_calibrated_this_stop_ && bg_acc_n_ >= bg_calib_frames_) {
        const Vec3d gyro_mean = gyr_acc_sum_ / static_cast<double>(bg_acc_n_);
        NavState s_bg = kf_.GetX();
        // >>>>> [ZUPT] FIX-1 bg 符号修正：模型 omega = gyro - bg（types.h get_f），
        //       静止时 ω_true=0 ⇒ bg = +gyro_mean（原 -gyro_mean 会引入 2×零偏虚假旋转）<<<<<
        s_bg.bg_ = gyro_mean;
        kf_.ChangeX(s_bg);

        // >>>>> [ZUPT] FIX-2 P 手术改为 PSD 安全：bg 与他态交叉项清零、对角置目标
        //       （原按 sqrt(target/old) 缩放交叉项会在 old 很小时放大最高 1000×，破坏 PSD）<<<<<
        ESKF::CovType P_bg = kf_.GetP();
        P_bg.block<3, 3>(15, 15) = bg_p_target_ * Eigen::Matrix3d::Identity();
        for (int i = 0; i < ESKF::state_dim_; i++) {
            if (i >= 15 && i < 18) continue;
            P_bg.block<1, 3>(i, 15).setZero();
            P_bg.block<3, 1>(15, i).setZero();
        }
        kf_.ChangeP(P_bg);
        bg_calibrated_this_stop_ = true;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[ZUPT] bg calibrated: " << gyro_mean.transpose() * 180.0 / M_PI << " deg/s";
    }
    if (zupt_active_ && bg_calibrated_this_stop_ && stationary_cnt_ >= gravity_align_frames_) {
        const bool g_ok = kf_.Update(ESKF::ObsType::ACC_AS_GRAVITY, 1.0);
        SYS_LOG(syslog::LogSeverity::INFO, true) << "[ZUPT] gravity align: ok=" << g_ok;
    }
    // >>>>> [ZUPT] END <<<<<
    // >>>>> [ZUPT/NHC] BEGIN 静止判定已上移至更新序列之前（speed_for_map/stationary_now/gnss_freeze 在上方定义）<<<<<
    // >>>>> [ZUPT/NHC] END <<<<<


    // 插值gnss到lidar的末端时间点
    have_new_gnss_ = false;
    ConsumeGnssForFrame(measures_.lidar_end_time_);
    if (use_gnss_ && have_new_gnss_ && !gnss_freeze) {
        // 先用残差简单判断是否更新
        const SO3 gps_R = EulerZYXToSO3(Vec3d(hc_heading_, hc_pitch_, hc_roll_));
        const Vec3d r = (kf_.GetX().rot_.inverse() * gps_R).log(); // 切空间转化为向量
        const double residual_deg = r.norm() * 180.0 / M_PI;
        // ===== [GNSS UPD] 更新前试算诊断（单位：度）=====
        const double RAD2DEG = 180.0 / M_PI;
        const Vec3d lio_rpy = SO3ToEulerZYX_Robust(kf_.GetX().rot_); // (yaw,pitch,roll)
        const ESKF::CovType P_now = kf_.GetP();
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS UPD] lio_yaw=" << lio_rpy[0] * RAD2DEG
                << " lio_pitch=" << lio_rpy[1] * RAD2DEG
                << " lio_roll=" << lio_rpy[2] * RAD2DEG
                << " | rel_hdg=" << hc_heading_ * RAD2DEG
                << " rel_pitch=" << hc_pitch_ * RAD2DEG
                << " rel_roll=" << hc_roll_ * RAD2DEG
                << " | residual=" << residual_deg
                << " rx=" << r[0] * RAD2DEG << " ry=" << r[1] * RAD2DEG << " rz=" << r[2] * RAD2DEG
                << " | P_rot=" << P_now(3, 3) << "," << P_now(4, 4) << "," << P_now(5, 5)
                << " P_prior=" << P_propagated(3, 3) << "," << P_propagated(4, 4) << "," << P_propagated(5, 5);

        ESKF::CovType P_cur = kf_.GetP(); // 获取更新后的协方差矩阵
        P_cur.block<3, 3>(3, 3) = P_propagated.block<3, 3>(3, 3);
        kf_.ChangeP(P_cur); // 更新为lidar更新前旋转协方差矩阵
        const NavState state_before_gps = kf_.GetX();
        const bool gps_accepted = kf_.Update(ESKF::ObsType::GPS, gnss_attitude_noise_ * gnss_attitude_noise_);
        const NavState state_after_gps = kf_.GetX();
        have_new_gnss_ = false;
        SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[GNSS UPD] accepted=" << gps_accepted
            << " gnss_noise=" << gnss_attitude_noise_;
        if (gps_accepted)
        {
            const Vec3d d = (state_before_gps.rot_.inverse() * state_after_gps.rot_).log();
            SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS UPD] applied=" << d.norm() * RAD2DEG
                << " deg (rx=" << d[0] * RAD2DEG
                << " ry=" << d[1] * RAD2DEG
                << " rz=" << d[2] * RAD2DEG << ")";
        }
    } else if (use_gnss_ && have_new_gnss_ && gnss_freeze){
        SYS_LOG(syslog::LogSeverity::INFO, true) << "[GNSS UPD] frozen: stationary_cnt=" << stationary_cnt_;
    }
    
    
    // if (use_gnss_ && have_new_gnss_) {
    //     kf_.Update(ESKF::ObsType::GPS, gnss_attitude_noise_ * gnss_attitude_noise_);
    //     have_new_gnss_ = false;
    //     // std::cout << "use_gnss_  done " << std::endl;
    //     SYS_LOG(syslog::LogSeverity::INFO, false)
    //             << "use_gnss_  done ";
    // }

    // >>>>> [NHC] BEGIN 行驶中非完整约束：车体系 vy=vz=0（设计文档 §4）<<<<<
    // 变坡点门控：俯仰角速度大（变坡/凹凸竖曲线/颠簸）时瞬时 vz≠0 是真实运动，禁用 NHC
    const bool nhc_grade_ok = std::fabs(gyr_mean(1)) < nhc_pitchrate_thresh_;
    // [NHC] FIX-6 转弯门控：偏航角速度大时杆臂假侧向速度（ω×r）大，禁用 NHC（杆臂未标定前的保险）
    const bool nhc_yaw_ok = std::fabs(gyr_mean(2)) < nhc_yawrate_thresh_;
    // 与 1D 轮速互补（1D 管前向大小、NHC 管侧/垂向）；3D 轮速已含 NHC 信息，同开=重复计数
    if (use_nhc_ && use_wheel_1d_ && !zupt_active_ && nhc_grade_ok && nhc_yaw_ok &&
        state_point_.vel_.norm() > nhc_min_speed_) {
        const NavState st_n0 = kf_.GetX();
        const bool nhc_ok = kf_.Update(ESKF::ObsType::NHC, nhc_var_);
        const Vec3d v_body = kf_.GetX().rot_.inverse() * kf_.GetX().vel_;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[NHC] ok=" << nhc_ok << " vy=" << v_body(1) << " vz=" << v_body(2)
                << " (before: " << (st_n0.rot_.inverse() * st_n0.vel_).transpose() << ")";
    }
    // >>>>> [NHC] END <<<<<

    state_point_ = kf_.GetX();

    if (keep_first_imu_estimation_ && frame_num_ < 5 &&
        (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180.0) {
        kf_.ChangeX(old_state);
        state_point_ = old_state;
    }
    // 与原版一致：时间戳在 keep_first 回退判断之后无条件写入，保证回退路径也带扫描末端时刻。
    state_point_.timestamp_ = measures_.lidar_end_time_;
    frame_num_++;

    euler_cur_ = state_point_.rot_;
    pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;
    // 静止时不插入地图
    // const double speed_for_map = use_wheel_speed_ ? std::fabs(wheel_speed_obs_) : state_point_.vel_.norm();
    if (stationary_cnt_ >= gnss_freeze_frames_ ) {
        std::cout << "--------skip mapincrement--------" << std::endl;
        restart_cnt_ = map_skip_frames_;
    } else if (restart_cnt_ > 0) {
        restart_cnt_--;
    } else {
        MapIncremental();
    }
    

    // 局部地图范围剔除：控制 iVox 规模，避免地图无限增长导致每帧处理/发布变慢
    if (map_trim_radius_ > 0.0f && (map_trim_interval_frames_ <= 1 || frame_num_ % map_trim_interval_frames_ == 0)) {
        PointType center;
        center.x = static_cast<float>(pos_lidar_.x());
        center.y = static_cast<float>(pos_lidar_.y());
        center.z = static_cast<float>(pos_lidar_.z());
        ivox_->RemoveFarPoints(center, map_trim_radius_);
    }

    // kf_imu_ = kf_;
    // if (!measures_.imu_.empty()) {
    //     double t = measures_.imu_.back()->timestamp;
    //     for (auto& imu : imu_buffer_) {
    //         double dt = imu->timestamp - t;
    //         kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
    //         t = imu->timestamp;
    //     }
    // }
    // LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
    //          << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;
    // std::cout << "[ odometry ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
    //           << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_ << std::endl;

    // if (SHOULD_LOG(2.0)) {
    //     SYS_LOG(syslog::LogSeverity::INFO, true) 
    //             << "[ odometry ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
    //             << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;
    // }
    SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[ odometry ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
                << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    // ===== [STATE] 每帧轨迹/运动状态（发散定位用；单位：度/米/米每秒）=====
    // 判读：dpos≈0 而 wheel/vel>0 = 位姿冻结；dpos 突增/乱走 = 发散；
    //       effect 骤降 = 匹配失效。
    {
        const Vec3d rpy = SO3ToEulerZYX_Robust(state_point_.rot_);
        const Vec3d dpos = state_point_.pos_ - old_state.pos_;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[STATE] t=" << state_point_.timestamp_
                << " pos=" << state_point_.pos_.x() << "," << state_point_.pos_.y() << "," << state_point_.pos_.z()
                << " yaw=" << rpy[0] * 180.0 / M_PI
                << " pitch=" << rpy[1] * 180.0 / M_PI
                << " roll=" << rpy[2] * 180.0 / M_PI
                << " | dpos=" << dpos.norm()
                << " vel=" << state_point_.vel_.norm()
                << " wheel=" << wheel_speed_obs_
                << " effect=" << effect_feat_num_
                << " lidar_ok=" << lidar_accepted;
    }


    return true;
}

void LaserOdometryCore::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    // 与原版一致：严格串行。iVox 节点是 10 点 FIFO 上限，只有按点索引（扫描）顺序
    // 串行插入，erase(begin) 淘汰的才是"最老帧的点"；par + 锁会使入列顺序随机化，
    // 节点里长期残留跨期老点，转弯时 KNN 取到涂抹点，平面法向质量下降。
    std::for_each(index.begin(), index.end(), [&](const size_t& i) {
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);
        PointType& point_world = scan_down_world_->points[i];

        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector& points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5f) *
                filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5f * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5f * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5f * filter_size_map_min_) {
                point_no_need_downsample.push_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= static_cast<size_t>(fasterlio::NUM_MATCH_POINTS)) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6f) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.push_back(point_world);
            }
        } else {
            points_to_add.push_back(point_world);
        }
    });

    ivox_->AddPoints(points_to_add);
    ivox_->AddPoints(point_no_need_downsample);
}

CloudPtr LaserOdometryCore::GetEffectScan() const {
    CloudPtr effect_scan(new PointCloudType());
    effect_scan->reserve(effect_feat_num_);

    for (int i = 0; i < effect_feat_num_; ++i) {
        PointType p;
        Vec3d p_global(state_point_.rot_ *
                           (state_point_.offset_R_lidar_ * corr_pts_[i].head<3>().cast<double>() +
                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);
        p.x = p_global(0);
        p.y = p_global(1);
        p.z = p_global(2);
        p.intensity = corr_pts_[i][3];
        effect_scan->push_back(p);
    }

    return effect_scan;
}

void LaserOdometryCore::ObsModel(NavState& s, ESKF::CustomObservationModel& obs) {
    int cnt_pts = scan_down_body_->size();

    const Mat3f R_wl = (s.rot_ * s.offset_R_lidar_).matrix().cast<float>();
    const Vec3f t_wl = (s.rot_ * s.offset_t_lidar_ + s.pos_).cast<float>();

    std::for_each(std::execution::par_unseq, scan_down_body_->points.cbegin(), scan_down_body_->points.cend(),
                  [&](const auto& point_body) {
                      size_t i = &point_body - &scan_down_body_->points[0];
                      PointType& point_world = scan_down_world_->points[i];

                      Vec3f p_body = point_body.getVector3fMap();
                      point_world.getVector3fMap() = R_wl * p_body + t_wl;
                      point_world.intensity = point_body.intensity;

                      auto& points_near = nearest_points_[i];

                      // points_near.clear();
                      const bool found = ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                      point_selected_surf_[i] = 
                          points_near.size() >= static_cast<size_t>(fasterlio::MIN_NUM_MATCH_POINTS);
                      if (point_selected_surf_[i]) {
                          point_selected_surf_[i] =
                              esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                      }

                      if (point_selected_surf_[i]) {
                          auto temp = point_world.getVector4fMap();
                          temp[3] = 1.0f;
                          float pd2 = plane_coef_[i].dot(temp);

                          bool valid_corr = p_body.norm() > 81.0f * pd2 * pd2;
                          if (valid_corr) {
                              point_selected_surf_[i] = true;
                              residuals_[i] = pd2;
                          }
                          // 与原版 lightning-lm 严格一致：valid_corr=false 时什么都不做——
                          // point_selected_surf_ 保持 true，residuals_[i] 不重写（Run 中
                          // resize 不清已有元素，即沿用上一帧同索引的小残差/初始 0）。
                          // 这相当于把当前帧的大残差错配"软化"为历史小残差，转弯时阻止错配点
                          // 强力拉动姿态；不能显式置 false 丢弃，否则 effect 数骤降、剩余点权重失衡。
                      }
                  });

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
        obs.valid_ = false;
        return;
    }

    obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
    obs.residual_.resize(effect_feat_num_);

    const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();
    const Vec3f off_t = s.offset_t_lidar_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    for (int i = 0; i < effect_feat_num_; i++) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Mat3f point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = SKEW_SYM_MATRIX(point_this);

        Vec3f norm_vec = corr_norm_[i].head<3>();

        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        if (extrinsic_est_en_) {
            Vec3f B(point_be_crossmat * off_R.transpose() * C);
            obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0], B[1],
                B[2], C[0], C[1], C[2];
        } else {
            obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        obs.residual_(i) = -corr_pts_[i][3];
    }

    std::vector<double> res_sq2;
    for (size_t i = 0; i < static_cast<size_t>(cnt_pts); ++i) {
        if (point_selected_surf_[i]) {
            double r = residuals_[i];
            res_sq2.push_back(r * r);
        }
    }

    std::sort(res_sq2.begin(), res_sq2.end());
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];

    obs.valid_ = true;
}
// [testwheel]
// void LaserOdometryCore::ObsModelWelspeed(NavState& s, ESKF::CustomObservationModel& obs) {
//     obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
//     obs.residual_.resize(3);
//     obs.R_ = wheel_velocity_noise_ * Eigen::MatrixXd::Identity(3, 3);
//     obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

//     Vec3d vel_odom(wheel_speed_obs_, 0.0, 0.0);
//     Vec3d vel_world = s.rot_ * vel_odom;
//     auto tem_v = s.vel_ - vel_world;
//     obs.residual_ = -tem_v;
//     obs.valid_ = true;
// }
void LaserOdometryCore::ObsModelWheelspeed(NavState& s, ESKF::CustomObservationModel& obs) {
    // >>>>> [ZUPT] BEGIN 静止模式输出"速度=0"观测（设计文档 §3.2：z=0, h=v, residual=-v, H=I3）<<<<<
    if (zupt_active_) {
        obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
        obs.residual_.resize(3);
        obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        obs.R_ = zupt_var_ * Eigen::Matrix3d::Identity();
        obs.residual_ = -s.vel_;
        obs.valid_ = true;
        return;
    }
    // >>>>> [ZUPT] END <<<<<
    if (use_wheel_1d_) {
        // 仅约束车体系x方向速度
        const Vec3d x_axis_w = s.rot_.matrix() * Vec3d::UnitX();
        const double v_body_x = x_axis_w.dot(s.vel_);

        // >>>>> [WHEEL-1D] FIX-5 观测输入修正：wheel_v_mean_ 只由被注释掉的
        //       ConsumeWheelSpeedForFrame 产生（恒=0），必须用 FeedWheelSpeed
        //       实时更新的 wheel_speed_obs_，否则 1D 轮速退化成"零速约束"把速度碾平 <<<<<
        double r = wheel_speed_obs_ - v_body_x;
        double w = 1.0;
        const double ar = std::fabs(r);
        if (ar > wheel_huber_delta_) {
            w = wheel_huber_delta_ / ar;
        }
        obs.h_x_ = Eigen::MatrixXd::Zero(1, 3);
        obs.residual_.resize(1);
        obs.R_ = Eigen::MatrixXd::Identity(1, 1);
        obs.residual_(0) = r;
        // >>>>> [WHEEL-1D] FIX 符号修正：约定 residual=z-h、H=∂h/∂v=+x̂_wᵀ <<<<<
        obs.h_x_.block<1, 3>(0, 0)  = x_axis_w.transpose();
        obs.R_(0, 0) = wheel_sigma_ * wheel_sigma_ / w;
        obs.valid_ = true;
    } else {
        // ---------- 旧三维路径，保留用于 A/B 与一键回退 ----------
        obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
        obs.residual_.resize(3);
        obs.R_ = wheel_sigma_ * Eigen::Matrix3d::Identity();
        obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Vec3d vel_odom(wheel_speed_obs_, 0.0, 0.0);
        Vec3d vel_world = s.rot_ * vel_odom;  // 转到了世界坐标系
        obs.residual_ = vel_world - s.vel_;
        obs.valid_ = true;
    }
}

void LaserOdometryCore::ObsModelGPS(NavState& s, ESKF::CustomObservationModel& obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
    obs.residual_.resize(3);
    obs.R_ = gnss_attitude_noise_ * Eigen::MatrixXd::Identity(3, 3);
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    auto RPR = Vec3d(hc_heading_, hc_pitch_, hc_roll_);
    SO3 gps_R = EulerZYXToSO3(RPR);
    obs.residual_ = (s.rot_.inverse() * gps_R).log();

    // const double rx = std::fabs(obs.residual_(0)) * 180.0 / M_PI;
    // const double ry = std::fabs(obs.residual_(1)) * 180.0 / M_PI;
    // const double rz = std::fabs(obs.residual_(2)) * 180.0 / M_PI;
    // if (rx < 0.5) {obs.residual_(0) = 0.0; obs.h_x_(0, 0) = 0.0;}
    // if (ry < 0.3) {obs.residual_(1) = 0.0; obs.h_x_(1, 1) = 0.0;}
    // if (rz < 2.0) {obs.residual_(2) = 0.0; obs.h_x_(2, 2) = 0.0;}


    // obs.residual_(2) = 0.0; // 不约束yaw
    // obs.h_x_(2, 2) = 0.0; // 观测也不约束yaw
       // 残差限制
    // const double max_rad = 30.0 * M_PI / 180.0;
    // const double r_norm = obs.residual_.norm();
    // if (r_norm > max_rad) {
    //     obs.residual_ *= (max_rad / r_norm);
    // }

    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[GNSS OBS] residual(rx,ry,rz)=" << obs.residual_(0) * 180.0 / M_PI << ","
            << obs.residual_(1) * 180.0 / M_PI << "," << obs.residual_(2) * 180.0 / M_PI
            << " deg | norm=" << obs.residual_.norm() * 180.0 / M_PI << " deg";
    obs.valid_ = true;
}

// >>>>> [ZUPT] BEGIN 重力对齐观测（设计文档 §3.5）
// 形式：静止时 get_f 给出 R(acc-ba)+grav=0 ⇒ 观测 R̂·a + grav_ = 0
//   residual = R̂·a + grav_  (m/s²)，H_rot = R̂·hat(a)（右扰动，residual=z-h 约定）
//   该式用状态里的 grav_（S2）作参考，世界系无需水平（坡道停车正确）；
//   yaw 天然不可观（hat(a)·â=0，yaw 方向在 H 零空间），无需手工置零。
// [ZUPT] FIX-3：替换原"水平欧拉角构造目标姿态"版本——那版假设世界 z=铅垂，
//   而本 ESKF 世界系=初始机体系，坡上停车会把姿态拉偏坡度角（起步即跳）。
// >>>>> [ZUPT] END <<<<<
void LaserOdometryCore::ObsModelGravity(NavState& s, ESKF::CustomObservationModel& obs) {
    const Vec3d a = (bg_acc_n_ > 0) ? (acc_acc_sum_ / static_cast<double>(bg_acc_n_))
                                    : Vec3d(0.0, 0.0, G_m_s2);  // 静止期加计均值（body）
    obs.residual_.resize(3);
    obs.residual_ = s.rot_ * a + s.grav_.vec_;          // 静止且姿态正确时 ≈ 0
    obs.h_x_ = s.rot_.matrix() * SO3::hat(a);           // ∂h/∂δθ = R̂·hat(a)，映射到 rot 块
    // 角度噪声 σ_θ 换算为等效加计噪声 g·σ_θ（如 1° → 0.17 m/s²）
    const double sig = gravity_noise_deg_ * M_PI / 180.0 * G_m_s2;
    obs.R_ = sig * sig * Eigen::Matrix3d::Identity();
    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[ZUPT] gravity obs: residual=" << obs.residual_.norm() << " m/s2"
            << " (g=" << G_m_s2 << ")";
    obs.valid_ = true;
}

// >>>>> [NHC] BEGIN 非完整约束观测（设计文档 §4：车体系 vy=vz=0）
// [NHC] FIX-6 杆臂补偿：约束参考点（后轴）与 IMU 相距 r，转弯时 IMU 处车体系
//   假侧向速度 = ω×r（实测 22.6°/s 转弯 vy 达 3.6 m/s）。补偿后残差=-(v_body-ω×r)，
//   ω×r 视为已知数据（陀螺×标定杆臂），雅可比结构不变。
// >>>>> [NHC] END <<<<<
void LaserOdometryCore::ObsModelNHC(NavState& s, ESKF::CustomObservationModel& obs) {
    const Vec3d v_body = s.rot_.inverse() * s.vel_;
    const Vec3d wxb = gyro_last_.cross(nhc_lever_);   // ω×r：转弯时的杆臂假速度（body）
    obs.residual_.resize(2);
    obs.residual_(0) = -(v_body(1) - wxb(1));   // z_y=0 → residual = z - h = -(vy - (ω×r)_y)
    obs.residual_(1) = -(v_body(2) - wxb(2));   // z_z=0

    const Mat3d Rt = s.rot_.inverse().matrix();
    // h_x_(2×12) 布局：前 3 列=rot，3..5=外参（置零），6..8=vel（分支代码贴到 12..14）
    obs.h_x_ = Eigen::MatrixXd::Zero(2, 12);
    // ∂h/∂δθ = E·hat(v_body)（右扰动 R→R·exp(δ)：h=E·exp(-δ)·v_body≈E(v_body+hat(v_body)δ)），取后两行
    const Mat3d hat_vb = SO3::hat(v_body);
    obs.h_x_.block<2, 3>(0, 0) = hat_vb.bottomRows<2>();
    // ∂h/∂v = Rᵀ，取后两行
    obs.h_x_.block<2, 3>(0, 6) = Rt.bottomRows<2>();

    obs.R_ = nhc_var_ * Eigen::Matrix2d::Identity();
    obs.valid_ = true;
}
// >>>>> [NHC] END <<<<<

// [testwheel]
void LaserOdometryCore::FeedWheelSpeed(double timestamp, double speed_kmh, int gear) {
    if (gear == 2) {
        can_speed_double_ = -speed_kmh;
    } else {
        can_speed_double_ = speed_kmh;
    }
    wheel_speed_obs_ = can_speed_double_ / 3.6;
    have_new_wheel_speed_ = true;
}

// void LaserOdometryCore::FeedWheelSpeed(double timestamp, double speed_kmh, int gear) {
//     const double signed_speed_kmh = (gear == 2) ? -speed_kmh : speed_kmh;
    
//     WheelSpeedSample sample;
//     sample.timestamp = timestamp;
//     sample.wheel_speed = signed_speed_kmh / 3.6;
//     // 防止出现时间戳回退
//     if (!wheel_speed_buf_.empty() && timestamp < wheel_speed_buf_.back().timestamp) {
//         wheel_speed_buf_.clear();
//     } 
//     wheel_speed_buf_.push_back(sample);
//     while (wheel_speed_buf_.size() > 2000) {
//         wheel_speed_buf_.pop_front();
//     }
// }

void LaserOdometryCore::ConsumeWheelSpeedForFrame(double t_beg, double t_end) {
    wheel_valid_this_frame_ = false;
    if (wheel_speed_buf_.empty()) {
        return;
    }

    // ① 消费所有时间戳 <= t_end 的样本，记录最后一条 (v0, t0)
    double v0 = 0.0, t0 = -1.0;
    int    n = 0;
    while (!wheel_speed_buf_.empty() &&
           wheel_speed_buf_.front().timestamp <= t_end) {
        const auto s = wheel_speed_buf_.front();
        wheel_speed_buf_.pop_front();
        if (s.timestamp >= t_beg - 1e-6) {
            v0 = s.wheel_speed;
            t0 = s.timestamp;
            ++n;
        }
    }
    if (n == 0 || t0 < 0.0) {
        return;  // 帧区间内没有任何轮速样本，本帧不约束
    }

    // ② 最近值年龄门：t0 离扫描末端太远说明 CAN 已停发，不约束
    if (t_end - t0 > wheel_max_age_) {
        return;
    }

    // ③ 线性插值到 t_end：peek 第一条 > t_end 的样本作为 (v1, t1)，不弹出
    double v_interp = v0;  // 默认退化为最近值
    if (!wheel_speed_buf_.empty()) {
        const double t1 = wheel_speed_buf_.front().timestamp;
        const double v1 = wheel_speed_buf_.front().wheel_speed;
        const double dt = t1 - t0;
        std::cout << "dt: " << dt << std::endl;
        // 前提：t1 紧跟在 t0 之后且跨过 t_end；t1 太远则不插值（用最近值）
        if (t1 > t_end && t1 - t_end <= wheel_max_age_ && dt > 1e-6) {
            const double alpha = (t_end - t0) / dt;         // 0..1
            v_interp = v0 + (v1 - v0) * alpha;              // 线性插值到 t_end
        }
    }

    wheel_v_mean_ = v_interp;   // 现在代表扫描末端的瞬时前向速度
    wheel_valid_this_frame_ = true;
  
}

void LaserOdometryCore::FeedGnssAttitude(double timestamp, double heading_deg, double pitch_deg, double roll_deg, int status, int nsv) {
    GnssAttitudeSample s;
    s.timestamp = timestamp;
    s.heading_deg = heading_deg;
    s.pitch_deg = pitch_deg;
    s.roll_deg = roll_deg;
    s.status = status;
    s.nsv = nsv;
    if (!gnss_buf_.empty() && timestamp < gnss_buf_.back().timestamp) gnss_buf_.clear();  // 时间回退保护
    gnss_buf_.push_back(s);
    while (gnss_buf_.size() > 2000) gnss_buf_.pop_front();

}


bool LaserOdometryCore::ConsumeGnssForFrame(double t_end)  {
    if (gnss_buf_.empty()) return false;

    // ① 队首调整为 <= t_end 的最近样本（更老的丢弃）
    while (gnss_buf_.size() > 1 && gnss_buf_[1].timestamp <= t_end) gnss_buf_.pop_front();
    const GnssAttitudeSample& s0 = gnss_buf_.front();
    if (s0.timestamp > t_end) return false;            // 全部在未来，等下一帧

    // ② 年龄门：队首太老说明 GNSS 停发，本帧不约束
    if (t_end - s0.timestamp > 0.2) return false;

    double h = s0.heading_deg, p = s0.pitch_deg, r = s0.roll_deg;

    // ③ 与下一帧样本线性插值到 t_end（航向走最短圆弧，防 359→1 回绕）
    if (gnss_buf_.size() >= 2)
    {
        const GnssAttitudeSample &s1 = gnss_buf_[1];
        const double dt = s1.timestamp - s0.timestamp;
        if (dt > 1e-6 && s1.timestamp >= t_end)
        {
            const double a = (t_end - s0.timestamp) / dt;
            double dh = s1.heading_deg - s0.heading_deg;
            if (dh > 180.0)
                dh -= 360.0;
            if (dh < -180.0)
                dh += 360.0;
            h = s0.heading_deg + a * dh;
            p = s0.pitch_deg + a * (s1.pitch_deg - s0.pitch_deg);
            r = s0.roll_deg + a * (s1.roll_deg - s0.roll_deg);
        }
    }

    ApplyGnssRaw(h, p, r);   // ④ 复用原基准/滑窗/日志逻辑
    return true;
}





// 代替原来的FeedGnssAttitude，直接应用GNSS原始数据
void LaserOdometryCore::ApplyGnssRaw(double heading_deg, double pitch_deg, double roll_deg) {
    // >>>>> [GNSS-REZERO] BEGIN —— 停车期 GNSS 基准重零（回退：删 BEGIN/END 之间改动，或配置 gnss_rezero: false）<<<<<
    // 原始角（沿用原符号约定），pitch 滑窗只调用一次，基准与 hc_ 用同一滤波值
    const double h_rad = -heading_deg * M_PI / 180.0;
    const double p_rad = -filter(pitch_deg * M_PI / 180.0);
    const double r_rad = roll_deg * M_PI / 180.0;
    // >>>>> [GNSS-REZERO] END <<<<<

    if (first_gnss_) {
        // >>>>> [GNSS-REZERO] BEGIN（等价改写：原三行赋值 = 下列 h_rad/p_rad/r_rad）<<<<<
        hc_heading_b_ = h_rad;
        hc_pitch_b_ = p_rad;
        hc_roll_b_ = r_rad;
        // >>>>> [GNSS-REZERO] END <<<<<
        first_gnss_ = false;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS FIRST] bias hdg=" << hc_heading_b_ * 180.0 / M_PI
                << " pitch=" << hc_pitch_b_ * 180.0 / M_PI
                << " roll=" << hc_roll_b_ * 180.0 / M_PI << " deg"
                << " | raw hdg=" << heading_deg << " pitch=" << pitch_deg << " roll=" << roll_deg;
        return;
    }

    // >>>>> [GNSS-REZERO] BEGIN —— 静止期：基准重锚到当前 LIO 姿态，残差恒 0、起步无台阶 <<<<<
    // 原理：车不动 ⇒ 姿态恒定。GNSS 相对姿态强制等于当前 LIO 姿态（hc_* = SO3ToEulerZYX_Robust(R̂)），
    //       反解基准 b = signed(raw) - target，使 hc = signed(raw) - b ≡ target（残差=0）。
    //       停车期 INS 漂移不进状态；基准同步重设 ⇒ 起步后 hc_ 从 LIO 姿态连续演化。
    if (gnss_rezero_enable_ && stationary_cnt_ >= gnss_freeze_frames_) {
        const Vec3d ypr = SO3ToEulerZYX_Robust(kf_.GetX().rot_);   // (yaw, pitch, roll)
        hc_heading_b_ = h_rad - ypr[0];
        hc_pitch_b_ = p_rad - ypr[1];
        hc_roll_b_ = r_rad - ypr[2];

        hc_heading_ = ypr[0];
        hc_pitch_ = ypr[1];
        hc_roll_ = ypr[2];
        have_new_gnss_ = true;   // 数据照常流转；残差≈0，GNSS 更新为无操作
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS REZERO] baseline->LIO ypr=("
                << ypr[0] * 180.0 / M_PI << "," << ypr[1] * 180.0 / M_PI << ","
                << ypr[2] * 180.0 / M_PI << ") deg";
        return;
    }
    // >>>>> [GNSS-REZERO] END <<<<<

    // 行驶路径（原 else 分支原样）
    hc_heading_ = h_rad - hc_heading_b_;  // rad
    hc_pitch_ = p_rad - hc_pitch_b_;
    hc_roll_ = r_rad - hc_roll_b_;
    have_new_gnss_ = true;
    // std::cout << "status: " << status << " nsv: " << nsv << std::endl;
    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[GNSS RAW] hdg=" << heading_deg << " pitch=" << pitch_deg << " roll=" << roll_deg
            << " deg "
            << " | rel_hdg=" << hc_heading_ * 180.0 / M_PI
            << " rel_pitch=" << hc_pitch_ * 180.0 / M_PI
            << " rel_roll=" << hc_roll_ * 180.0 / M_PI << " deg";
}



double LaserOdometryCore::filter(double new_value) {
    window_.push_back(new_value);
    sum_ += new_value;
    if (static_cast<int>(window_.size()) > window_size_) {
        sum_ -= window_.front();
        window_.pop_front();
    }
    return sum_ / window_.size();
}

const NavState& LaserOdometryCore::GetIMUState() const {
    if (p_imu_->IsIMUInited()) {
        return kf_imu_.GetX();
    } else {
        static NavState s;
        s.pose_is_ok_ = false;
        return s;
    }
}

}  // namespace slam_odometry