#include "lio_odometry/core/voxelmap/point_covariance.h"
#include "lio_odometry/core/laser_odometry_core.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <execution>
#include <fstream>

#include "lio_odometry/core/eskf.h"
#include <chrono>
#include "syslog/logger.h"

namespace slam_odometry {

namespace fasterlio {
int NUM_MAX_ITERATIONS = 4;
float ESTI_PLANE_THRESHOLD = 0.1f;
float lidar_time_interval = 0.1;
}  // namespace fasterlio

// 每行号独立的节流器（替代 ROS_INFO_THROTTLE）
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

    Vec3f normvec;

    if (point.size() == static_cast<size_t>(fasterlio::NUM_MATCH_POINTS)) {
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

    float n = normvec.norm();
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
    const auto solver = YAML::LoadFile(config_yaml)["fasterlio"];
    eskf_options.use_aa_ = use_aa_;
    eskf_options.use_pose_converge_for_lidar_ = solver["use_pose_convergence"].as<bool>(false);
    eskf_options.lidar_converged_times_required_ = solver["converged_times_required"].as<int>(1);
    eskf_options.lidar_rot_converge_deg_ = solver["rotation_convergence_deg"].as<double>(0.01);
    eskf_options.lidar_pos_converge_m_ = solver["position_convergence_m"].as<double>(0.00015);
    if (eskf_options.lidar_converged_times_required_ < 1 ||
        eskf_options.lidar_rot_converge_deg_ <= 0 || eskf_options.lidar_pos_converge_m_ <= 0) return false;
    kf_.Init(eskf_options);
    std::cout << "[voxelmap] AA=" << use_aa_ << " adaptive_R=" << use_adaptive_r_
              << " pose_convergence=" << eskf_options.use_pose_converge_for_lidar_
              << " required_convergences=" << eskf_options.lidar_converged_times_required_
              << " neighbor_radius=" << vm_config_.neighbor_search_radius_
              << " matching_threads=" << vm_config_.matching_threads_ << std::endl;

    return true;
}

bool LaserOdometryCore::LoadParamsFromYAML(const std::string& yaml_file) {
    int lidar_type;
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

        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

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
        lidar_update_r_ = yaml["fasterlio"]["lidar_update_r"].as<double>(0.001);
        r_base_ = yaml["fasterlio"]["r_base"].as<double>(0.001);
        use_adaptive_r_ = yaml["fasterlio"]["use_adaptive_r"].as<bool>(true);
        min_effective_points_ = yaml["fasterlio"]["min_effective_points"].as<int>(20);
        min_effective_ratio_ = yaml["fasterlio"]["min_effective_ratio"].as<double>(0.05);
        if (!std::isfinite(r_base_) || r_base_ <= 0 || min_effective_points_ < 3 ||
            min_effective_ratio_ < 0 || min_effective_ratio_ > 1) return false;
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>(false);

        // ===== VoxelMap 配置加载 =====
        vm_config_.max_voxel_size_ = yaml["voxel_map"]["max_voxel_size"].as<double>(3.0);
        vm_config_.max_layer_ = yaml["voxel_map"]["max_layer"].as<int>(3);
        vm_config_.layer_init_num_ = yaml["voxel_map"]["layer_init_num"].as<std::vector<int>>(std::vector<int>{5, 5, 5, 5, 5});
        vm_config_.max_points_num_ = yaml["voxel_map"]["max_points_num"].as<int>(100);
        vm_config_.max_root_points_size_ = yaml["voxel_map"]["max_root_points_size"].as<int>(2000);
        vm_config_.planner_threshold_ = yaml["voxel_map"]["planner_threshold"].as<double>(0.01);
        vm_config_.beam_err_ = yaml["voxel_map"]["beam_err"].as<double>(0.02);
        vm_config_.dept_err_ = yaml["voxel_map"]["dept_err"].as<double>(0.05);
        vm_config_.update_size_threshold_ = yaml["voxel_map"]["update_size_threshold"].as<int>(5);
        vm_config_.sigma_num_ = yaml["voxel_map"]["sigma_num"].as<double>(3.0);
        vm_config_.map_sliding_en_ = yaml["voxel_map"]["map_sliding_en"].as<bool>(true);
        vm_config_.sliding_thresh_ = yaml["voxel_map"]["sliding_thresh"].as<double>(20.0);
        vm_config_.half_map_size_ = yaml["voxel_map"]["half_map_size"].as<int>(100);

        // 新增：LRU 和 reinit 配置
        vm_config_.lru_capacity_ = yaml["voxel_map"]["lru_capacity"].as<int>(500);
        vm_config_.reinit_size_vec_ = yaml["voxel_map"]["reinit_size_vec"].as<std::vector<int>>(std::vector<int>{100, 200, 300, 500, 700, 1000});
        vm_config_.stop_reinit_threshold_ = yaml["voxel_map"]["stop_reinit_threshold"].as<int>(1000);
        vm_config_.update_inlier_threshold_ = yaml["voxel_map"]["update_inlier_threshold"].as<double>(0.1);

        // ===== RANSAC 配置读取 =====
        vm_config_.use_ransac_ = yaml["voxel_map"]["use_ransac"].as<bool>(false);
        vm_config_.ransac_max_iter_ = yaml["voxel_map"]["ransac_max_iter"].as<int>(10);
        vm_config_.ransac_sample_num_ = yaml["voxel_map"]["ransac_sample_num"].as<int>(4);
        vm_config_.ransac_inlier_threshold_ = yaml["voxel_map"]["ransac_inlier_threshold"].as<double>(0.1);
        vm_config_.ransac_isplane_p_threshold_ = yaml["voxel_map"]["ransac_isplane_p_threshold"].as<double>(0.8);
        vm_config_.valid_check_max_layer_ = yaml["voxel_map"]["valid_check_max_layer"].as<int>(0);
        vm_config_.valid_check_min_points_size_ = yaml["voxel_map"]["valid_check_min_points_size"].as<int>(5);
        vm_config_.valid_check_resolution_ = yaml["voxel_map"]["valid_check_resolution"].as<int>(4);

        vm_config_.neighbor_search_on_failure_ = yaml["voxel_map"]["neighbor_search_on_failure"].as<bool>(true);
        vm_config_.matching_threads_ = yaml["voxel_map"]["matching_threads"].as<int>(4);
        vm_config_.neighbor_search_radius_ = yaml["voxel_map"]["neighbor_search_radius"].as<int>(1);
        vm_config_.sample_seed_ = yaml["voxel_map"]["sample_seed"].as<int>(42);
        if (vm_config_.matching_threads_ < 1 || vm_config_.lru_capacity_ < 1 ||
            vm_config_.update_size_threshold_ < 1 || vm_config_.sigma_num_ <= 0 ||
            vm_config_.beam_err_ <= 0 || vm_config_.dept_err_ < 0 ||
            vm_config_.max_points_num_ < 3 || vm_config_.max_root_points_size_ < 3 ||
            vm_config_.ransac_max_iter_ < 1 || vm_config_.ransac_sample_num_ < 3 ||
            vm_config_.ransac_isplane_p_threshold_ <= 0 || vm_config_.ransac_isplane_p_threshold_ >= 1 ||
            vm_config_.max_layer_ < 0 || vm_config_.max_layer_ >= int(vm_config_.layer_init_num_.size()) ||
            vm_config_.max_voxel_size_ <= 0 || vm_config_.neighbor_search_radius_ < 0 ||
            vm_config_.neighbor_search_radius_ > 2 || vm_config_.valid_check_resolution_ < 1 ||
            !std::is_sorted(vm_config_.reinit_size_vec_.begin(), vm_config_.reinit_size_vec_.end()) ||
            std::any_of(vm_config_.layer_init_num_.begin(), vm_config_.layer_init_num_.end(), [](int n){return n<3;}) ||
            std::any_of(vm_config_.reinit_size_vec_.begin(), vm_config_.reinit_size_vec_.end(), [](int n){return n<1;})) return false;
        // 初始化 VoxelMapManager
        voxel_map_mgr_.Init(vm_config_);

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
    voxel_map_mgr_.Init(vm_config_);
    pv_list_.clear();
    ptpl_list_.clear();
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
    // ===== [TIME_LOG] Run() 总耗时 =====
    auto t_run_start = std::chrono::high_resolution_clock::now();

    if (!SyncPackages()) {
        return false;
    }

    // ===== [TIME_LOG] IMU 处理耗时 =====
    auto t_imu_start = std::chrono::high_resolution_clock::now();

    // 每帧处理前重置输出点云：IMU 空包/去畸变跳过时 Process 不会写 scan_undistort_，
    // 必须保证下方空检查看到的是"本帧的空"而不是上一帧残留的点云，否则会用旧点云
    // 重复更新并把同一批点再次插入地图。
    scan_undistort_.reset(new PointCloudType());
    p_imu_->Process(measures_, kf_, scan_undistort_);

    auto t_imu_end = std::chrono::high_resolution_clock::now();
    double ms_imu = std::chrono::duration<double, std::milli>(t_imu_end - t_imu_start).count();

    if (!scan_undistort_ || scan_undistort_->empty()) {
        return false;
    }
    // 如果使用轮速，先消费轮速
    // if (use_wheel_speed_) {
    //     ConsumeWheelSpeedForFrame(measures_.lidar_begin_time_, measures_.lidar_end_time_);
    // }

    if (flg_first_scan_) {
        state_point_ = kf_.GetX();

        // 首帧降采样（与后续帧保持一致）
        voxel_scan_.setInputCloud(scan_undistort_);
        voxel_scan_.filter(*scan_down_body_);

        int cur_pts = scan_down_body_->size();
        if (cur_pts < 5) {
            return false;
        }
        scan_down_world_->resize(cur_pts);

        // 从 ESKF 获取状态协方差（用于协方差传播）

        const Mat3d off_R = state_point_.offset_R_lidar_.matrix();
        const Vec3d off_t = state_point_.offset_t_lidar_;
        const Mat3d R_wl = state_point_.rot_.matrix();
        const Vec3d t_wl = state_point_.pos_;

        // 构建 pv_list_ 用于首帧建图
        pv_list_.resize(cur_pts);
        for (int i = 0; i < cur_pts; i++) {
            slam_odometry::pointWithVar &pv = pv_list_[i];
            const auto &pt = scan_down_body_->points[i];
            pv.point_b << pt.x, pt.y, pt.z;

            // 计算 body 协方差
            // 参数顺序: (point, ranging_cov, angle_cov, body_var)
            // beam_err = 测距误差(米), dept_err = 角度误差(度)
            slam_odometry::VoxelMapManagerFASTLIVO::CalcBodyCov(
                pv.point_b, vm_config_.beam_err_, vm_config_.dept_err_, pv.body_var);

            // 计算 IMU 坐标系下的点（含外参）
            pv.point_i = off_R * pv.point_b + off_t;

            // 变换到世界坐标系
            Vec3d p_global = R_wl * pv.point_i + t_wl;
            pv.point_w = p_global;
            scan_down_world_->points[i].x = p_global(0);
            scan_down_world_->points[i].y = p_global(1);
            scan_down_world_->points[i].z = p_global(2);

            // 世界坐标系协方差（完整传播，含状态协方差）
            pv.var_nostate = SensorCovarianceWorld(state_point_, pv.body_var);
            pv.var = PointCovarianceWorld(state_point_, kf_.GetP(), pv.point_b, pv.body_var, extrinsic_est_en_);
        }

        // R-VoxelMap 首帧建图
        voxel_map_mgr_.UpdateMap(pv_list_);

        // 首帧也执行 ESKF 更新（参考 FAST-LIVO2 的 StateEstimation）
        // kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_);
        // state_point_ = kf_.GetX();
        pv_list_.clear();

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        euler_cur_ = state_point_.rot_;
        pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;

        // 用 ESKF 更新后的状态重新建图
        // MapIncremental();

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

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        std::cout << "skip lidar " << std::endl;
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    // ===== [TIME_LOG] 降采样耗时 =====
    auto t_downsample_start = std::chrono::high_resolution_clock::now();

    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    auto t_downsample_end = std::chrono::high_resolution_clock::now();
    double ms_downsample = std::chrono::duration<double, std::milli>(t_downsample_end - t_downsample_start).count();

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        return false;
    }
    scan_down_world_->resize(cur_pts);

    auto old_state = kf_.GetX();
    const auto old_cov = kf_.GetP();
    const ESKF::CovType P_propagated = kf_.GetP(); // 获取更新前的协方差矩阵

    // ===== [TIME_LOG] ESKF Update 耗时（包含 ObsModel + BuildResidualListOMP） =====
    auto t_eskf_start = std::chrono::high_resolution_clock::now();

    // if (!kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_)) {
    //     std::cerr << "[voxelmap] LiDAR update rejected; map unchanged" << std::endl;
    //     return false;
    // }
    const bool lidar_accepted = kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_);
    // [LIDAR] 匹配质量：iters=IEKF迭代数, res=残差收敛比(1=无改善)
    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[LIDAR] ok=" << lidar_accepted
            << " iters=" << kf_.GetIterations()
            << " res=" << kf_.GetFinalRes()
            << " dpos=" << (kf_.GetX().pos_ - old_state.pos_).norm()
            << " dyaw=" << (old_state.rot_.inverse() * kf_.GetX().rot_).log().norm() * 180.0 / M_PI;

    auto t_eskf_end = std::chrono::high_resolution_clock::now();
    double ms_eskf = std::chrono::duration<double, std::milli>(t_eskf_end - t_eskf_start).count();

    if (use_wheel_speed_ && have_new_wheel_speed_) {
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
    if (use_gnss_ && have_new_gnss_) {
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
        if (std::abs(residual_deg) > 180.0) {
            SYS_LOG(syslog::LogSeverity::WARNING, true)
                    << "[GNSS UPD] residual=" << residual_deg << " deg > 180, skip update";
        } else {
            ESKF::CovType P_cur = kf_.GetP(); // 获取更新后的协方差矩阵
            P_cur.block<3,3>(3,3) = P_propagated.block<3,3>(3,3);
            kf_.ChangeP(P_cur); // 更新为lidar更新前旋转协方差矩阵
            const NavState state_before_gps = kf_.GetX();
            const bool gps_accepted = kf_.Update(ESKF::ObsType::GPS, gnss_attitude_noise_ * gnss_attitude_noise_);
            const NavState state_after_gps = kf_.GetX();
            have_new_gnss_ = false;
            SYS_LOG(syslog::LogSeverity::INFO, true)
                    << "[GNSS UPD] accepted=" << gps_accepted
                    << " gnss_noise=" << gnss_attitude_noise_;
            if (gps_accepted) {
                const Vec3d d = (state_before_gps.rot_.inverse() * state_after_gps.rot_).log();
                SYS_LOG(syslog::LogSeverity::INFO, true)
                        << "[GNSS UPD] applied=" << d.norm() * RAD2DEG
                        << " deg (rx=" << d[0] * RAD2DEG
                        << " ry=" << d[1] * RAD2DEG
                        << " rz=" << d[2] * RAD2DEG << ")";
            }
        }
    }

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    if (keep_first_imu_estimation_ && frame_num_ < 5 &&
        (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180.0) {
        kf_.ChangeX(old_state);
        kf_.ChangeP(old_cov);
        return false;
    }
    frame_num_++;

    euler_cur_ = state_point_.rot_;
    pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;

    // ===== [TIME_LOG] MapIncremental 耗时 =====
    auto t_map_start = std::chrono::high_resolution_clock::now();

    MapIncremental();

    auto t_map_end = std::chrono::high_resolution_clock::now();
    double ms_map = std::chrono::duration<double, std::milli>(t_map_end - t_map_start).count();

    // kf_imu_ 已删除（GetIMUState 从未被调用，是死代码）

    // ===== [TIME_LOG] Run() 总耗时 =====
    auto t_run_end = std::chrono::high_resolution_clock::now();
    double ms_run_total = std::chrono::duration<double, std::milli>(t_run_end - t_run_start).count();

    // ===== [TIME_LOG] 各阶段耗时汇总 =====
    // std::cout << "[ TIME ] IMU: " << std::fixed << std::setprecision(1) << ms_imu << "ms"
    //           << " | downsample: " << ms_downsample << "ms"
    //           << " | ESKF: " << ms_eskf << "ms"
    //           << " | MapUpdate: " << ms_map << "ms"
    //           << " | Run_total: " << ms_run_total << "ms"
    //           << " | pts: " << cur_pts
    //           << " effect: " << effect_feat_num_
    //           << " voxels: " << voxel_map_mgr_.GetTotalPointsNum()
    //           << std::endl;

    if (SHOULD_LOG(2.0)) {

        std::cout << "[ TIME ] IMU: " << std::fixed << std::setprecision(1) << ms_imu << "ms"
              << " | downsample: " << ms_downsample << "ms"
              << " | ESKF: " << ms_eskf << "ms"
              << " | MapUpdate: " << ms_map << "ms"
              << " | Run_total: " << ms_run_total << "ms"
              << " | pts: " << cur_pts
              << " effect: " << effect_feat_num_
              << " voxels: " << voxel_map_mgr_.GetTotalPointsNum()
              << std::endl;
    }

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
    // R-VoxelMap 增量更新
    // pv_list_ 可能来自两种途径：
    //   1) ObsModel 首次 ESKF 迭代时填充（使用当时的预测位姿）
    //   2) 本函数因 skip_lidar 等情况为空，需重新计算
    // 无论哪种情况，都必须用最终修正后的 state_point_ 重新计算世界坐标

    size_t cur_pts = scan_down_body_->size();
    if (cur_pts == 0) return;

    const Mat3d off_R = state_point_.offset_R_lidar_.matrix();
    const Vec3d off_t = state_point_.offset_t_lidar_;
    const Mat3d R_wl = state_point_.rot_.matrix();
    const Vec3d t_wl = state_point_.pos_;

    // 从 ESKF 获取最终状态协方差（用于协方差传播，与 ObsModel 一致）


    if (pv_list_.empty()) {
        // ObsModel 未被调用（skip_lidar 等情况）：从头构建 pv_list_
        pv_list_.resize(cur_pts);

        for (size_t i = 0; i < cur_pts; i++) {
            Vec3d point_body(scan_down_body_->points[i].x,
                           scan_down_body_->points[i].y,
                           scan_down_body_->points[i].z);

            slam_odometry::pointWithVar &pv = pv_list_[i];
            pv.point_b = point_body;
            pv.point_i = off_R * point_body + off_t;

            // 计算 body 协方差
            slam_odometry::VoxelMapManagerFASTLIVO::CalcBodyCov(
                point_body, vm_config_.beam_err_, vm_config_.dept_err_, pv.body_var);

            // 变换到世界坐标系
            Vec3d p_global = R_wl * pv.point_i + t_wl;
            pv.point_w = p_global;
            scan_down_world_->points[i].x = p_global(0);
            scan_down_world_->points[i].y = p_global(1);
            scan_down_world_->points[i].z = p_global(2);

            // 世界坐标系协方差（完整传播，含状态协方差）
            pv.var_nostate = SensorCovarianceWorld(state_point_, pv.body_var);
            pv.var = PointCovarianceWorld(state_point_, kf_.GetP(), pv.point_b, pv.body_var, extrinsic_est_en_);
        }
    } else {
        // pv_list_ 在 ObsModel 中已填充，用最终修正位姿重算世界坐标和协方差
        for (size_t i = 0; i < std::min(pv_list_.size(), cur_pts); i++) {
            slam_odometry::pointWithVar &pv = pv_list_[i];
            Vec3d point_i = off_R * pv.point_b + off_t;
            pv.point_i = point_i;
            Vec3d p_global = R_wl * point_i + t_wl;
            pv.point_w = p_global;

            // 更新世界坐标系协方差（完整传播）
            pv.var_nostate = SensorCovarianceWorld(state_point_, pv.body_var);
            pv.var = PointCovarianceWorld(state_point_, kf_.GetP(), pv.point_b, pv.body_var, extrinsic_est_en_);

            // 同步 scan_down_world_（用于可视化等）
            scan_down_world_->points[i].x = p_global(0);
            scan_down_world_->points[i].y = p_global(1);
            scan_down_world_->points[i].z = p_global(2);
        }
    }

    // R-VoxelMap 增量更新（UpdateMap 内部已有详细耗时日志）
    voxel_map_mgr_.UpdateMap(pv_list_);

    if (vm_config_.map_sliding_en_) voxel_map_mgr_.MapSliding(state_point_.pos_);

    // 清空 pv_list_，下一帧重新计算
    pv_list_.clear();
}

CloudPtr LaserOdometryCore::GetEffectScan() const {
    CloudPtr effect_scan(new PointCloudType());
    effect_scan->reserve(effect_feat_num_);

    for (int i = 0; i < effect_feat_num_; ++i) {
        const auto &ptpl = ptpl_list_[i];
        PointType p;
        Vec3d p_global(state_point_.rot_ *
                           (state_point_.offset_R_lidar_ * ptpl.point_b_ +
                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);
        p.x = p_global(0);
        p.y = p_global(1);
        p.z = p_global(2);
        p.intensity = ptpl.dis_to_plane_;
        effect_scan->push_back(p);
    }

    return effect_scan;
}

void LaserOdometryCore::ObsModel(NavState& s, ESKF::CustomObservationModel& obs) {
    effect_feat_num_ = 0;
    // ===== [TIME_LOG] ObsModel 整体耗时 =====
    auto t_obs_start = std::chrono::high_resolution_clock::now();

    int cnt_pts = scan_down_body_->size();
    // 与原版/ivox 版一致：点数下限在 Run() 层检查（cur_pts < 5），此处不再额外拒绝。

    // ===== [TIME_LOG] pv_list_ 重建 + 协方差传播耗时 =====
    auto t_pvlist_start = std::chrono::high_resolution_clock::now();

    // ===== 每轮 IEKF 迭代重新构建 pv_list_（用当前状态 s） =====
    pv_list_.resize(cnt_pts);

    const Mat3d off_R = s.offset_R_lidar_.matrix();
    const Vec3d off_t = s.offset_t_lidar_;
    const Mat3d R_wl = s.rot_.matrix();
    const Vec3d t_wl = s.pos_;

    // 从 ESKF 获取当前状态协方差（用于协方差传播）


    for (int i = 0; i < cnt_pts; i++) {
        Vec3d point_body(scan_down_body_->points[i].x,
                       scan_down_body_->points[i].y,
                       scan_down_body_->points[i].z);

        // 1) body 协方差（仅传感器噪声）
        Mat3d body_var;
        slam_odometry::VoxelMapManagerFASTLIVO::CalcBodyCov(
            point_body, vm_config_.beam_err_, vm_config_.dept_err_, body_var);

        // 2) 叉乘矩阵（点云的叉乘矩阵，用于旋转雅可比）
        Mat3d crossmat;
        crossmat << 0.0, -point_body[2], point_body[1],
                    point_body[2], 0.0, -point_body[0],
                   -point_body[1], point_body[0], 0.0;

        // 3) 变换到 IMU 坐标系
        Vec3d point_imu = off_R * point_body + off_t;

        // 4) 变换到世界坐标系
        Vec3d point_world = R_wl * point_imu + t_wl;

        // 5) 世界坐标系总协方差（完整传播，含状态不确定性）
        //    传感器噪声传播: term1 = R * body_var * R^T
        const Mat3d term1 = SensorCovarianceWorld(s, body_var);
        const Mat3d total_var = PointCovarianceWorld(s, kf_.GetObservationPriorP(), point_body, body_var, extrinsic_est_en_);

        // 6) 填充 pointWithVar
        slam_odometry::pointWithVar &pv = pv_list_[i];
        pv.point_b = point_body;
        pv.point_i = point_imu;
        pv.point_w = point_world;
        pv.body_var = body_var;
        pv.point_crossmat = crossmat;
        pv.var = total_var;
        pv.var_nostate = term1;

        // 同步 scan_down_world_（用于可视化）
        scan_down_world_->points[i].x = point_world(0);
        scan_down_world_->points[i].y = point_world(1);
        scan_down_world_->points[i].z = point_world(2);
    }

    auto t_pvlist_end = std::chrono::high_resolution_clock::now();
    double ms_pvlist = std::chrono::duration<double, std::milli>(t_pvlist_end - t_pvlist_start).count();

    // ===== 每轮迭代重新搜索地图对应关系 =====
    // 这与 R-VoxelMap/FAST-LIVO2 一致：每次迭代用更新后的状态重新匹配
    ptpl_list_.clear();
    voxel_map_mgr_.BuildResidualListOMP(pv_list_, ptpl_list_);

    // ===== [TIME_LOG] Jacobian 构建耗时 =====
    auto t_jacobian_start = std::chrono::high_resolution_clock::now();

    // ===== 构建 ESKF 观测 =====
    int effect_num = ptpl_list_.size();
    // 与原版 lightning-lm / ivox 版一致：唯一门控是至少 1 个有效约束。
    // voxel 地图侧已有平面判定 + 3σ Mahalanobis 门控，不再叠加 20 点/5% 硬门，
    // 避免转弯预测偏差大时整帧 invalid 触发滤波器丢更新。
    if (effect_num < 1) {
        obs.valid_ = false;
        return;
    }

    obs.h_x_ = Eigen::MatrixXd::Zero(effect_num, 12);
    obs.residual_.resize(effect_num);
    obs.variance_.resize(effect_num);
    obs.valid_ = true;

    for (int idx = 0; idx < effect_num; idx++) {
        const auto &ptpl = ptpl_list_[idx];

        // 使用当前状态重新计算点在世界坐标系下的坐标
        Vec3d point_body = ptpl.point_b_;
        Vec3d point_imu = off_R * point_body + off_t;
        Vec3d point_world = R_wl * point_imu + t_wl;

        // 法向量（来自地图平面）
        Vec3d norm_vec = ptpl.normal_;

        // ===== 残差 = 点到平面距离 =====
        obs.residual_(idx) = -(norm_vec.dot(point_world) + ptpl.d_);

        // ===== 自适应 R 矩阵（FAST-LIVO2 风格） =====
        // R_i = r_base + sigma_l + n^T * var * n
        // 其中：
        // - r_base: 观测噪声基线
        // - sigma_l: 平面不确定性（从 plane_var_ 传播）
        // - n^T * var * n: 点不确定性在法向量上的投影

        // 1) 计算 sigma_l（平面不确定性传播）
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl.center_;
        J_nq.block<1, 3>(0, 3) = -norm_vec;
        double sigma_l = J_nq * ptpl.plane_var_ * J_nq.transpose();

        // 2) 计算点不确定性在法向量上的投影
        // 使用 body_cov_ 从 PointToPlane，并传播到世界坐标系
        Mat3d point_var = SensorCovarianceWorld(s, ptpl.body_cov_);
        double sigma_p = norm_vec.transpose() * point_var * norm_vec;

        const double variance = use_adaptive_r_ ? r_base_ + sigma_l + sigma_p : lidar_update_r_;
        if (!std::isfinite(variance) || variance <= 0.0) { obs.valid_ = false; return; }
        obs.variance_(idx) = variance;

        // ===== Jacobian H =====
        // h_x = [d_res/d_pos, d_res/d_rot, d_res/d_off_R, d_res/d_off_t]
        //       (3)          (3)          (3)             (3)
        obs.h_x_.block<1, 3>(idx, 0) = norm_vec.transpose();                                      // d/d(pos)
        obs.h_x_.block<1, 3>(idx, 3) = point_imu.cross(R_wl.transpose() * norm_vec).transpose();  // d/d(rot)

        if (extrinsic_est_en_) {
            // 从 point_b 计算 crossmat
            Mat3d crossmat;
            crossmat << 0.0, -point_body[2], point_body[1],
                        point_body[2], 0.0, -point_body[0],
                       -point_body[1], point_body[0], 0.0;
            obs.h_x_.block<1, 3>(idx, 6) = (point_body.cross(off_R.transpose() * R_wl.transpose() * norm_vec)).transpose(); // d/d(off_R)
            obs.h_x_.block<1, 3>(idx, 9) = (R_wl.transpose() * norm_vec).transpose();             // d/d(off_t)
        } else {
            obs.h_x_.block<1, 3>(idx, 6).setZero();
            obs.h_x_.block<1, 3>(idx, 9).setZero();
        }
    }

    // 残差统计与原版/ivox 版一致：平方残差排序后的中位数作为 mean_，
    // 末位（最大平方残差）作为 max_。中位数对转弯时少量错配大残差不敏感，
    // 也保证 AA 残差回升判定的口径与原版相同。
    std::vector<double> res_sq2(effect_num);
    for (int i = 0; i < effect_num; i++) {
        res_sq2[i] = obs.residual_(i) * obs.residual_(i);
    }
    std::sort(res_sq2.begin(), res_sq2.end());
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    obs.lidar_residual_max_ = res_sq2.back();
    effect_feat_num_ = effect_num;

    // ===== [TIME_LOG] Jacobian 构建耗时 =====
    auto t_jacobian_end = std::chrono::high_resolution_clock::now();
    double ms_jacobian = std::chrono::duration<double, std::milli>(t_jacobian_end - t_jacobian_start).count();

    // ===== [TIME_LOG] ObsModel 整体耗时 =====
    auto t_obs_end = std::chrono::high_resolution_clock::now();
    double ms_obs = std::chrono::duration<double, std::milli>(t_obs_end - t_obs_start).count();

    // std::cout << "    [ ObsModel ] pvlist: " << std::fixed << std::setprecision(1) << ms_pvlist << "ms"
    //           << " | jacobian: " << ms_jacobian << "ms"
    //           << " | total: " << ms_obs << "ms"
    //           << " | effect: " << effect_num << "/" << cnt_pts
    //           << std::endl;
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
    if (use_wheel_1d_) {
        // 仅约束车体系x方向速度
        const Vec3d x_axis_w = s.rot_.matrix() * Vec3d::UnitX();
        const double v_body_x = x_axis_w.dot(s.vel_);

        double r = wheel_v_mean_ - v_body_x;
        double w = 1.0;
        const double ar = std::fabs(r);
        if (ar > wheel_huber_delta_) {
            w = wheel_huber_delta_ / ar;
        }
        obs.h_x_ = Eigen::MatrixXd::Zero(1, 3);
        obs.residual_.resize(1);
        obs.R_ = Eigen::MatrixXd::Identity(1, 1);
        obs.residual_(0) = r;
        obs.h_x_.block<1, 3>(0, 0)  = -x_axis_w.transpose();
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

    SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[GNSS OBS] residual(rx,ry,rz)=" << obs.residual_(0) * 180.0 / M_PI << ","
            << obs.residual_(1) * 180.0 / M_PI << "," << obs.residual_(2) * 180.0 / M_PI
            << " deg | norm=" << obs.residual_.norm() * 180.0 / M_PI << " deg";
    obs.valid_ = true;
}

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
//
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

void LaserOdometryCore::FeedGnssAttitude(double heading_deg, double pitch_deg, double roll_deg, int status, int nsv) {
    if (first_gnss_) {
        hc_heading_b_ = -heading_deg * M_PI / 180.0;
        hc_pitch_b_ = -filter(pitch_deg * M_PI / 180.0);
        hc_roll_b_ = roll_deg * M_PI / 180.0;
        first_gnss_ = false;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS FIRST] bias hdg=" << hc_heading_b_ * 180.0 / M_PI
                << " pitch=" << hc_pitch_b_ * 180.0 / M_PI
                << " roll=" << hc_roll_b_ * 180.0 / M_PI << " deg"
                << " | raw hdg=" << heading_deg << " pitch=" << pitch_deg << " roll=" << roll_deg
                << " status=" << status << " nsv=" << nsv;
    } else {
        hc_heading_ = -heading_deg * M_PI / 180.0 - hc_heading_b_;  // rad
        hc_pitch_ = -filter(pitch_deg * M_PI / 180.0) - hc_pitch_b_;
        hc_roll_ = roll_deg * M_PI / 180.0 - hc_roll_b_;
        have_new_gnss_ = true;
        SYS_LOG(syslog::LogSeverity::INFO, true)
                << "[GNSS RAW] hdg=" << heading_deg << " pitch=" << pitch_deg << " roll=" << roll_deg
                << " deg | status=" << status << " nsv=" << nsv
                << " | rel_hdg=" << hc_heading_ * 180.0 / M_PI
                << " rel_pitch=" << hc_pitch_ * 180.0 / M_PI
                << " rel_roll=" << hc_roll_ * 180.0 / M_PI << " deg";
    }
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

}  // namespace slam_odometry