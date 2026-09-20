#include "lio_odometry/core/laser_odometry_core.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <execution>
#include <fstream>

#include "lio_odometry/core/eskf.h"

namespace slam_odometry {

namespace fasterlio {
int NUM_MAX_ITERATIONS = 4;
float ESTI_PLANE_THRESHOLD = 0.1f;
float lidar_time_interval = 0.1;
}  // namespace fasterlio

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
        ObsModelWelspeed(s, obs);
    };
    eskf_options.gps_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModelGPS(s, obs); };
    // VoxelMap 模式下禁用 AA（与 FAST-LIVO2 一致）
    eskf_options.use_aa_ = false;
    // VoxelMap 模式使用 Pose-only 收敛准则（与 FAST-LIVO2 一致）
    eskf_options.use_pose_converge_for_lidar_ = true;
    eskf_options.lidar_converged_times_required_ = 2;
    kf_.Init(eskf_options);

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
        window_size_ = yaml["vel_gnss"]["gnss_pitch_window_size"].as<int>(50);
        lidar_update_r_ = yaml["fasterlio"]["lidar_update_r"].as<double>(0.001);
        r_base_ = yaml["fasterlio"]["r_base"].as<double>(0.001);
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>(false);

        // ===== VoxelMap 配置加载 =====
        vm_config_.max_voxel_size_ = yaml["voxel_map"]["max_voxel_size"].as<double>(3.0);
        vm_config_.max_layer_ = yaml["voxel_map"]["max_layer"].as<int>(3);
        vm_config_.layer_init_num_ = yaml["voxel_map"]["layer_init_num"].as<std::vector<int>>(std::vector<int>{5, 5, 5, 5, 5});
        vm_config_.max_points_num_ = yaml["voxel_map"]["max_points_num"].as<int>(100);
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

        // ===== BIEVR 凹凸图配置加载 =====
        vm_config_.bump_config_.enable = yaml["voxel_map"]["bump_enable"].as<bool>(false);
        vm_config_.bump_config_.image_size = yaml["voxel_map"]["bump_image_size"].as<int>(16);
        vm_config_.bump_config_.pixel_size = yaml["voxel_map"]["bump_pixel_size"].as<double>(0.05);
        vm_config_.bump_config_.lazy_update_threshold = yaml["voxel_map"]["bump_lazy_update_threshold"].as<int>(5);
        vm_config_.bump_config_.normal_change_tol_deg = yaml["voxel_map"]["bump_normal_change_tol_deg"].as<double>(5.0);
        vm_config_.bump_config_.use_grad_cache = yaml["voxel_map"]["bump_use_grad_cache"].as<bool>(true);
        vm_config_.bump_config_.enable_smooth = yaml["voxel_map"]["bump_enable_smooth"].as<bool>(true);
        vm_config_.bump_config_.smooth_sigma = yaml["voxel_map"]["bump_smooth_sigma"].as<double>(0.8);
        vm_config_.bump_config_.bump_weight_in_residual = yaml["voxel_map"]["bump_weight_in_residual"].as<double>(0.0);
        vm_config_.bump_config_.map_informed_sampling = yaml["voxel_map"]["map_informed_sampling"].as<bool>(false);
        vm_config_.bump_config_.informed_high_ratio = yaml["voxel_map"]["informed_high_ratio"].as<int>(3);
        vm_config_.bump_config_.flat_voxel_bump_thresh = yaml["voxel_map"]["flat_voxel_bump_thresh"].as<double>(0.02);
        vm_config_.bump_config_.enable_per_pixel_confidence = yaml["voxel_map"]["enable_per_pixel_confidence"].as<bool>(true);

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

    return true;
}

void LaserOdometryCore::ProcessIMU(const IMUPtr& imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(imu);
}

void LaserOdometryCore::ProcessPointCloud(const CloudPtr& cloud, double timestamp) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    scan_count_++;

    if (timestamp < last_timestamp_lidar_) {
        lidar_buffer_.clear();
    }

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
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / 1000.0 < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / 1000.0;
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / 1000.0 - lidar_mean_scantime_) / scan_num_;
        }

        fasterlio::lidar_time_interval = lidar_mean_scantime_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
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

    p_imu_->Process(measures_, kf_, scan_undistort_);

    auto t_imu_end = std::chrono::high_resolution_clock::now();
    double ms_imu = std::chrono::duration<double, std::milli>(t_imu_end - t_imu_start).count();

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        return false;
    }

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
        const Mat3d R_cov = kf_.GetP().block<3, 3>(0, 0);
        const Mat3d t_cov = kf_.GetP().block<3, 3>(3, 3);
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
            if (pv.point_b[2] == 0) pv.point_b[2] = 0.001;

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
            Mat3d term1 = R_wl * pv.body_var * R_wl.transpose();
            Mat3d point_imu_cross;
            point_imu_cross << 0.0, -pv.point_i[2], pv.point_i[1],
                                pv.point_i[2], 0.0, -pv.point_i[0],
                               -pv.point_i[1], pv.point_i[0], 0.0;
            Mat3d term2 = R_wl * point_imu_cross * R_cov * point_imu_cross.transpose() * R_wl.transpose();
            pv.var = term1 + term2 + t_cov;
            pv.var_nostate = term1;
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

    // ===== 地图引导采样（Map-Informed Sampling） =====
    if (vm_config_.bump_config_.enable && vm_config_.bump_config_.map_informed_sampling
        && flg_EKF_inited_ && scan_down_body_->size() > 50) {
        auto t_mis_start = std::chrono::high_resolution_clock::now();

        const Mat3d R_wl_mis = state_point_.rot_.matrix();
        const Vec3d t_wl_mis = state_point_.pos_;
        const Mat3d off_R_mis = state_point_.offset_R_lidar_.matrix();
        const Vec3d off_t_mis = state_point_.offset_t_lidar_;

        // 为每个点计算得分
        std::vector<std::pair<int, double>> scored_idx;
        scored_idx.reserve(scan_down_body_->size());
        for (size_t i = 0; i < scan_down_body_->size(); ++i) {
            const auto &pt = scan_down_body_->points[i];
            Eigen::Vector3d point_world = R_wl_mis * (off_R_mis * Eigen::Vector3d(pt.x, pt.y, pt.z) + off_t_mis) + t_wl_mis;

            // 查找对应的体素
            VOXEL_LOCATION loc = voxel_map_mgr_.GetVoxelLocation(point_world);
            const auto &voxel_map = voxel_map_mgr_.GetMap();
            auto it = voxel_map.find(loc);

            double score = 0.5;  // 默认中等分
            if (it != voxel_map.end() && it->second->plane_ptr_->is_plane_) {
                const VoxelPlane *plane = it->second->plane_ptr_;
                if (plane->bump_enabled && plane->bump_observed_pixels_ > 0) {
                    // 得分 = bump_mean_dist_（非平坦区域得分高）
                    score = std::min(plane->bump_mean_dist_ * 20.0, 1.0);  // 归一化到 [0,1]
                }
            }
            scored_idx.emplace_back(i, score);
        }

        // 按得分排序（高→低）
        std::sort(scored_idx.begin(), scored_idx.end(),
                  [](const auto &a, const auto &b) { return a.second > b.second; });

        // 用高:低 = informed_high_ratio:1 的比例采样
        int target_num = static_cast<int>(scored_idx.size());
        int high_num = std::min(target_num,
            static_cast<int>(target_num * vm_config_.bump_config_.informed_high_ratio
                             / static_cast<double>(vm_config_.bump_config_.informed_high_ratio + 1)));
        int low_num = target_num - high_num;
        // 低分区域进一步降采样
        if (low_num > high_num) low_num = high_num / vm_config_.bump_config_.informed_high_ratio;

        CloudPtr sampled_cloud(new PointCloudType());
        sampled_cloud->reserve(high_num + low_num);

        // 取前 high_num 高分点
        for (int i = 0; i < high_num && i < static_cast<int>(scored_idx.size()); ++i) {
            sampled_cloud->push_back(scan_down_body_->points[scored_idx[i].first]);
        }

        // 从剩余低分点均匀采样 low_num 个
        int remaining_start = high_num;
        int remaining_count = static_cast<int>(scored_idx.size()) - high_num;
        if (remaining_count > 0 && low_num > 0) {
            int step = std::max(1, remaining_count / low_num);
            for (int i = 0; i < low_num && remaining_start + i * step < static_cast<int>(scored_idx.size()); ++i) {
                sampled_cloud->push_back(scan_down_body_->points[scored_idx[remaining_start + i * step].first]);
            }
        }

        *scan_down_body_ = *sampled_cloud;

        auto t_mis_end = std::chrono::high_resolution_clock::now();
        double ms_mis = std::chrono::duration<double, std::milli>(t_mis_end - t_mis_start).count();
        std::cout << "[ TIME ] MapInformedSample: " << std::fixed << std::setprecision(1) << ms_mis << "ms"
                  << " | kept: " << sampled_cloud->size() << " (high:" << high_num << " low:" << low_num << ")"
                  << std::endl;
    }

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        return false;
    }
    scan_down_world_->resize(cur_pts);

    auto old_state = kf_.GetX();

    // ===== [TIME_LOG] ESKF Update 耗时（包含 ObsModel + BuildResidualListOMP） =====
    auto t_eskf_start = std::chrono::high_resolution_clock::now();

    kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_);

    auto t_eskf_end = std::chrono::high_resolution_clock::now();
    double ms_eskf = std::chrono::duration<double, std::milli>(t_eskf_end - t_eskf_start).count();

    if (use_wheel_speed_ && have_new_wheel_speed_) {
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, wheel_velocity_noise_ * wheel_velocity_noise_);
        have_new_wheel_speed_ = false;
    }
    if (use_gnss_ && have_new_gnss_) {
        kf_.Update(ESKF::ObsType::GPS, 1e-3);
        have_new_gnss_ = false;
    }

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    if (keep_first_imu_estimation_ && frame_num_ < 5 &&
        (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180.0) {
        kf_.ChangeX(old_state);
        state_point_ = old_state;
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
    std::cout << "[ TIME ] IMU: " << std::fixed << std::setprecision(1) << ms_imu << "ms"
              << " | downsample: " << ms_downsample << "ms"
              << " | ESKF: " << ms_eskf << "ms"
              << " | MapUpdate: " << ms_map << "ms"
              << " | Run_total: " << ms_run_total << "ms"
              << " | pts: " << cur_pts
              << " effect: " << effect_feat_num_
              << " voxels: " << voxel_map_mgr_.GetTotalPointsNum()
              << std::endl;


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
    const Mat3d R_cov_map = kf_.GetP().block<3, 3>(0, 0);
    const Mat3d t_cov_map = kf_.GetP().block<3, 3>(3, 3);

    if (pv_list_.empty()) {
        // ObsModel 未被调用（skip_lidar 等情况）：从头构建 pv_list_
        pv_list_.resize(cur_pts);

        for (size_t i = 0; i < cur_pts; i++) {
            Vec3d point_body(scan_down_body_->points[i].x,
                           scan_down_body_->points[i].y,
                           scan_down_body_->points[i].z);
            if (point_body[2] == 0) point_body[2] = 0.001;

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
            Mat3d term1 = R_wl * pv.body_var * R_wl.transpose();
            Mat3d point_imu_cross;
            point_imu_cross << 0.0, -pv.point_i[2], pv.point_i[1],
                                pv.point_i[2], 0.0, -pv.point_i[0],
                               -pv.point_i[1], pv.point_i[0], 0.0;
            Mat3d term2 = R_wl * point_imu_cross * R_cov_map * point_imu_cross.transpose() * R_wl.transpose();
            pv.var = term1 + term2 + t_cov_map;
            pv.var_nostate = term1;
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
            Mat3d term1 = R_wl * pv.body_var * R_wl.transpose();
            Mat3d point_imu_cross;
            point_imu_cross << 0.0, -point_i[2], point_i[1],
                                point_i[2], 0.0, -point_i[0],
                               -point_i[1], point_i[0], 0.0;
            Mat3d term2 = R_wl * point_imu_cross * R_cov_map * point_imu_cross.transpose() * R_wl.transpose();
            pv.var = term1 + term2 + t_cov_map;
            pv.var_nostate = term1;

            // 同步 scan_down_world_（用于可视化等）
            scan_down_world_->points[i].x = p_global(0);
            scan_down_world_->points[i].y = p_global(1);
            scan_down_world_->points[i].z = p_global(2);
        }
    }

    // R-VoxelMap 增量更新（UpdateMap 内部已有详细耗时日志）
    voxel_map_mgr_.UpdateMap(pv_list_);

    // 滑动窗口管理
    if (vm_config_.map_sliding_en_) {
        static Vec3d last_slide_pos = Vec3d::Zero();
        double dist_to_last_slide = (state_point_.pos_ - last_slide_pos).norm();
        if (dist_to_last_slide > vm_config_.sliding_thresh_) {
            voxel_map_mgr_.MapSliding(state_point_.pos_);
            last_slide_pos = state_point_.pos_;
        }
    }

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
    // ===== [TIME_LOG] ObsModel 整体耗时 =====
    auto t_obs_start = std::chrono::high_resolution_clock::now();

    int cnt_pts = scan_down_body_->size();
    if (cnt_pts < 10) {
        obs.valid_ = false;
        return;
    }

    // ===== [TIME_LOG] pv_list_ 重建 + 协方差传播耗时 =====
    auto t_pvlist_start = std::chrono::high_resolution_clock::now();

    // ===== 每轮 IEKF 迭代重新构建 pv_list_（用当前状态 s） =====
    pv_list_.resize(cnt_pts);

    const Mat3d off_R = s.offset_R_lidar_.matrix();
    const Vec3d off_t = s.offset_t_lidar_;
    const Mat3d R_wl = s.rot_.matrix();
    const Vec3d t_wl = s.pos_;

    // 从 ESKF 获取当前状态协方差（用于协方差传播）
    const Mat3d R_cov = kf_.GetP().block<3, 3>(0, 0);    // 姿态协方差 (SO3)
    const Mat3d t_cov = kf_.GetP().block<3, 3>(3, 3);    // 位置协方差

    for (int i = 0; i < cnt_pts; i++) {
        Vec3d point_body(scan_down_body_->points[i].x,
                       scan_down_body_->points[i].y,
                       scan_down_body_->points[i].z);
        if (point_body[2] == 0) point_body[2] = 0.001;

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
        Mat3d term1 = R_wl * body_var * R_wl.transpose();

        //    姿态不确定性传播: p_i = off_R * p_b + off_t
        //    d(p_w)/d(δθ) = R * p_i^  (右扰动模型)
        Mat3d point_imu_cross;
        point_imu_cross << 0.0, -point_imu[2], point_imu[1],
                           point_imu[2], 0.0, -point_imu[0],
                          -point_imu[1], point_imu[0], 0.0;
        Mat3d term2 = R_wl * point_imu_cross * R_cov * point_imu_cross.transpose() * R_wl.transpose();

        //    总协方差 = 传感器噪声 + 姿态不确定性 + 位置不确定性
        Mat3d total_var = term1 + term2 + t_cov;

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
    if (effect_num < 3) {
        obs.valid_ = false;
        return;
    }

    obs.h_x_ = Eigen::MatrixXd::Zero(effect_num, 12);
    obs.residual_.resize(effect_num);
    obs.R_ = Eigen::MatrixXd::Zero(effect_num, effect_num);  // 自适应 R 矩阵
    obs.valid_ = true;

    for (int idx = 0; idx < effect_num; idx++) {
        const auto &ptpl = ptpl_list_[idx];

        // 使用当前状态重新计算点在世界坐标系下的坐标
        Vec3d point_body = ptpl.point_b_;
        Vec3d point_imu = off_R * point_body + off_t;
        Vec3d point_world = R_wl * point_imu + t_wl;

        // 法向量（来自地图平面）
        Vec3d norm_vec = ptpl.normal_;

        // ===== 残差 = 点到平面距离（基础项） =====
        double d_plane = norm_vec.dot(point_world) + ptpl.d_;

        // ===== 凹凸图修正 =====
        double bump_val = 0.0;
        double dbump_dwx = 0.0, dbump_dwy = 0.0, dbump_dwz = 0.0;
        int bump_num_valid = 4;  // 默认 4 个插值邻居全有效
        bool bump_active = vm_config_.bump_config_.enable;
        if (bump_active && ptpl.plane_ptr_ != nullptr) {
            const VoxelPlane *bp = ptpl.plane_ptr_;
            if (bp->bump_enabled && bp->bump_observed_pixels_ > 0 && !bp->bump_flat) {
                // 将世界坐标点投影到局部平面坐标 (u, v)，双线性插值 bump
                Eigen::Vector3d centered = point_world - bp->center_;
                float u = static_cast<float>(bp->x_normal_.dot(centered));
                float v = static_cast<float>(bp->y_normal_.dot(centered));
                float inv_ps = 1.0f / static_cast<float>(bp->bump_pixel_size_);
                float half_sz = kDefaultBumpImageSize / 2.0f;
                u = u * inv_ps + half_sz;
                v = v * inv_ps + half_sz;

                int ui0 = static_cast<int>(std::floor(u)), vi0 = static_cast<int>(std::floor(v));
                int ui1 = ui0 + 1, vi1 = vi0 + 1;
                float du = u - ui0, dv = v - vi0;

                auto getIdx = [bp](int r, int c) { return bp->bumpIdx(r, c); };

                bool v00 = (vi0 >= 0 && vi0 < kDefaultBumpImageSize && ui0 >= 0 && ui0 < kDefaultBumpImageSize
                            && bp->bump_weights_[getIdx(vi0, ui0)] > 0);
                bool v10 = (vi0 >= 0 && vi0 < kDefaultBumpImageSize && ui1 >= 0 && ui1 < kDefaultBumpImageSize
                            && bp->bump_weights_[getIdx(vi0, ui1)] > 0);
                bool v01 = (vi1 >= 0 && vi1 < kDefaultBumpImageSize && ui0 >= 0 && ui0 < kDefaultBumpImageSize
                            && bp->bump_weights_[getIdx(vi1, ui0)] > 0);
                bool v11 = (vi1 >= 0 && vi1 < kDefaultBumpImageSize && ui1 >= 0 && ui1 < kDefaultBumpImageSize
                            && bp->bump_weights_[getIdx(vi1, ui1)] > 0);

                float w00 = v00 ? (1.0f - du) * (1.0f - dv) : 0.0f;
                float w10 = v10 ? du * (1.0f - dv) : 0.0f;
                float w01 = v01 ? (1.0f - du) * dv : 0.0f;
                float w11 = v11 ? du * dv : 0.0f;
                float wsum = w00 + w10 + w01 + w11;

                if (wsum > 1e-10f) {
                    float b00 = v00 ? bp->bump_img_[getIdx(vi0, ui0)] : 0.0f;
                    float b10 = v10 ? bp->bump_img_[getIdx(vi0, ui1)] : 0.0f;
                    float b01 = v01 ? bp->bump_img_[getIdx(vi1, ui0)] : 0.0f;
                    float b11 = v11 ? bp->bump_img_[getIdx(vi1, ui1)] : 0.0f;
                    bump_val = static_cast<double>((b00 * w00 + b10 * w10 + b01 * w01 + b11 * w11) / wsum);

                    // bump 在局部坐标下的梯度 (∂bump/∂u, ∂bump/∂v)，转为世界梯度
                    float gu00 = v00 ? bp->bump_grad_du_[getIdx(vi0, ui0)] : 0.0f;
                    float gu10 = v10 ? bp->bump_grad_du_[getIdx(vi0, ui1)] : 0.0f;
                    float gu01 = v01 ? bp->bump_grad_du_[getIdx(vi1, ui0)] : 0.0f;
                    float gu11 = v11 ? bp->bump_grad_du_[getIdx(vi1, ui1)] : 0.0f;
                    float gv00 = v00 ? bp->bump_grad_dv_[getIdx(vi0, ui0)] : 0.0f;
                    float gv10 = v10 ? bp->bump_grad_dv_[getIdx(vi0, ui1)] : 0.0f;
                    float gv01 = v01 ? bp->bump_grad_dv_[getIdx(vi1, ui0)] : 0.0f;
                    float gv11 = v11 ? bp->bump_grad_dv_[getIdx(vi1, ui1)] : 0.0f;
                    float g_u = (gu00 * w00 + gu10 * w10 + gu01 * w01 + gu11 * w11) / wsum;
                    float g_v = (gv00 * w00 + gv10 * w10 + gv01 * w01 + gv11 * w11) / wsum;

                    // dBump/dp_w = (∂b/∂u) * (∂u/∂p_w) + (∂b/∂v) * (∂v/∂p_w)
                    // ∂u/∂p_w = x_normal / pixel_size, ∂v/∂p_w = y_normal / pixel_size
                    Eigen::Vector3d dbump_dpw = (g_u * bp->x_normal_ + g_v * bp->y_normal_) / bp->bump_pixel_size_;
                    dbump_dwx = dbump_dpw(0);
                    dbump_dwy = dbump_dpw(1);
                    dbump_dwz = dbump_dpw(2);

                    // 记录插值邻居有效数（供后续自适应 R）
                    bump_num_valid = (v00 ? 1 : 0) + (v10 ? 1 : 0) + (v01 ? 1 : 0) + (v11 ? 1 : 0);
                }
            }
        }

        // ===== 复合残差: r = -(d_plane + bump) =====
        double bump_weight = vm_config_.bump_config_.bump_weight_in_residual;
        obs.residual_(idx) = -(d_plane + bump_weight * bump_val);

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
        Mat3d point_var = R_wl * ptpl.body_cov_ * R_wl.transpose();
        double sigma_p = norm_vec.transpose() * point_var * norm_vec;

        // 3) 自适应 R
        double adaptive_R = r_base_ + sigma_l + sigma_p;
        adaptive_R = std::max(adaptive_R, r_base_);  // 最小值为 r_base_（防止退化场景过度信任）
        adaptive_R = std::min(adaptive_R, 1.0);      // 防止过大

        // 凹凸图像素置信度：稀疏像素放大观测噪声
        if (bump_active && vm_config_.bump_config_.enable_per_pixel_confidence && bump_num_valid < 4) {
            adaptive_R *= (1.0 + 2.0 * (4 - bump_num_valid));
        }

        obs.R_(idx, idx) = adaptive_R;

        // ===== Jacobian H =====
        // h_x = [d_res/d_pos, d_res/d_rot, d_res/d_off_R, d_res/d_off_t]
        //       (3)          (3)          (3)             (3)
        // 基础 Jacobian: ∂r/∂pos = -n^T, ∂r/∂rot = -(p_i × (R^T * n))^T
        obs.h_x_.block<1, 3>(idx, 0) = norm_vec.transpose();                                      // d/d(pos)
        obs.h_x_.block<1, 3>(idx, 3) = point_imu.cross(R_wl.transpose() * norm_vec).transpose();  // d/d(rot)

        // 凹凸图 Jacobian 修正: ∂r/∂pos -= bump_weight * dBump/dp_w
        if (bump_active && bump_weight > 0.0 && std::abs(dbump_dwx) + std::abs(dbump_dwy) + std::abs(dbump_dwz) > 1e-10) {
            Vec3d dbump_dpw_vec(dbump_dwx, dbump_dwy, dbump_dwz);

            // ∂bump/∂t_G: dBump/dp_w (世界坐标系) → 残差的平移 Jacobian
            obs.h_x_.block<1, 3>(idx, 0) -= bump_weight * dbump_dpw_vec.transpose();

            // ∂bump/∂θ: dBump/dp_w^T * (-R * p_imu^) = (p_imu × (R^T * dbump))^T
            obs.h_x_.block<1, 3>(idx, 3) -= bump_weight *
                point_imu.cross(R_wl.transpose() * dbump_dpw_vec).transpose();
        }

        if (extrinsic_est_en_) {
            // 从 point_b 计算 crossmat
            Mat3d crossmat;
            crossmat << 0.0, -point_body[2], point_body[1],
                        point_body[2], 0.0, -point_body[0],
                       -point_body[1], point_body[0], 0.0;
            obs.h_x_.block<1, 3>(idx, 6) = (R_wl.transpose() * norm_vec).transpose() * crossmat; // d/d(off_R)
            obs.h_x_.block<1, 3>(idx, 9) = (R_wl.transpose() * norm_vec).transpose();             // d/d(off_t)
        } else {
            obs.h_x_.block<1, 3>(idx, 6).setZero();
            obs.h_x_.block<1, 3>(idx, 9).setZero();
        }
    }

    obs.lidar_residual_mean_ = obs.residual_.cwiseAbs().mean();
    obs.lidar_residual_max_ = obs.residual_.cwiseAbs().maxCoeff();
    effect_feat_num_ = effect_num;

    // ===== [TIME_LOG] Jacobian 构建耗时 =====
    auto t_jacobian_end = std::chrono::high_resolution_clock::now();
    double ms_jacobian = std::chrono::duration<double, std::milli>(t_jacobian_end - t_jacobian_start).count();

    // ===== [TIME_LOG] ObsModel 整体耗时 =====
    auto t_obs_end = std::chrono::high_resolution_clock::now();
    double ms_obs = std::chrono::duration<double, std::milli>(t_obs_end - t_obs_start).count();

    std::cout << "    [ ObsModel ] pvlist: " << std::fixed << std::setprecision(1) << ms_pvlist << "ms"
              << " | jacobian: " << ms_jacobian << "ms"
              << " | total: " << ms_obs << "ms"
              << " | effect: " << effect_num << "/" << cnt_pts
              << std::endl;
}

void LaserOdometryCore::ObsModelWelspeed(NavState& s, ESKF::CustomObservationModel& obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
    obs.residual_.resize(3);
    obs.R_ = wheel_velocity_noise_ * Eigen::MatrixXd::Identity(3, 3);
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Vec3d vel_odom(wheel_speed_obs_, 0.0, 0.0);
    Vec3d vel_world = s.rot_ * vel_odom;
    auto tem_v = s.vel_ - vel_world;
    obs.residual_ = -tem_v;
    obs.valid_ = true;
}

void LaserOdometryCore::ObsModelGPS(NavState& s, ESKF::CustomObservationModel& obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
    obs.residual_.resize(3);
    obs.R_ = gnss_attitude_noise_ * Eigen::MatrixXd::Identity(3, 3);
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    auto RPR = Vec3d(hc_heading_, hc_pitch_, hc_roll_);
    SO3 gps_R = EulerZYXToSO3(RPR);
    obs.residual_ = (s.rot_.inverse() * gps_R).log();
    obs.valid_ = true;
}

void LaserOdometryCore::FeedWheelSpeed(double timestamp, double speed_kmh, int gear) {
    if (gear == 2) {
        can_speed_double_ = -speed_kmh;
    } else {
        can_speed_double_ = speed_kmh;
    }
    wheel_speed_obs_ = can_speed_double_ / 3.6;
    have_new_wheel_speed_ = true;
}

void LaserOdometryCore::FeedGnssAttitude(double heading_deg, double pitch_deg, double roll_deg) {
    if (first_gnss_) {
        hc_heading_b_ = -heading_deg * M_PI / 180.0;
        hc_pitch_b_ = -filter(pitch_deg * M_PI / 180.0);
        hc_roll_b_ = roll_deg * M_PI / 180.0;
        first_gnss_ = false;
    } else {
        hc_heading_ = -heading_deg * M_PI / 180.0 - hc_heading_b_;
        hc_pitch_ = -filter(pitch_deg * M_PI / 180.0) - hc_pitch_b_;
        hc_roll_ = roll_deg * M_PI / 180.0 - hc_roll_b_;
        have_new_gnss_ = true;
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