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
        ObsModelWelspeed(s, obs);
    };
    eskf_options.gps_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModelGPS(s, obs); };
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
        window_size_ = yaml["vel_gnss"]["gnss_pitch_window_size"].as<int>(50);
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

    return true;
}

void LaserOdometryCore::ProcessIMU(const IMUPtr& imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        imu_buffer_.clear();
    }

    // if (p_imu_->IsIMUInited()) {
    //     kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity,
    //                     imu->linear_acceleration);
    // }

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
    if (!SyncPackages()) {
        // std::cout << "SyncPackages failed " << std::endl;
        return false;
    }

    p_imu_->Process(measures_, kf_, scan_undistort_);
    // std::cout << "p_imu_->Process done " << std::endl;
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        std::cout << "scan_undistort is empty " << std::endl;
        return false;
    }

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

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        std::cout << "skip lidar " << std::endl;
        return false;
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
    //std::cout << "update lidar " << std::endl;
    kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_);
    // std::cout << "update lidar done " << std::endl;
    if (use_wheel_speed_ && have_new_wheel_speed_) {
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, wheel_velocity_noise_ * wheel_velocity_noise_);
        have_new_wheel_speed_ = false;
        std::cout << "use_wheel_speed_  done " << std::endl;
    }
    if (use_gnss_ && have_new_gnss_) {
        kf_.Update(ESKF::ObsType::GPS, 1e-3);
        have_new_gnss_ = false;
        std::cout << "use_gnss_  done " << std::endl;
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

    MapIncremental();

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
    std::cout << "[ odometry ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_ << std::endl;


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

    std::mutex mtx;
    // std::execution::par_unseq
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
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
                std::lock_guard<std::mutex> lock(mtx);
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
                std::lock_guard<std::mutex> lock(mtx);
                points_to_add.push_back(point_world);
            }
        } else {
            std::lock_guard<std::mutex> lock(mtx);
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

    auto R_wl = (s.rot_ * s.offset_R_lidar_).cast<float>();
    auto t_wl = (s.rot_ * s.offset_t_lidar_ + s.pos_).cast<float>();

    std::for_each(std::execution::par_unseq, scan_down_body_->points.cbegin(), scan_down_body_->points.cend(),
                  [&](const auto& point_body) {
                      size_t i = &point_body - &scan_down_body_->points[0];
                      PointType& point_world = scan_down_world_->points[i];

                      Vec3f p_body = point_body.getVector3fMap();
                      point_world.getVector3fMap() = R_wl * p_body + t_wl;
                      point_world.intensity = point_body.intensity;

                      auto& points_near = nearest_points_[i];

                      ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                      point_selected_surf_[i] = points_near.size() >= static_cast<size_t>(fasterlio::MIN_NUM_MATCH_POINTS);
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
                          } else {
                              point_selected_surf_[i] = false;
                          }
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