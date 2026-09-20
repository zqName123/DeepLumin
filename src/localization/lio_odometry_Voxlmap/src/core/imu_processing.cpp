#include "lio_odometry/core/imu_processing.h"

#include <cmath>
#include <algorithm>

#include "lio_odometry/core/eskf.h"

namespace slam_odometry {

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_.setZero();
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
    cov_acc_ = Vec3d(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    last_imu_.reset(new IMU());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    angvel_last_.setZero();

    imu_need_init_ = true;
    init_iter_num_ = 1;
    imu_queue_.clear();
    imu_pose_.clear();
    last_imu_.reset(new IMU());
    cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const Vec3d& transl, const Mat3d& rot) {
    t_lidar_imu_ = transl;
    R_lidar_imu_ = rot;
}

void ImuProcess::SetGyrCov(const Vec3d& scaler) { cov_gyr_scale_ = scaler; }
void ImuProcess::SetAccCov(const Vec3d& scaler) { cov_acc_scale_ = scaler; }
void ImuProcess::SetGyrBiasCov(const Vec3d& b_g) { cov_bias_gyr_ = b_g; }
void ImuProcess::SetAccBiasCov(const Vec3d& b_a) { cov_bias_acc_ = b_a; }

void ImuProcess::IMUInit(const MeasureGroup& meas, ESKF& kf_state, int& N) {
    Vec3d cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto& imu_acc = meas.imu_.front()->linear_acceleration;
        const auto& gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ = imu_acc;
        mean_gyr_ = gyr_acc;
    }

    for (const auto& imu : meas.imu_) {
        cur_acc = imu->linear_acceleration;
        cur_gyr = imu->angular_velocity;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ = cov_acc_ * (N - 1.0) / N +
                   (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                   (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    auto init_state = kf_state.GetX();
    init_state.timestamp_ = meas.imu_.back()->timestamp;
    init_state.grav_ = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
    init_state.bg_ = mean_gyr_;
    init_state.offset_t_lidar_ = t_lidar_imu_;
    init_state.offset_R_lidar_ = R_lidar_imu_;
    kf_state.ChangeX(init_state);

    auto init_P = kf_state.GetP();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.ChangeP(init_P);

    last_imu_ = meas.imu_.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup& meas, ESKF& kf_state, CloudPtr& pcl_out) {
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double& imu_end_time = v_imu.back()->timestamp;

    const double& pcl_beg_time = meas.lidar_begin_time_;
    const double& pcl_end_time = meas.lidar_end_time_;

    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());

    Vec3d angvel_avr, acc_avr, vel_imu, pos_imu;
    Mat3d R_imu;

    double dt = 0;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_) {
            continue;
        }

        angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);

        acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();

        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc = acc_avr;
        gyro = angvel_avr;

        if (dt > 0.1) {
            break;
        }

        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf_state.Predict(dt, Q_, gyro, acc);

        imu_state = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        acc_s_last_ = imu_state.rot_ * (acc_avr - imu_state.ba_);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav_[i];
        }

        double&& offs_t = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(
            Pose6D(offs_t, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()));
    }

    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q_, gyro, acc);

    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; });

    if (pcl_out->empty()) {
        return;
    }

    auto it_pcl = pcl_out->points.end() - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = head->rot;
        vel_imu = head->vel;
        pos_imu = head->pos;
        Vec3d acc_imu = tail->acc;
        angvel_avr = tail->gyr;

        for (; it_pcl->time / 1000.0 > head->offset_time; it_pcl--) {
            dt = it_pcl->time / 1000.0 - head->offset_time;

            Mat3d R_i(R_imu * math::exp(angvel_avr, dt));

            Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos_);
            Vec3d p_compensate = imu_state.offset_R_lidar_.inverse() *
                                 (imu_state.rot_.inverse() *
                                      (R_i * (imu_state.offset_R_lidar_ * P_i + imu_state.offset_t_lidar_) + T_ei) -
                                  imu_state.offset_t_lidar_);

            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out->points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const MeasureGroup& meas, ESKF& kf_state, CloudPtr& scan) {
    if (meas.imu_.empty()) {
        std::cout << "[ImuProcess] meas.imu_ is empty" << std::endl;
        return;
    }

    if (imu_need_init_) {
        std::cout << "[ImuProcess] imu_need_init, meas.imu_.size()=" << meas.imu_.size()
                  << " iter=" << init_iter_num_ << std::endl;

        IMUInit(meas, kf_state, init_iter_num_);

        std::cout << "[ImuProcess] IMUInit done, init_iter_num_=" << init_iter_num_ << std::endl;

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();
        auto state = kf_state.GetX();

        if (init_iter_num_ > kMaxInitCount) {
            cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            std::cout << "[ImuProcess] IMU initialization complete!" << std::endl;
        }

        return;
    }

    UndistortPcl(meas, kf_state, scan);
}

}  // namespace slam_odometry