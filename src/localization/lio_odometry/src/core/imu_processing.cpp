#include "lio_odometry/core/imu_processing.h"

#include <cmath>
#include <algorithm>
#include "syslog/logger.h"
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
    cov_acc_scale_ = cov_acc_;
    cov_gyr_scale_ = cov_gyr_;
    last_imu_.reset(new IMU());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    angvel_last_.setZero();
    acc_s_last_.setZero();
    cov_acc_.setZero();
    cov_gyr_.setZero();
    init_start_time_ = -1.0;
    last_lidar_end_time_ = 0.0;
    stream_realigned_ = false;
    b_first_frame_ = true;

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

void ImuProcess::RestartStreaming() {
    // 仅重置跨帧积分衔接，不触碰均值/偏置/初始化标志与滤波器状态。
    last_lidar_end_time_ = 0.0;
    imu_pose_.clear();
    stream_realigned_ = true;
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
        init_start_time_ = meas.imu_.front()->timestamp;
    }

    for (const auto& imu : meas.imu_) {
        cur_acc = imu->linear_acceleration;
        cur_gyr = imu->angular_velocity;

        const Vec3d acc_delta = cur_acc - mean_acc_;
        const Vec3d gyr_delta = cur_gyr - mean_gyr_;
        mean_acc_ += acc_delta / N;
        mean_gyr_ += gyr_delta / N;
        // Running population variance (Welford); do not underestimate motion at startup.
        cov_acc_ = (cov_acc_ * (N - 1.0) + acc_delta.cwiseProduct(cur_acc - mean_acc_)) / N;
        cov_gyr_ = (cov_gyr_ * (N - 1.0) + gyr_delta.cwiseProduct(cur_gyr - mean_gyr_)) / N;

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

    // 时间回退/流重启后的首帧：以新流首个 IMU 作为衔接点并把状态时间戳对齐过去，
    // 避免用上一时期的 last_imu_ 与新流拼出巨大 dt 区间；滤波器状态本身不重置。
    if (stream_realigned_) {
        last_imu_ = meas.imu_.front();
        kf_state.SetTime(last_imu_->timestamp);
        stream_realigned_ = false;
    }

    v_imu.push_front(last_imu_);
    const double& imu_end_time = v_imu.back()->timestamp;

    const double& pcl_beg_time = meas.lidar_begin_time_;
    const double& pcl_end_time = meas.lidar_end_time_;

    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    // First pose always at offset 0.0, matching original lightning-lm.
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_,
                           imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());

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

        // 与原版一致：首段裁到上一帧扫描末端，其余用整段 head->tail
        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc = acc_avr;
        gyro = angvel_avr;

        // 断流判据只看【整段有效积分 dt】，用与 slam_odometry_v2/原版一致的计算。
        // 短丢帧（几十 ms，如数据集里的 60ms 规律缺口）dt 很小，直接积分填补、
        // 不中断——避免把正常短丢误判成断流导致每帧 SetTime+break 冻结、状态时间
        // 基准与激光错位而飘飞。仅当整段 dt 超过 imu_gap_limit_（默认 0.15s，真正
        // 长断流）时才冻结：状态空间凝固、时间戳对齐区间起点。
        if (dt > imu_gap_limit_) {
            std::cerr << "[ImuProcess] real IMU gap: " << dt
                      << "s > " << imu_gap_limit_
                      << "s; freeze state and skip gap" << std::endl;
            kf_state.SetTime(head->timestamp);
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

    // 帧尾外推，与原版一致：dt 带符号。IMU 晚于扫描末端时用负 dt 反向积分退回 pcl_end，
    // 保证去畸变基准严格对齐扫描末端（转弯时末端点不再有几 ms 的系统性旋转偏差）。
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q_, gyro, acc);

    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; });

    if (pcl_out->empty() || pcl_out->points.back().time <= 0.0) {
        // 无逐点时间时保留扫描几何，不虚构扫描时长进行补偿。
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

            Mat3d R_i = R_imu * SO3::exp(angvel_avr * dt).matrix();

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
        // std::cout << "[ImuProcess] meas.imu_ is empty" << std::endl;
        SYS_LOG(syslog::LogSeverity::INFO, true) << "[ImuProcess] meas.imu_ is empty";
        return;
    }

    for (const auto& imu : meas.imu_) {
        if (!imu->linear_acceleration.allFinite() || !imu->angular_velocity.allFinite() ||
            !std::isfinite(imu->timestamp)) return;
    }
    if (imu_need_init_) {
        IMUInit(meas, kf_state, init_iter_num_);
        if (init_iter_num_ > kMaxInitCount) {
            imu_need_init_ = false;
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            last_lidar_end_time_ = meas.lidar_end_time_;
            angvel_last_ = last_imu_->angular_velocity - mean_gyr_;
            acc_s_last_ = last_imu_->linear_acceleration * G_m_s2 / mean_acc_.norm() +
                          kf_state.GetX().grav_.vec_;
            SYS_LOG(syslog::LogSeverity::INFO, true) 
                        << "[ImuProcess] initialized: mean_acc=" << mean_acc_.transpose()
                        << " gravity=" << kf_state.GetX().grav_.vec_.transpose()
                        << " gyro_bias=" << mean_gyr_.transpose();
        }
        return;
    }

    UndistortPcl(meas, kf_state, scan);
}

}  // namespace slam_odometry