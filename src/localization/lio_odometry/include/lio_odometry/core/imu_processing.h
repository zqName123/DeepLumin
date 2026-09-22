#pragma once

#include <deque>
#include <memory>
#include <vector>

#include "lio_odometry/core/types.h"
#include "syslog/logger.h"

namespace slam_odometry {

constexpr double G_m_s2 = S2::length_;

class ESKF;

class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const Vec3d& transl, const Mat3d& rot);
    void SetGyrCov(const Vec3d& scaler);
    void SetAccCov(const Vec3d& scaler);
    void SetGyrBiasCov(const Vec3d& b_g);
    void SetAccBiasCov(const Vec3d& b_a);

    void Process(const MeasureGroup& meas, ESKF& kf_state, CloudPtr& scan);
    bool IsIMUInited() const { return imu_need_init_ == false; }
    double GetMeanAccNorm() const { return mean_acc_.norm(); }

    Eigen::Matrix<double, 12, 12> Q_;
    Vec3d cov_acc_;
    Vec3d cov_gyr_;
    Vec3d cov_acc_scale_;
    Vec3d cov_gyr_scale_;
    Vec3d cov_bias_gyr_;
    Vec3d cov_bias_acc_;

   private:
    void IMUInit(const MeasureGroup& meas, ESKF& kf_state, int& N);
    void UndistortPcl(const MeasureGroup& meas, ESKF& kf_state, CloudPtr& pcl_out);

    static constexpr int kMaxInitCount = 20;

    CloudPtr cur_pcl_un_ = nullptr;
    IMUPtr last_imu_ = nullptr;
    std::deque<IMUPtr> imu_queue_;

    std::vector<Pose6D> imu_pose_;
    Mat3d R_lidar_imu_ = Mat3d::Identity();
    Vec3d t_lidar_imu_ = Vec3d::Zero();
    Vec3d mean_acc_ = Vec3d::Zero();
    Vec3d mean_gyr_ = Vec3d::Zero();
    Vec3d angvel_last_ = Vec3d::Zero();
    Vec3d acc_s_last_ = Vec3d::Zero();

    double last_lidar_end_time_ = 0;
    double init_start_time_ = -1.0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

}  // namespace slam_odometry