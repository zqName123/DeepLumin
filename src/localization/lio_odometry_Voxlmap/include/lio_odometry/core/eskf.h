#pragma once

#include <functional>

#include "lio_odometry/core/anderson_acceleration.h"
#include "lio_odometry/core/types.h"

namespace slam_odometry {

class ESKF {
   public:
    static constexpr int process_noise_dim_ = 12;
    static constexpr int state_dim_ = NavState::dim;
    using StateVecType = NavState::VectState;
    using CovType = Eigen::Matrix<double, state_dim_, state_dim_>;
    using ProcessNoiseType = Eigen::Matrix<double, process_noise_dim_, process_noise_dim_>;

    enum class ObsType {
        LIDAR,
        WHEEL_SPEED,
        WHEEL_SPEED_AND_LIDAR,
        ACC_AS_GRAVITY,
        GPS,
        BIAS,
    };

    struct CustomObservationModel {
        Eigen::VectorXd variance_; // LiDAR diagonal R; empty means scalar fallback.
        bool valid_ = true;
        bool converge_ = true;

        Eigen::Matrix<double, Eigen::Dynamic, 1> residual_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x_;
        Eigen::Matrix<double, Eigen::Dynamic, 1> s_;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_;

        double lidar_residual_mean_ = 0;
        double lidar_residual_max_ = 0;
    };

    using CustomObsFunction = std::function<void(NavState& s, CustomObservationModel& obs)>;

    struct Options {
        CustomObsFunction lidar_obs_func_;
        CustomObsFunction wheelspeed_obs_func_;
        CustomObsFunction acc_as_gravity_obs_func_;
        CustomObsFunction gps_obs_func_;
        CustomObsFunction bias_obs_func_;
        int max_iterations_ = 4;
        StateVecType epsi_;
        bool use_aa_ = false;
        // FAST-LIVO2 风格 Pose-only 收敛选项
        bool use_pose_converge_for_lidar_ = false;
        int lidar_converged_times_required_ = 1;
        double lidar_rot_converge_deg_ = 0.01;    // 旋转收敛阈值 (deg)
        double lidar_pos_converge_m_ = 0.00015;   // 平移收敛阈值 (m)
    };

   public:
    explicit ESKF(const NavState& x = NavState(), const CovType& P = CovType::Identity(), bool use_aa = true)
        : x_(x), P_(P), use_aa_(use_aa) {}

    ~ESKF() {}

    void Init(Options options) {
        lidar_obs_func_ = options.lidar_obs_func_;
        wheelspeed_obs_func_ = options.wheelspeed_obs_func_;
        acc_as_gravity_obs_func_ = options.acc_as_gravity_obs_func_;
        gps_obs_func_ = options.gps_obs_func_;
        bias_obs_func_ = options.bias_obs_func_;
        maximum_iter_ = options.max_iterations_;
        limit_ = options.epsi_;
        use_aa_ = options.use_aa_;
        // Pose-only 收敛选项
        use_pose_converge_for_lidar_ = options.use_pose_converge_for_lidar_;
        lidar_converged_times_required_ = options.lidar_converged_times_required_;
        lidar_rot_converge_deg_ = options.lidar_rot_converge_deg_;
        lidar_pos_converge_m_ = options.lidar_pos_converge_m_;
    }

    void Predict(const double& dt, const ProcessNoiseType& Q, const Vec3d& gyro, const Vec3d& acce);

    bool Update(ObsType obs, const double& R);

    const NavState& GetX() const { return x_; }
    const CovType& GetP() const { return P_; }
    const CovType& GetObservationPriorP() const { return observation_prior_P_; }
    const double& GetStamp() const { return stamp_; }

    void ChangeX(const NavState& state) { x_ = state; }
    void ChangeP(const CovType& P) { P_ = P; }
    void ChangeStamp(const double& stamp) { stamp_ = stamp; }

    bool GetUseAA() const { return use_aa_; }
    void SetUseAA(bool use_aa) { use_aa_ = use_aa; }
    void SetTime(double timestamp) { x_.timestamp_ = timestamp; }

    int GetIterations() const { return iterations_; }
    double GetFinalRes() const { return final_res_; }

   private:
    double stamp_ = 0.0;

    NavState x_;
    CovType P_ = CovType::Identity();
    CovType observation_prior_P_ = CovType::Identity();
    CovType F_x1_ = CovType::Identity();
    CovType L_ = CovType::Identity();

    CustomObservationModel custom_obs_model_;
    CustomObsFunction lidar_obs_func_, wheelspeed_obs_func_, acc_as_gravity_obs_func_, gps_obs_func_, bias_obs_func_;

    int maximum_iter_ = 0;
    StateVecType limit_;

    int iterations_ = 0;
    double final_res_ = 0.0;

    bool use_aa_ = false;
    AndersonAcceleration<double, state_dim_, 10> aa_;

    // Pose-only 收敛选项
    bool use_pose_converge_for_lidar_ = false;
    int lidar_converged_times_required_ = 1;
    double lidar_rot_converge_deg_ = 0.01;
    double lidar_pos_converge_m_ = 0.00015;
};

}  // namespace slam_odometry