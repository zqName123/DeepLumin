#include "lio_odometry/core/eskf.h"

#include <cmath>

namespace slam_odometry {

const std::vector<NavState::MetaInfo> NavState::vect_states_{
    {0, 0, 3},
    {9, 9, 3},
    {12, 12, 3},
    {15, 15, 3},
    {18, 18, 3},
};

const std::vector<NavState::MetaInfo> NavState::SO3_states_{
    {3, 3, 3},
    {6, 6, 3},
};

const std::vector<NavState::MetaInfo> NavState::S2_states_{
    {21, 21, 3},
};

void ESKF::Predict(const double& dt, const ESKF::ProcessNoiseType& Q, const Vec3d& gyro, const Vec3d& acce) {
    Eigen::Matrix<double, 24, 1> f_ = x_.get_f(gyro, acce);
    Eigen::Matrix<double, 24, 23> f_x_ = x_.df_dx(acce);

    Eigen::Matrix<double, 24, 12> f_w_ = x_.df_dw();
    Eigen::Matrix<double, 23, process_noise_dim_> f_w_final;

    NavState x_before = x_;
    x_.oplus(f_, dt);

    F_x1_ = CovType::Identity();

    CovType f_x_final;
    for (const auto& st : x_.vect_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        int dof = st.dof_;

        for (int i = 0; i < 23; i++) {
            for (int j = 0; j < dof; j++) {
                f_x_final(idx + j, i) = f_x_(dim + j, i);
            }
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            for (int j = 0; j < dof; j++) {
                f_w_final(idx + j, i) = f_w_(dim + j, i);
            }
        }
    }

    Mat3d res_temp_SO3;
    Vec3d seg_SO3;
    for (const auto& st : x_.SO3_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_SO3(i) = -1 * f_(dim + i) * dt;
        }

        F_x1_.block<3, 3>(idx, idx) = math::exp(seg_SO3, 0.5).matrix();

        res_temp_SO3 = math::A_matrix(seg_SO3);
        for (int i = 0; i < state_dim_; i++) {
            f_x_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_x_.block<3, 1>(dim, i));
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_w_.block<3, 1>(dim, i));
        }
    }

    Eigen::Matrix<double, 2, 3> res_temp_S2;
    Vec3d seg_S2;
    for (const auto& st : x_.S2_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_S2(i) = f_(dim + i) * dt;
        }

        SO3 res = math::exp(seg_S2, 0.5);

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
        Eigen::Matrix<double, 3, 2> Mx = x_before.grav_.S2_Mx(vec);

        F_x1_.block<2, 2>(idx, idx) = Nx * res.matrix() * Mx;

        Eigen::Matrix<double, 3, 3> x_before_hat = x_before.grav_.S2_hat();
        res_temp_S2 = -Nx * res.matrix() * x_before_hat * math::A_matrix(seg_S2).transpose();

        for (int i = 0; i < state_dim_; i++) {
            f_x_final.block<2, 1>(idx, i) = res_temp_S2 * (f_x_.block<3, 1>(dim, i));
        }
        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.block<2, 1>(idx, i) = res_temp_S2 * (f_w_.block<3, 1>(dim, i));
        }
    }

    F_x1_ += f_x_final * dt;
    P_ = (F_x1_)*P_ * (F_x1_).transpose() + (dt * f_w_final) * Q * (dt * f_w_final).transpose();
}

bool ESKF::Update(ESKF::ObsType obs, const double& R) {
    custom_obs_model_.valid_ = true;
    custom_obs_model_.converge_ = true;

    CovType P_propagated = P_;

    Eigen::Matrix<double, 23, 1> K_r;
    Eigen::Matrix<double, 23, 23> K_H;

    StateVecType dx_current = StateVecType::Zero();

    NavState start_x = x_;
    NavState last_x = x_;
    const auto reject = [&]() {
        x_ = start_x;
        P_ = P_propagated;
        return false;
    };
    if (!std::isfinite(R) || R <= 0.0) return reject();

    int converged_times = 0;
    double last_lidar_res = 0;

    double init_res = 0.0;
    static double iterated_num = 0;
    static double update_num = 0;
    update_num += 1;
    for (int i = -1; i < maximum_iter_; i++) {
        custom_obs_model_.valid_ = true;

        if (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) {
            lidar_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::WHEEL_SPEED) {
            wheelspeed_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::GPS) {
            gps_obs_func_(x_, custom_obs_model_);
        // >>>>> [ZUPT] BEGIN 重力对齐观测分发 <<<<<
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            acc_as_gravity_obs_func_(x_, custom_obs_model_);
        // >>>>> [ZUPT] END <<<<<
        // >>>>> [NHC] BEGIN 非完整约束观测分发 <<<<<
        } else if (obs == ObsType::NHC) {
            nhc_obs_func_(x_, custom_obs_model_);
        // >>>>> [NHC] END <<<<<
        }

        if (use_aa_ && i > -1 && (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) &&
            custom_obs_model_.lidar_residual_mean_ >= last_lidar_res * 1.01) {
            // 与原版 lightning-lm 一致：回退到上一个被接受的状态后结束迭代。
            // 注意 P_ 必须原样保留——此刻它是上一轮迭代中重线性化后的传播协方差
            // (J·P_propagated·Jᵀ，位于 last_x 切空间)，不能重置回 start_x 切空间
            // 下的 P_propagated。
            x_ = last_x;
            // 在被接受的状态上重算一次观测，使 nearest_points_ 与最终位姿一致，供 MapIncremental 使用。
            lidar_obs_func_(x_, custom_obs_model_);
            // 与原版无条件 break 接受一致：即使重算恰好无效（退化场景当帧有效点临时为 0），
            // 也接受 last_x——它是残差回升前最后一个已验证状态，比整帧拒绝（退回纯 IMU
            // 预测）安全得多；最近邻停留在旧状态仅影响本帧加点取舍。
            return true;
        }
        iterated_num += 1;

        if (!custom_obs_model_.valid_) {
            continue;
        }

        if (obs == ObsType::WHEEL_SPEED) {
            const int m = custom_obs_model_.h_x_.rows(); 
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(m, state_dim_);
            h_x_cur.block(0, 12, m, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        } else if (obs == ObsType::GPS) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
            h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        // >>>>> [ZUPT] BEGIN 重力对齐更新分支（与 GPS 分支同构：3×3 旋转块观测，yaw 在 ObsModel 内置零）<<<<<
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
            h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        // >>>>> [ZUPT] END <<<<<
        // >>>>> [NHC] BEGIN 2 行观测：h_x_(2×12) 前3列→rot(3..5)，6..8列→vel(12..14) <<<<<
        } else if (obs == ObsType::NHC) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(2, state_dim_);
            h_x_cur.block(0, 3, 2, 3) = custom_obs_model_.h_x_.leftCols<3>();
            h_x_cur.block(0, 12, 2, 3) = custom_obs_model_.h_x_.block<2, 3>(0, 6);
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        // >>>>> [NHC] END <<<<<
        }

        if (i == -1) {
            init_res = custom_obs_model_.lidar_residual_mean_;
            if (init_res < 1e-9) {
                init_res = 1e-9;
            }
        }

        iterations_ = i + 2;
        final_res_ = custom_obs_model_.lidar_residual_mean_ / init_res;

        int dof_measurement = custom_obs_model_.h_x_.rows();
        StateVecType dx = x_.boxminus(start_x);
        dx_current = dx;

        P_ = P_propagated;

        for (const auto& it : x_.SO3_states_) {
            int idx = it.idx_;
            Vec3d seg_SO3 = dx.block<3, 1>(idx, 0);
            Mat3d res_temp_SO3 = math::A_matrix(seg_SO3).transpose();

            dx_current.block<3, 1>(idx, 0) = res_temp_SO3 * dx.block<3, 1>(idx, 0);

            for (int j = 0; j < state_dim_; j++) {
                P_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
            }
            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
            }
        }

        for (const auto& it : x_.S2_states_) {
            int idx = it.idx_;

            Vec2d seg_S2 = dx.block<2, 1>(idx, 0);

            Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
            Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
            Mat2d res_temp_S2 = Nx * Mx;

            dx_current.block<2, 1>(idx, 0) = res_temp_S2 * dx.block<2, 1>(idx, 0);

            for (int j = 0; j < state_dim_; j++) {
                P_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
            }

            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
            }
        }

        if (state_dim_ > dof_measurement) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(dof_measurement, state_dim_);
            h_x_cur.topLeftCorner(dof_measurement, 12) = custom_obs_model_.h_x_;
            custom_obs_model_.R_ = R * Eigen::MatrixXd::Identity(dof_measurement, dof_measurement);

            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            K_r = K * custom_obs_model_.residual_;
            K_H = K * h_x_cur;
        } else {
            Eigen::Matrix<double, 12, 12> HTH = custom_obs_model_.h_x_.transpose() * custom_obs_model_.h_x_;

            CovType P_temp = (P_ / R).inverse();
            P_temp.block<12, 12>(0, 0) += HTH;
            CovType Q_inv = P_temp.inverse();

            K_r = Q_inv.template block<23, 12>(0, 0) * custom_obs_model_.h_x_.transpose() *
                  custom_obs_model_.residual_;

            K_H.setZero();
            K_H.template block<23, 12>(0, 0) = Q_inv.template block<23, 12>(0, 0) * HTH;
        }

        dx_current = K_r + (K_H - Eigen::Matrix<double, 23, 23>::Identity()) * dx_current;

        if (!dx_current.allFinite()) return reject();

        if (!use_aa_) {
            x_ = x_.boxplus(dx_current);
        } else {
            x_ = x_.boxplus(dx_current);

            if (i == -1) {
                aa_.init(dx_current);
            } else {
                auto dx_all = x_.boxminus(start_x);
                auto new_dx_all = aa_.compute(dx_all);
                x_ = start_x.boxplus(new_dx_all);
            }
        }

        // Keep dx_current as the KF increment (before AA), matching the original
        // lightning-lm behavior.  Convergence and covariance reset must both use
        // the KF step, not the (typically larger) AA actual-manifold increment.
        if (!dx_current.allFinite()) return reject();

        last_x = x_;
        last_lidar_res = custom_obs_model_.lidar_residual_mean_;
        custom_obs_model_.converge_ = true;

        for (int j = 0; j < 23; j++) {
            if (std::fabs(dx_current[j]) > limit_[j]) {
                custom_obs_model_.converge_ = false;
                break;
            }
        }

        if (custom_obs_model_.converge_) {
            converged_times++;
        }

        if (!converged_times && i == maximum_iter_ - 2) {
            custom_obs_model_.converge_ = true;
        }

        if (converged_times > 0 || i == maximum_iter_ - 1) {
            // Reset posterior covariance into the tangent space of the accepted state.
            CovType J = CovType::Identity();
            for (const auto& st : x_.SO3_states_) {
                const Vec3d d = dx_current.segment<3>(st.idx_);
                J.block<3, 3>(st.idx_, st.idx_) = math::A_matrix(d).transpose();
            }
            const Vec2d dg = dx_current.segment<2>(21);
            // dx_current 的 S2 分量表达在 start_x 切空间，Mx 的基点必须是 start_x，
            // 与原版 eskf.cc 收敛后验块一致（不能用迭代中间状态 before_update）。
            J.block<2, 2>(21, 21) = x_.grav_.S2_Nx_yy() * start_x.grav_.S2_Mx(dg);
            const CovType posterior = (CovType::Identity() - K_H) * P_;
            P_ = J * posterior * J.transpose();
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            if (!P_.allFinite()) return reject();
            return true;
        }
    }
    return reject();
}

}  // namespace slam_odometry