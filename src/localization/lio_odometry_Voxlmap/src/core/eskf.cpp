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

void ESKF::Update(ESKF::ObsType obs, const double& R) {
    custom_obs_model_.valid_ = true;
    custom_obs_model_.converge_ = true;

    CovType P_propagated = P_;

    Eigen::Matrix<double, 23, 1> K_r;
    Eigen::Matrix<double, 23, 23> K_H;

    StateVecType dx_current = StateVecType::Zero();

    NavState start_x = x_;
    NavState last_x = x_;

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
        }

        if (use_aa_ && i > -1 && (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) &&
            custom_obs_model_.lidar_residual_mean_ >= last_lidar_res * 1.01) {
            x_ = last_x;
            break;
        }
        iterated_num += 1;

        if (!custom_obs_model_.valid_) {
            continue;
        }

        if (obs == ObsType::WHEEL_SPEED) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
            h_x_cur.block(0, 12, 3, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            x_ = x_.boxplus(dx_current);
            break;
        } else if (obs == ObsType::GPS) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
            h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            x_ = x_.boxplus(dx_current);
            break;
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

        // R 矩阵处理：支持自适应 R（obs.R_ 已设置）或标量 R
        if (state_dim_ > dof_measurement) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(dof_measurement, state_dim_);
            h_x_cur.topLeftCorner(dof_measurement, 12) = custom_obs_model_.h_x_;

            // 如果 obs.R_ 已被外部设置（自适应 R），则使用；否则使用标量 R
            if (custom_obs_model_.R_.rows() != dof_measurement || custom_obs_model_.R_.cols() != dof_measurement) {
                custom_obs_model_.R_ = R * Eigen::MatrixXd::Identity(dof_measurement, dof_measurement);
            }

            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            K_r = K * custom_obs_model_.residual_;
            K_H = K * h_x_cur;
        } else {
            // 信息形式优化（当测量维度 >= 状态维度时）
            // 支持对角 R 矩阵（自适应 R）
            Eigen::VectorXd R_inv_diag(dof_measurement);
            if (custom_obs_model_.R_.rows() == dof_measurement && custom_obs_model_.R_.cols() == dof_measurement) {
                // 使用自定义 R 矩阵的对角元素
                for (int j = 0; j < dof_measurement; j++) {
                    R_inv_diag(j) = 1.0 / custom_obs_model_.R_(j, j);
                }
            } else {
                // 使用标量 R
                custom_obs_model_.R_ = R * Eigen::MatrixXd::Identity(dof_measurement, dof_measurement);
                R_inv_diag.setConstant(1.0 / R);
            }

            // 计算 H^T * R^{-1} * H (12 x 12 矩阵)
            Eigen::Matrix<double, 12, 12> HTR_invH = Eigen::Matrix<double, 12, 12>::Zero();
            for (int j = 0; j < dof_measurement; j++) {
                double r_inv = R_inv_diag(j);
                HTR_invH += r_inv * custom_obs_model_.h_x_.row(j).transpose() * custom_obs_model_.h_x_.row(j);
            }

            // 计算 H^T * R^{-1} * residual (12 x 1 向量)
            Eigen::Matrix<double, 12, 1> HTR_inv_z = Eigen::Matrix<double, 12, 1>::Zero();
            for (int j = 0; j < dof_measurement; j++) {
                HTR_inv_z += R_inv_diag(j) * custom_obs_model_.h_x_.row(j).transpose() * custom_obs_model_.residual_(j);
            }

            CovType P_temp = P_.inverse();
            P_temp.block<12, 12>(0, 0) += HTR_invH;
            CovType Q_inv = P_temp.inverse();

            K_r = Q_inv.template block<23, 12>(0, 0) * HTR_inv_z;

            K_H.setZero();
            K_H.template block<23, 12>(0, 0) = Q_inv.template block<23, 12>(0, 0) * HTR_invH;
        }

        dx_current = K_r + (K_H - Eigen::Matrix<double, 23, 23>::Identity()) * dx_current;

        for (int j = 0; j < 23; ++j) {
            if (std::isnan(dx_current(j, 0))) {
                return;
            }
        }

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

        last_x = x_;

        last_lidar_res = custom_obs_model_.lidar_residual_mean_;
        custom_obs_model_.converge_ = true;

        // 收敛检查：支持 Pose-only 或全状态收敛
        if (use_pose_converge_for_lidar_ && (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR)) {
            // Pose-only 收敛：仅检查旋转和平移增量（与 FAST-LIVO2 一致）
            Vec3d rot_add = dx_current.block<3, 1>(3, 0);  // 旋转增量
            Vec3d t_add = dx_current.block<3, 1>(0, 0);    // 平移增量
            if ((rot_add.norm() * 57.3 < lidar_rot_converge_deg_) && (t_add.norm() < lidar_pos_converge_m_)) {
                converged_times++;
            } else {
                converged_times = 0;
            }
        } else {
            // 全状态收敛检查
            for (int j = 0; j < 23; j++) {
                if (std::fabs(dx_current[j]) > limit_[j]) {
                    custom_obs_model_.converge_ = false;
                    break;
                }
            }
            if (custom_obs_model_.converge_) {
                converged_times++;
            }
        }

        // 要求连续收敛指定次数
        if (converged_times >= lidar_converged_times_required_) {
            custom_obs_model_.converge_ = true;
        }

        if (!converged_times && i == maximum_iter_ - 2) {
            custom_obs_model_.converge_ = true;
        }

        if (converged_times > 0 || i == maximum_iter_ - 1) {
            L_ = P_;
            Mat3d res_temp_SO3;
            Vec3d seg_SO3;
            for (const auto& it : x_.SO3_states_) {
                int idx = it.idx_;
                for (int j = 0; j < 3; j++) {
                    seg_SO3(j) = dx_current(j + idx);
                }

                res_temp_SO3 = math::A_matrix(seg_SO3).transpose();
                for (int j = 0; j < 23; j++) {
                    L_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
                }

                for (int j = 0; j < 15; j++) {
                    K_H.block<3, 1>(idx, j) = res_temp_SO3 * (K_H.block<3, 1>(idx, j));
                }

                for (int j = 0; j < 23; j++) {
                    L_.block<1, 3>(j, idx) = (L_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                    P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                }
            }

            Mat2d res_temp_S2;
            Vec2d seg_S2;
            for (const auto& it : x_.S2_states_) {
                int idx = it.idx_;

                for (int j = 0; j < 2; j++) {
                    seg_S2(j) = dx_current(j + idx);
                }

                Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
                Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
                res_temp_S2 = Nx * Mx;

                for (int j = 0; j < 23; j++) {
                    L_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
                }

                for (int j = 0; j < 15; j++) {
                    K_H.block<2, 1>(idx, j) = res_temp_S2 * (K_H.block<2, 1>(idx, j));
                }

                for (int j = 0; j < 23; j++) {
                    L_.block<1, 2>(j, idx) = (L_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                    P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                }
            }

            P_ = L_ - K_H.block<23, 15>(0, 0) * P_.template block<15, 23>(0, 0);

            break;
        }
    }
}

}  // namespace slam_odometry