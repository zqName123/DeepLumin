#pragma once
#include "voxel_map_types.h"
#include "voxel_map_config.h"
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <functional>

namespace slam_odometry {

class OctoTree {
public:
    std::list<pointWithVar> plane_points_;     // 平面内点
    std::list<pointWithVar> not_plane_points_; // 非平面点
    std::list<pointWithVar> new_points_;       // 新加入的点

    VoxelPlane *plane_ptr_;
    OctoTree *root;
    int max_layer_;
    bool indoor_mode_;
    int layer_;
    int octo_state_;          // 0: 叶子节点, 1: 非叶子节点
    OctoTree *leaves_[8];
    double voxel_center_[3];
    std::vector<int> layer_init_num_;
    float quater_length_;
    float planer_threshold_;
    int init_size_threshold_;
    int update_size_threshold_;

    int all_points_num_root_;
    int all_points_num_;
    int new_points_num_;

    int max_points_size_;
    int max_cov_points_size_;
    bool init_octo_;
    bool update_cov_enable_;
    bool update_enable_;
    bool root_update_enable_;

    bool stop_reinit_;
    int reinit_state_;

    VoxelMapConfig config_;

    OctoTree(int layer, const VoxelMapConfig &config)
        : layer_(layer), config_(config) {
        max_layer_ = config.max_layer_;
        layer_init_num_ = config.layer_init_num_;
        max_points_size_ = config.max_points_num_;
        max_cov_points_size_ = config.max_cov_points_size_;
        planer_threshold_ = config.planner_threshold_;
        update_size_threshold_ = config.update_size_threshold_;

        new_points_.clear();
        plane_points_.clear();
        not_plane_points_.clear();
        octo_state_ = 0;
        new_points_num_ = 0;
        all_points_num_ = 0;
        all_points_num_root_ = 0;
        init_octo_ = false;
        root_update_enable_ = true;
        root = this;
        update_enable_ = true;
        update_cov_enable_ = true;
        init_size_threshold_ = layer_init_num_[layer_];
        stop_reinit_ = false;
        reinit_state_ = 0;

        voxel_center_[0] = 0.0;
        voxel_center_[1] = 0.0;
        voxel_center_[2] = 0.0;
        quater_length_ = 0.0f;

        for (int i = 0; i < 8; i++) {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new VoxelPlane;
    }

    ~OctoTree() {
        delete plane_ptr_;
        for (int i = 0; i < 8; i++) {
            if (leaves_[i] != nullptr) {
                delete leaves_[i];
            }
        }
    }

    // ==============================
    //  平面初始化（RANSAC + 特征值分解）
    // ==============================
    void init_plane_ransac() {
        VoxelPlane *plane = plane_ptr_;
        all_points_num_ = new_points_.size();

        srand(config_.sample_seed_);

        if (!ransac()) {
            plane->is_plane_ = false;
            plane->is_init_ = true;
            return;
        }

        plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        plane->covariance_ = Eigen::Matrix3d::Zero();
        plane->center_ = Eigen::Vector3d::Zero();
        plane->normal_ = Eigen::Vector3d::Zero();
        plane->points_size_ = 0;
        plane->radius_ = 0;

        Eigen::Matrix3d sum_ppt = Eigen::Matrix3d::Zero();
        for (const auto &pv : plane_points_) {
            sum_ppt += pv.point_w * pv.point_w.transpose();
            plane->center_ += pv.point_w;
            plane->points_size_++;
        }

        plane->center_ = plane->center_ / plane->points_size_;
        plane->covariance_ = sum_ppt / plane->points_size_ - plane->center_ * plane->center_.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance_);
        Eigen::Matrix3d evecs = es.eigenvectors();
        Eigen::Vector3d evals = es.eigenvalues();

        // SelfAdjointEigenSolver 返回递增序特征值：evals(0) <= evals(1) <= evals(2)
        // 直接使用索引避免 minCoeff/maxCoeff 在所有特征值相等时返回相同索引的 bug
        int evalsMin = 0;
        int evalsMid = 1;
        int evalsMax = 2;
        Eigen::Vector3d evecMin = evecs.col(evalsMin);
        Eigen::Vector3d evecMid = evecs.col(evalsMid);
        Eigen::Vector3d evecMax = evecs.col(evalsMax);

        plane->normal_ = evecs.col(evalsMin);
        plane->y_normal_ = evecs.col(evalsMid);
        plane->x_normal_ = plane->y_normal_.cross(plane->normal_);

        plane->min_eigen_value_ = evals(0);
        plane->mid_eigen_value_ = evals(1);
        plane->max_eigen_value_ = evals(2);

        // 平面有效检查
        if (layer_ <= config_.valid_check_max_layer_ &&
            static_cast<int>(plane_points_.size()) >= config_.valid_check_min_points_size_) {
            if (!plane_valid_check()) {
                plane->points_size_ = 0;
                plane->is_init_ = true;
                plane->is_plane_ = false;
                return;
            }
        }

        // 移除 RANSAC 后 new_points_ 中残留的离群点
        if (new_points_.size() > 0) {
            // 从 sum_ppt 和 center 中减去离群点的贡献
            for (auto iter = new_points_.begin(); iter != new_points_.end(); iter++) {
                Eigen::Vector3d point = iter->point_w;
                sum_ppt -= point * point.transpose();
                plane->center_ -= point;
            }
            plane->points_size_ -= static_cast<int>(new_points_.size());
            plane->center_ = plane->center_ / plane->points_size_;
            plane->covariance_ = sum_ppt / plane->points_size_ - plane->center_ * plane->center_.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es2(plane->covariance_);
            evecs = es2.eigenvectors();
            evals = es2.eigenvalues();
            evalsMin = 0;
            evalsMax = 2;
            evalsMid = 1;
            evecMin = evecs.col(evalsMin);
            evecMid = evecs.col(evalsMid);
            evecMax = evecs.col(evalsMax);

            not_plane_points_.splice(not_plane_points_.end(), new_points_);
        }

        if (evals(evalsMin) < planer_threshold_) {
            // 计算平面协方差
            Eigen::Matrix3d J_Q;
            J_Q << 1.0 / plane->points_size_, 0, 0,
                   0, 1.0 / plane->points_size_, 0,
                   0, 0, 1.0 / plane->points_size_;

            for (const auto &pv : plane_points_) {
                Eigen::Matrix<double, 6, 3> J;
                Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
                for (int m = 0; m < 3; m++) {
                    if (m != (int)evalsMin) {
                        Eigen::Matrix<double, 1, 3> F_m =
                            (pv.point_w - plane->center_).transpose() /
                            ((plane->points_size_) * (evals(evalsMin) - evals(m))) *
                            (evecs.col(m) * evecs.col(evalsMin).transpose() +
                             evecs.col(evalsMin) * evecs.col(m).transpose());
                        F.row(m) = F_m;
                    }
                }
                J.block<3, 3>(0, 0) = evecs * F;
                J.block<3, 3>(3, 0) = J_Q;
                plane->plane_var_ += J * pv.var * J.transpose();
            }

            plane->normal_ << evecs(0, evalsMin), evecs(1, evalsMin), evecs(2, evalsMin);
            plane->y_normal_ << evecs(0, evalsMid), evecs(1, evalsMid), evecs(2, evalsMid);
            plane->x_normal_ = plane->y_normal_.cross(plane->normal_);
            plane->min_eigen_value_ = evals(evalsMin);
            plane->mid_eigen_value_ = evals(evalsMid);
            plane->max_eigen_value_ = evals(evalsMax);
            plane->radius_ = sqrt(evals(evalsMax));
            plane->d_ = -(plane->normal_.dot(plane->center_));
            plane->is_plane_ = true;
            if (plane->last_update_points_size_ == 0) {
                plane->last_update_points_size_ = plane->points_size_;
                plane->is_update_ = true;
            } else if (plane->points_size_ - plane->last_update_points_size_ > 100) {
                plane->last_update_points_size_ = plane->points_size_;
                plane->is_update_ = true;
            }
            if (!plane->is_init_) {
                plane->is_init_ = true;
            }
        } else {
            if (!plane->is_init_) {
                plane->is_init_ = true;
            }
            not_plane_points_.splice(not_plane_points_.end(), plane_points_);
            plane->is_plane_ = false;
        }
    }

    // ==============================
    //  更新平面协方差
    // ==============================
    void update_plane_cov() {
        VoxelPlane *plane = plane_ptr_;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance_);
        Eigen::Matrix3d evecs = es.eigenvectors();
        Eigen::Vector3d evals = es.eigenvalues();

        int evalsMin = 0;
        int evalsMid = 1;
        int evalsMax = 2;

        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane->points_size_, 0, 0,
               0, 1.0 / plane->points_size_, 0,
               0, 0, 1.0 / plane->points_size_;

        plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        for (auto &pv : plane_points_) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
            const Eigen::Vector3d point_centered = pv.point_w - plane->center_;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    const Eigen::Vector3d evec_m = evecs.col(m);
                    F.row(m) = 1.0 / (plane->points_size_ * (evals[evalsMin] - evals[m])) *
                               point_centered.transpose() *
                               (evec_m * evecs.col(evalsMin).transpose() +
                                evecs.col(evalsMin) * evec_m.transpose());
                }
            }
            J.block<3, 3>(0, 0) = evecs * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_var_.noalias() += J * pv.var * J.transpose();
        }
        plane->cov_need_update_ = false;
        plane->is_update_ = true;
    }

    // ==============================
    //  增量更新检查：尝试将新点匹配到已有平面
    // ==============================
    bool check_and_update(const pointWithVar &pv) {
        VoxelPlane *plane = plane_ptr_;
        int curr_points_num = plane->points_size_;

        Eigen::Vector3d curr_mean = plane->center_;
        Eigen::Matrix3d curr_cov = plane->covariance_;

        Eigen::Vector3d p_vec(pv.point_w[0], pv.point_w[1], pv.point_w[2]);
        Eigen::Vector3d new_mean = (curr_mean * curr_points_num + p_vec) / (curr_points_num + 1);

        Eigen::Matrix3d new_cov = (curr_cov + curr_mean * curr_mean.transpose()) * curr_points_num;
        new_cov = new_cov + p_vec * p_vec.transpose();
        new_cov = new_cov / (curr_points_num + 1) - new_mean * new_mean.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
        Eigen::Vector3d eigen_values = es.eigenvalues();

        double min_eigen_value = eigen_values(0);
        if (min_eigen_value >= config_.planner_threshold_) {
            return false;
        } else {
            plane->center_ = new_mean;
            plane->covariance_ = new_cov;
            plane->points_size_ = curr_points_num + 1;

            Eigen::Matrix3d evecs = es.eigenvectors();
            plane->normal_ << evecs(0, 0), evecs(1, 0), evecs(2, 0);
            plane->y_normal_ << evecs(0, 1), evecs(1, 1), evecs(2, 1);
            plane->x_normal_ = plane->y_normal_.cross(plane->normal_);

            plane->min_eigen_value_ = eigen_values(0);
            plane->mid_eigen_value_ = eigen_values(1);
            plane->max_eigen_value_ = eigen_values(2);
            plane->radius_ = sqrt(eigen_values(2));
            plane->d_ = -plane->normal_.dot(plane->center_);
            plane->is_update_ = true;
            plane->cov_need_update_ = true;
            return true;
        }
    }

    // ==============================
    //  八叉树初始化
    // ==============================
    void init_octo_tree() {
        all_points_num_ = new_points_.size();
        if (all_points_num_ > init_size_threshold_) {
            init_plane_ransac();
            if (not_plane_points_.size() > 0) {
                octo_state_ = 1;
                cut_octo_tree();
            } else {
                octo_state_ = 0;
            }
            init_octo_ = true;
        }
        new_points_num_ = 0;
    }

    // ==============================
    //  八叉树重初始化
    // ==============================
    void reinit_octo_tree() {
        collect_points(new_points_, layer_);

        for (int i = 0; i < 8; ++i) {
            if (leaves_[i] != nullptr) {
                delete leaves_[i];
                leaves_[i] = nullptr;
            }
        }

        init_octo_ = false;
        plane_ptr_->is_init_ = false;

        init_octo_tree();
    }

    // ==============================
    //  八叉树分裂
    // ==============================
    void cut_octo_tree() {
        if (layer_ >= max_layer_) {
            octo_state_ = 0;
            return;
        }
        for (auto it = not_plane_points_.begin(); it != not_plane_points_.end();) {
            auto &pv = *it;
            int xyz[3] = {0, 0, 0};
            if (pv.point_w[0] > voxel_center_[0]) xyz[0] = 1;
            if (pv.point_w[1] > voxel_center_[1]) xyz[1] = 1;
            if (pv.point_w[2] > voxel_center_[2]) xyz[2] = 1;
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

            if (leaves_[leafnum] == nullptr) {
                leaves_[leafnum] = new OctoTree(layer_ + 1, config_);
                leaves_[leafnum]->root = root;
                leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            }

            auto move_it = it++;
            leaves_[leafnum]->new_points_.splice(leaves_[leafnum]->new_points_.end(), not_plane_points_, move_it);
        }
        for (uint i = 0; i < 8; i++) {
            if (leaves_[i] != nullptr) {
                leaves_[i]->init_octo_tree();
            }
        }
    }

    // ==============================
    //  增量更新八叉树
    // ==============================
    void update_octo_tree() {
        if (!init_octo_) {
            init_octo_tree();
            return;
        }

        // reinit 机制：当点数达到阈值时重新初始化八叉树
        if (!stop_reinit_ && all_points_num_root_ > config_.reinit_size_vec_[reinit_state_]) {
            reinit_octo_tree();
            while (all_points_num_root_ > config_.reinit_size_vec_[reinit_state_]) {
                reinit_state_ += 1;
            }
            if (reinit_state_ > static_cast<int>(config_.reinit_size_vec_.size()) - 1) {
                stop_reinit_ = true;
            }
            return;
        }

        std::unordered_set<OctoTree *> update_cache;
        for (auto &pv : new_points_) {
            OctoTree *voxel = push_to_subvoxel(pv);
            if (voxel != nullptr) {
                update_cache.insert(voxel);
            }
        }
        new_points_.clear();

        for (auto voxel : update_cache) {
            voxel->update_single_voxel();
            if (voxel->plane_ptr_->is_plane_ && voxel->root->stop_reinit_ &&
                static_cast<int>(voxel->plane_points_.size()) > config_.max_points_num_) {
                voxel->update_enable_ = false;
            }
        }
    }

    // ==============================
    //  单个体素更新
    // ==============================
    void update_single_voxel() {
        if (!init_octo_) {
            init_octo_tree();
            return;
        }

        if (new_points_num_ < update_size_threshold_) {
            return;
        }

        if (plane_ptr_->is_plane_ && plane_ptr_->cov_need_update_) {
            update_plane_cov();
        }

        // 如果 is_plane_ 但比例不足，重新 init
        if (octo_state_ == 0 &&
            1.0 * plane_points_.size() / (plane_points_.size() + not_plane_points_.size()) < config_.ransac_isplane_p_threshold_) {
            new_points_.splice(new_points_.end(), plane_points_);
            new_points_.splice(new_points_.end(), not_plane_points_);
            init_octo_tree();
        }

        new_points_num_ = 0;
    }

    // ==============================
    //  查找最合适的子体素插入点
    // ==============================
    OctoTree *push_to_subvoxel(const pointWithVar &pv) {
        std::vector<OctoTree *> subvoxel_candidates = find_subvoxel_candidates(pv);
        double min_residual = 1000000;
        OctoTree *best_voxel = nullptr;

        for (auto curr_voxel : subvoxel_candidates) {
            if (curr_voxel->plane_ptr_->is_plane_) {
                VoxelPlane *plane = curr_voxel->plane_ptr_;
                double residual = fabs(plane->normal_.dot(pv.point_w) + plane->d_);
                double distance2 = (pv.point_w - plane->center_).squaredNorm();
                double range_dis = sqrt(distance2 - residual * residual);
                if (range_dis < 3 * plane->radius_) {
                    if (residual < 3 * sqrt(plane->min_eigen_value_) && residual < min_residual) {
                        min_residual = residual;
                        best_voxel = curr_voxel;
                    }
                }
            }
        }

        if (best_voxel != nullptr && min_residual < config_.update_inlier_distance_threshold_) {
            if (!best_voxel->update_enable_) {
                return nullptr;
            }
            if (best_voxel->check_and_update(pv)) {
                best_voxel->plane_points_.push_back(pv);
                best_voxel->new_points_num_++;
                return best_voxel;
            }
        }

        auto last_voxel = subvoxel_candidates.back();
        if (last_voxel->octo_state_ == 0) {
            best_voxel = last_voxel;
            if (best_voxel->init_octo_) {
                best_voxel->not_plane_points_.push_back(pv);
                best_voxel->new_points_num_++;
            } else {
                best_voxel->new_points_.push_back(pv);
            }
        } else {
            int xyz[3] = {0, 0, 0};
            if (pv.point_w[0] > last_voxel->voxel_center_[0]) xyz[0] = 1;
            if (pv.point_w[1] > last_voxel->voxel_center_[1]) xyz[1] = 1;
            if (pv.point_w[2] > last_voxel->voxel_center_[2]) xyz[2] = 1;
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

            last_voxel->leaves_[leafnum] = new OctoTree(last_voxel->layer_ + 1, last_voxel->config_);
            last_voxel->leaves_[leafnum]->root = root;
            last_voxel->leaves_[leafnum]->voxel_center_[0] =
                last_voxel->voxel_center_[0] + (2 * xyz[0] - 1) * last_voxel->quater_length_;
            last_voxel->leaves_[leafnum]->voxel_center_[1] =
                last_voxel->voxel_center_[1] + (2 * xyz[1] - 1) * last_voxel->quater_length_;
            last_voxel->leaves_[leafnum]->voxel_center_[2] =
                last_voxel->voxel_center_[2] + (2 * xyz[2] - 1) * last_voxel->quater_length_;
            last_voxel->leaves_[leafnum]->quater_length_ = last_voxel->quater_length_ / 2;

            last_voxel->leaves_[leafnum]->new_points_.push_back(pv);
            best_voxel = last_voxel->leaves_[leafnum];
        }

        return best_voxel;
    }

    // ==============================
    //  查找子体素候选链
    // ==============================
    std::vector<OctoTree *> find_subvoxel_candidates(const pointWithVar &pv) {
        std::vector<OctoTree *> subvoxel_candidates;
        OctoTree *curr = this;
        subvoxel_candidates.push_back(curr);
        while (curr->octo_state_ != 0) {
            int xyz[3] = {0, 0, 0};
            if (pv.point_w[0] > curr->voxel_center_[0]) xyz[0] = 1;
            if (pv.point_w[1] > curr->voxel_center_[1]) xyz[1] = 1;
            if (pv.point_w[2] > curr->voxel_center_[2]) xyz[2] = 1;
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
            if (curr->leaves_[leafnum] == nullptr) break;
            curr = curr->leaves_[leafnum];
            subvoxel_candidates.push_back(curr);
        }
        return subvoxel_candidates;
    }

    // ==============================
    //  残差构建：单个点的平面匹配
    // ==============================
    void build_single_residual(const pointWithVar &pv, bool &is_success,
                               double &prob, PointToPlane &single_ptpl) {
        double radius_k = 3;
        Eigen::Vector3d p_w = pv.point_w;
        if (plane_ptr_->is_plane_) {
            VoxelPlane &plane = *plane_ptr_;
            float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) +
                                      plane.normal_(2) * p_w(2) + plane.d_);
            float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                                  (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                                  (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
            float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

            if (range_dis <= radius_k * plane.radius_) {
                Eigen::Matrix<double, 1, 6> J_nq;
                J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
                J_nq.block<1, 3>(0, 3) = -plane.normal_;
                double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
                sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
                if (dis_to_plane < config_.sigma_num_ * sqrt(sigma_l)) {
                    is_success = true;
                    double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                    if (this_prob > prob) {
                        prob = this_prob;
                        single_ptpl.plane_var_ = plane.plane_var_;
                        single_ptpl.normal_ = plane.normal_;
                        single_ptpl.center_ = plane.center_;
                        single_ptpl.d_ = plane.d_;
                        single_ptpl.layer_ = layer_;
                        single_ptpl.point_b_ = pv.point_b;
                        single_ptpl.point_w_ = pv.point_w;
                        single_ptpl.body_cov_ = pv.body_var;
                        single_ptpl.eigen_value_ = plane.min_eigen_value_;
                        single_ptpl.is_valid_ = true;
                        single_ptpl.dis_to_plane_ = plane.normal_.dot(p_w) + plane.d_;
                    }
                }
            }
        }

        if (layer_ < max_layer_ && octo_state_ == 1) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->build_single_residual(pv, is_success, prob, single_ptpl);
                }
            }
        }
    }

    // ==============================
    //  查找体素对应
    // ==============================
    OctoTree *find_correspond(const Eigen::Vector3d &pw) {
        if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

        int xyz[3] = {0, 0, 0};
        xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
        xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
        xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
    }

    // ==============================
    //  收集所有点（用于 reinit）
    // ==============================
    void collect_points(std::list<pointWithVar> &collect_list, int collect_layer) {
        if (plane_ptr_->is_plane_) {
            collect_list.splice(collect_list.end(), plane_points_);
        }
        if (octo_state_ == 0) {
            collect_list.splice(collect_list.end(), not_plane_points_);
        }
        if (!init_octo_ && layer_ != collect_layer) {
            collect_list.splice(collect_list.end(), new_points_);
        }
        if (octo_state_ == 1) {
            for (int i = 0; i < 8; i++) {
                if (leaves_[i] != nullptr) {
                    leaves_[i]->collect_points(collect_list, collect_layer);
                }
            }
        }
    }

private:
    // ==============================
    //  RANSAC 平面拟合（使用 config 参数）
    // ==============================
    bool ransac() {
        Eigen::Vector3d best_normal, best_center;
        int max_inliers = 0;
        int sample_num = config_.ransac_sample_num_;

        std::vector<std::list<pointWithVar>::iterator> iter_vec;
        iter_vec.reserve(new_points_.size());
        for (auto it = new_points_.begin(); it != new_points_.end(); ++it) {
            iter_vec.push_back(it);
        }
        all_points_num_ = iter_vec.size();

        if (sample_num < 3) {
            sample_num = 3;
        }
        if (sample_num > all_points_num_) {
            sample_num = all_points_num_;
        }

        for (int i = 0; i < config_.ransac_max_iter_; ++i) {
            std::vector<Eigen::Vector3d> sample_points(sample_num);
            std::unordered_set<int> sampled_set;
            int cnt = 0;
            while (cnt < sample_num) {
                int sample = rand() % all_points_num_;
                if (sampled_set.find(sample) == sampled_set.end()) {
                    sample_points[cnt] = iter_vec[sample]->point_w;
                    sampled_set.insert(sample);
                    cnt++;
                }
            }

            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (int j = 0; j < sample_num; j++) {
                center += sample_points[j];
                cov += sample_points[j] * sample_points[j].transpose();
            }
            center = center / sample_num;
            cov = cov / sample_num - center * center.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
            Eigen::Vector3d normal = es.eigenvectors().col(0);
            normal.normalize();

            int inliers = 0;
            for (int k = 0; k < all_points_num_; k++) {
                Eigen::Vector3d current_point = iter_vec[k]->point_w;
                double distance = fabs((current_point - center).dot(normal));
                if (distance < config_.ransac_inlier_threshold_) {
                    inliers++;
                }
            }

            if (inliers > max_inliers) {
                max_inliers = inliers;
                best_normal = normal;
                best_center = center;
            }
        }

        if (1.0f * max_inliers / all_points_num_ > config_.ransac_isplane_p_threshold_) {
            for (int i = 0; i < all_points_num_; i++) {
                auto &iter = iter_vec[i];
                double distance = fabs((iter->point_w - best_center).dot(best_normal));
                if (distance < config_.ransac_inlier_threshold_) {
                    plane_points_.splice(plane_points_.end(), new_points_, iter);
                } else {
                    not_plane_points_.splice(not_plane_points_.end(), new_points_, iter);
                }
            }
            return true;
        } else {
            not_plane_points_.splice(not_plane_points_.end(), new_points_);
            return false;
        }
    }

    // ==============================
    //  平面有效检查（连通域分析）
    // ==============================
    bool plane_valid_check() {
        VoxelPlane *plane = plane_ptr_;

        double resolution = quater_length_ * 4 / config_.valid_check_resolution_;
        Eigen::Vector3d x_axis = plane->x_normal_;
        Eigen::Vector3d y_axis = plane->y_normal_;
        Eigen::Vector3d origin = plane->center_;

        std::map<std::pair<int, int>, std::vector<std::list<pointWithVar>::iterator>> grid_map;

        for (auto iter = plane_points_.begin(); iter != plane_points_.end(); iter++) {
            const Eigen::Vector3d &point = iter->point_w;
            Eigen::Vector3d relative_point = point - origin;
            double x_coord = relative_point.dot(x_axis);
            double y_coord = relative_point.dot(y_axis);

            double grid_x_f = x_coord / resolution;
            if (x_coord < 0) grid_x_f += -1;
            double grid_y_f = y_coord / resolution;
            if (y_coord < 0) grid_y_f += -1;

            int grid_x = (int)grid_x_f;
            int grid_y = (int)grid_y_f;
            grid_map[{grid_x, grid_y}].push_back(iter);
        }

        std::map<std::pair<int, int>, bool> visited;
        std::vector<std::vector<std::pair<int, int>>> clusters;

        std::function<void(int, int, std::vector<std::pair<int, int>> &)> dfs =
            [&](int x, int y, std::vector<std::pair<int, int>> &cluster) {
                auto key = std::make_pair(x, y);
                if (visited[key] || grid_map.find(key) == grid_map.end()) return;
                visited[key] = true;
                cluster.push_back(key);
                dfs(x + 1, y, cluster);
                dfs(x - 1, y, cluster);
                dfs(x, y + 1, cluster);
                dfs(x, y - 1, cluster);
            };

        for (const auto &grid_pair : grid_map) {
            auto key = grid_pair.first;
            if (!visited[key]) {
                std::vector<std::pair<int, int>> cluster;
                dfs(key.first, key.second, cluster);
                clusters.push_back(cluster);
            }
        }

        int max_cluster_size = 0;
        int max_cluster_idx = -1;
        for (size_t i = 0; i < clusters.size(); i++) {
            int cluster_size = 0;
            for (auto grid_pair : clusters[i]) {
                cluster_size += grid_map[grid_pair].size();
            }
            if (cluster_size > max_cluster_size) {
                max_cluster_size = cluster_size;
                max_cluster_idx = i;
            }
        }

        if (1.0 * max_cluster_size / all_points_num_ > config_.ransac_isplane_p_threshold_) {
            for (int i = 0; i < static_cast<int>(clusters.size()); i++) {
                if (i == max_cluster_idx) continue;
                for (auto grid_pair : clusters[i]) {
                    for (auto iter : grid_map[grid_pair]) {
                        new_points_.splice(new_points_.end(), plane_points_, iter);
                    }
                }
            }
            return true;
        } else {
            not_plane_points_.splice(not_plane_points_.end(), plane_points_);
            return false;
        }
    }
};

} // namespace slam_odometry
