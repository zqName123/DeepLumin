#include "lio_odometry/core/voxelmap/voxel_map_fastlivo.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <map>
#include <functional>
#include <glog/logging.h>

namespace slam_odometry {

// ============================================================
//  VoxelOctoTreeFASTLIVO 实现
// ============================================================

void VoxelOctoTreeFASTLIVO::init_plane(const std::list<pointWithVar> &points, VoxelPlane *plane) {
    plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->sum_ppt_ = Eigen::Matrix3d::Zero();
    plane->center_ = Eigen::Vector3d::Zero();
    plane->normal_ = Eigen::Vector3d::Zero();
    plane->points_size_ = points.size();
    plane->radius_ = 0;

    // 计算累积和 sum_ppt 和中心
    for (const auto &pv : points) {
        plane->sum_ppt_ += pv.point_w * pv.point_w.transpose();
        plane->center_ += pv.point_w;
    }
    plane->center_ = plane->center_ / plane->points_size_;
    plane->covariance_ = plane->sum_ppt_ / plane->points_size_ - plane->center_ * plane->center_.transpose();

    // 优化：使用 SelfAdjointEigenSolver（比 EigenSolver 快 3~5 倍）
    // 协方差矩阵是对称半正定矩阵，可以直接使用 SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    Eigen::Matrix3d evecs = es.eigenvectors();     // 直接实数矩阵
    Eigen::Vector3d evals = es.eigenvalues();       // 直接实数向量

    // SelfAdjointEigenSolver 返回递增序特征值：evals(0) <= evals(1) <= evals(2)
    int evalsMin = 0;
    int evalsMid = 1;
    int evalsMax = 2;

    // 退化特征值保护
    if (evalsMin == evalsMax || evalsMid < 0 || evalsMid > 2) {
        plane->is_update_ = true;
        plane->is_plane_ = false;
        return;
    }

    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size_, 0, 0,
           0, 1.0 / plane->points_size_, 0,
           0, 0, 1.0 / plane->points_size_;

    if (evals(evalsMin) < planer_threshold_) {
        // 计算平面协方差传播
        for (const auto &pv : points) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m =
                        (pv.point_w - plane->center_).transpose() /
                        ((double)(plane->points_size_) *
                         (evals[evalsMin] - evals[m])) *
                        (evecs.col(m) *
                         evecs.col(evalsMin).transpose() +
                         evecs.col(evalsMin) *
                         evecs.col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_var_.noalias() += J * pv.var * J.transpose();
        }

        plane->normal_ << evecs(0, evalsMin),
                          evecs(1, evalsMin),
                          evecs(2, evalsMin);
        plane->y_normal_ << evecs(0, evalsMid),
                            evecs(1, evalsMid),
                            evecs(2, evalsMid);
        plane->x_normal_ << evecs(0, evalsMax),
                            evecs(1, evalsMax),
                            evecs(2, evalsMax);
        plane->min_eigen_value_ = evals(evalsMin);
        plane->mid_eigen_value_ = evals(evalsMid);
        plane->max_eigen_value_ = evals(evalsMax);
        plane->radius_ = sqrt(evals(evalsMax));
        plane->d_ = -(plane->normal_(0) * plane->center_(0) +
                      plane->normal_(1) * plane->center_(1) +
                      plane->normal_(2) * plane->center_(2));
        plane->is_plane_ = true;
        plane->is_update_ = true;
        if (!plane->is_init_) {
            plane->id_ = voxel_plane_id;
            voxel_plane_id++;
            plane->is_init_ = true;
        }
    } else {
        plane->is_update_ = true;
        plane->is_plane_ = false;
    }
}

// ============================================================
//  RANSAC 平面拟合模块（可选，use_ransac_=true 时启用）
// ============================================================

void VoxelOctoTreeFASTLIVO::init_plane_ransac() {
    VoxelPlane *plane = plane_ptr_;
    all_points_num_ = static_cast<int>(new_points_.size());

    srand(config_ptr_->sample_seed_);

    if (!ransac()) {
        plane->is_plane_ = false;
        plane->is_init_ = true;
        return;
    }

    plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->sum_ppt_ = Eigen::Matrix3d::Zero();
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
    plane->sum_ppt_ = sum_ppt;
    plane->covariance_ = sum_ppt / plane->points_size_ - plane->center_ * plane->center_.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    Eigen::Matrix3d evecs = es.eigenvectors();
    Eigen::Vector3d evals = es.eigenvalues();

    int evalsMin = 0, evalsMid = 1, evalsMax = 2;

    plane->normal_ = evecs.col(evalsMin);
    plane->y_normal_ = evecs.col(evalsMid);
    plane->x_normal_ = plane->y_normal_.cross(plane->normal_);
    plane->min_eigen_value_ = static_cast<float>(evals(evalsMin));
    plane->mid_eigen_value_ = static_cast<float>(evals(evalsMid));
    plane->max_eigen_value_ = static_cast<float>(evals(evalsMax));

    if (layer_ <= config_ptr_->valid_check_max_layer_ &&
        static_cast<int>(plane_points_.size()) >= config_ptr_->valid_check_min_points_size_) {
        if (!plane_valid_check()) {
            plane->points_size_ = 0;
            plane->is_init_ = true;
            plane->is_plane_ = false;
            return;
        }
    }

    if (new_points_.size() > 0) {
        for (auto iter = new_points_.begin(); iter != new_points_.end(); iter++) {
            Eigen::Vector3d point = iter->point_w;
            sum_ppt -= point * point.transpose();
            plane->center_ -= point;
        }
        plane->points_size_ -= static_cast<int>(new_points_.size());
        plane->center_ = plane->center_ / plane->points_size_;
        plane->sum_ppt_ = sum_ppt;
        plane->covariance_ = sum_ppt / plane->points_size_ - plane->center_ * plane->center_.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es2(plane->covariance_);
        evecs = es2.eigenvectors();
        evals = es2.eigenvalues();
        evalsMin = 0; evalsMid = 1; evalsMax = 2;

        not_plane_points_.splice(not_plane_points_.end(), new_points_);
    }

    if (evals(evalsMin) < planer_threshold_) {
        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane->points_size_, 0, 0,
               0, 1.0 / plane->points_size_, 0,
               0, 0, 1.0 / plane->points_size_;

        for (const auto &pv : plane_points_) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
            for (int m = 0; m < 3; m++) {
                if (m != evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m =
                        (pv.point_w - plane->center_).transpose() /
                        (plane->points_size_ * (evals(evalsMin) - evals(m))) *
                        (evecs.col(m) * evecs.col(evalsMin).transpose() +
                         evecs.col(evalsMin) * evecs.col(m).transpose());
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_var_ += J * pv.var * J.transpose();
        }

        plane->normal_ = evecs.col(evalsMin);
        plane->y_normal_ = evecs.col(evalsMid);
        plane->x_normal_ = plane->y_normal_.cross(plane->normal_);
        plane->min_eigen_value_ = static_cast<float>(evals(evalsMin));
        plane->mid_eigen_value_ = static_cast<float>(evals(evalsMid));
        plane->max_eigen_value_ = static_cast<float>(evals(evalsMax));
        plane->radius_ = static_cast<float>(sqrt(evals(evalsMax)));
        plane->d_ = static_cast<float>(-(plane->normal_.dot(plane->center_)));
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

bool VoxelOctoTreeFASTLIVO::ransac() {
    Eigen::Vector3d best_normal, best_center;
    int max_inliers = 0;
    int sample_num = config_ptr_->ransac_sample_num_;

    std::vector<std::list<pointWithVar>::iterator> iter_vec;
    iter_vec.reserve(new_points_.size());
    for (auto it = new_points_.begin(); it != new_points_.end(); ++it) {
        iter_vec.push_back(it);
    }
    all_points_num_ = static_cast<int>(iter_vec.size());

    if (sample_num < 3) {
        sample_num = 3;
    }
    if (sample_num > all_points_num_) {
        sample_num = all_points_num_;
    }

    for (int i = 0; i < config_ptr_->ransac_max_iter_; ++i) {
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
            double distance = fabs((iter_vec[k]->point_w - center).dot(normal));
            if (distance < config_ptr_->ransac_inlier_threshold_) {
                inliers++;
            }
        }

        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_normal = normal;
            best_center = center;
        }
    }

    if (1.0f * max_inliers / all_points_num_ > config_ptr_->ransac_isplane_p_threshold_) {
        for (int i = 0; i < all_points_num_; i++) {
            auto &iter = iter_vec[i];
            double distance = fabs((iter->point_w - best_center).dot(best_normal));
            if (distance < config_ptr_->ransac_inlier_threshold_) {
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

bool VoxelOctoTreeFASTLIVO::plane_valid_check() {
    VoxelPlane *plane = plane_ptr_;

    double resolution = quater_length_ * 4 / config_ptr_->valid_check_resolution_;
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
            cluster_size += static_cast<int>(grid_map[grid_pair].size());
        }
        if (cluster_size > max_cluster_size) {
            max_cluster_size = cluster_size;
            max_cluster_idx = static_cast<int>(i);
        }
    }

    if (1.0 * max_cluster_size / all_points_num_ > config_ptr_->ransac_isplane_p_threshold_) {
        for (size_t i = 0; i < clusters.size(); i++) {
            if (static_cast<int>(i) == max_cluster_idx) continue;
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

void VoxelOctoTreeFASTLIVO::init_octo_tree() {
    all_points_num_ = new_points_.size();
    if (all_points_num_ > points_size_threshold_) {
        // ===== RANSAC 模式开关 =====
        if (use_ransac_) {
            init_plane_ransac();
        } else {
            init_plane(new_points_, plane_ptr_);
        }
        if (plane_ptr_->is_plane_) {
            octo_state_ = 0;
            if (new_points_.size() > max_points_num_) {
                update_enable_ = false;
                std::list<pointWithVar>().swap(new_points_);
                new_points_num_ = 0;
            }
        } else {
            octo_state_ = 1;
            cut_octo_tree();
        }
        init_octo_ = true;
        new_points_num_ = 0;
    }
}

void VoxelOctoTreeFASTLIVO::cut_octo_tree() {
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
            leaves_[leafnum] = new VoxelOctoTreeFASTLIVO(
                max_layer_, layer_ + 1, layer_init_num_[layer_ + 1],
                max_points_num_, planer_threshold_);
            leaves_[leafnum]->root = root;
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->reinit_size_vec_ = reinit_size_vec_;
            leaves_[leafnum]->update_inlier_threshold_ = update_inlier_threshold_;
            leaves_[leafnum]->use_ransac_ = use_ransac_;
            leaves_[leafnum]->config_ptr_ = config_ptr_;
            leaves_[leafnum]->max_root_points_size_ = max_root_points_size_;
            leaves_[leafnum]->root_update_enable_ = root_update_enable_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }

        // 使用 splice 零拷贝转移
        auto move_it = it++;
        leaves_[leafnum]->new_points_.splice(
            leaves_[leafnum]->new_points_.end(), not_plane_points_, move_it);
    }
    for (uint i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            leaves_[i]->init_octo_tree();
        }
    }
}

void VoxelOctoTreeFASTLIVO::update_octo_tree() {
    if (!init_octo_) {
        init_octo_tree();
        return;
    }

    // reinit 机制：当点数达到阈值时重新初始化八叉树
    if (!stop_reinit_ && all_points_num_root_ > reinit_size_vec_[reinit_state_]) {
        reinit_octo_tree();
        while (all_points_num_root_ > reinit_size_vec_[reinit_state_]) {
            reinit_state_ += 1;
        }
        if (reinit_state_ > static_cast<int>(reinit_size_vec_.size()) - 1) {
            stop_reinit_ = true;
        }
        return;
    }

    // 使用 push_to_subvoxel 智能插入
    std::unordered_set<VoxelOctoTreeFASTLIVO *> update_cache;
    for (auto &pv : new_points_) {
        VoxelOctoTreeFASTLIVO *voxel = push_to_subvoxel(pv);
        if (voxel != nullptr) {
            update_cache.insert(voxel);
        }
    }
    new_points_.clear();

    for (auto voxel : update_cache) {
        voxel->update_single_voxel();
        if (voxel->plane_ptr_->is_plane_ && voxel->root->stop_reinit_ &&
            static_cast<int>(voxel->plane_points_.size()) > max_points_num_) {
            voxel->update_enable_ = false;
        }
    }
}

void VoxelOctoTreeFASTLIVO::update_single_voxel() {
    if (!init_octo_) {
        init_octo_tree();
        return;
    }

    if (new_points_num_ < update_size_threshold_) {
        return;
    }

    if (plane_ptr_->is_plane_ && plane_ptr_->cov_need_update_) {
        // 更新平面协方差
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
        Eigen::Matrix3d evecs = es.eigenvectors();
        Eigen::Vector3d evals = es.eigenvalues();

        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane_ptr_->points_size_, 0, 0,
               0, 1.0 / plane_ptr_->points_size_, 0,
               0, 0, 1.0 / plane_ptr_->points_size_;

        plane_ptr_->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        for (const auto &pv : plane_points_) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
            const Eigen::Vector3d point_centered = pv.point_w - plane_ptr_->center_;
            for (int m = 0; m < 3; m++) {
                if (m != 0) {  // min_eigen_value index is 0
                    const Eigen::Vector3d evec_m = evecs.col(m);
                    F.row(m) = 1.0 / (plane_ptr_->points_size_ * (evals[0] - evals[m])) *
                               point_centered.transpose() *
                               (evec_m * evecs.col(0).transpose() +
                                evecs.col(0) * evec_m.transpose());
                }
            }
            J.block<3, 3>(0, 0) = evecs * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane_ptr_->plane_var_.noalias() += J * pv.var * J.transpose();
        }
        plane_ptr_->cov_need_update_ = false;
        plane_ptr_->is_update_ = true;
    }

    // 如果平面点比例不足，重新初始化
    if (octo_state_ == 0 &&
        1.0 * plane_points_.size() / (plane_points_.size() + not_plane_points_.size()) < 0.8) {
        new_points_.splice(new_points_.end(), plane_points_);
        new_points_.splice(new_points_.end(), not_plane_points_);
        init_octo_tree();
    }

    new_points_num_ = 0;
}

VoxelOctoTreeFASTLIVO *VoxelOctoTreeFASTLIVO::push_to_subvoxel(const pointWithVar &pv) {
    // 收集候选体素路径
    std::vector<VoxelOctoTreeFASTLIVO *> candidates = find_subvoxel_candidates(pv);

    // 遍历候选体素，选择残差最小的平面
    double min_residual = std::numeric_limits<double>::max();
    VoxelOctoTreeFASTLIVO *best_voxel = nullptr;

    for (auto *voxel : candidates) {
        if (voxel->plane_ptr_->is_plane_) {
            VoxelPlane *plane = voxel->plane_ptr_;

            // 计算点到平面的距离
            double residual = std::abs(plane->normal_.dot(pv.point_w) + plane->d_);
            double distance2 = (pv.point_w - plane->center_).squaredNorm();
            double range_dis = std::sqrt(distance2 - residual * residual);

            // 点在平面覆盖范围内且残差最小
            if (range_dis < 3.0 * plane->radius_ && residual < min_residual) {
                if (residual < 3.0 * std::sqrt(plane->min_eigen_value_)) {
                    min_residual = residual;
                    best_voxel = voxel;
                }
            }
        }
    }

    // 找到匹配平面 → 尝试 O(1) 递推更新
    if (best_voxel != nullptr && min_residual < update_inlier_threshold_) {
        if (!best_voxel->update_enable_) {
            return nullptr;
        }
        if (best_voxel->check_and_update(pv)) {
            best_voxel->plane_points_.push_back(pv);  // 内点
            best_voxel->new_points_num_++;
            return best_voxel;
        }
    }

    // 无匹配 → 加入 not_plane_points_
    auto last_voxel = candidates.back();
    if (last_voxel->octo_state_ == 0) {
        best_voxel = last_voxel;
        if (best_voxel->init_octo_) {
            best_voxel->not_plane_points_.push_back(pv);
            best_voxel->new_points_num_++;
        } else {
            best_voxel->new_points_.push_back(pv);
        }
    } else {
        // 非叶子 → 创建子体素
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > last_voxel->voxel_center_[0]) xyz[0] = 1;
        if (pv.point_w[1] > last_voxel->voxel_center_[1]) xyz[1] = 1;
        if (pv.point_w[2] > last_voxel->voxel_center_[2]) xyz[2] = 1;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        last_voxel->leaves_[leafnum] = new VoxelOctoTreeFASTLIVO(
            last_voxel->max_layer_, last_voxel->layer_ + 1,
            last_voxel->layer_init_num_[last_voxel->layer_ + 1],
            last_voxel->max_points_num_, last_voxel->planer_threshold_);
        last_voxel->leaves_[leafnum]->root = root;
        last_voxel->leaves_[leafnum]->layer_init_num_ = layer_init_num_;
        last_voxel->leaves_[leafnum]->reinit_size_vec_ = reinit_size_vec_;
        last_voxel->leaves_[leafnum]->update_inlier_threshold_ = update_inlier_threshold_;
        last_voxel->leaves_[leafnum]->use_ransac_ = use_ransac_;
        last_voxel->leaves_[leafnum]->config_ptr_ = config_ptr_;
        last_voxel->leaves_[leafnum]->max_root_points_size_ = max_root_points_size_;
        last_voxel->leaves_[leafnum]->root_update_enable_ = root_update_enable_;
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

std::vector<VoxelOctoTreeFASTLIVO *> VoxelOctoTreeFASTLIVO::find_subvoxel_candidates(const pointWithVar &pv) {
    std::vector<VoxelOctoTreeFASTLIVO *> subvoxel_candidates;
    VoxelOctoTreeFASTLIVO *curr = this;
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

bool VoxelOctoTreeFASTLIVO::check_and_update(const pointWithVar &pv) {
    VoxelPlane *plane = plane_ptr_;
    int N = plane->points_size_;

    // 递推更新（无需遍历所有点）
    Eigen::Matrix3d new_sum_ppt = plane->sum_ppt_ + pv.point_w * pv.point_w.transpose();
    Eigen::Vector3d new_center = (plane->center_ * N + pv.point_w) / (N + 1);
    Eigen::Matrix3d new_cov = new_sum_ppt / (N + 1) - new_center * new_center.transpose();

    // 对新协方差做 EigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
    double min_eigen_value = es.eigenvalues()(0);

    if (min_eigen_value >= planer_threshold_) {
        return false;  // 加入该点后不再是平面
    }

    // 仍满足平面条件 → O(1) 更新参数
    plane->sum_ppt_ = new_sum_ppt;
    plane->center_ = new_center;
    plane->covariance_ = new_cov;
    plane->points_size_ = N + 1;
    plane->normal_ = es.eigenvectors().col(0);
    plane->y_normal_ = es.eigenvectors().col(1);
    plane->x_normal_ = plane->y_normal_.cross(plane->normal_);
    plane->d_ = -plane->normal_.dot(plane->center_);
    plane->radius_ = std::sqrt(es.eigenvalues()(2));
    plane->min_eigen_value_ = min_eigen_value;
    plane->mid_eigen_value_ = es.eigenvalues()(1);
    plane->max_eigen_value_ = es.eigenvalues()(2);
    plane->is_plane_ = true;
    plane->is_update_ = true;
    plane->cov_need_update_ = true;
    return true;
}

void VoxelOctoTreeFASTLIVO::reinit_octo_tree() {
    // 1. 从所有子体素收集点
    collect_points(new_points_, layer_);

    // 2. 删除所有子体素
    for (int i = 0; i < 8; ++i) {
        if (leaves_[i] != nullptr) {
            delete leaves_[i];
            leaves_[i] = nullptr;
        }
    }

    // 3. 重置状态，重新初始化
    init_octo_ = false;
    plane_ptr_->is_init_ = false;
    plane_ptr_->is_plane_ = false;

    init_octo_tree();
}

void VoxelOctoTreeFASTLIVO::collect_points(std::list<pointWithVar> &collect_list, int collect_layer) {
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

void VoxelOctoTreeFASTLIVO::build_single_residual(
    pointWithVar &pv, const int current_layer, bool &is_success, double &prob,
    PointToPlane &single_ptpl, double sigma_num, int max_layer) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_w;
    if (plane_ptr_->is_plane_) {
        VoxelPlane &plane = *plane_ptr_;
        float dis_to_plane =
            fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) +
                 plane.normal_(2) * p_w(2) + plane.d_);
        float dis_to_center =
            (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
            (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
            (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius_) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
            J_nq.block<1, 3>(0, 3) = -plane.normal_;
            double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
            sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                is_success = true;
                double this_prob = 1.0 / (sqrt(sigma_l)) *
                                   exp(-0.5 * dis_to_plane * dis_to_plane /
                                       sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    pv.normal = plane.normal_;
                    single_ptpl.body_cov_ = pv.body_var;
                    single_ptpl.point_b_ = pv.point_b;
                    single_ptpl.point_w_ = pv.point_w;
                    single_ptpl.plane_var_ = plane.plane_var_;
                    single_ptpl.normal_ = plane.normal_;
                    single_ptpl.center_ = plane.center_;
                    single_ptpl.d_ = plane.d_;
                    single_ptpl.layer_ = current_layer;
                    single_ptpl.dis_to_plane_ =
                        plane.normal_(0) * p_w(0) +
                        plane.normal_(1) * p_w(1) +
                        plane.normal_(2) * p_w(2) + plane.d_;
                }
                return;
            } else {
                return;
            }
        } else {
            return;
        }
    } else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (leaves_[leafnum] != nullptr) {
                    VoxelOctoTreeFASTLIVO *leaf_octo = leaves_[leafnum];
                    leaf_octo->build_single_residual(
                        pv, current_layer + 1, is_success, prob, single_ptpl,
                        sigma_num, max_layer);
                }
            }
            return;
        } else {
            return;
        }
    }
}

// ============================================================
//  VoxelMapManagerFASTLIVO 实现
// ============================================================

VoxelMapManagerFASTLIVO::~VoxelMapManagerFASTLIVO() {
    for (auto &pair : voxel_map_) {
        delete pair.second;
    }
    voxel_map_.clear();
}

void VoxelMapManagerFASTLIVO::Init(const VoxelMapConfig &config) {
    config_ = config;
    voxel_size_ = config_.max_voxel_size_;
    last_slide_pos_.setZero();
    total_points_num_ = 0;
    voxel_plane_id = 0;
    lru_capacity_ = config_.lru_capacity_;  // 从配置读取 LRU 容量
}

void VoxelMapManagerFASTLIVO::UpdateMap(const std::vector<pointWithVar> &input_points) {
    // ===== [TIME_LOG] UpdateMap 整体耗时 =====
    auto t_update_start = std::chrono::high_resolution_clock::now();

    // 使用 update_cache 只遍历当前帧有新增点的体素，避免全表遍历
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTreeFASTLIVO *> update_cache;
    uint plsize = input_points.size();

    // ===== [TIME_LOG] 点插入耗时 =====
    auto t_insert_start = std::chrono::high_resolution_clock::now();

    for (uint i = 0; i < plsize; i++) {
        const pointWithVar &p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size_;
            if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                                 (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            if (!iter->second->root_update_enable_) {
                continue;  // 根体素已锁死，不再加入新点
            }
            iter->second->new_points_.push_back(p_v);
            iter->second->all_points_num_root_++;
        } else {
            VoxelOctoTreeFASTLIVO *octo_tree =
                new VoxelOctoTreeFASTLIVO(config_.max_layer_, 0,
                                           config_.layer_init_num_[0],
                                           config_.max_points_num_,
                                           config_.planner_threshold_);
            voxel_map_[position] = octo_tree;
            octo_tree->root = octo_tree;
            octo_tree->layer_init_num_ = config_.layer_init_num_;
            octo_tree->reinit_size_vec_ = config_.reinit_size_vec_;
            octo_tree->update_inlier_threshold_ = config_.update_inlier_threshold_;
            octo_tree->use_ransac_ = config_.use_ransac_;
            octo_tree->config_ptr_ = &config_;
            octo_tree->max_root_points_size_ = config_.max_root_points_size_;
            octo_tree->quater_length_ = voxel_size_ / 4;
            octo_tree->voxel_center_[0] = (0.5 + position.x) * voxel_size_;
            octo_tree->voxel_center_[1] = (0.5 + position.y) * voxel_size_;
            octo_tree->voxel_center_[2] = (0.5 + position.z) * voxel_size_;
            octo_tree->new_points_.push_back(p_v);
            octo_tree->all_points_num_root_++;
        }
        // 记录到 update_cache
        update_cache[position] = voxel_map_[position];

        // 更新 LRU
        UpdateLRU(position);
    }
    total_points_num_ += input_points.size();

    auto t_insert_end = std::chrono::high_resolution_clock::now();
    double ms_insert = std::chrono::duration<double, std::milli>(t_insert_end - t_insert_start).count();

    // ===== [TIME_LOG] 八叉树更新耗时 =====
    auto t_octree_start = std::chrono::high_resolution_clock::now();

    // 优化：只遍历 update_cache（当前帧点真正触达的体素），而非 voxel_map_ 全表
    for (auto &pair : update_cache) {
        if (pair.second->new_points_.size() > 0) {
            pair.second->update_octo_tree();
        }
        // 根体素点数超过上限 → 锁死，不再接受新点（对齐 R-VoxelMap）
        if (pair.second->all_points_num_root_ > pair.second->max_root_points_size_) {
            pair.second->root_update_enable_ = false;
        }
    }

    auto t_octree_end = std::chrono::high_resolution_clock::now();
    double ms_octree = std::chrono::duration<double, std::milli>(t_octree_end - t_octree_start).count();

    // ===== [TIME_LOG] LRU 淘汰耗时 =====
    auto t_lru_start = std::chrono::high_resolution_clock::now();

    LRUEvict();

    auto t_lru_end = std::chrono::high_resolution_clock::now();
    double ms_lru = std::chrono::duration<double, std::milli>(t_lru_end - t_lru_start).count();

    auto t_update_end = std::chrono::high_resolution_clock::now();
    double ms_total = std::chrono::duration<double, std::milli>(t_update_end - t_update_start).count();

    std::cout << "  [ UpdateMap ] insert: " << std::fixed << std::setprecision(1) << ms_insert << "ms"
              << " | octree: " << ms_octree << "ms"
              << " | LRU: " << ms_lru << "ms"
              << " | total: " << ms_total << "ms"
              << " | cache_size: " << update_cache.size()
              << std::endl;
}

void VoxelMapManagerFASTLIVO::BuildResidualListOMP(
    const std::vector<pointWithVar> &pv_list,
    std::vector<PointToPlane> &ptpl_list) {
    // ===== [TIME_LOG] BuildResidualListOMP 整体耗时 =====
    auto t_residual_start = std::chrono::high_resolution_clock::now();

    int max_layer = config_.max_layer_;
    double sigma_num = config_.sigma_num_;
    std::mutex mylock;
    ptpl_list.clear();
    std::vector<PointToPlane> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    for (size_t i = 0; i < pv_list.size(); ++i) {
        useful_ptpl[i] = false;
    }
#ifdef MP_EN
#pragma omp parallel for
#endif
    for (size_t i = 0; i < pv_list.size(); i++) {
        const pointWithVar &pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_w[j] / voxel_size_;
            if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                                 (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            // 更新 LRU（读操作）
            {
                std::lock_guard<std::mutex> lock(lru_mutex_);
                UpdateLRU(position);
            }

            VoxelOctoTreeFASTLIVO *current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_success = false;
            double prob = 0;
            current_octo->build_single_residual(
                const_cast<pointWithVar &>(pv), 0, is_success, prob,
                single_ptpl, sigma_num, max_layer);
            if (!is_success) {
                VOXEL_LOCATION near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] +
                                  current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                          current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] +
                                  current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                          current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] +
                                  current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                          current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map_.find(near_position);
                if (iter_near != voxel_map_.end()) {
                    iter_near->second->build_single_residual(
                        const_cast<pointWithVar &>(pv), 0, is_success, prob,
                        single_ptpl, sigma_num, max_layer);
                }
            }
            if (is_success) {
                std::lock_guard<std::mutex> lock(mylock);
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) ptpl_list.push_back(all_ptpl_list[i]);
    }

    // ===== [TIME_LOG] BuildResidualListOMP 耗时 =====
    auto t_residual_end = std::chrono::high_resolution_clock::now();
    double ms_residual = std::chrono::duration<double, std::milli>(t_residual_end - t_residual_start).count();

    std::cout << "  [ Residual ] total: " << std::fixed << std::setprecision(1) << ms_residual << "ms"
              << " | input_pts: " << pv_list.size()
              << " | matched: " << ptpl_list.size()
              << " | hit_rate: " << std::setprecision(0) << (100.0 * ptpl_list.size() / std::max(1u, (unsigned)pv_list.size())) << "%"
              << std::endl;
}

void VoxelMapManagerFASTLIVO::MapSliding(const Eigen::Vector3d &center) {
    if (last_slide_pos_.isZero()) {
        last_slide_pos_ = center;
        return;
    }
    if ((center - last_slide_pos_).norm() < config_.sliding_thresh_) {
        return;
    }
    last_slide_pos_ = center;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = center[j] / voxel_size_;
        if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
    }
    clearMemOutOfMap((int64_t)loc_xyz[0] + config_.half_map_size_,
                     (int64_t)loc_xyz[0] - config_.half_map_size_,
                     (int64_t)loc_xyz[1] + config_.half_map_size_,
                     (int64_t)loc_xyz[1] - config_.half_map_size_,
                     (int64_t)loc_xyz[2] + config_.half_map_size_,
                     (int64_t)loc_xyz[2] - config_.half_map_size_);
}

void VoxelMapManagerFASTLIVO::UpdateLRU(const VOXEL_LOCATION &position) {
    auto it = loc2lru_.find(position);
    if (it != loc2lru_.end()) {
        // 已存在 → O(1) 移到链表头部
        lru_cache_.splice(lru_cache_.begin(), lru_cache_, it->second);
        it->second = lru_cache_.begin();
    } else {
        // 新体素 → 插入头部
        lru_cache_.push_front(position);
        loc2lru_[position] = lru_cache_.begin();
    }
}

void VoxelMapManagerFASTLIVO::LRUEvict() {
    while (lru_cache_.size() > lru_capacity_) {
        VOXEL_LOCATION victim = lru_cache_.back();  // 最久未使用
        lru_cache_.pop_back();
        loc2lru_.erase(victim);
        // std::cout << "LRUEvict: " << lru_cache_.size() << std::endl;

        auto it = voxel_map_.find(victim);
        if (it != voxel_map_.end()) {
            delete it->second;  // 释放八叉树内存
            voxel_map_.erase(it);  // 从哈希表中移除
        }
    }
}

void VoxelMapManagerFASTLIVO::clearMemOutOfMap(int x_max, int x_min, int y_max,
                                                int y_min, int z_max,
                                                int z_min) {
    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
        const VOXEL_LOCATION &loc = it->first;
        bool should_remove = loc.x > x_max || loc.x < x_min ||
                             loc.y > y_max || loc.y < y_min ||
                             loc.z > z_max || loc.z < z_min;
        if (should_remove) {
            // 同时从 LRU 中移除
            auto lru_it = loc2lru_.find(loc);
            if (lru_it != loc2lru_.end()) {
                lru_cache_.erase(lru_it->second);
                loc2lru_.erase(lru_it);
            }
            delete it->second;
            it = voxel_map_.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelMapManagerFASTLIVO::CalcBodyCov(const Eigen::Vector3d &point,
                                          double ranging_cov, double angle_cov,
                                          Eigen::Matrix3d &body_var) {
    // 原始实现（设计文档要求无变化）
    Eigen::Vector3d p = point;
    if (point[2] == 0) {
        p[2] = 0.0001;
        return;
    }
    double range = p.norm();
    Eigen::Vector3d direction(p);
    direction.normalize();
    double range_var = ranging_cov * ranging_cov;
    // double angle_var = pow(sin(angle_cov * M_PI / 180.0), 2);
    Eigen::Matrix2d angle_var;
    angle_var << pow(sin(angle_cov * M_PI / 180.0), 2), 0, 0, pow(sin(angle_cov * M_PI / 180.0), 2);
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1),
                     direction(2), 0, -direction(0),
                     -direction(1), direction(0), 0;
    // Eigen::Vector3d base_vector1 = direction.cross(Eigen::Vector3d::UnitX());
    // if (base_vector1.norm() < 1e-6) {
    //     base_vector1 = direction.cross(Eigen::Vector3d::UnitY());
    // }
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0),
         base_vector1(1), base_vector2(1),
         base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    body_var = direction * range_var * direction.transpose() +
               A * angle_var * A.transpose();
}

} // namespace slam_odometry