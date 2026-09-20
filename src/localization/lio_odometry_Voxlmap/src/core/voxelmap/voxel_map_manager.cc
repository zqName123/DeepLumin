#include "lio_odometry/core/voxelmap/voxel_map_manager.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <unordered_set>
#include <glog/logging.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace slam_odometry {

VoxelMapManager::~VoxelMapManager() {
    for (auto &pair : feat_map_) {
        if (pair.second != nullptr) {
            delete pair.second;
        }
    }
    feat_map_.clear();
}

void VoxelMapManager::Init(const VoxelMapConfig &config) {
    config_ = config;
    voxel_size_ = config_.max_voxel_size_;

    total_points_num_ = 0;

    LOG(INFO) << "R-VoxelMap initialized: voxel_size=" << voxel_size_
              << ", max_layer=" << config_.max_layer_
              << ", ransac_max_iter=" << config_.ransac_max_iter_
              << ", ransac_sample_num=" << config_.ransac_sample_num_;
}

void VoxelMapManager::UpdateMap(const std::vector<pointWithVar> &pv_list) {
    if (pv_list.empty()) return;

    std::unordered_map<VOXEL_LOCATION, OctoTree *, std::hash<VOXEL_LOCATION>> update_cache;

    for (const auto &pv : pv_list) {
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_w[j] / voxel_size_;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

        auto iter = feat_map_.find(position);
        if (iter != feat_map_.end()) {
            if (iter->second->root_update_enable_) {
                iter->second->new_points_.push_back(pv);
                iter->second->all_points_num_root_++;
                total_points_num_++;
            }
        } else {
            OctoTree *octo_tree = new OctoTree(0, config_);
            octo_tree->root = octo_tree;
            octo_tree->quater_length_ = voxel_size_ / 4;
            octo_tree->voxel_center_[0] = (0.5 + position.x) * voxel_size_;
            octo_tree->voxel_center_[1] = (0.5 + position.y) * voxel_size_;
            octo_tree->voxel_center_[2] = (0.5 + position.z) * voxel_size_;
            octo_tree->new_points_.push_back(pv);
            octo_tree->all_points_num_root_++;
            total_points_num_++;
            feat_map_[position] = octo_tree;
        }
        update_cache[position] = feat_map_[position];
    }

    for (auto &pair : update_cache) {
        pair.second->update_octo_tree();
    }
}

void VoxelMapManager::BuildResidualListOMP(const std::vector<pointWithVar> &pv_list,
                                           std::vector<PointToPlane> &ptpl_list) {
    if (pv_list.empty()) return;
    if (feat_map_.empty()) return;

    ptpl_list.clear();
    ptpl_list.resize(pv_list.size());

#pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < pv_list.size(); i++) {
        const auto &pv = pv_list[i];
        bool is_success = false;
        double prob = 0.0;
        PointToPlane &single_ptpl = ptpl_list[i];
        single_ptpl.is_valid_ = false;
        single_ptpl.source_idx_ = i;  // 记录源点索引

        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_w[j] / voxel_size_;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

        auto iter = feat_map_.find(position);
        if (iter != feat_map_.end()) {
            OctoTree *current_octo = iter->second;
            iter->second->build_single_residual(pv, is_success, prob, single_ptpl);

            if (!is_success) {
                VOXEL_LOCATION near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }

                if (near_position.x != position.x || near_position.y != position.y || near_position.z != position.z) {
                    auto iter_near = feat_map_.find(near_position);
                    if (iter_near != feat_map_.end()) {
                        iter_near->second->build_single_residual(pv, is_success, prob, single_ptpl);
                    }
                }
            }
        }
    }

    auto new_end = std::remove_if(ptpl_list.begin(), ptpl_list.end(),
                                  [](const PointToPlane &p) { return !p.is_valid_; });
    ptpl_list.erase(new_end, ptpl_list.end());
}

void VoxelMapManager::MapSliding(const Eigen::Vector3d &center) {
    double half_size = config_.half_map_size_ * voxel_size_;

    int center_loc[3];
    for (int j = 0; j < 3; j++) {
        center_loc[j] = (int)(center(j) / voxel_size_);
        if (center(j) < 0) center_loc[j] -= 1;
    }

    auto iter = feat_map_.begin();
    while (iter != feat_map_.end()) {
        const auto &loc = iter->first;
        double dx = fabs((loc.x - center_loc[0]) * voxel_size_);
        double dy = fabs((loc.y - center_loc[1]) * voxel_size_);
        double dz = fabs((loc.z - center_loc[2]) * voxel_size_);

        if (dx > half_size || dy > half_size || dz > half_size) {
            total_points_num_ -= iter->second->all_points_num_root_;
            delete iter->second;
            iter = feat_map_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void VoxelMapManager::CalcBodyCov(const Eigen::Vector3d &point, double ranging_cov, double angle_cov, Eigen::Matrix3d &body_var) {
    if (point[2] == 0) {
        body_var = Eigen::Matrix3d::Identity() * 1e-4;
        return;
    }
    double range = point.norm();
    Eigen::Vector3d direction = point / range;
    double range_var = ranging_cov * ranging_cov;
    double angle_var = pow(sin(angle_cov * M_PI / 180.0), 2);
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1),
                     direction(2), 0, -direction(0),
                     -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1 = direction.cross(Eigen::Vector3d::UnitX());
    if (base_vector1.norm() < 1e-6) {
        base_vector1 = direction.cross(Eigen::Vector3d::UnitY());
    }
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
