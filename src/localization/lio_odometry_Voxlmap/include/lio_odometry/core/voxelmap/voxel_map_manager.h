#pragma once
#include "voxel_map_types.h"
#include "voxel_map_config.h"
#include "voxel_map_octree.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>

namespace slam_odometry {

class VoxelMapManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoxelMapManager() = default;
    ~VoxelMapManager();

    void Init(const VoxelMapConfig &config);

    void UpdateMap(const std::vector<pointWithVar> &pv_list);

    void BuildResidualListOMP(const std::vector<pointWithVar> &pv_list,
                              std::vector<PointToPlane> &ptpl_list);

    void MapSliding(const Eigen::Vector3d &center);

    static void CalcBodyCov(const Eigen::Vector3d &point, double ranging_cov, double angle_cov, Eigen::Matrix3d &body_var);

    int GetTotalPointsNum() const { return total_points_num_; }

    const std::unordered_map<VOXEL_LOCATION, OctoTree *> &GetFeatMap() const { return feat_map_; }

private:
    VoxelMapConfig config_;
    double voxel_size_;
    std::unordered_map<VOXEL_LOCATION, OctoTree *> feat_map_;
    int total_points_num_ = 0;
};

} // namespace slam_odometry
