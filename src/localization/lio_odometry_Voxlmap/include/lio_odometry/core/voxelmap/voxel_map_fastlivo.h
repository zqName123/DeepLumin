#pragma once
#include "lio_odometry/core/voxelmap/voxel_map_types.h"
#include "lio_odometry/core/voxelmap/voxel_map_config.h"
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <limits>
#include <Eigen/Dense>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

namespace slam_odometry {

static int voxel_plane_id = 0;

class VoxelOctoTreeFASTLIVO {
public:
    VoxelOctoTreeFASTLIVO() = default;

    // ===== 点存储（使用 list 支持 splice 零拷贝） =====
    std::list<pointWithVar> plane_points_;      // 内点列表（属于平面的点）
    std::list<pointWithVar> not_plane_points_;  // 外点列表（离群点/非平面点）
    std::list<pointWithVar> new_points_;        // 待处理的新点

    VoxelPlane *plane_ptr_;
    int layer_;
    int octo_state_; // 0: leaf, 1: not leaf
    VoxelOctoTreeFASTLIVO *leaves_[8];
    double voxel_center_[3];
    std::vector<int> layer_init_num_;
    float quater_length_;
    float planer_threshold_;
    int points_size_threshold_;
    int update_size_threshold_;
    int max_points_num_;
    int max_layer_;
    int new_points_num_;  // 新点计数器
    bool init_octo_;
    bool update_enable_;

    // ===== 新增：reinit 机制 =====
    VoxelOctoTreeFASTLIVO *root;
    int all_points_num_root_;
    int all_points_num_;
    bool stop_reinit_;
    int reinit_state_;
    std::vector<int> reinit_size_vec_;

    // ===== 新增：增量更新阈值 =====
    double update_inlier_threshold_;

    // ===== 新增：RANSAC 模式开关和 config 引用 =====
    bool use_ransac_;
    const VoxelMapConfig *config_ptr_;

    // ===== 新增：根体素点数上限（对齐 R-VoxelMap max_root_points_size） =====
    int max_root_points_size_;
    bool root_update_enable_;

    VoxelOctoTreeFASTLIVO(int max_layer, int layer, int points_size_threshold,
                          int max_points_num, float planer_threshold)
        : max_layer_(max_layer), layer_(layer),
          points_size_threshold_(points_size_threshold),
          max_points_num_(max_points_num),
          planer_threshold_(planer_threshold) {
        new_points_.clear();
        plane_points_.clear();
        not_plane_points_.clear();
        octo_state_ = 0;
        new_points_num_ = 0;
        all_points_num_ = 0;
        all_points_num_root_ = 0;
        init_octo_ = false;
        update_enable_ = true;
        root = this;
        stop_reinit_ = false;
        reinit_state_ = 0;
        update_inlier_threshold_ = 0.1;
        use_ransac_ = false;
        config_ptr_ = nullptr;
        max_root_points_size_ = 2000;
        root_update_enable_ = true;
        for (int i = 0; i < 8; i++) {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new VoxelPlane;
    }

    ~VoxelOctoTreeFASTLIVO() {
        delete plane_ptr_;
        for (int i = 0; i < 8; i++) {
            delete leaves_[i];
        }
    }

    // ===== 核心方法 =====
    void init_plane(const std::list<pointWithVar> &points, VoxelPlane *plane);
    void init_octo_tree();
    void cut_octo_tree();
    void update_octo_tree();
    void update_single_voxel();

    // ===== 新增：智能点插入 =====
    VoxelOctoTreeFASTLIVO *push_to_subvoxel(const pointWithVar &pv);
    std::vector<VoxelOctoTreeFASTLIVO *> find_subvoxel_candidates(const pointWithVar &pv);
    bool check_and_update(const pointWithVar &pv);

    // ===== 新增：reinit 机制 =====
    void reinit_octo_tree();
    void collect_points(std::list<pointWithVar> &collect_list, int collect_layer);

    // ===== 匹配方法 =====
    void build_single_residual(pointWithVar &pv, const int current_layer,
                               bool &is_success, double &prob,
                               PointToPlane &single_ptpl,
                               double sigma_num, int max_layer);

    // ===== RANSAC 平面拟合（可选模块） =====
    void init_plane_ransac();
    bool ransac();
    bool plane_valid_check();
};

class VoxelMapManagerFASTLIVO {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoxelMapManagerFASTLIVO() = default;
    ~VoxelMapManagerFASTLIVO();

    void Init(const VoxelMapConfig &config);

    void UpdateMap(const std::vector<pointWithVar> &input_points);

    void BuildResidualListOMP(const std::vector<pointWithVar> &pv_list,
                              std::vector<PointToPlane> &ptpl_list);

    void MapSliding(const Eigen::Vector3d &center);

    static void CalcBodyCov(const Eigen::Vector3d &point,
                            double ranging_cov, double angle_cov,
                            Eigen::Matrix3d &body_var);

    int GetTotalPointsNum() const { return total_points_num_; }

    const std::unordered_map<VOXEL_LOCATION, VoxelOctoTreeFASTLIVO *,
                             std::hash<VOXEL_LOCATION>> &
    GetMap() const { return voxel_map_; }

private:
    VoxelMapConfig config_;
    double voxel_size_;
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTreeFASTLIVO *,
                       std::hash<VOXEL_LOCATION>> voxel_map_;
    int total_points_num_ = 0;
    Eigen::Vector3d last_slide_pos_ = Eigen::Vector3d::Zero();

    // ===== LRU 缓存管理 =====
    std::list<VOXEL_LOCATION> lru_cache_;
    std::unordered_map<VOXEL_LOCATION, std::list<VOXEL_LOCATION>::iterator> loc2lru_;
    int lru_capacity_;
    std::mutex lru_mutex_;

    enum AccessType { ACCESS_READ, ACCESS_WRITE };

    void UpdateLRU(const VOXEL_LOCATION &position);
    void LRUEvict();
    void clearMemOutOfMap(int x_max, int x_min, int y_max, int y_min,
                          int z_max, int z_min);
};

} // namespace slam_odometry