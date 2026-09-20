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

    // ===== BIEVR 凹凸图（Bump Image）方法 =====
    void updateBumpImage();                                        // 更新凹凸图（懒更新内部判断）
    void resetBumpImage();                                         // 法向量变化时重置凹凸图
    void buildBumpImageFromPoints(const std::list<pointWithVar> &points);  // 从点列表构建凹凸图
    void gaussianSmoothBump();                                     // 对凹凸图做高斯平滑
    void computeBumpGradients();                                   // 预计算凹凸图梯度缓存
    void computeBumpScore();                                       // 计算凹凸评分 + 平坦判定

    // ===== 凹凸图匹配方法 =====
    bool sampleBumpWithGradient(const Eigen::Vector3d &point_w,
                                float &bump_val,
                                float &dbump_du, float &dbump_dv) const;
    float sampleBumpBilinear(const Eigen::Vector3d &point_w, bool use_smooth = true) const;
    int countBumpValidPixels() const;                               // 统计凹凸图有效像素数
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

    // ===== BIEVR 凹凸图采样接口 =====
    int GetTotalPointsNum() const { return total_points_num_; }

    VOXEL_LOCATION GetVoxelLocation(const Eigen::Vector3d &point_w) const;

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