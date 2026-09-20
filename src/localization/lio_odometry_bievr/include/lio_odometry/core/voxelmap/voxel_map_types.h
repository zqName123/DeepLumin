#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace slam_odometry {

static constexpr int kDefaultBumpImageSize = 16;  // 固定凹凸图分辨率
static constexpr int kDefaultBumpTotalPixels = kDefaultBumpImageSize * kDefaultBumpImageSize;  // 256

// ===== 点与协方差（参考 FAST-LIVO2 的 pointWithVar） =====
struct pointWithVar {
    Eigen::Vector3d point_b;        // body 坐标系下的点
    Eigen::Vector3d point_i;        // IMU 坐标系下的点
    Eigen::Vector3d point_w;        // 世界坐标系下的点
    Eigen::Matrix3d var_nostate;    // 去除状态协方差后的点协方差（仅传感器噪声）
    Eigen::Matrix3d body_var;       // body 坐标系下的协方差
    Eigen::Matrix3d var;            // 世界坐标系下的总协方差
    Eigen::Matrix3d point_crossmat; // 点的叉乘矩阵（用于 Jacobian 计算）
    Eigen::Vector3d normal;         // 法向量（用于残差计算）

    pointWithVar()
        : point_b(Eigen::Vector3d::Zero())
        , point_i(Eigen::Vector3d::Zero())
        , point_w(Eigen::Vector3d::Zero())
        , var_nostate(Eigen::Matrix3d::Zero())
        , body_var(Eigen::Matrix3d::Zero())
        , var(Eigen::Matrix3d::Zero())
        , point_crossmat(Eigen::Matrix3d::Zero())
        , normal(Eigen::Vector3d::Zero()) {}
};

// ===== 点面匹配结果（参考 FAST-LIVO2 的 PointToPlane） =====
struct VoxelPlane;   // 前置声明

struct PointToPlane {
    Eigen::Vector3d point_b_;       // body 坐标系点
    Eigen::Vector3d point_w_;       // 世界坐标系点
    Eigen::Vector3d normal_;        // 平面法向量
    Eigen::Vector3d center_;        // 平面中心
    Eigen::Matrix<double, 6, 6> plane_var_; // 平面参数协方差 (normal3 + center3)
    Eigen::Matrix3d body_cov_;      // body 坐标系协方差
    int layer_;                     // 匹配到的八叉树层
    double d_;                      // 平面方程: n·x + d = 0
    double eigen_value_;            // 最小特征值
    bool is_valid_;                 // 是否有效
    float dis_to_plane_;            // 点到平面距离
    int source_idx_ = -1;           // 对应原始点云在 pv_list_ 中的索引（用于 Jacobian 重算）

    VoxelPlane *plane_ptr_ = nullptr; // 所属体素平面（用于凹凸图匹配）
};

// ===== Voxel 平面信息（参考 FAST-LIVO2/R-VoxelMap 的 VoxelPlane） =====
struct VoxelPlane {
    Eigen::Vector3d center_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d y_normal_;
    Eigen::Vector3d x_normal_;
    Eigen::Matrix3d covariance_;
    Eigen::Matrix3d sum_ppt_;   // Σ p_i * p_i^T 累积和（用于 O(1) 递推更新）
    Eigen::Matrix<double, 6, 6> plane_var_;
    float radius_ = 0;
    float min_eigen_value_ = 1;
    float mid_eigen_value_ = 1;
    float max_eigen_value_ = 1;
    float d_ = 0;
    int points_size_ = 0;
    int last_update_points_size_ = 0;
    bool is_plane_ = false;
    bool is_init_ = false;
    int id_ = 0;
    bool is_update_ = false;
    bool cov_need_update_ = false;

    // ===== BIEVR 凹凸图（Bump Image）成员 =====
    bool bump_enabled = false;               // 此体素是否启用凹凸图
    bool bump_flat = false;                  // 平坦体素标记（bump≈0，自动禁用凹凸匹配）
    int bump_observed_pixels_ = 0;           // 凹凸图中有效（被观测到）的像素数
    double bump_mean_dist_ = 0.0;            // 平均 |bump| 值（用于采样评分和平坦判定）

    float bump_img_[kDefaultBumpTotalPixels] = {};       // 凹凸高度值（按行优先存储）
    float bump_weights_[kDefaultBumpTotalPixels] = {};   // 累加权重
    uint8_t bump_valid_[kDefaultBumpTotalPixels] = {};   // 有效掩码

    float bump_grad_du_[kDefaultBumpTotalPixels] = {};   // 预计算梯度 ∂bump/∂u
    float bump_grad_dv_[kDefaultBumpTotalPixels] = {};   // 预计算梯度 ∂bump/∂v
    bool bump_grad_dirty_ = true;                         // 梯度缓存是否需要重新计算

    double bump_pixel_size_ = 0.05;          // 像素物理尺寸
    double bump_u_min_ = 0.0;               // 图像 u 坐标最小值（世界单位）
    double bump_v_min_ = 0.0;               // 图像 v 坐标最小值（世界单位）
    Eigen::Vector3d bump_last_normal_ = Eigen::Vector3d::Zero(); // 上一次凹凸图对应的法向量

    int bump_last_update_size_ = 0;          // 上一次凹凸图更新时的 points_size_
    int bump_lazy_threshold_ = 5;            // 懒更新阈值

    inline int bumpIdx(int r, int c) const { return r * kDefaultBumpImageSize + c; }

    VoxelPlane() {
        plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        covariance_ = Eigen::Matrix3d::Zero();
        sum_ppt_ = Eigen::Matrix3d::Zero();
        center_ = Eigen::Vector3d::Zero();
        normal_ = Eigen::Vector3d::Zero();
    }
};

// ===== 体素坐标哈希（参考 FAST-LIVO2 的 VOXEL_LOCATION） =====
#define RV_HASH_P 116101
#define RV_MAX_N 10000000000

struct VOXEL_LOCATION {
    int64_t x, y, z;
    VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}
    bool operator==(const VOXEL_LOCATION &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

} // namespace slam_odometry

// Hash 特化（必须在 std 命名空间）
namespace std {
template <>
struct hash<slam_odometry::VOXEL_LOCATION> {
    int64_t operator()(const slam_odometry::VOXEL_LOCATION &s) const {
        using std::hash;
        return ((((s.z) * RV_HASH_P) % RV_MAX_N + (s.y)) * RV_HASH_P) % RV_MAX_N + (s.x);
    }
};
} // namespace std