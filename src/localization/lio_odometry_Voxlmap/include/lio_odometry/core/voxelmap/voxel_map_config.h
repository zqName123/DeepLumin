#pragma once
#include <vector>
#include <Eigen/Core>

namespace slam_odometry {

struct VoxelMapConfig {
    // ===== 八叉树参数 =====
    double max_voxel_size_ = 3.0;          // 根体素边长 (米)
    int max_layer_ = 3;                    // 八叉树最大深度
    std::vector<int> layer_init_num_ = {5, 5, 5, 5, 5};  // 每层初始化所需最少点数
    int max_points_num_ = 100;             // 单体素最大点数（停止更新阈值）
    int max_cov_points_size_ = 1000;       // 最大参与协方差计算的点数
    int max_root_points_size_ = 2000;      // 根体素最大点数
    double planner_threshold_ = 0.01;      // 平面判定阈值（最小特征值）

    // ===== 传感器噪声 =====
    double beam_err_ = 0.02;               // 激光束测距误差 (m)
    double dept_err_ = 0.05;               // 深度方向误差 (m)

    // ===== 平面拟合模式选择 =====
    bool use_ransac_ = false;                 // false=EVD 直接分解, true=RANSAC+EVD+平面验证

    // ===== RANSAC（R-VoxelMap 通过 config 传递参数） =====
    int ransac_max_iter_ = 10;
    int ransac_sample_num_ = 4;
    double ransac_inlier_threshold_ = 0.1;
    double ransac_isplane_p_threshold_ = 0.8;
    int sample_seed_ = 42;

    // ===== 增量更新 =====
    int update_size_threshold_ = 5;        // 每 N 个新点触发一次更新
    double update_inlier_distance_threshold_ = 0.3;  // 点到平面距离阈值（匹配到已有平面）

    // ===== Reinit 重初始化 =====
    std::vector<int> reinit_size_vec_ = {100, 200, 300, 500, 700, 1000};
    int stop_reinit_threshold_ = 100000;   // 达到此点数后停止 reinit
    int reinit_threshold_ = 100;           // 旧版 reinit 阈值（每 N 个点 reinit 一次）

    // ===== Mahalanobis 门控 =====
    double sigma_num_ = 3.0;               // 3-sigma 门控

    // ===== 平面有效检查 =====
    int valid_check_max_layer_ = 0;        // 进行有效检查的最大层
    int valid_check_min_points_size_ = 5;  // 最小点数
    int valid_check_resolution_ = 4;       // 网格分辨率（valid_check）

    // ===== 滑动窗口 =====
    double sliding_thresh_ = 10.0;         // 滑动触发距离阈值 (米)
    bool map_sliding_en_ = true;           // 是否启用滑动窗口
    int half_map_size_ = 50;               // 半个地图窗口大小 (体素单位)

    // ===== LRU 缓存管理 =====
    int lru_capacity_ = 500;               // 体素数量上限

    // ===== 增量更新阈值 =====
    double update_inlier_threshold_ = 0.1; // 内点判定阈值 (m)

    // ===== 发布（可选） =====
    bool is_pub_plane_map_ = false;
};

} // namespace slam_odometry