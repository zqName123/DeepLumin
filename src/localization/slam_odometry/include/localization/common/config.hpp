/**
 * @file config.hpp
 * @brief 各子模块配置结构体（由 ros_converters 从 YAML 加载）
 *
 * 注意 slam/faster_lio/mapping/* 为 LIO 内部 IMU 噪声，
 * 与 dr/imu/* 独立（前者给 Faster-LIO，后者给 DR/ESKF）
 */
#pragma once

#include "localization/common/types.hpp"
#include <string>
#include <vector>

namespace localization {

/**
 * DR 模块配置（YAML: dr/*，由 loadDrConfig 填充）
 * 流向：dr_debug → DrOdometryEstimator::initialize → DrEskfCore
 */
struct DrConfig {
    double output_rate = 100.0;          ///< tick 发布频率 (Hz)

    // IMU 过程噪声（进入 ESKF 的 Q）
    double gyro_noise = 0.01;
    double accel_noise = 0.1;
    double gyro_bias_noise = 0.0001;
    double accel_bias_noise = 0.001;

    Vec3d initial_gyro_bias = Vec3d::Zero();
    Vec3d initial_accel_bias = Vec3d::Zero();
    double initial_wheel_scale = 1.0;
    double wheel_velocity_noise = 0.05;  ///< 轮速观测噪声下限 (m/s)
    double initial_bias_cov = 0.01;      ///< P 中 bg/ba 初始协方差对角元

    // 在线标定：静止 ZUPT + 直行时 wheel_scale 低通
    bool online_calibration_enable = true;
    double static_detection_duration = 2.0;   ///< 连续静止多久触发 ZUPT (s)
    double static_gyro_threshold = 0.02;      ///< |ω-bg| 静止阈值
    double static_accel_threshold = 0.15;     ///< ||a||-g 静止阈值
    double wheel_scale_update_interval = 10.0;
};

struct SlamConfig {
    std::string type = "faster_lio";
    double scan_rate = 10.0;

    // preprocess（与 ysw_loc ouster64.yaml 对齐）
    int lidar_type = 3;
    int scan_line = 64;
    double blind = 3.0;
    double time_scale = 1e-3;
    int point_filter_num = 2;

    // mapping IMU 噪声（LIO 内部，与 DR 的 imu 噪声独立）
    double acc_cov = 0.1;
    double gyr_cov = 0.1;
    double b_acc_cov = 0.0001;
    double b_gyr_cov = 0.0001;
    double fov_degree = 180.0;
    double det_range = 150.0;

    // iVox / 滤波
    double ivox_grid_resolution = 0.5;
    int ivox_nearby_type = 18;
    double cube_side_length = 2000.0;
    double filter_size_surf = 0.2;
    double filter_size_map = 0.2;
    int max_iteration = 3;
    double esti_plane_threshold = 0.1;

    bool estimate_extrinsic = false;
    Vec3d extrinsic_T = Vec3d::Zero();
    Mat3d extrinsic_R = Mat3d::Identity();
    double degeneracy_threshold = 100.0;
};

struct FusionConfig {
    double output_rate = 100.0;
    double initial_position_cov = 1.0;
    double initial_velocity_cov = 1.0;
    double initial_rotation_cov = 0.1;
    double initial_bias_cov = 0.01;
    double initial_scale_cov = 0.01;
    bool adaptive_covariance_enable = true;
    double slam_position_noise = 0.05;
    double slam_rotation_noise = 0.02;
    double gnss_position_noise = 0.1;
    int smooth_transition_frames = 10;
};

struct KeyFrameConfig {
    double dist_thresh = 1.0;
    double rad_thresh = 0.2;
    double time_thresh = 0.5;
};

struct DescriptorConfig {
    std::string type = "scan_context";
    int ring_num = 20;
    int sector_num = 60;
    double max_radius = 80.0;
};

struct MatcherConfig {
    std::string type = "ndt";
    double resolution = 1.0;
    int max_iterations = 30;
    double transformation_epsilon = 0.01;
    double fitness_thresh = 0.3;
};

struct LoopClosureConfig {
    bool activate = true;
    int top_k = 5;
    int min_index_gap = 20;
    double time_thresh = 30.0;
    DescriptorConfig descriptor;
    MatcherConfig verification;
};

struct PoseGraphConfig {
    std::string optimizer = "gtsam";
    Vec3d odom_noise_rot = Vec3d(1e-6, 1e-6, 1e-6);
    Vec3d odom_noise_pos = Vec3d(1e-4, 1e-4, 1e-6);
    double loop_noise_scale = 1.0;
};

struct RelocConfig {
    bool enable = false;
    double thread_rate = 1.0;
    double pose_jump_threshold = 2.0;
    double cov_threshold = 50.0;
    int degenerate_frames_threshold = 30;
    DescriptorConfig global_descriptor;
    MatcherConfig fine_matcher;
    RelocSearchConfig search;
    int smooth_transition_frames = 10;
};

struct SceneConfig {
    SceneState default_state = SceneState::UNDERGROUND;
    double gnss_signal_strength_threshold = 30.0;
    double fade_decay_rate = 0.95;
    double transition_timeout = 10.0;
};

/** TF/里程计坐标系名；dr_debug 主要用 odom + base_link */
struct FrameConfig {
    std::string map = "map";
    std::string odom = "odom";
    std::string base_link = "base_link";
    std::string imu = "imu_link";
    std::string lidar = "lidar_link";
};

}  // namespace localization
