#pragma once

/**
 * @file types.hpp
 * @brief dr_odometry_v2 核心数据类型（纯 C++，零 ROS）
 *
 * 状态约定：15 维误差状态 ESKF
 *   x = [p(3), v(3), θ(3), b_g(3), b_a(3)]
 * 姿态右扰动：q ← q ⊗ δq(δθ)，δθ 在机体系；协方差块序 [p|v|θ|bg|ba]。
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>

namespace dr_odometry_v2 {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat3d = Eigen::Matrix3d;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Vec15d = Eigen::Matrix<double, 15, 1>;

struct Pose3d {
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
};

// ============================== 传感器样本 ==============================

struct ImuSample {
  double stamp = 0.0;
  Vec3d gyro = Vec3d::Zero();   ///< rad/s（base 系，外参已转换）
  Vec3d accel = Vec3d::Zero();  ///< m/s² 比力（base 系）
};

struct WheelSample {
  double stamp = 0.0;
  double speed_mps = 0.0;  ///< 前进为正（倒档取负）
  bool valid = true;
};

struct GnssSample {
  double stamp = 0.0;
  // 姿态：roll/pitch（弧度）、heading（原始弧度，约定由 GnssConfig::heading_convention 解释）
  double roll = 0.0;
  double pitch = 0.0;
  double heading = 0.0;
  bool attitude_valid = false;
  double speed_mps = 0.0;  ///< 地速（航向低速门限用）
  // 加速度（gnss/accel_frame 决定解释）
  Vec3d accel = Vec3d::Zero();
  bool accel_valid = false;
  // 速度 / 位置（可选观测）
  Vec3d velocity_enu = Vec3d::Zero();
  bool velocity_valid = false;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt = 0.0;
  bool position_valid = false;
};

/// 定位结果观测（localization_v3 的 map->base + twist）
struct PoseSample {
  double stamp = 0.0;
  Pose3d map_base;
  Vec3d velocity_body = Vec3d::Zero();
  bool has_velocity = false;
};

// ============================== 状态 / 输出 ==============================

struct DrState {
  double timestamp = 0.0;
  Vec3d position = Vec3d::Zero();
  Vec3d velocity = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
  Vec3d gyro_bias = Vec3d::Zero();
  Vec3d accel_bias = Vec3d::Zero();
  Mat15d covariance = Mat15d::Identity();
  bool initialized = false;
};

struct OdomResult {
  double timestamp = 0.0;
  Pose3d pose;
  Vec3d velocity = Vec3d::Zero();
  bool valid = false;
};

struct DrHealth {
  std::uint64_t imu_count = 0;
  std::uint64_t wheel_count = 0;
  std::uint64_t gnss_count = 0;
  std::uint64_t pose_count = 0;
  std::uint64_t predict_count = 0;
  std::uint64_t wheel_update_count = 0;
  std::uint64_t gnss_attitude_update_count = 0;
  std::uint64_t gnss_accel_update_count = 0;
  std::uint64_t gnss_velocity_update_count = 0;
  std::uint64_t gnss_position_update_count = 0;
  std::uint64_t pose_update_count = 0;
  std::uint64_t pose_step_count = 0;        ///< 跳变确认后的台阶修正次数
  std::uint64_t pose_reject_count = 0;      ///< 伪跳变拒收次数
  std::uint64_t pose_pending_count = 0;     ///< 大跳待确认次数
  std::uint64_t invalid_sample_count = 0;
  std::uint64_t nonpositive_dt_count = 0;
  std::uint64_t stale_wheel_count = 0;
  std::uint64_t stale_gnss_count = 0;
  std::uint64_t stale_pose_count = 0;
  double last_update_time = 0.0;
};

// ============================== 配置（按功能分组） ==============================

struct ImuConfig {
  bool enabled = true;
  bool use_accel = true;  ///< 预测是否用加速度（false 时速度靠观测维持）
  double max_dt = 0.05;
  double gyro_noise = 0.01;
  double accel_noise = 0.2;
  double gyro_bias_noise = 1.0e-5;
  double accel_bias_noise = 1.0e-4;
  double gyro_bias_step_clamp = 2.0e-4;   ///< 零偏单步修正上限（rad/s/次），防瞬态误估暴走
  double accel_bias_step_clamp = 1.0e-3;  ///< 同上（m/s²/次）
  double gravity = 9.80665;
};

struct WheelConfig {
  bool enabled = true;
  bool speed_is_kmh = true;
  bool use_valid_flag = false;
  double velocity_noise = 0.25;      ///< 纵向
  double nonholonomic_noise = 0.10;  ///< 侧向/垂向
  double timeout = 0.5;
};

struct GnssConfig {
  bool enabled = true;
  bool use_attitude = true;
  /// 姿态观测模式：
  ///   relative：只用姿态变化量（基准=初始/静止重锚，世界系=初始姿态系，与 LIO 一致；默认）
  ///   absolute：绝对姿态（世界系锚 ENU，需航向约定换算）
  std::string attitude_mode = "relative";
  /// 静止时重锚基准。注意与 lio 的差异：lio 基准是可信的激光姿态，重锚无副作用；
  /// 本模块基准是滤波状态本身，静止重锚会让 GNSS 停止修正漂移、压制零偏学习——默认关。
  bool rezero_when_static = false;
  double static_speed_eps = 0.1;      ///< 静止判定速度阈值（m/s）
  double attitude_noise_roll_deg = 1.0;
  double attitude_noise_pitch_deg = 1.0;
  double attitude_noise_yaw_deg = 2.0;
  /// 航向约定（2026-07-14 bag 实测确认 GPCHC 为罗盘约定）：
  ///   compass：北基准顺时针（GPCHC 默认）→ yaw = π/2 - heading
  ///   enu    ：东基准逆时针 → yaw = heading
  /// 相对模式下绝对基准消去，仅转向符号依赖此约定。
  std::string heading_convention = "compass";
  double heading_offset_deg = 0.0;    ///< 约定换算后的附加零位偏移
  double min_speed_for_heading = 0.2; ///< 低于此地速 yaw 噪声拉大
  bool use_accel = false;
  std::string accel_frame = "world";  ///< world | body
  double accel_noise = 0.5;
  bool use_velocity = false;
  double velocity_noise = 0.4;
  bool use_position = false;
  double position_noise = 1.5;
  double timeout = 1.0;
};

struct LocalizationConfig {
  bool enabled = true;
  bool require_tracking = true;    ///< status.state==TRACKING && output_valid
  bool require_slam_fresh = true;  ///< last_slam_age 限内（防 DR 模式回喂闭环）
  double slam_age_limit = 0.2;
  double max_latency = 0.5;
  double pose_noise_trans = 0.05;
  double pose_noise_yaw_deg = 0.5;
  double pose_noise_att_deg = 0.5;
  double vel_noise = 0.2;
  // 跳变判断三档（防伪跳变污染）
  double jump_soft_trans = 0.5;
  double jump_soft_yaw_deg = 2.0;
  double jump_hard_trans = 5.0;
  double jump_hard_yaw_deg = 15.0;
  int jump_confirm_frames = 3;
  double jump_confirm_consistency_trans = 0.5;
  double jump_confirm_consistency_yaw_deg = 2.0;
};

struct OutputConfig {
  double rate = 100.0;
  bool publish_tf = false;
  int path_max_size = 100000;        ///< 轨迹长度上限（稀疏追加后约 1~2h）
  double path_min_step = 0.05;       ///< 轨迹稀疏追加步长（m）；0=每帧追加
  double path_publish_rate = 2.0;    ///< 轨迹发布频率（Hz，累计量无需高频）
  double max_extrapolate = 0.1;
};

struct TimeSyncConfig {
  int queue_size = 512;
  int reordered_drop_warn = 100;
};

struct DrConfig {
  ImuConfig imu;
  WheelConfig wheel;
  GnssConfig gnss;
  LocalizationConfig localization;
  OutputConfig output;
  TimeSyncConfig time_sync;
  double initial_yaw_deg = 0.0;
};

}  // namespace dr_odometry_v2
