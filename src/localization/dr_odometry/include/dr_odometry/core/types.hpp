#pragma once

/**
 * @file types.hpp
 * @brief DR 核心层公共数据类型（与 ROS 解耦）。
 *
 * 状态量布局（误差状态 15 维，与 DrEskf 协方差块索引一致）：
 *   [0:3)   δp   位置误差 (m)
 *   [3:6)   δv   速度误差 (m/s)
 *   [6:9)   δθ   姿态误差 (rad，小角度)
 *   [9:12)  δbg  陀螺零偏误差 (rad/s)
 *   [12:15) δba  加计零偏误差 (m/s^2)
 *
 * 坐标系约定：
 *   - 世界系近似 ENU（东-北-天），重力为 (0,0,-g)；
 *   - 姿态四元数表示 body→world；
 *   - 车体速度观测中 x 为前进方向。
 */

#include <Eigen/Dense>
#include <string>

namespace dr_odometry {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat3d = Eigen::Matrix3d;
using Mat15d = Eigen::Matrix<double, 15, 15>;

/** @brief IMU 一次采样（角速度、比力）。单位：rad/s，m/s^2。 */
struct ImuData {
  double timestamp = 0.0;
  Vec3d gyro = Vec3d::Zero();
  Vec3d accel = Vec3d::Zero();
};

/**
 * @brief 底盘/CAN 车速观测。
 * speed_mps：已带符号的车体前进速度（倒车为负），单位 m/s。
 */
struct CanData {
  double timestamp = 0.0;
  double speed_mps = 0.0;
  double gear = 0.0;
  bool valid = true;
};

/**
 * @brief GNSS / 组合导航观测（由 Gpchc 适配而来）。
 * heading_rad 等已是 ENU 语义；*_valid 供滤波分支门控。
 */
struct GnssData {
  double timestamp = 0.0;
  double latitude = 0.0;   ///< 度
  double longitude = 0.0;  ///< 度
  double altitude = 0.0;   ///< 米
  double heading_rad = 0.0;
  double pitch_rad = 0.0;
  double roll_rad = 0.0;
  Vec3d velocity_enu = Vec3d::Zero();  ///< (ve, vn, vu)
  double speed_mps = 0.0;
  int nsv1 = 0;
  int nsv2 = 0;
  int status = 0;
  bool heading_valid = false;
  bool position_valid = false;
  bool velocity_valid = false;
};

/** @brief 滤波器与节点运行参数（由 yaml dr/* 加载，debug 节点可覆盖开关）。 */
struct DrConfig {
  double output_rate = 100.0;
  bool use_imu_accel = true;
  bool use_wheel = true;
  bool use_gnss = true;
  bool use_gnss_position = false;
  bool use_gnss_velocity = true;
  bool use_gnss_heading = true;
  bool publish_tf = true;

  double gyro_noise = 0.01;
  double accel_noise = 0.2;
  double gyro_bias_noise = 1.0e-5;
  double accel_bias_noise = 1.0e-4;
  double wheel_velocity_noise = 0.25;
  double nonholonomic_noise = 0.10;
  double gnss_heading_noise_deg = 2.0;
  double gnss_position_noise = 1.5;
  double gnss_velocity_noise = 0.4;
  double gravity = 9.80665;
  double max_dt = 0.05;
  double min_wheel_speed_for_heading = 0.2;
  double gnss_timeout = 1.0;
  double wheel_timeout = 0.5;
  double initial_yaw_deg = 0.0;
  Vec3d initial_gyro_bias = Vec3d::Zero();
  Vec3d initial_accel_bias = Vec3d::Zero();
  Vec3d initial_position = Vec3d::Zero();
};

/**
 * @brief ESKF 标称状态（名义量）+ 15×15 协方差。
 * initialized：收到首帧有效 IMU 后置 true，节点据此决定是否发布。
 */
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

/** @brief 健康/可用性摘要，映射到 LocalizationStatus。 */
struct DrHealth {
  bool has_imu = false;
  bool has_wheel = false;
  bool has_gnss = false;
  bool gnss_heading_used = false;
  bool gnss_position_used = false;
  bool wheel_used = false;
  double last_update_time = 0.0;
  double wheel_age = 1e9;
  double gnss_age = 1e9;
};

/**
 * @brief 对外里程计结果。
 * covariance：6×6，块顺序为 [位置 3 + 姿态 3]（姿态取自误差状态 δθ 的协方差）。
 */
struct OdomResult {
  double timestamp = 0.0;
  Vec3d position = Vec3d::Zero();
  Vec3d velocity = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  bool valid = false;
  std::string frame_id = "odom";
};

}  // namespace dr_odometry
