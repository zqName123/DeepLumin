/**
 * @file ros_adapter.cpp
 * @brief ROS 适配层实现：参数读写、传感器消息解析、输出消息装配。
 */

#include "dr_odometry/ros/ros_adapter.hpp"
#include "dr_odometry/core/math_utils.hpp"

#include <cmath>
#include <vector>

namespace dr_odometry_ros {
namespace {
constexpr double kDegToRad = M_PI / 180.0;

/**
 * @brief GPCHC 航向角 → ENU yaw。
 *
 * GPCHC heading：0° 指北，顺时针为正（导航惯例）；
 * ENU yaw：0 rad 指东，逆时针为正（右手系）。
 * 转换：yaw_rad = normalize( (90° - heading_deg) * π/180 )。
 */
double headingDegToYawRad(double heading_deg) {
  return dr_odometry::math::normalizeAngle((90.0 - heading_deg) * kDegToRad);
}

/** @brief 粗判 GNSS 解状态：status>0 视为可用（具体枚举依华测/驱动定义）。 */
bool gnssStatusValid(int status) {
  return status > 0;
}

bool loadVector3(ros::NodeHandle& nh, const std::string& key, dr_odometry::Vec3d& value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 3) {
    return false;
  }
  value << flat[0], flat[1], flat[2];
  return true;
}

bool loadMatrix3(ros::NodeHandle& nh, const std::string& key, dr_odometry::Mat3d& value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 9) {
    return false;
  }
  value << flat[0], flat[1], flat[2], flat[3], flat[4], flat[5], flat[6], flat[7], flat[8];
  return true;
}

SensorExtrinsic loadSensorExtrinsic(ros::NodeHandle& nh, const std::string& key) {
  SensorExtrinsic ext;
  loadVector3(nh, key + "/translation", ext.translation);
  loadMatrix3(nh, key + "/rotation", ext.rotation);
  return ext;
}

double yawFromRotation(const dr_odometry::Mat3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}
}  // namespace

dr_odometry::DrConfig loadDrConfig(ros::NodeHandle& nh) {
  dr_odometry::DrConfig cfg;
  // 发布与观测开关
  nh.param<double>("dr/output_rate", cfg.output_rate, cfg.output_rate);
  nh.param<bool>("dr/use_imu_accel", cfg.use_imu_accel, cfg.use_imu_accel);
  nh.param<bool>("dr/use_wheel", cfg.use_wheel, cfg.use_wheel);
  nh.param<bool>("dr/use_gnss", cfg.use_gnss, cfg.use_gnss);
  nh.param<bool>("dr/use_gnss_position", cfg.use_gnss_position, cfg.use_gnss_position);
  nh.param<bool>("dr/use_gnss_velocity", cfg.use_gnss_velocity, cfg.use_gnss_velocity);
  nh.param<bool>("dr/use_gnss_heading", cfg.use_gnss_heading, cfg.use_gnss_heading);
  nh.param<bool>("dr/publish_tf", cfg.publish_tf, cfg.publish_tf);
  // 噪声与门限（缺省用 DrConfig 默认值，再被 yaml 覆盖）
  nh.param<double>("dr/gyro_noise", cfg.gyro_noise, cfg.gyro_noise);
  nh.param<double>("dr/accel_noise", cfg.accel_noise, cfg.accel_noise);
  nh.param<double>("dr/gyro_bias_noise", cfg.gyro_bias_noise, cfg.gyro_bias_noise);
  nh.param<double>("dr/accel_bias_noise", cfg.accel_bias_noise, cfg.accel_bias_noise);
  nh.param<double>("dr/wheel_velocity_noise", cfg.wheel_velocity_noise, cfg.wheel_velocity_noise);
  nh.param<double>("dr/nonholonomic_noise", cfg.nonholonomic_noise, cfg.nonholonomic_noise);
  nh.param<double>("dr/gnss_heading_noise_deg", cfg.gnss_heading_noise_deg, cfg.gnss_heading_noise_deg);
  nh.param<double>("dr/gnss_position_noise", cfg.gnss_position_noise, cfg.gnss_position_noise);
  nh.param<double>("dr/gnss_velocity_noise", cfg.gnss_velocity_noise, cfg.gnss_velocity_noise);
  nh.param<double>("dr/max_dt", cfg.max_dt, cfg.max_dt);
  nh.param<double>("dr/min_wheel_speed_for_heading", cfg.min_wheel_speed_for_heading,
                   cfg.min_wheel_speed_for_heading);
  nh.param<double>("dr/gnss_timeout", cfg.gnss_timeout, cfg.gnss_timeout);
  nh.param<double>("dr/wheel_timeout", cfg.wheel_timeout, cfg.wheel_timeout);
  nh.param<double>("dr/initial_yaw_deg", cfg.initial_yaw_deg, cfg.initial_yaw_deg);
  return cfg;
}

TopicConfig loadTopicConfig(ros::NodeHandle& nh) {
  TopicConfig cfg;
  nh.param<std::string>("topics/imu", cfg.imu, cfg.imu);
  nh.param<std::string>("topics/can", cfg.can, cfg.can);
  nh.param<std::string>("topics/gnss", cfg.gnss, cfg.gnss);
  nh.param<std::string>("topics/odom", cfg.odom, cfg.odom);
  nh.param<std::string>("topics/status", cfg.status, cfg.status);
  nh.param<std::string>("topics/path", cfg.path, cfg.path);
  return cfg;
}

FrameConfig loadFrameConfig(ros::NodeHandle& nh) {
  FrameConfig cfg;
  nh.param<std::string>("frames/odom", cfg.odom, cfg.odom);
  nh.param<std::string>("frames/base_link", cfg.base_link, cfg.base_link);
  nh.param<std::string>("frames/imu", cfg.imu, cfg.imu);
  nh.param<std::string>("frames/gnss", cfg.gnss, cfg.gnss);
  nh.param<std::string>("frames/can", cfg.can, cfg.can);
  return cfg;
}

ExtrinsicConfig loadExtrinsicConfig(ros::NodeHandle& nh) {
  ExtrinsicConfig cfg;
  cfg.base_to_imu = loadSensorExtrinsic(nh, "extrinsics/base_to_imu");
  cfg.base_to_gnss = loadSensorExtrinsic(nh, "extrinsics/base_to_gnss");
  cfg.base_to_can = loadSensorExtrinsic(nh, "extrinsics/base_to_can");
  return cfg;
}

dr_odometry::ImuData fromRos(const sensor_msgs::Imu& msg) {
  dr_odometry::ImuData data;
  data.timestamp = msg.header.stamp.toSec();
  data.gyro = dr_odometry::Vec3d(msg.angular_velocity.x, msg.angular_velocity.y,
                                 msg.angular_velocity.z);
  data.accel = dr_odometry::Vec3d(msg.linear_acceleration.x, msg.linear_acceleration.y,
                                  msg.linear_acceleration.z);
  return data;
}

dr_odometry::ImuData transformImuToBase(const dr_odometry::ImuData& data,
                                        const SensorExtrinsic& base_to_imu) {
  dr_odometry::ImuData out = data;
  out.gyro = base_to_imu.rotation * data.gyro;
  out.accel = base_to_imu.rotation * data.accel;
  return out;
}

dr_odometry::CanData fromRos(const deeplumin_msgs::CanReceiveInfo& msg, bool speed_is_kmh,
                              bool use_valid_flag) {
  dr_odometry::CanData data;
  // stamp 为空时用当前时间，避免未知时间戳被滤波判定为过期
  data.timestamp = msg.header.stamp.isZero() ? ros::Time::now().toSec() : msg.header.stamp.toSec();
  // gear≈2 视为倒档，速度取负；其余前进为正
  const double direction = (static_cast<int>(std::round(msg.gear)) == 2) ? -1.0 : 1.0;
  data.speed_mps = direction * msg.speed * (speed_is_kmh ? (1.0 / 3.6) : 1.0);
  data.gear = msg.gear;
  data.valid = use_valid_flag ? msg.valid : std::isfinite(data.speed_mps);
  return data;
}

dr_odometry::GnssData fromRos(const deeplumin_msgs::Gpchc& msg) {
  dr_odometry::GnssData data;
  data.timestamp = msg.header.stamp.toSec();
  data.latitude = msg.latitude;
  data.longitude = msg.longitude;
  data.altitude = msg.altitude;
  data.heading_rad = headingDegToYawRad(msg.heading);
  data.pitch_rad = msg.pitch * kDegToRad;
  data.roll_rad = msg.roll * kDegToRad;
  // GPCHC：ve/vn/vu 为东/北/天速度
  data.velocity_enu = dr_odometry::Vec3d(msg.ve, msg.vn, msg.vu);
  data.speed_mps = msg.v;
  data.nsv1 = msg.nsv1;
  data.nsv2 = msg.nsv2;
  data.status = msg.status;
  // 各观测独立有效性：供 ESKF correctGnss 分支门控
  data.heading_valid = gnssStatusValid(msg.status) && std::isfinite(msg.heading);
  data.position_valid = gnssStatusValid(msg.status) && std::isfinite(msg.latitude) &&
                        std::isfinite(msg.longitude) && std::abs(msg.latitude) > 1e-8 &&
                        std::abs(msg.longitude) > 1e-8;
  data.velocity_valid = gnssStatusValid(msg.status) && std::isfinite(msg.ve) &&
                        std::isfinite(msg.vn) && std::isfinite(msg.vu);
  return data;
}

dr_odometry::GnssData transformGnssToBase(const dr_odometry::GnssData& data,
                                          const SensorExtrinsic& base_to_gnss) {
  dr_odometry::GnssData out = data;
  const double sensor_yaw_in_base = yawFromRotation(base_to_gnss.rotation);
  out.heading_rad = dr_odometry::math::normalizeAngle(data.heading_rad - sensor_yaw_in_base);
  return out;
}

nav_msgs::Odometry toRosOdom(const dr_odometry::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(odom.timestamp);
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.pose.pose.position.x = odom.position.x();
  msg.pose.pose.position.y = odom.position.y();
  msg.pose.pose.position.z = odom.position.z();
  msg.pose.pose.orientation.x = odom.orientation.x();
  msg.pose.pose.orientation.y = odom.orientation.y();
  msg.pose.pose.orientation.z = odom.orientation.z();
  msg.pose.pose.orientation.w = odom.orientation.w();
  // 6x6 pose 协方差行主序展平到 36 元数组
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = odom.covariance(i / 6, i % 6);
  }
  // twist 在输出中给的是世界系（odom）线速度；角速度未填
  msg.twist.twist.linear.x = odom.velocity.x();
  msg.twist.twist.linear.y = odom.velocity.y();
  msg.twist.twist.linear.z = odom.velocity.z();
  return msg;
}

geometry_msgs::TransformStamped toRosTransform(const dr_odometry::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame) {
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time(odom.timestamp);
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;
  tf.transform.translation.x = odom.position.x();
  tf.transform.translation.y = odom.position.y();
  tf.transform.translation.z = odom.position.z();
  tf.transform.rotation.x = odom.orientation.x();
  tf.transform.rotation.y = odom.orientation.y();
  tf.transform.rotation.z = odom.orientation.z();
  tf.transform.rotation.w = odom.orientation.w();
  return tf;
}

deeplumin_msgs::LocalizationStatus toStatusMsg(const dr_odometry::DrHealth& health,
                                               const ros::Time& stamp) {
  deeplumin_msgs::LocalizationStatus msg;
  msg.header.stamp = stamp;
  // 当前策略：无 IMU 则定位失败；有 IMU 视为 DR 可用（轮速/GNSS 质量后续可细化）
  msg.level = health.has_imu ? deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL
                             : deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE;
  msg.failure_reason = health.has_imu ? "" : "no imu input";
  msg.quality_score = health.has_imu ? 1.0f : 0.0f;
  msg.is_gnss_available = health.has_gnss;
  msg.is_slam_available = false;
  msg.is_dr_available = health.has_imu;
  msg.current_mode = "DR_ODOMETRY";
  return msg;
}

}  // namespace dr_odometry_ros
