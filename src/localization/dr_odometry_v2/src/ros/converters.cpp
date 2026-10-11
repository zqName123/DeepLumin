#include "dr_odometry_v2/ros/converters.hpp"

#include <cmath>

#include "dr_odometry_v2/core/math_utils.hpp"

namespace dr_odometry_v2_ros {

namespace {
bool gnssStatusValid(int status) { return status > 0; }
}  // namespace

ImuSample fromRos(const sensor_msgs::Imu& msg, const Extrinsic& base_to_imu) {
  ImuSample out;
  out.stamp = msg.header.stamp.toSec();
  // 外参仅取旋转（加计/陀螺为自由向量）：T_base_sensor 的旋转把 sensor 系向量转到 base
  out.gyro = base_to_imu.rotation * Vec3d(msg.angular_velocity.x, msg.angular_velocity.y,
                                          msg.angular_velocity.z);
  out.accel = base_to_imu.rotation * Vec3d(msg.linear_acceleration.x, msg.linear_acceleration.y,
                                           msg.linear_acceleration.z);
  return out;
}

WheelSample fromRos(const deeplumin_msgs::CanReceiveInfo& msg, const WheelConfig& cfg) {
  WheelSample out;
  out.stamp = msg.header.stamp.isZero() ? 0.0 : msg.header.stamp.toSec();
  // gear≈2 视为倒档，速度取负
  const double direction = (static_cast<int>(std::round(msg.gear)) == 2) ? -1.0 : 1.0;
  out.speed_mps = direction * msg.speed * (cfg.speed_is_kmh ? (1.0 / 3.6) : 1.0);
  out.valid = cfg.use_valid_flag ? msg.valid : std::isfinite(out.speed_mps);
  return out;
}

GnssSample fromRos(const deeplumin_msgs::Gpchc& msg) {
  GnssSample out;
  out.stamp = msg.header.stamp.toSec();
  const bool valid = gnssStatusValid(msg.status);

  out.heading = math::deg2rad(msg.heading);  // 约定换算（compass/enu）由 eskf 应用
  out.pitch = math::deg2rad(msg.pitch);
  out.roll = math::deg2rad(msg.roll);
  out.attitude_valid = valid && std::isfinite(msg.heading) && std::isfinite(msg.pitch) &&
                       std::isfinite(msg.roll);
  out.speed_mps = std::isfinite(msg.v) ? msg.v : 0.0;

  out.accel = Vec3d(msg.acc_x, msg.acc_y, msg.acc_z);
  out.accel_valid = valid && std::isfinite(msg.acc_x) && std::isfinite(msg.acc_y) &&
                    std::isfinite(msg.acc_z);

  out.velocity_enu = Vec3d(msg.ve, msg.vn, msg.vu);
  out.velocity_valid = valid && std::isfinite(msg.ve) && std::isfinite(msg.vn) &&
                       std::isfinite(msg.vu);

  out.lat_deg = msg.latitude;
  out.lon_deg = msg.longitude;
  out.alt = msg.altitude;
  out.position_valid = valid && std::isfinite(msg.latitude) && std::isfinite(msg.longitude) &&
                       std::abs(msg.latitude) > 1e-8 && std::abs(msg.longitude) > 1e-8;
  return out;
}

PoseSample fromRos(const nav_msgs::Odometry& msg) {
  PoseSample out;
  out.stamp = msg.header.stamp.toSec();
  out.map_base.position = Vec3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                msg.pose.pose.position.z);
  out.map_base.orientation = Quatd(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (!out.map_base.orientation.coeffs().allFinite() ||
      out.map_base.orientation.norm() < 1e-9) {
    out.map_base.orientation = Quatd::Identity();
  } else {
    out.map_base.orientation.normalize();
  }
  // twist 按 nav_msgs 约定在 child 系（base）→ 直接作 body 速度观测
  out.velocity_body = Vec3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z);
  out.has_velocity = math::finiteVec(out.velocity_body);
  return out;
}

LocalizationGateInfo fromRos(const deeplumin_msgs::FusionStatus& msg) {
  LocalizationGateInfo info;
  info.tracking = (msg.state == "TRACKING") && msg.output_valid;
  info.output_valid = msg.output_valid;
  info.slam_age = msg.last_slam_age;
  return info;
}

nav_msgs::Odometry toRos(const OdomResult& odom, const std::string& frame,
                         const std::string& child) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(odom.timestamp);
  msg.header.frame_id = frame;
  msg.child_frame_id = child;
  msg.pose.pose.position.x = odom.pose.position.x();
  msg.pose.pose.position.y = odom.pose.position.y();
  msg.pose.pose.position.z = odom.pose.position.z();
  const Quatd q = odom.pose.orientation.normalized();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  // nav_msgs/Odometry 约定：twist 在 child 系（base）——世界系速度须转回机体系
  const Vec3d v_body = q.toRotationMatrix().transpose() * odom.velocity;
  msg.twist.twist.linear.x = v_body.x();
  msg.twist.twist.linear.y = v_body.y();
  msg.twist.twist.linear.z = v_body.z();
  return msg;
}

deeplumin_msgs::LocalizationStatus toStatusMsg(const DrHealth& h, double now) {
  deeplumin_msgs::LocalizationStatus msg;
  msg.header.stamp = ros::Time(now);
  msg.level = deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL;
  msg.failure_reason = "";
  msg.quality_score = 1.0;
  msg.is_slam_available = true;
  msg.current_mode = "DR_ODOMETRY_V2";
  return msg;
}

}  // namespace dr_odometry_v2_ros
