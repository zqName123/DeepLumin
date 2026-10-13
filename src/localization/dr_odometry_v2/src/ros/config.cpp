#include "dr_odometry_v2/ros/config.hpp"

#include <vector>

namespace dr_odometry_v2_ros {

namespace {

std::string loadString(ros::NodeHandle& pnh, const std::string& key,
                       const std::string& def) {
  std::string v;
  pnh.param<std::string>(key, v, def);
  return v;
}

double loadDouble(ros::NodeHandle& pnh, const std::string& key, double def) {
  double v = def;
  pnh.param<double>(key, v, def);
  return v;
}

int loadInt(ros::NodeHandle& pnh, const std::string& key, int def) {
  int v = def;
  pnh.param<int>(key, v, def);
  return v;
}

bool loadBool(ros::NodeHandle& pnh, const std::string& key, bool def) {
  bool v = def;
  pnh.param<bool>(key, v, def);
  return v;
}

Extrinsic loadExtrinsic(ros::NodeHandle& pnh, const std::string& key) {
  Extrinsic ext;
  std::vector<double> t(3, 0.0);
  std::vector<double> r(9, 0.0);
  r[0] = r[4] = r[8] = 1.0;
  pnh.param<std::vector<double>>(key + "/translation", t, t);
  pnh.param<std::vector<double>>(key + "/rotation", r, r);
  if (t.size() == 3) {
    ext.translation = Eigen::Vector3d(t[0], t[1], t[2]);
  }
  if (r.size() == 9) {
    ext.rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }
  return ext;
}

}  // namespace

LoadedConfig loadConfig(ros::NodeHandle& pnh) {
  LoadedConfig out;
  DrConfig& cfg = out.core;

  // ---- frames / topics ----
  out.frames.odom = loadString(pnh, "frames/odom", out.frames.odom);
  out.frames.base_link = loadString(pnh, "frames/base_link", out.frames.base_link);

  out.topics.imu = loadString(pnh, "topics/imu", out.topics.imu);
  out.topics.can = loadString(pnh, "topics/can", out.topics.can);
  out.topics.gnss = loadString(pnh, "topics/gnss", out.topics.gnss);
  out.topics.localization_pose =
      loadString(pnh, "topics/localization_pose", out.topics.localization_pose);
  out.topics.localization_status =
      loadString(pnh, "topics/localization_status", out.topics.localization_status);
  out.topics.odom = loadString(pnh, "topics/odom", out.topics.odom);
  out.topics.status = loadString(pnh, "topics/status", out.topics.status);
  out.topics.path = loadString(pnh, "topics/path", out.topics.path);

  // ---- extrinsics ----
  out.extrinsics.base_to_imu = loadExtrinsic(pnh, "extrinsics/base_to_imu");
  out.extrinsics.base_to_gnss = loadExtrinsic(pnh, "extrinsics/base_to_gnss");
  out.extrinsics.base_to_can = loadExtrinsic(pnh, "extrinsics/base_to_can");

  // ---- imu ----
  cfg.imu.enabled = loadBool(pnh, "imu/enabled", cfg.imu.enabled);
  cfg.imu.use_accel = loadBool(pnh, "imu/use_accel", cfg.imu.use_accel);
  cfg.imu.max_dt = loadDouble(pnh, "imu/max_dt", cfg.imu.max_dt);
  cfg.imu.gyro_noise = loadDouble(pnh, "imu/gyro_noise", cfg.imu.gyro_noise);
  cfg.imu.accel_noise = loadDouble(pnh, "imu/accel_noise", cfg.imu.accel_noise);
  cfg.imu.gyro_bias_noise = loadDouble(pnh, "imu/gyro_bias_noise", cfg.imu.gyro_bias_noise);
  cfg.imu.accel_bias_noise = loadDouble(pnh, "imu/accel_bias_noise", cfg.imu.accel_bias_noise);
  cfg.imu.gravity = loadDouble(pnh, "imu/gravity", cfg.imu.gravity);

  // ---- wheel ----
  cfg.wheel.enabled = loadBool(pnh, "wheel/enabled", cfg.wheel.enabled);
  cfg.wheel.speed_is_kmh = loadBool(pnh, "wheel/speed_is_kmh", cfg.wheel.speed_is_kmh);
  cfg.wheel.use_valid_flag = loadBool(pnh, "wheel/use_valid_flag", cfg.wheel.use_valid_flag);
  cfg.wheel.velocity_noise = loadDouble(pnh, "wheel/velocity_noise", cfg.wheel.velocity_noise);
  cfg.wheel.nonholonomic_noise =
      loadDouble(pnh, "wheel/nonholonomic_noise", cfg.wheel.nonholonomic_noise);
  cfg.wheel.timeout = loadDouble(pnh, "wheel/timeout", cfg.wheel.timeout);

  // ---- gnss ----
  cfg.gnss.enabled = loadBool(pnh, "gnss/enabled", cfg.gnss.enabled);
  cfg.gnss.use_attitude = loadBool(pnh, "gnss/use_attitude", cfg.gnss.use_attitude);
  cfg.gnss.attitude_mode = loadString(pnh, "gnss/attitude_mode", cfg.gnss.attitude_mode);
  cfg.gnss.rezero_when_static =
      loadBool(pnh, "gnss/rezero_when_static", cfg.gnss.rezero_when_static);
  cfg.gnss.static_speed_eps = loadDouble(pnh, "gnss/static_speed_eps", cfg.gnss.static_speed_eps);
  cfg.gnss.attitude_noise_roll_deg =
      loadDouble(pnh, "gnss/attitude_noise_roll_deg", cfg.gnss.attitude_noise_roll_deg);
  cfg.gnss.attitude_noise_pitch_deg =
      loadDouble(pnh, "gnss/attitude_noise_pitch_deg", cfg.gnss.attitude_noise_pitch_deg);
  cfg.gnss.attitude_noise_yaw_deg =
      loadDouble(pnh, "gnss/attitude_noise_yaw_deg", cfg.gnss.attitude_noise_yaw_deg);
  cfg.gnss.heading_convention =
      loadString(pnh, "gnss/heading_convention", cfg.gnss.heading_convention);
  cfg.gnss.heading_offset_deg = loadDouble(pnh, "gnss/heading_offset_deg", cfg.gnss.heading_offset_deg);
  cfg.gnss.min_speed_for_heading =
      loadDouble(pnh, "gnss/min_speed_for_heading", cfg.gnss.min_speed_for_heading);
  cfg.gnss.use_accel = loadBool(pnh, "gnss/use_accel", cfg.gnss.use_accel);
  cfg.gnss.accel_frame = loadString(pnh, "gnss/accel_frame", cfg.gnss.accel_frame);
  cfg.gnss.accel_noise = loadDouble(pnh, "gnss/accel_noise", cfg.gnss.accel_noise);
  cfg.gnss.use_velocity = loadBool(pnh, "gnss/use_velocity", cfg.gnss.use_velocity);
  cfg.gnss.velocity_noise = loadDouble(pnh, "gnss/velocity_noise", cfg.gnss.velocity_noise);
  cfg.gnss.use_position = loadBool(pnh, "gnss/use_position", cfg.gnss.use_position);
  cfg.gnss.position_noise = loadDouble(pnh, "gnss/position_noise", cfg.gnss.position_noise);
  cfg.gnss.timeout = loadDouble(pnh, "gnss/timeout", cfg.gnss.timeout);

  // ---- localization ----
  cfg.localization.enabled = loadBool(pnh, "localization/enabled", cfg.localization.enabled);
  cfg.localization.require_tracking =
      loadBool(pnh, "localization/require_tracking", cfg.localization.require_tracking);
  cfg.localization.max_latency =
      loadDouble(pnh, "localization/max_latency", cfg.localization.max_latency);
  cfg.localization.pose_noise_trans =
      loadDouble(pnh, "localization/pose_noise_trans", cfg.localization.pose_noise_trans);
  cfg.localization.pose_noise_yaw_deg =
      loadDouble(pnh, "localization/pose_noise_yaw_deg", cfg.localization.pose_noise_yaw_deg);
  cfg.localization.pose_noise_att_deg =
      loadDouble(pnh, "localization/pose_noise_att_deg", cfg.localization.pose_noise_att_deg);
  cfg.localization.vel_noise = loadDouble(pnh, "localization/vel_noise", cfg.localization.vel_noise);
  cfg.localization.use_velocity =
      loadBool(pnh, "localization/use_velocity", cfg.localization.use_velocity);
  cfg.localization.bias_freeze_timeout =
      loadDouble(pnh, "localization/bias_freeze_timeout", cfg.localization.bias_freeze_timeout);
  cfg.localization.jump_soft_trans =
      loadDouble(pnh, "localization/jump_soft_trans", cfg.localization.jump_soft_trans);
  cfg.localization.jump_soft_yaw_deg =
      loadDouble(pnh, "localization/jump_soft_yaw_deg", cfg.localization.jump_soft_yaw_deg);
  cfg.localization.jump_hard_trans =
      loadDouble(pnh, "localization/jump_hard_trans", cfg.localization.jump_hard_trans);
  cfg.localization.jump_hard_yaw_deg =
      loadDouble(pnh, "localization/jump_hard_yaw_deg", cfg.localization.jump_hard_yaw_deg);
  cfg.localization.jump_confirm_frames =
      loadInt(pnh, "localization/jump_confirm_frames", cfg.localization.jump_confirm_frames);
  cfg.localization.jump_confirm_consistency_trans =
      loadDouble(pnh, "localization/jump_confirm_consistency_trans",
                 cfg.localization.jump_confirm_consistency_trans);
  cfg.localization.jump_confirm_consistency_yaw_deg =
      loadDouble(pnh, "localization/jump_confirm_consistency_yaw_deg",
                 cfg.localization.jump_confirm_consistency_yaw_deg);

  // ---- output ----
  cfg.output.rate = loadDouble(pnh, "output/rate", cfg.output.rate);
  cfg.output.publish_tf = loadBool(pnh, "output/publish_tf", cfg.output.publish_tf);
  cfg.output.path_max_size = loadInt(pnh, "output/path_max_size", cfg.output.path_max_size);
  cfg.output.path_min_step = loadDouble(pnh, "output/path_min_step", cfg.output.path_min_step);
  cfg.output.path_publish_rate =
      loadDouble(pnh, "output/path_publish_rate", cfg.output.path_publish_rate);
  cfg.output.max_extrapolate = loadDouble(pnh, "output/max_extrapolate", cfg.output.max_extrapolate);

  // ---- time_sync ----
  cfg.time_sync.queue_size = loadInt(pnh, "time_sync/queue_size", cfg.time_sync.queue_size);
  cfg.time_sync.reordered_drop_warn =
      loadInt(pnh, "time_sync/reordered_drop_warn", cfg.time_sync.reordered_drop_warn);
  cfg.time_sync.pose_late_tolerance =
      loadDouble(pnh, "time_sync/pose_late_tolerance", cfg.time_sync.pose_late_tolerance);

  cfg.initial_yaw_deg = loadDouble(pnh, "imu/initial_yaw_deg", cfg.initial_yaw_deg);

  // ---- map（可视化）----
  out.map_viz.enabled = loadBool(pnh, "map/enabled", out.map_viz.enabled);
  out.map_viz.pcd_path = loadString(pnh, "map/pcd_path", out.map_viz.pcd_path);
  out.map_viz.topic = loadString(pnh, "map/topic", out.map_viz.topic);
  out.map_viz.frame_id = loadString(pnh, "map/frame_id", out.map_viz.frame_id);
  out.map_viz.voxel_leaf = loadDouble(pnh, "map/voxel_leaf", out.map_viz.voxel_leaf);
  out.map_viz.publish_period = loadDouble(pnh, "map/publish_period", out.map_viz.publish_period);
  out.map_viz.offset_x = loadDouble(pnh, "map/offset_x", out.map_viz.offset_x);
  out.map_viz.offset_y = loadDouble(pnh, "map/offset_y", out.map_viz.offset_y);
  out.map_viz.offset_z = loadDouble(pnh, "map/offset_z", out.map_viz.offset_z);
  out.map_viz.offset_yaw_deg = loadDouble(pnh, "map/offset_yaw_deg", out.map_viz.offset_yaw_deg);
  return out;
}

}  // namespace dr_odometry_v2_ros
