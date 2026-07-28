#include "slam_mapping/ros/ros_converters.hpp"

#include <localization/ros/ros_converters.hpp>

namespace slam_mapping_ros {

localization::ImuData fromRos(const sensor_msgs::Imu& msg) { return localization_ros::fromRos(msg); }
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg) { return localization_ros::fromRos(msg); }
localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg,
                                            const localization::SlamConfig& cfg) { return localization_ros::fromRos(msg, cfg); }

localization::CanVelocityData fromRos(const deeplumin_msgs::CanReceiveInfo& msg) {
  localization::CanVelocityData out;
  out.timestamp = msg.header.stamp.toSec();
  out.speed_kmh = (msg.gear == 2) ? -msg.speed : msg.speed;
  out.valid = true;
  return out;
}

localization::GnssAttitudeData fromRos(const deeplumin_msgs::Gpchc& msg) {
  localization::GnssAttitudeData out;
  out.timestamp = msg.header.stamp.toSec();
  out.heading_deg = msg.heading;
  out.pitch_deg = msg.pitch;
  out.roll_deg = msg.roll;
  out.valid = true;
  return out;
}

nav_msgs::Odometry toRosOdom(const localization::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame) {
  return localization_ros::toRosOdom(odom, parent_frame, child_frame);
}

sensor_msgs::PointCloud2 toRosPointCloud(const localization::TimestampedPointCloud& pc) {
  return localization_ros::toRosPointCloud(pc);
}

geometry_msgs::TransformStamped toRosTransform(const localization::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame) {
  return localization_ros::toRosTransform(odom, parent_frame, child_frame);
}

deeplumin_msgs::MappingStatus toRosStatus(const slam_mapping::MappingRuntimeStatus& status,
                                          const ros::Time& stamp) {
  deeplumin_msgs::MappingStatus msg;
  msg.header.stamp = stamp;
  msg.task_id = status.task_id;
  msg.task_name = status.task_name;
  msg.state = slam_mapping::toString(status.state);
  msg.status_message = status.message;
  msg.ready = status.state == slam_mapping::RuntimeState::READY;
  msg.running = status.running;
  msg.saving = status.saving;
  msg.loop_enabled = status.loop_enabled;
  msg.loop_supported = status.loop_supported;
  msg.keyframe_export_supported = true;
  msg.trajectory_export_supported = true;
  msg.output_valid = status.output_valid;
  msg.output_dir = status.output_dir;
  msg.map_path = status.map_path;
  msg.trajectory_path = status.trajectory_path;
  msg.keyframe_dir = status.keyframe_dir;
  msg.keyframe_pose_path = status.keyframe_pose_path;
  msg.loop_result_path = status.loop_result_path;
  msg.frame_count = status.frame_count;
  msg.keyframe_count = status.keyframe_count;
  msg.loop_count = status.loop_count;
  msg.accumulated_point_count = status.accumulated_point_count;
  msg.duration_sec = status.duration_sec;
  return msg;
}

slam_mapping::RuntimeStartRequest fromRos(const deeplumin_msgs::StartMappingRuntime::Request& req) {
  slam_mapping::RuntimeStartRequest out;
  out.task_id = req.task_id;
  out.task_name = req.task_name;
  out.config_path = req.config_path;
  out.output_dir = req.output_dir;
  out.enable_loop_closure = req.enable_loop_closure;
  out.enable_visualization = req.enable_visualization;
  out.reset_before_start = req.reset_before_start;
  return out;
}

slam_mapping::RuntimeStopRequest fromRos(const deeplumin_msgs::StopMappingRuntime::Request& req) {
  slam_mapping::RuntimeStopRequest out;
  out.task_id = req.task_id;
  out.save_map = req.save_map;
  out.save_keyframes = req.save_keyframes;
  out.save_keyframe_poses = req.save_keyframe_poses;
  out.save_trajectory = req.save_trajectory;
  return out;
}

slam_mapping::SaveRequest fromRos(const deeplumin_msgs::SaveMappingRuntime::Request& req) {
  slam_mapping::SaveRequest out;
  out.output_dir = req.output_dir;
  out.map_name = req.map_name;
  out.resolution = req.resolution;
  out.save_keyframes = req.save_keyframes;
  out.save_keyframe_poses = req.save_keyframe_poses;
  out.save_trajectory = req.save_trajectory;
  return out;
}

void fillRos(const slam_mapping::MappingRuntimeStatus& status,
             deeplumin_msgs::GetMappingRuntimeStatus::Response* res,
             const ros::Time& stamp) {
  res->status = toRosStatus(status, stamp);
}

void fillRos(const slam_mapping::SaveResult& result,
             deeplumin_msgs::SaveMappingRuntime::Response* res) {
  res->success = result.success;
  res->message = result.message;
  res->map_path = result.map_path;
  res->keyframe_dir = result.keyframe_dir;
  res->keyframe_pose_path = result.keyframe_pose_path;
  res->trajectory_path = result.trajectory_path;
}

}  // namespace slam_mapping_ros
