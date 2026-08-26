#include "slam_mapping_v2/ros/converters.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace slam_mapping_v2_ros {
using namespace slam_mapping_v2;

FrameInput fromOdomMsg(const nav_msgs::Odometry& msg) {
  FrameInput f;
  f.timestamp = msg.header.stamp.toSec();
  f.position << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  f.orientation = Quatd(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  f.frame_id = msg.header.frame_id.empty() ? "odom" : msg.header.frame_id;
  f.valid = true;
  return f;
}

FrameInput fromScanMsg(const sensor_msgs::PointCloud2& msg) {
  FrameInput f;
  f.timestamp = msg.header.stamp.toSec();
  f.frame_id = msg.header.frame_id;
  f.cloud.reset(new PointCloud());
  pcl::fromROSMsg(msg, *f.cloud);
  f.valid = !f.cloud->empty();
  return f;
}

GnssObservation fromGnssMsg(const deeplumin_msgs::Gpchc& msg) {
  GnssObservation g;
  g.timestamp = msg.header.stamp.toSec();
  g.latitude = msg.latitude;
  g.longitude = msg.longitude;
  g.altitude = msg.altitude;
  g.status = msg.status;
  g.nsv = msg.nsv1 + msg.nsv2;
  g.valid = (std::abs(g.latitude) > 1e-9 && std::abs(g.longitude) > 1e-9);
  return g;
}

deeplumin_msgs::MappingStatus toStatusMsg(const RuntimeStatus& status,
                                          const ros::Time& stamp) {
  deeplumin_msgs::MappingStatus msg;
  msg.header.stamp = stamp;
  msg.task_id = status.task_id;
  msg.task_name = status.task_name;
  msg.state = toString(status.state);
  msg.status_message = status.message;
  msg.ready = status.state == RuntimeState::READY;
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
  msg.last_loop_from = status.last_loop_from;
  msg.last_loop_to = status.last_loop_to;
  msg.last_loop_score = status.last_loop_score;
  msg.loop_updated_last_frame = status.loop_updated_last_frame;
  msg.accumulated_point_count = 0;
  msg.duration_sec = status.duration_sec;
  return msg;
}

StartRequest fromStartReq(const deeplumin_msgs::StartMappingRuntime::Request& req) {
  StartRequest out;
  out.task_id = req.task_id;
  out.task_name = req.task_name;
  out.output_dir = req.output_dir;
  out.enable_loop_closure = req.enable_loop_closure;
  out.reset_before_start = req.reset_before_start;
  return out;
}

StopRequest fromStopReq(const deeplumin_msgs::StopMappingRuntime::Request& req) {
  StopRequest out;
  out.task_id = req.task_id;
  out.save_map = req.save_map;
  out.save_keyframes = req.save_keyframes;
  out.save_keyframe_poses = req.save_keyframe_poses;
  out.save_trajectory = req.save_trajectory;
  return out;
}

SaveRequest fromSaveReq(const deeplumin_msgs::SaveMappingRuntime::Request& req) {
  SaveRequest out;
  out.output_dir = req.output_dir;
  out.map_name = req.map_name;
  out.resolution = req.resolution;
  out.save_keyframes = req.save_keyframes;
  out.save_keyframe_poses = req.save_keyframe_poses;
  out.save_trajectory = req.save_trajectory;
  out.save_loop_results = true;
  return out;
}

void fillStatusRes(const RuntimeStatus& status,
                   deeplumin_msgs::GetMappingRuntimeStatus::Response* res,
                   const ros::Time& stamp) {
  if (res) res->status = toStatusMsg(status, stamp);
}

void fillSaveRes(const SaveResult& result, deeplumin_msgs::SaveMappingRuntime::Response* res) {
  if (!res) return;
  res->success = result.success;
  res->message = result.message;
  res->map_path = result.map_path;
  res->keyframe_dir = result.keyframe_dir;
  res->keyframe_pose_path = result.keyframe_pose_path;
  res->trajectory_path = result.trajectory_path;
}

}  // namespace slam_mapping_v2_ros
