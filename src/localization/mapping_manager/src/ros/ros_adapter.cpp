#include "mapping_manager/ros/ros_adapter.hpp"

namespace mapping_manager_ros {
namespace {

mapping_manager::MappingState stateFromString(const std::string& state) {
  if (state == "CHECKING_ENV") return mapping_manager::MappingState::CHECKING_ENV;
  if (state == "READY") return mapping_manager::MappingState::READY;
  if (state == "STARTING") return mapping_manager::MappingState::STARTING;
  if (state == "RUNNING") return mapping_manager::MappingState::RUNNING;
  if (state == "STOPPING") return mapping_manager::MappingState::STOPPING;
  if (state == "SAVING_RESULTS" || state == "SAVING") return mapping_manager::MappingState::SAVING_RESULTS;
  if (state == "FINISHED") return mapping_manager::MappingState::FINISHED;
  if (state == "FAILED" || state == "ERROR") return mapping_manager::MappingState::FAILED;
  return mapping_manager::MappingState::IDLE;
}

}  // namespace

mapping_manager::ManagerConfig loadConfig(ros::NodeHandle& pnh) {
  mapping_manager::ManagerConfig cfg;
  pnh.param<std::string>("topics/status", cfg.topics.status, cfg.topics.status);
  pnh.param<std::string>("topics/record_status", cfg.topics.record_status, cfg.topics.record_status);
  pnh.param<std::string>("topics/environment_last_result", cfg.topics.environment_last_result,
                         cfg.topics.environment_last_result);

  pnh.param<std::string>("services/check_environment", cfg.services.check_environment,
                         cfg.services.check_environment);
  pnh.param<std::string>("services/get_status", cfg.services.get_status, cfg.services.get_status);
  pnh.param<std::string>("services/start_mapping", cfg.services.start_mapping,
                         cfg.services.start_mapping);
  pnh.param<std::string>("services/stop_mapping", cfg.services.stop_mapping,
                         cfg.services.stop_mapping);
  pnh.param<std::string>("services/start_bag_recording", cfg.services.start_bag_recording,
                         cfg.services.start_bag_recording);
  pnh.param<std::string>("services/stop_bag_recording", cfg.services.stop_bag_recording,
                         cfg.services.stop_bag_recording);

  pnh.param<double>("runtime/status_hz", cfg.runtime.status_hz, cfg.runtime.status_hz);
  pnh.param<double>("runtime/startup_delay_sec", cfg.runtime.startup_delay_sec,
                    cfg.runtime.startup_delay_sec);
  pnh.param<double>("runtime/save_delay_sec", cfg.runtime.save_delay_sec,
                    cfg.runtime.save_delay_sec);
  pnh.param<double>("runtime/auto_finish_delay_sec", cfg.runtime.auto_finish_delay_sec,
                    cfg.runtime.auto_finish_delay_sec);

  pnh.param<std::string>("defaults/output_root", cfg.defaults.output_root, cfg.defaults.output_root);
  pnh.param<std::string>("defaults/bag_output_dir", cfg.defaults.bag_output_dir,
                         cfg.defaults.bag_output_dir);

  pnh.param<bool>("backend/enabled", cfg.backend.enabled, cfg.backend.enabled);
  pnh.param<std::string>("backend/start_service", cfg.backend.start_service, cfg.backend.start_service);
  pnh.param<std::string>("backend/stop_service", cfg.backend.stop_service, cfg.backend.stop_service);
  pnh.param<std::string>("backend/get_status_service", cfg.backend.get_status_service, cfg.backend.get_status_service);
  return cfg;
}

deeplumin_msgs::MappingEnvironmentItem toRos(const mapping_manager::EnvironmentItem& src) {
  deeplumin_msgs::MappingEnvironmentItem msg;
  msg.name = src.name;
  msg.ok = src.ok;
  msg.level = src.level;
  msg.detail = src.detail;
  return msg;
}

deeplumin_msgs::MappingEnvironmentStatus toRos(const mapping_manager::EnvironmentStatusData& src,
                                               const ros::Time& stamp) {
  deeplumin_msgs::MappingEnvironmentStatus msg;
  msg.header.stamp = stamp;
  msg.overall_ok = src.overall_ok;
  msg.summary = src.summary;
  msg.items.reserve(src.items.size());
  for (const auto& item : src.items) {
    msg.items.push_back(toRos(item));
  }
  return msg;
}

deeplumin_msgs::MappingStatus toRos(const mapping_manager::MappingStatusData& src,
                                    const ros::Time& stamp) {
  deeplumin_msgs::MappingStatus msg;
  msg.header.stamp = stamp;
  msg.task_id = src.task_id;
  msg.task_name = src.task_name;
  msg.state = mapping_manager::toString(src.state);
  msg.status_message = src.status_message;
  msg.ready = src.ready;
  msg.running = src.running;
  msg.saving = src.saving;
  msg.loop_enabled = src.loop_enabled;
  msg.loop_supported = src.loop_supported;
  msg.keyframe_export_supported = src.keyframe_export_supported;
  msg.trajectory_export_supported = src.trajectory_export_supported;
  msg.output_valid = src.output_valid;
  msg.output_dir = src.output_dir;
  msg.map_path = src.map_path;
  msg.trajectory_path = src.trajectory_path;
  msg.keyframe_dir = src.keyframe_dir;
  msg.keyframe_pose_path = src.keyframe_pose_path;
  msg.loop_result_path = src.loop_result_path;
  msg.frame_count = src.frame_count;
  msg.keyframe_count = src.keyframe_count;
  msg.loop_count = src.loop_count;
  msg.last_loop_from = src.last_loop_from;
  msg.last_loop_to = src.last_loop_to;
  msg.last_loop_score = src.last_loop_score;
  msg.loop_updated_last_frame = src.loop_updated_last_frame;
  msg.accumulated_point_count = src.accumulated_point_count;
  msg.duration_sec = src.duration_sec;
  return msg;
}

deeplumin_msgs::MappingRecordStatus toRos(const mapping_manager::RecordStatusData& src,
                                          const ros::Time& stamp) {
  deeplumin_msgs::MappingRecordStatus msg;
  msg.header.stamp = stamp;
  msg.task_id = src.task_id;
  msg.state = mapping_manager::toString(src.state);
  msg.active = src.active;
  msg.output_valid = src.output_valid;
  msg.bag_path = src.bag_path;
  msg.message = src.message;
  msg.duration_sec = src.duration_sec;
  msg.bytes_written = src.bytes_written;
  return msg;
}

mapping_manager::StartMappingRequestData fromRos(const deeplumin_msgs::StartMapping::Request& req) {
  mapping_manager::StartMappingRequestData out;
  out.task_name = req.task_name;
  out.config_path = req.config_path;
  out.output_root = req.output_root;
  out.enable_loop_closure = req.enable_loop_closure;
  out.enable_visualization = req.enable_visualization;
  out.auto_start_recording = req.auto_start_recording;
  return out;
}

mapping_manager::StopMappingRequestData fromRos(const deeplumin_msgs::StopMapping::Request& req) {
  mapping_manager::StopMappingRequestData out;
  out.task_id = req.task_id;
  out.save_map = req.save_map;
  out.save_keyframes = req.save_keyframes;
  out.save_keyframe_poses = req.save_keyframe_poses;
  out.save_trajectory = req.save_trajectory;
  return out;
}

mapping_manager::StartBagRecordingRequestData fromRos(const deeplumin_msgs::StartBagRecording::Request& req) {
  mapping_manager::StartBagRecordingRequestData out;
  out.task_id = req.task_id;
  out.bag_name = req.bag_name;
  out.output_dir = req.output_dir;
  out.topics.assign(req.topics.begin(), req.topics.end());
  return out;
}

mapping_manager::StopBagRecordingRequestData fromRos(const deeplumin_msgs::StopBagRecording::Request& req) {
  mapping_manager::StopBagRecordingRequestData out;
  out.task_id = req.task_id;
  out.save_now = req.save_now;
  return out;
}

mapping_manager::MappingStatusData fromRos(const deeplumin_msgs::MappingStatus& msg) {
  mapping_manager::MappingStatusData out;
  out.task_id = msg.task_id;
  out.task_name = msg.task_name;
  out.state = stateFromString(msg.state);
  out.status_message = msg.status_message;
  out.ready = msg.ready;
  out.running = msg.running;
  out.saving = msg.saving;
  out.loop_enabled = msg.loop_enabled;
  out.loop_supported = msg.loop_supported;
  out.keyframe_export_supported = msg.keyframe_export_supported;
  out.trajectory_export_supported = msg.trajectory_export_supported;
  out.output_valid = msg.output_valid;
  out.output_dir = msg.output_dir;
  out.map_path = msg.map_path;
  out.trajectory_path = msg.trajectory_path;
  out.keyframe_dir = msg.keyframe_dir;
  out.keyframe_pose_path = msg.keyframe_pose_path;
  out.loop_result_path = msg.loop_result_path;
  out.frame_count = msg.frame_count;
  out.keyframe_count = msg.keyframe_count;
  out.loop_count = msg.loop_count;
  out.last_loop_from = msg.last_loop_from;
  out.last_loop_to = msg.last_loop_to;
  out.last_loop_score = msg.last_loop_score;
  out.loop_updated_last_frame = msg.loop_updated_last_frame;
  out.accumulated_point_count = msg.accumulated_point_count;
  out.duration_sec = msg.duration_sec;
  return out;
}

void fillRos(const mapping_manager::EnvironmentStatusData& src,
             deeplumin_msgs::CheckMappingEnvironment::Response* res) {
  res->overall_ok = src.overall_ok;
  res->summary = src.summary;
  res->items.clear();
  res->items.reserve(src.items.size());
  for (const auto& item : src.items) {
    res->items.push_back(toRos(item));
  }
}

void fillRos(const mapping_manager::StartMappingResponseData& src,
             deeplumin_msgs::StartMapping::Response* res) {
  res->accepted = src.accepted;
  res->task_id = src.task_id;
  res->state = src.state;
  res->reason = src.reason;
}

void fillRos(const mapping_manager::StopMappingResponseData& src,
             deeplumin_msgs::StopMapping::Response* res) {
  res->accepted = src.accepted;
  res->state = src.state;
  res->reason = src.reason;
}

void fillRos(const mapping_manager::StartBagRecordingResponseData& src,
             deeplumin_msgs::StartBagRecording::Response* res) {
  res->accepted = src.accepted;
  res->state = src.state;
  res->reason = src.reason;
  res->bag_path = src.bag_path;
}

void fillRos(const mapping_manager::StopBagRecordingResponseData& src,
             deeplumin_msgs::StopBagRecording::Response* res) {
  res->accepted = src.accepted;
  res->state = src.state;
  res->reason = src.reason;
  res->bag_path = src.bag_path;
}

void fillRos(const mapping_manager::GetStatusResponseData& src,
             deeplumin_msgs::GetMappingStatus::Response* res,
             const ros::Time& stamp) {
  res->status = toRos(src.status, stamp);
  res->record_status = toRos(src.record_status, stamp);
  res->environment_status = toRos(src.environment_status, stamp);
}

}  // namespace mapping_manager_ros
