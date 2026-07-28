#pragma once

#include <string>
#include <vector>

namespace mapping_manager {

enum class MappingState {
  IDLE,
  CHECKING_ENV,
  READY,
  STARTING,
  RUNNING,
  STOPPING,
  SAVING_RESULTS,
  FINISHED,
  FAILED,
};

enum class RecordState {
  REC_IDLE,
  REC_STARTING,
  REC_RUNNING,
  REC_STOPPING,
  REC_SAVED,
  REC_FAILED,
};

struct EnvironmentItem {
  std::string name;
  bool ok = false;
  std::string level;
  std::string detail;
};

struct EnvironmentStatusData {
  bool overall_ok = false;
  std::string summary;
  std::vector<EnvironmentItem> items;
  double stamp = 0.0;
};

struct MappingStatusData {
  std::string task_id;
  std::string task_name;
  MappingState state = MappingState::IDLE;
  std::string status_message = "空闲";
  bool ready = false;
  bool running = false;
  bool saving = false;
  bool loop_enabled = false;
  bool loop_supported = false;
  bool keyframe_export_supported = false;
  bool trajectory_export_supported = false;
  bool output_valid = false;
  std::string output_dir;
  std::string map_path;
  std::string trajectory_path;
  std::string keyframe_dir;
  std::string keyframe_pose_path;
  std::string loop_result_path;
  unsigned int frame_count = 0;
  unsigned int keyframe_count = 0;
  unsigned int loop_count = 0;
  unsigned long long accumulated_point_count = 0;
  double duration_sec = 0.0;
  double stamp = 0.0;
};

struct RecordStatusData {
  std::string task_id;
  RecordState state = RecordState::REC_IDLE;
  bool active = false;
  bool output_valid = false;
  std::string bag_path;
  std::string message = "未录制";
  double duration_sec = 0.0;
  unsigned long long bytes_written = 0;
  double stamp = 0.0;
};

struct StartMappingRequestData {
  std::string task_name;
  std::string config_path;
  std::string output_root;
  bool enable_loop_closure = false;
  bool enable_visualization = false;
  bool auto_start_recording = false;
};

struct StartMappingResponseData {
  bool accepted = false;
  std::string task_id;
  std::string state;
  std::string reason;
};

struct StopMappingRequestData {
  std::string task_id;
  bool save_map = true;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
};

struct StopMappingResponseData {
  bool accepted = false;
  std::string state;
  std::string reason;
};

struct StartBagRecordingRequestData {
  std::string task_id;
  std::string bag_name;
  std::string output_dir;
  std::vector<std::string> topics;
};

struct StartBagRecordingResponseData {
  bool accepted = false;
  std::string state;
  std::string reason;
  std::string bag_path;
};

struct StopBagRecordingRequestData {
  std::string task_id;
  bool save_now = true;
};

struct StopBagRecordingResponseData {
  bool accepted = false;
  std::string state;
  std::string reason;
  std::string bag_path;
};

struct GetStatusResponseData {
  MappingStatusData status;
  RecordStatusData record_status;
  EnvironmentStatusData environment_status;
};

std::string toString(MappingState state);
std::string toString(RecordState state);

}  // namespace mapping_manager
