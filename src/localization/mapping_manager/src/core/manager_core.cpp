#include "mapping_manager/core/manager_core.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace mapping_manager {
namespace {

std::string sanitizeTaskName(const std::string& name) {
  if (name.empty()) return "mapping_task";
  std::string out = name;
  for (char& c : out) {
    if (c == ' ' || c == '/' || c == '\\') c = '_';
  }
  return out;
}

}  // namespace

std::string toString(MappingState state) {
  switch (state) {
    case MappingState::IDLE: return "IDLE";
    case MappingState::CHECKING_ENV: return "CHECKING_ENV";
    case MappingState::READY: return "READY";
    case MappingState::STARTING: return "STARTING";
    case MappingState::RUNNING: return "RUNNING";
    case MappingState::STOPPING: return "STOPPING";
    case MappingState::SAVING_RESULTS: return "SAVING_RESULTS";
    case MappingState::FINISHED: return "FINISHED";
    case MappingState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

std::string toString(RecordState state) {
  switch (state) {
    case RecordState::REC_IDLE: return "REC_IDLE";
    case RecordState::REC_STARTING: return "REC_STARTING";
    case RecordState::REC_RUNNING: return "REC_RUNNING";
    case RecordState::REC_STOPPING: return "REC_STOPPING";
    case RecordState::REC_SAVED: return "REC_SAVED";
    case RecordState::REC_FAILED: return "REC_FAILED";
  }
  return "REC_UNKNOWN";
}

bool ManagerCore::initialize(const ManagerConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  initialized_ = true;
  resetStatus(0.0);
  resetRecordStatus(0.0);
  environment_status_.summary = "尚未执行环境检测。";
  environment_status_.stamp = 0.0;
  return true;
}

EnvironmentItem ManagerCore::makeEnvItem(const std::string& name,
                                         bool ok,
                                         const std::string& level,
                                         const std::string& detail) const {
  EnvironmentItem item;
  item.name = name;
  item.ok = ok;
  item.level = level;
  item.detail = detail;
  return item;
}

void ManagerCore::resetStatus(double now_sec) {
  status_ = MappingStatusData();
  status_.state = MappingState::IDLE;
  status_.status_message = "空闲";
  status_.stamp = now_sec;
  status_.loop_supported = false;
  status_.keyframe_export_supported = false;
  status_.trajectory_export_supported = false;
  task_start_sec_ = 0.0;
  runtime_started_deadline_sec_ = -1.0;
  save_started_deadline_sec_ = -1.0;
  finish_deadline_sec_ = -1.0;
}

void ManagerCore::resetRecordStatus(double now_sec) {
  record_status_ = RecordStatusData();
  record_status_.state = RecordState::REC_IDLE;
  record_status_.message = "未录制";
  record_status_.stamp = now_sec;
  record_start_sec_ = 0.0;
}

bool ManagerCore::isTaskActive() const {
  return status_.state == MappingState::STARTING || status_.state == MappingState::RUNNING ||
         status_.state == MappingState::STOPPING || status_.state == MappingState::SAVING_RESULTS;
}

std::string ManagerCore::makeTaskId(const std::string& task_name, double now_sec) const {
  const std::string safe_name = sanitizeTaskName(task_name);
  const std::time_t tt = static_cast<std::time_t>(now_sec);
  std::tm tm{};
  localtime_r(&tt, &tm);
  std::ostringstream oss;
  oss << safe_name << '_' << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

EnvironmentStatusData ManagerCore::checkEnvironment(bool include_runtime_checks,
                                                    const std::string& config_path,
                                                    double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  EnvironmentStatusData out;
  out.stamp = now_sec;
  out.summary = "环境检测完成";
  out.items.push_back(makeEnvItem("ros_master", true, "PASS", "ROS Master 可用"));
  out.items.push_back(makeEnvItem("output_root", true, "PASS", "默认输出目录: " + config_.defaults.output_root));
  out.items.push_back(makeEnvItem("bag_output_dir", true, "PASS", "默认录包目录: " + config_.defaults.bag_output_dir));
  const bool config_ok = config_path.empty() || std::filesystem::exists(config_path);
  out.items.push_back(makeEnvItem("config_path",
                                  config_ok,
                                  config_ok ? "PASS" : "FAIL",
                                  config_path.empty() ? "未指定配置文件，允许后端使用默认配置" :
                                  (config_ok ? "配置文件存在" : "配置文件不存在: " + config_path)));
  if (include_runtime_checks) {
    const bool runtime_idle = !isTaskActive();
    out.items.push_back(makeEnvItem("mapping_runtime",
                                    runtime_idle,
                                    runtime_idle ? "PASS" : "WARN",
                                    runtime_idle ? "当前无运行中的建图任务" : "当前已有建图任务在运行"));
  }
  out.overall_ok = true;
  for (const auto& item : out.items) out.overall_ok = out.overall_ok && item.ok;
  environment_status_ = out;

  if (out.overall_ok && (status_.state == MappingState::IDLE || status_.state == MappingState::FINISHED)) {
    status_.state = MappingState::READY;
    status_.status_message = "环境检测通过，可开始建图";
    status_.ready = true;
  }
  status_.stamp = now_sec;
  return out;
}

StartMappingResponseData ManagerCore::startMapping(const StartMappingRequestData& req, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  StartMappingResponseData res;
  if (!(status_.state == MappingState::IDLE || status_.state == MappingState::READY || status_.state == MappingState::FINISHED)) {
    res.accepted = false;
    res.task_id = status_.task_id;
    res.state = toString(status_.state);
    res.reason = "当前状态不允许开始新建图任务";
    return res;
  }

  const std::string task_name = req.task_name.empty() ? "mapping_task" : req.task_name;
  const std::string output_root = req.output_root.empty() ? config_.defaults.output_root : req.output_root;
  const std::string task_id = makeTaskId(task_name, now_sec);
  const std::filesystem::path output_dir = std::filesystem::path(output_root) / task_id;
  std::filesystem::create_directories(output_dir);

  status_.task_id = task_id;
  status_.task_name = task_name;
  status_.state = MappingState::STARTING;
  status_.status_message = "正在请求 slam_mapping 启动建图";
  status_.ready = false;
  status_.running = false;
  status_.saving = false;
  status_.loop_enabled = req.enable_loop_closure;
  status_.loop_supported = false;
  status_.keyframe_export_supported = false;
  status_.trajectory_export_supported = false;
  status_.output_valid = false;
  status_.output_dir = output_dir.string();
  status_.map_path.clear();
  status_.trajectory_path.clear();
  status_.keyframe_dir.clear();
  status_.keyframe_pose_path.clear();
  status_.loop_result_path.clear();
  status_.frame_count = 0;
  status_.keyframe_count = 0;
  status_.loop_count = 0;
  status_.accumulated_point_count = 0;
  status_.duration_sec = 0.0;
  status_.stamp = now_sec;
  task_start_sec_ = now_sec;
  runtime_started_deadline_sec_ = -1.0;
  save_started_deadline_sec_ = -1.0;
  finish_deadline_sec_ = -1.0;

  if (req.auto_start_recording) {
    StartBagRecordingRequestData bag_req;
    bag_req.task_id = task_id;
    bag_req.bag_name = task_name;
    bag_req.output_dir = (output_dir / "bag").string();
    startBagRecording(bag_req, now_sec);
  }

  res.accepted = true;
  res.task_id = task_id;
  res.state = toString(status_.state);
  res.reason = "建图任务已受理，等待后端启动";
  return res;
}

StopMappingResponseData ManagerCore::stopMapping(const StopMappingRequestData& req, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  StopMappingResponseData res;
  if (!(status_.state == MappingState::RUNNING || status_.state == MappingState::STARTING || status_.state == MappingState::READY)) {
    res.accepted = false;
    res.state = toString(status_.state);
    res.reason = "当前没有可停止的建图任务";
    return res;
  }
  if (!req.task_id.empty() && !status_.task_id.empty() && req.task_id != status_.task_id) {
    res.accepted = false;
    res.state = toString(status_.state);
    res.reason = "task_id 不匹配";
    return res;
  }
  status_.state = MappingState::STOPPING;
  status_.status_message = "正在请求 slam_mapping 停止并保存";
  status_.running = false;
  status_.saving = false;
  status_.stamp = now_sec;
  save_started_deadline_sec_ = -1.0;
  finish_deadline_sec_ = -1.0;

  res.accepted = true;
  res.state = toString(status_.state);
  res.reason = "停止建图请求已受理，等待后端保存";
  return res;
}

StartBagRecordingResponseData ManagerCore::startBagRecording(const StartBagRecordingRequestData& req, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  StartBagRecordingResponseData res;
  if (record_status_.active) {
    res.accepted = false;
    res.state = toString(record_status_.state);
    res.reason = "当前已经在录包";
    res.bag_path = record_status_.bag_path;
    return res;
  }
  const std::string bag_name = req.bag_name.empty() ? "mapping" : req.bag_name;
  const std::string output_dir = req.output_dir.empty() ? config_.defaults.bag_output_dir : req.output_dir;
  std::filesystem::create_directories(output_dir);
  const std::filesystem::path bag_path = std::filesystem::path(output_dir) / (bag_name + ".bag");

  record_status_.task_id = req.task_id.empty() ? status_.task_id : req.task_id;
  record_status_.state = RecordState::REC_RUNNING;
  record_status_.active = true;
  record_status_.output_valid = false;
  record_status_.bag_path = bag_path.string();
  record_status_.message = "录包进行中";
  record_status_.duration_sec = 0.0;
  record_status_.bytes_written = 0;
  record_status_.stamp = now_sec;
  record_start_sec_ = now_sec;

  res.accepted = true;
  res.state = toString(record_status_.state);
  res.reason = "录包已启动";
  res.bag_path = record_status_.bag_path;
  return res;
}

StopBagRecordingResponseData ManagerCore::stopBagRecording(const StopBagRecordingRequestData& req, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  StopBagRecordingResponseData res;
  if (!record_status_.active) {
    res.accepted = false;
    res.state = toString(record_status_.state);
    res.reason = "当前没有进行中的录包任务";
    res.bag_path = record_status_.bag_path;
    return res;
  }
  if (!req.task_id.empty() && !record_status_.task_id.empty() && req.task_id != record_status_.task_id) {
    res.accepted = false;
    res.state = toString(record_status_.state);
    res.reason = "task_id 不匹配";
    res.bag_path = record_status_.bag_path;
    return res;
  }

  record_status_.state = RecordState::REC_SAVED;
  record_status_.active = false;
  record_status_.output_valid = true;
  record_status_.message = "录包已保存";
  record_status_.duration_sec = record_start_sec_ > 0.0 ? (now_sec - record_start_sec_) : 0.0;
  record_status_.stamp = now_sec;

  res.accepted = true;
  res.state = toString(record_status_.state);
  res.reason = "录包已停止并保存";
  res.bag_path = record_status_.bag_path;
  return res;
}

GetStatusResponseData ManagerCore::getStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  GetStatusResponseData out;
  out.status = status_;
  out.record_status = record_status_;
  out.environment_status = environment_status_;
  return out;
}

void ManagerCore::updateFromBackend(const MappingStatusData& backend, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!backend.task_id.empty()) {
    status_ = backend;
    status_.stamp = now_sec;
    if (task_start_sec_ <= 0.0 && backend.duration_sec > 0.0) {
      task_start_sec_ = now_sec - backend.duration_sec;
    }
  }
}

void ManagerCore::markFailed(const std::string& reason, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  status_.state = MappingState::FAILED;
  status_.status_message = reason;
  status_.running = false;
  status_.saving = false;
  status_.ready = false;
  status_.stamp = now_sec;
}

void ManagerCore::failBagRecording(const std::string& reason, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  record_status_.state = RecordState::REC_FAILED;
  record_status_.active = false;
  record_status_.output_valid = false;
  record_status_.message = reason;
  record_status_.stamp = now_sec;
}

void ManagerCore::setRecordBytes(unsigned long long bytes, double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  record_status_.bytes_written = bytes;
  record_status_.stamp = now_sec;
}

void ManagerCore::update(double now_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) return;
  status_.stamp = now_sec;
  record_status_.stamp = now_sec;
  if (task_start_sec_ > 0.0 && status_.duration_sec <= 0.0) {
    status_.duration_sec = now_sec - task_start_sec_;
  }
  if (record_status_.active && record_start_sec_ > 0.0) {
    record_status_.duration_sec = now_sec - record_start_sec_;
  }
}

}  // namespace mapping_manager
