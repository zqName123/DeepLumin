#include "slam_mapping/core/mapping_runtime.hpp"

#include <localization/factory/factory_registry.hpp>

namespace slam_mapping {

namespace {
localization::Pose poseFromOdom(const localization::OdomResult& odom) {
  localization::Pose pose;
  pose.timestamp = odom.timestamp;
  pose.position = odom.position;
  pose.orientation = odom.orientation;
  pose.covariance = odom.covariance;
  pose.frame_id = odom.frame_id;
  return pose;
}
}  // namespace

std::string toString(RuntimeState state) {
  switch (state) {
    case RuntimeState::IDLE: return "IDLE";
    case RuntimeState::READY: return "READY";
    case RuntimeState::RUNNING: return "RUNNING";
    case RuntimeState::STOPPING: return "STOPPING";
    case RuntimeState::SAVING: return "SAVING";
    case RuntimeState::FINISHED: return "FINISHED";
    case RuntimeState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

bool MappingRuntime::initialize(const SlamMappingConfig& cfg) {
  config_ = cfg;
  localization::registerDefaultPlugins();
  slam_ = localization::SlamFactory::instance().create(config_.slam_backend.type);
  if (!slam_) {
    status_.state = RuntimeState::FAILED;
    status_.message = "failed to create slam backend";
    return false;
  }
  if (!slam_->initialize(config_.slam_backend)) {
    status_.state = RuntimeState::FAILED;
    status_.message = "failed to initialize slam backend";
    return false;
  }
  keyframe_manager_.configure(config_.keyframe);
  map_saver_.configure(config_.map_saver);
  loop_backend_.configure(config_.loop_closure);
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = loop_backend_.supported();
  return true;
}

bool MappingRuntime::start(const RuntimeStartRequest& req, double now_sec) {
  if (!slam_) return false;
  if (req.reset_before_start) reset();
  status_.task_id = req.task_id;
  status_.task_name = req.task_name;
  status_.output_dir = req.output_dir;
  status_.loop_enabled = req.enable_loop_closure;
  status_.loop_supported = loop_backend_.supported();
  status_.state = RuntimeState::RUNNING;
  status_.message = "running";
  status_.running = true;
  status_.saving = false;
  status_.output_valid = false;
  status_.start_time = now_sec;
  return true;
}

bool MappingRuntime::stop(const RuntimeStopRequest& req) {
  if (status_.state != RuntimeState::RUNNING && status_.state != RuntimeState::READY) return false;
  status_.state = RuntimeState::STOPPING;
  status_.message = "stopping";
  status_.running = false;
  if (req.save_map && config_.map_saver.save_on_stop) {
    SaveRequest save_req;
    save_req.output_dir = status_.output_dir;
    save_req.map_name = config_.map_saver.map_filename;
    save_req.resolution = config_.map_saver.default_resolution;
    save_req.save_keyframes = req.save_keyframes;
    save_req.save_keyframe_poses = req.save_keyframe_poses;
    save_req.save_trajectory = req.save_trajectory;
    return save(save_req);
  }
  status_.state = RuntimeState::FINISHED;
  status_.message = "finished";
  return true;
}

bool MappingRuntime::save(const SaveRequest& req) {
  status_.state = RuntimeState::SAVING;
  status_.saving = true;
  status_.message = "saving";
  SaveResult result;
  const bool ok = map_saver_.saveAll(req, keyframe_manager_.keyframes(), &result);
  status_.saving = false;
  status_.output_valid = ok;
  status_.map_path = result.map_path;
  status_.keyframe_dir = result.keyframe_dir;
  status_.keyframe_pose_path = result.keyframe_pose_path;
  status_.trajectory_path = result.trajectory_path;
  status_.state = ok ? RuntimeState::FINISHED : RuntimeState::FAILED;
  status_.message = result.message;
  status_.running = false;
  return ok;
}

void MappingRuntime::reset() {
  if (slam_) slam_->reset();
  keyframe_manager_.reset();
  loop_backend_.reset();
  current_odom_ = localization::OdomResult();
  current_scan_ = localization::TimestampedPointCloud();
  local_map_ = localization::TimestampedPointCloud();
  loop_pairs_.clear();
  status_ = MappingRuntimeStatus();
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = loop_backend_.supported();
}

void MappingRuntime::feedImu(const localization::ImuData& imu) { if (slam_) slam_->feedImu(imu); }
void MappingRuntime::feedLidar(const localization::TimestampedPointCloud& cloud) { if (slam_) slam_->feedLidar(cloud); }
void MappingRuntime::feedCanVelocity(const localization::CanVelocityData& can) { if (slam_) slam_->feedCanVelocity(can); }
void MappingRuntime::feedGnssAttitude(const localization::GnssAttitudeData& gnss) { if (slam_) slam_->feedGnssAttitude(gnss); }

bool MappingRuntime::processOnce() {
  if (!slam_ || status_.state != RuntimeState::RUNNING) return false;
  if (!slam_->processOnce()) return false;
  current_odom_ = slam_->getOdometry();
  current_scan_ = slam_->getCurrentScan();
  local_map_ = slam_->getLocalMap();
  if (!current_odom_.valid || !current_scan_.cloud || current_scan_.cloud->empty()) return false;
  return handleFrame(current_odom_, current_scan_, local_map_);
}

bool MappingRuntime::handleFrame(const localization::OdomResult& odom,
                                 const localization::TimestampedPointCloud& scan,
                                 const localization::TimestampedPointCloud&) {
  const bool keyframe_added = keyframe_manager_.add(odom, scan);
  LoopClosureResult loop_result;
  if (keyframe_added && status_.loop_enabled) {
    loop_result = loop_backend_.update(&keyframe_manager_.mutableKeyframes());
    loop_pairs_ = loop_result.loop_pairs;
    status_.loop_count = loop_result.loop_count;
    if (!loop_result.loop_pairs.empty()) {
      status_.last_loop_from = loop_result.loop_pairs.back().first;
      status_.last_loop_to = loop_result.loop_pairs.back().second;
    }
  }
  status_.loop_updated_last_frame = loop_result.updated;
  status_.last_loop_score = loop_result.latest_score;
  updateStatusOnFrame(odom, scan);
  status_.keyframe_count = keyframe_manager_.keyframes().size();
  if (loop_result.updated && status_.last_loop_from >= 0 && status_.last_loop_to >= 0) {
    status_.message = "loop_detected:" + std::to_string(status_.last_loop_from) + "->" +
                      std::to_string(status_.last_loop_to) + " score=" + std::to_string(status_.last_loop_score);
  } else if (status_.state == RuntimeState::RUNNING) {
    status_.message = "running";
  }
  return true;
}

void MappingRuntime::updateStatusOnFrame(const localization::OdomResult& odom,
                                         const localization::TimestampedPointCloud& scan) {
  status_.frame_count += 1;
  status_.last_frame_time = odom.timestamp;
  status_.duration_sec = status_.start_time > 0.0 ? (odom.timestamp - status_.start_time) : 0.0;
  status_.accumulated_point_count += scan.cloud ? scan.cloud->size() : 0;
}

MappingRuntimeStatus MappingRuntime::status() const { return status_; }
std::vector<MappingKeyframe> MappingRuntime::keyframes() const { return keyframe_manager_.keyframes(); }
std::vector<std::pair<int, int>> MappingRuntime::loopPairs() const { return loop_pairs_; }
localization::OdomResult MappingRuntime::currentOdom() const { return current_odom_; }
localization::TimestampedPointCloud MappingRuntime::currentScan() const { return current_scan_; }
localization::TimestampedPointCloud MappingRuntime::localMap() const { return local_map_; }

}  // namespace slam_mapping
