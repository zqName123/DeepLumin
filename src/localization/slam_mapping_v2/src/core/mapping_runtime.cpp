#include "slam_mapping_v2/core/mapping_runtime.hpp"
#include <ros/ros.h>

#include "slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp"

#include <pcl/common/transforms.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace slam_mapping_v2 {

namespace {

Eigen::Matrix4d poseToMatrix(const Pose& p) {
  Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
  t.block<3, 3>(0, 0) = p.orientation.normalized().toRotationMatrix();
  t.block<3, 1>(0, 3) = p.position;
  return t;
}

Pose poseFromFrame(const FrameInput& f) {
  Pose p;
  p.timestamp = f.timestamp;
  p.position = f.position;
  p.orientation = f.orientation;
  p.frame_id = f.frame_id;
  return p;
}

}  // namespace

MappingRuntime::~MappingRuntime() { stopThreads(); }

bool MappingRuntime::initialize(const MappingConfig& cfg) {
  cfg_ = cfg;
  kf_manager_.configure(cfg_.keyframe);
  global_map_.configure(cfg_.map);
  gnss_.configure(cfg_.loop_closure.gnss);
  optimizer_.configure(cfg_.loop_closure);
  map_saver_.configure(cfg_.map_saver);
  detector_ = std::make_unique<KdtreeGicpLoopDetector>();
  detector_->configure(cfg_.loop_closure);

  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = RuntimeStatus{};
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = true;
  initialized_ = true;
  return true;
}

bool MappingRuntime::start(const StartRequest& req, double now_sec) {
  if (!initialized_) return false;
  if (req.reset_before_start) reset();
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.task_id = req.task_id;
    status_.task_name = req.task_name;
    status_.output_dir = req.output_dir;
    status_.loop_enabled = req.enable_loop_closure;
    status_.loop_supported = true;
    status_.state = RuntimeState::RUNNING;
    status_.message = "running";
    status_.running = true;
    status_.saving = false;
    status_.output_valid = false;
    status_.start_time = now_sec;
  }
  loop_enabled_ = req.enable_loop_closure;
  ROS_INFO("[slam_mapping_v2] start RUNNING task=%s output=%s loop=%d",
           req.task_name.c_str(), req.output_dir.c_str(), req.enable_loop_closure ? 1 : 0);
  return true;
}

bool MappingRuntime::stop(const StopRequest& req) {
  const auto st = statusSnapshot();
  if (st.state != RuntimeState::RUNNING && st.state != RuntimeState::READY) return false;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::STOPPING;
    status_.message = "stopping";
    status_.running = false;
  }
  if (req.save_map && cfg_.map_saver.save_on_stop) {
    SaveRequest save_req;
    save_req.output_dir = st.output_dir;
    save_req.map_name = cfg_.map_saver.map_filename;
    save_req.resolution = cfg_.map_saver.default_resolution;
    save_req.save_keyframes = req.save_keyframes;
    save_req.save_keyframe_poses = req.save_keyframe_poses;
    save_req.save_trajectory = req.save_trajectory;
    save_req.save_loop_results = cfg_.map_saver.save_loop_results;
    return save(save_req);
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::FINISHED;
    status_.message = "finished";
  }
  return true;
}

bool MappingRuntime::save(const SaveRequest& req) {
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = RuntimeState::SAVING;
    status_.saving = true;
    status_.message = "saving";
  }
  std::vector<Keyframe> kfs;
  double lat = 0.0, lon = 0.0, alt = 0.0;
  {
    std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
    kfs = keyframes_;
    if (cfg_.loop_closure.gnss.enable) {
      gnss_.getEnuOrigin(&lat, &lon, &alt);
    }
  }
  LoopData ld = loopData();
  SaveResult result;
  const bool ok = map_saver_.saveAll(req, kfs, ld.loop_pairs, ld.current_submap,
                                     ld.history_submap, &result);
  if (cfg_.loop_closure.gnss.enable && ld.map_enu_valid) {
    map_saver_.saveGnssAlignment(req.output_dir + "/gnss_alignment.txt", ld.map_enu_rot,
                                 ld.map_enu_trans, lat, lon, alt);
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.saving = false;
    status_.output_valid = ok;
    status_.map_path = result.map_path;
    status_.keyframe_dir = result.keyframe_dir;
    status_.keyframe_pose_path = result.keyframe_pose_path;
    status_.trajectory_path = result.trajectory_path;
    status_.loop_result_path = result.loop_result_path;
    status_.state = ok ? RuntimeState::FINISHED : RuntimeState::FAILED;
    status_.message = ok ? "saved:" + result.map_path : "save failed";
  }
  return ok;
}

void MappingRuntime::reset() {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    odom_queue_.clear();
    scan_queue_.clear();
    gnss_queue_.clear();
    gnss_buffer_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.clear();
  }
  {
    std::unique_lock<std::shared_mutex> kf_lock(kf_mutex_);
    keyframes_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    loop_data_ = LoopData{};
  }
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_frame_ = FrameInput{};
  }
  last_keyframe_ = Keyframe{};
  frame_count_ = 0;
  loop_enabled_ = false;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_map_.reset();
  }
  gnss_.reset();
  optimizer_.reset();
  detector_->reset();
  map_saver_.configure(cfg_.map_saver);

  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = RuntimeStatus{};
  status_.state = RuntimeState::READY;
  status_.message = "ready";
  status_.loop_supported = true;
}

void MappingRuntime::feedOdom(const FrameInput& odom) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (odom_queue_.size() >= cfg_.threading.max_input_queue) odom_queue_.pop_front();
  odom_queue_.push_back(odom);
}

void MappingRuntime::feedScan(const FrameInput& scan) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (scan_queue_.size() >= cfg_.threading.max_input_queue) scan_queue_.pop_front();
  scan_queue_.push_back(scan);
}

void MappingRuntime::feedGnss(const GnssObservation& gnss) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  gnss_queue_.push_back(gnss);
  while (gnss_queue_.size() > 5000) gnss_queue_.pop_front();
}

bool MappingRuntime::popFrame(FrameInput* odom, FrameInput* scan) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  const double tol = cfg_.threading.time_sync_tolerance_sec;
  while (!odom_queue_.empty() && !scan_queue_.empty()) {
    const auto& o = odom_queue_.front();
    const auto& s = scan_queue_.front();
    const double dt = std::abs(o.timestamp - s.timestamp);
    if (dt <= tol) {
      *odom = o;
      *scan = s;
      odom_queue_.pop_front();
      scan_queue_.pop_front();
      return true;
    }
    if (o.timestamp < s.timestamp) {
      odom_queue_.pop_front();
    } else {
      scan_queue_.pop_front();
    }
  }
  return false;
}

bool MappingRuntime::processOnce() {
  if (!initialized_ || statusSnapshot().state != RuntimeState::RUNNING) {
    ROS_WARN_THROTTLE(5.0, "[slam_mapping_v2] processOnce skip: state=%s (call start or set runtime/auto_start)",
                      toString(statusSnapshot().state).c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (odom_queue_.empty() || scan_queue_.empty()) {
      ROS_WARN_THROTTLE(5.0, "[slam_mapping_v2] waiting input: odom_q=%zu scan_q=%zu",
                        odom_queue_.size(), scan_queue_.size());
    }
  }

  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    while (!gnss_queue_.empty()) {
      gnss_buffer_.push_back(gnss_queue_.front());
      gnss_queue_.pop_front();
    }
  }
  while (gnss_buffer_.size() > 5000) gnss_buffer_.pop_front();

  FrameInput odom, scan;
  if (!popFrame(&odom, &scan)) return false;
  ROS_INFO_THROTTLE(2.0, "[slam_mapping_v2] consumed frame ts=%.3f odom_valid=%d pts=%zu",
                    odom.timestamp, odom.valid ? 1 : 0, scan.cloud ? scan.cloud->size() : 0);
  ROS_INFO_THROTTLE(2.0, "[slam_mapping_v2] consumed frame ts=%.3f odom_valid=%d pts=%zu",
                    odom.timestamp, odom.valid ? 1 : 0, scan.cloud ? scan.cloud->size() : 0);
  if (!odom.valid || !scan.cloud || scan.cloud->empty()) return false;

  // 合并 odom 位姿与 scan 点云为完整帧（关键帧判定需要点云与位姿）
  FrameInput frame = scan;
  frame.position = odom.position;
  frame.orientation = odom.orientation;
  frame.valid = true;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_frame_ = frame;
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.last_frame_time = odom.timestamp;
    status_.duration_sec = odom.timestamp - status_.start_time;
  }
  ++frame_count_;

  // 全局地图增量插入（每帧，faster-lio 风格）
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    global_map_.insert(scan);
  }

  // 关键帧判定（主线程私有 last_keyframe_，无锁）
  if (kf_manager_.shouldAdd(last_keyframe_, frame, frame_count_)) {
    Keyframe kf;
    kf.timestamp = frame.timestamp;
    kf.pose_odom = poseFromFrame(frame);
    kf.pose_map = kf.pose_odom;
    kf.cloud = frame.cloud;
    kf.point_count = frame.cloud->size();
    kf.source_frame = frame_count_;
    if (cfg_.loop_closure.gnss.enable) {
      associateGnssToKeyframe(&kf);
    }
    last_keyframe_ = kf;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_.push_back(std::move(kf));
    }
    pending_cv_.notify_one();
  }
  return true;
}

void MappingRuntime::associateGnssToKeyframe(Keyframe* kf) {
  const double max_age = cfg_.loop_closure.gnss.max_age_sec;
  auto best = gnss_buffer_.end();
  double best_dt = max_age;
  for (auto it = gnss_buffer_.begin(); it != gnss_buffer_.end(); ++it) {
    const double dt = std::abs(it->timestamp - kf->timestamp);
    if (dt <= best_dt) {
      best_dt = dt;
      best = it;
    }
  }
  if (best != gnss_buffer_.end()) {
    kf->gnss = *best;
    gnss_buffer_.erase(best);
  }
  while (!gnss_buffer_.empty() && gnss_buffer_.front().timestamp < kf->timestamp - max_age) {
    gnss_buffer_.pop_front();
  }
}

void MappingRuntime::backendLoop() {
  while (!stop_threads_) {
    std::vector<Keyframe> batch;
    {
      std::unique_lock<std::mutex> lock(pending_mutex_);
      pending_cv_.wait_for(
          lock,
          std::chrono::milliseconds(
              static_cast<int>(cfg_.threading.backend_interval_sec * 1000.0)),
          [this]() { return stop_threads_ || !pending_.empty(); });
      if (stop_threads_) break;
      batch.swap(pending_);
    }
    if (batch.empty()) continue;
    if (statusSnapshot().state != RuntimeState::RUNNING) continue;

    LoopResult r;
    double latest_score = -1.0;
    const Keyframe* latest_kf = nullptr;
    {
      std::unique_lock<std::shared_mutex> kf_lock(kf_mutex_);
      for (auto& kf : batch) {
        kf.id = static_cast<int>(keyframes_.size());
        keyframes_.push_back(std::move(kf));
      }
      const int latest_index = static_cast<int>(keyframes_.size()) - 1;
      if (loop_enabled_ || cfg_.loop_closure.gnss.enable) {
        optimizer_.addOdomFactors(keyframes_);
        if (cfg_.loop_closure.gnss.enable) {
          gnss_.update(keyframes_, &optimizer_.graph(), latest_index);
        }
        LoopCandidate candidate;
        if (loop_enabled_) {
          candidate = detector_->detect(keyframes_);
          if (candidate.valid) {
            keyframes_[candidate.current_idx].loop_closed = true;
            keyframes_[candidate.history_idx].loop_closed = true;
            keyframes_[candidate.current_idx].loop_match_count += 1;
            keyframes_[candidate.history_idx].loop_match_count += 1;
            latest_score = candidate.score;
            optimizer_.addLoopFactor(candidate);
          }
        }
        optimizer_.optimize(&keyframes_);
      }
      if (!keyframes_.empty()) latest_kf = &keyframes_.back();
      ROS_INFO_THROTTLE(1.0, "[slam_mapping_v2] backend merged: kfs=%zu loops=%zu gnss_factors=%zu",
                        keyframes_.size(), detector_->loopPairs().size(), gnss_.factorCount());
      r.updated = !keyframes_.empty() && loop_enabled_ &&
                  !detector_->loopPairs().empty() &&
                  detector_->loopPairs().back().second == latest_index;
      r.loop_pairs = detector_->loopPairs();
      r.loop_count = static_cast<int>(r.loop_pairs.size());
      r.latest_score = latest_score;
      r.gnss_factor_count = gnss_.factorCount();
      r.gnss_rejected_count = gnss_.rejectedCount();
      r.gnss_aligned = gnss_.aligned();
      updateLoopData(latest_kf, r);
    }
    updateStatusFromLoop(r);
  }
}

void MappingRuntime::updateLoopData(const Keyframe* latest_kf, const LoopResult& r) {
  LoopData d;
  d.loop_pairs = r.loop_pairs;
  d.current_submap = detector_->latestCurrentSubmap();
  d.history_submap = detector_->latestHistorySubmap();
  d.gnss_factor_positions = gnss_.acceptedPositions();
  d.gnss_rejected_positions = gnss_.rejectedPositions();
  d.gnss_accepted_factors = gnss_.acceptedFactors();
  d.gnss_rejected_factors = gnss_.rejectedFactors();
  d.map_enu_valid = gnss_.getMapEnuTransform(&d.map_enu_rot, &d.map_enu_trans);
  if (latest_kf) {
    d.t_map_odom = poseToMatrix(latest_kf->pose_map) * poseToMatrix(latest_kf->pose_odom).inverse();
    d.map_odom_valid = d.t_map_odom.allFinite();
  }
  std::lock_guard<std::mutex> lock(loop_mutex_);
  loop_data_ = std::move(d);
}

void MappingRuntime::updateStatusFromLoop(const LoopResult& r) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.loop_count = r.loop_count;
  status_.gnss_factor_count = r.gnss_factor_count;
  status_.gnss_rejected_count = r.gnss_rejected_count;
  status_.gnss_aligned = r.gnss_aligned;
  status_.loop_updated_last_frame = r.updated;
  status_.last_loop_score = r.latest_score;
  if (!r.loop_pairs.empty()) {
    status_.last_loop_from = r.loop_pairs.back().first;
    status_.last_loop_to = r.loop_pairs.back().second;
  }
  if (r.updated && status_.last_loop_from >= 0 && status_.last_loop_to >= 0) {
    status_.message = "loop_detected:" + std::to_string(status_.last_loop_from) + "->" +
                      std::to_string(status_.last_loop_to) +
                      " score=" + std::to_string(status_.last_loop_score);
  } else if (status_.state == RuntimeState::RUNNING) {
    status_.message = "running";
  }
}

void MappingRuntime::startThreads() {
  if (!backend_thread_.joinable()) {
    stop_threads_ = false;
    backend_thread_ = std::thread(&MappingRuntime::backendLoop, this);
  }
}

void MappingRuntime::stopThreads() {
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    stop_threads_ = true;
  }
  pending_cv_.notify_all();
  if (backend_thread_.joinable()) backend_thread_.join();
}

void MappingRuntime::shutdown() {
  const auto st = statusSnapshot();
  if (st.state == RuntimeState::RUNNING) {
    StopRequest req;
    req.task_id = st.task_id;
    req.save_map = true;
    req.save_keyframes = cfg_.map_saver.save_keyframes;
    req.save_keyframe_poses = cfg_.map_saver.save_keyframe_poses;
    req.save_trajectory = cfg_.map_saver.save_trajectory;
    stop(req);
  }
  stopThreads();
}

RuntimeStatus MappingRuntime::statusSnapshot() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

RuntimeStatus MappingRuntime::status() const {
  auto st = statusSnapshot();
  {
    std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
    st.keyframe_count = keyframes_.size();
  }
  st.frame_count = frame_count_;
  if (st.start_time > 0.0) st.duration_sec = st.last_frame_time - st.start_time;
  return st;
}

std::vector<Keyframe> MappingRuntime::keyframes() const {
  std::shared_lock<std::shared_mutex> kf_lock(kf_mutex_);
  return keyframes_;
}

LoopData MappingRuntime::loopData() const {
  std::lock_guard<std::mutex> lock(loop_mutex_);
  return loop_data_;
}

FrameInput MappingRuntime::latestFrame() const {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  return latest_frame_;
}

bool MappingRuntime::mapOdomTransform(Eigen::Matrix4d* t_map_odom) const {
  std::lock_guard<std::mutex> lock(loop_mutex_);
  if (!loop_data_.map_odom_valid) return false;
  *t_map_odom = loop_data_.t_map_odom;
  return true;
}

PointCloudPtr MappingRuntime::mapCloud(double voxel_leaf) const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  auto cloud = global_map_.cloud(voxel_leaf);
  Eigen::Matrix4d t_map_odom;
  if (mapOdomTransform(&t_map_odom) && cloud && !cloud->empty()) {
    PointCloud tmp;
    pcl::transformPointCloud(*cloud, tmp, t_map_odom.cast<float>());
    cloud = PointCloudPtr(new PointCloud(tmp));
  }
  return cloud;
}
}  // namespace slam_mapping_v2
