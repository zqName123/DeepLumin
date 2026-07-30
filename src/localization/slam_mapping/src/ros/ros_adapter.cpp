#include "slam_mapping/ros/ros_adapter.hpp"

#include "slam_mapping/ros/param_loader.hpp"
#include "slam_mapping/ros/ros_converters.hpp"

namespace slam_mapping_ros {

bool SlamMappingRosAdapter::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  nh_ = nh;
  pnh_ = pnh;
  config_ = loadConfig(pnh_);
  if (!runtime_.initialize(config_)) {
    ROS_ERROR("[slam_mapping] runtime initialize failed");
    return false;
  }
  viz_.initialize(nh_, config_.visualization, config_.frames, config_.outputs);

  imu_sub_ = nh_.subscribe(config_.sensors.imu_topic, 200000, &SlamMappingRosAdapter::onImu, this);
  lidar_sub_ = nh_.subscribe(config_.sensors.lidar_topic, 200000, &SlamMappingRosAdapter::onLidar, this);
  can_sub_ = nh_.subscribe(config_.sensors.can_topic, 1000, &SlamMappingRosAdapter::onCan, this);
  gnss_sub_ = nh_.subscribe(config_.sensors.gnss_topic, 1000, &SlamMappingRosAdapter::onGnss, this);

  start_srv_ = nh_.advertiseService("/slam_mapping/start", &SlamMappingRosAdapter::handleStart, this);
  stop_srv_ = nh_.advertiseService("/slam_mapping/stop", &SlamMappingRosAdapter::handleStop, this);
  save_srv_ = nh_.advertiseService("/slam_mapping/save", &SlamMappingRosAdapter::handleSave, this);
  reset_srv_ = nh_.advertiseService("/slam_mapping/reset", &SlamMappingRosAdapter::handleReset, this);
  get_status_srv_ = nh_.advertiseService("/slam_mapping/get_status", &SlamMappingRosAdapter::handleGetStatus, this);

  if (config_.mapping.auto_start) {
    slam_mapping::RuntimeStartRequest req;
    req.task_id = config_.mapping.task_name;
    req.task_name = config_.mapping.task_name;
    req.output_dir = config_.mapping.output_root + "/" + config_.mapping.task_name;
    req.enable_loop_closure = config_.loop_closure.enable;
    req.enable_visualization = true;
    runtime_.start(req, ros::Time::now().toSec());
    ROS_INFO("[slam_mapping] auto_start task_name=%s output_dir=%s",
             req.task_name.c_str(), req.output_dir.c_str());
  }
  ROS_INFO("[slam_mapping] initialized imu=%s lidar=%s can=%s gnss=%s",
           config_.sensors.imu_topic.c_str(),
           config_.sensors.lidar_topic.c_str(),
           config_.sensors.can_topic.c_str(),
           config_.sensors.gnss_topic.c_str());
  return true;
}

void SlamMappingRosAdapter::spinOnce() {
  const bool ok = runtime_.processOnce();
  ROS_INFO_THROTTLE(2.0, "[slam_mapping] spinOnce state=%s processOnce=%s frame_count=%llu keyframes=%llu",
                    slam_mapping::toString(runtime_.status().state).c_str(),
                    ok ? "true" : "false",
                    static_cast<unsigned long long>(runtime_.status().frame_count),
                    static_cast<unsigned long long>(runtime_.status().keyframe_count));
  publishOutputs();
}

double SlamMappingRosAdapter::processRateHz() const { return config_.mapping.process_rate_hz; }

void SlamMappingRosAdapter::shutdown() {
  const auto status = runtime_.status();
  if (status.state != slam_mapping::RuntimeState::RUNNING) {
    ROS_INFO("[slam_mapping] shutdown skip autosave state=%s running=%s output_valid=%s",
             slam_mapping::toString(status.state).c_str(),
             status.running ? "true" : "false",
             status.output_valid ? "true" : "false");
    return;
  }

  slam_mapping::RuntimeStopRequest req;
  req.task_id = status.task_id;
  req.save_map = true;
  req.save_keyframes = config_.map_saver.save_keyframes;
  req.save_keyframe_poses = config_.map_saver.save_keyframe_poses;
  req.save_trajectory = config_.map_saver.save_trajectory;

  ROS_INFO("[slam_mapping] shutdown autosave task_id=%s output_dir=%s save_keyframes=%s save_keyframe_poses=%s save_trajectory=%s",
           status.task_id.c_str(),
           status.output_dir.c_str(),
           req.save_keyframes ? "true" : "false",
           req.save_keyframe_poses ? "true" : "false",
           req.save_trajectory ? "true" : "false");
  const bool ok = runtime_.stop(req);
  const auto after = runtime_.status();
  ROS_INFO("[slam_mapping] shutdown autosave result ok=%s state=%s message=%s map_path=%s",
           ok ? "true" : "false",
           slam_mapping::toString(after.state).c_str(),
           after.message.c_str(),
           after.map_path.c_str());
}

void SlamMappingRosAdapter::onImu(const sensor_msgs::Imu::ConstPtr& msg) { runtime_.feedImu(slam_mapping_ros::fromRos(*msg)); }
void SlamMappingRosAdapter::onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) { runtime_.feedLidar(slam_mapping_ros::fromRos(*msg, config_.slam_backend)); }
void SlamMappingRosAdapter::onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) { runtime_.feedCanVelocity(slam_mapping_ros::fromRos(*msg)); }
void SlamMappingRosAdapter::onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) { runtime_.feedGnssAttitude(slam_mapping_ros::fromRos(*msg)); }

bool SlamMappingRosAdapter::handleStart(deeplumin_msgs::StartMappingRuntime::Request& req,
                                        deeplumin_msgs::StartMappingRuntime::Response& res) {
  ROS_INFO("[slam_mapping] /slam_mapping/start task_id=%s task_name=%s output_dir=%s loop=%s viz=%s reset=%s",
           req.task_id.c_str(), req.task_name.c_str(), req.output_dir.c_str(),
           req.enable_loop_closure ? "true" : "false",
           req.enable_visualization ? "true" : "false",
           req.reset_before_start ? "true" : "false");
  const auto ok = runtime_.start(slam_mapping_ros::fromRos(req), ros::Time::now().toSec());
  res.accepted = ok;
  res.state = slam_mapping::toString(runtime_.status().state);
  res.reason = runtime_.status().message.empty() ? (ok ? "accepted" : "start failed") : runtime_.status().message;
  ROS_INFO("[slam_mapping] start result accepted=%s state=%s reason=%s",
           ok ? "true" : "false", res.state.c_str(), res.reason.c_str());
  return true;
}

bool SlamMappingRosAdapter::handleStop(deeplumin_msgs::StopMappingRuntime::Request& req,
                                       deeplumin_msgs::StopMappingRuntime::Response& res) {
  ROS_INFO("[slam_mapping] /slam_mapping/stop task_id=%s save_map=%s save_keyframes=%s save_keyframe_poses=%s save_trajectory=%s",
           req.task_id.c_str(),
           req.save_map ? "true" : "false",
           req.save_keyframes ? "true" : "false",
           req.save_keyframe_poses ? "true" : "false",
           req.save_trajectory ? "true" : "false");
  const auto ok = runtime_.stop(slam_mapping_ros::fromRos(req));
  const auto status = runtime_.status();
  res.accepted = ok;
  res.state = slam_mapping::toString(status.state);
  res.reason = status.message.empty() ? (ok ? "accepted" : "stop failed") : status.message;
  res.map_path = status.map_path;
  res.output_dir = status.output_dir;
  ROS_INFO("[slam_mapping] stop result accepted=%s state=%s reason=%s map_path=%s output_dir=%s",
           ok ? "true" : "false", res.state.c_str(), res.reason.c_str(),
           res.map_path.c_str(), res.output_dir.c_str());
  return true;
}

bool SlamMappingRosAdapter::handleSave(deeplumin_msgs::SaveMappingRuntime::Request& req,
                                       deeplumin_msgs::SaveMappingRuntime::Response& res) {
  ROS_INFO("[slam_mapping] /slam_mapping/save task_id=%s output_dir=%s map_name=%s resolution=%.3f",
           req.task_id.c_str(), req.output_dir.c_str(), req.map_name.c_str(), req.resolution);
  const auto ok = runtime_.save(slam_mapping_ros::fromRos(req));
  slam_mapping::SaveResult result;
  const auto status = runtime_.status();
  result.success = ok;
  result.message = status.message;
  result.map_path = status.map_path;
  result.keyframe_dir = status.keyframe_dir;
  result.keyframe_pose_path = status.keyframe_pose_path;
  result.trajectory_path = status.trajectory_path;
  slam_mapping_ros::fillRos(result, &res);
  ROS_INFO("[slam_mapping] save result success=%s message=%s map_path=%s",
           ok ? "true" : "false", res.message.c_str(), res.map_path.c_str());
  return true;
}

bool SlamMappingRosAdapter::handleReset(std_srvs::Trigger::Request&,
                                        std_srvs::Trigger::Response& res) {
  runtime_.reset();
  res.success = true;
  res.message = "reset";
  return true;
}

bool SlamMappingRosAdapter::handleGetStatus(deeplumin_msgs::GetMappingRuntimeStatus::Request&,
                                            deeplumin_msgs::GetMappingRuntimeStatus::Response& res) {
  ROS_INFO_THROTTLE(1.0, "[slam_mapping] /slam_mapping/get_status state=%s task_id=%s output_valid=%s",
                    slam_mapping::toString(runtime_.status().state).c_str(),
                    runtime_.status().task_id.c_str(),
                    runtime_.status().output_valid ? "true" : "false");
  slam_mapping_ros::fillRos(runtime_.status(), &res, ros::Time::now());
  return true;
}

void SlamMappingRosAdapter::publishOutputs() {
  const auto status = runtime_.status();
  ROS_INFO_THROTTLE(2.0, "[slam_mapping] publishOutputs state=%s output_valid=%s map_path=%s",
                    slam_mapping::toString(status.state).c_str(),
                    status.output_valid ? "true" : "false",
                    status.map_path.c_str());
  viz_.publishStatus(status);
  viz_.publishOdom(runtime_.currentOdom());
  viz_.publishPath(runtime_.keyframes());
  viz_.publishCurrentScan(runtime_.currentScan(), config_.frames.base_link);
  viz_.publishLocalMap(runtime_.localMap(), config_.frames.odom);
  viz_.publishMapPreview(runtime_.keyframes());
  viz_.publishLoopCurrentSubmap(runtime_.latestLoopCurrentSubmap(), config_.frames.map);
  viz_.publishLoopHistorySubmap(runtime_.latestLoopHistorySubmap(), config_.frames.map);
  viz_.publishKeyframeMarkers(runtime_.keyframes());
  viz_.publishLoopMarkers(runtime_.loopPairs(), runtime_.keyframes());
  if (config_.mapping.publish_tf && runtime_.currentOdom().valid) {
    tf_broadcaster_.sendTransform(slam_mapping_ros::toRosTransform(runtime_.currentOdom(), config_.frames.odom, config_.frames.base_link));
  }
}

}  // namespace slam_mapping_ros
