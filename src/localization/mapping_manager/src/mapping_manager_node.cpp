#include "mapping_manager/core/manager_core.hpp"
#include "mapping_manager/ros/ros_adapter.hpp"
#include "mapping_manager/ros/bag_recorder.hpp"

#include <deeplumin_msgs/CheckMappingEnvironment.h>
#include <deeplumin_msgs/GetMappingRuntimeStatus.h>
#include <deeplumin_msgs/GetMappingStatus.h>
#include <deeplumin_msgs/MappingEnvironmentStatus.h>
#include <deeplumin_msgs/MappingRecordStatus.h>
#include <deeplumin_msgs/MappingStatus.h>
#include <deeplumin_msgs/StartBagRecording.h>
#include <deeplumin_msgs/StartMapping.h>
#include <deeplumin_msgs/StartMappingRuntime.h>
#include <deeplumin_msgs/StopBagRecording.h>
#include <deeplumin_msgs/StopMapping.h>
#include <deeplumin_msgs/StopMappingRuntime.h>
#include <ros/ros.h>

#include <sys/stat.h>

class MappingManagerNode {
 public:
  MappingManagerNode() : nh_(), pnh_("~") {
    config_ = mapping_manager_ros::loadConfig(pnh_);
    core_.initialize(config_);

    status_pub_ = nh_.advertise<deeplumin_msgs::MappingStatus>(config_.topics.status, 10);
    record_status_pub_ = nh_.advertise<deeplumin_msgs::MappingRecordStatus>(config_.topics.record_status, 10);
    environment_pub_ = nh_.advertise<deeplumin_msgs::MappingEnvironmentStatus>(
        config_.topics.environment_last_result, 10, true);

    check_environment_srv_ = nh_.advertiseService(config_.services.check_environment,
                                                  &MappingManagerNode::handleCheckEnvironment, this);
    get_status_srv_ = nh_.advertiseService(config_.services.get_status,
                                           &MappingManagerNode::handleGetStatus, this);
    start_mapping_srv_ = nh_.advertiseService(config_.services.start_mapping,
                                              &MappingManagerNode::handleStartMapping, this);
    stop_mapping_srv_ = nh_.advertiseService(config_.services.stop_mapping,
                                             &MappingManagerNode::handleStopMapping, this);
    start_bag_srv_ = nh_.advertiseService(config_.services.start_bag_recording,
                                          &MappingManagerNode::handleStartBag, this);
    stop_bag_srv_ = nh_.advertiseService(config_.services.stop_bag_recording,
                                         &MappingManagerNode::handleStopBag, this);

    if (config_.backend.enabled) {
      backend_start_client_ = nh_.serviceClient<deeplumin_msgs::StartMappingRuntime>(config_.backend.start_service);
      backend_stop_client_ = nh_.serviceClient<deeplumin_msgs::StopMappingRuntime>(config_.backend.stop_service);
      backend_get_status_client_ = nh_.serviceClient<deeplumin_msgs::GetMappingRuntimeStatus>(config_.backend.get_status_service);
      backend_status_sub_ = nh_.subscribe(config_.backend.status_topic, 10,
                                          &MappingManagerNode::onBackendStatus, this);
    }

    const auto initial_env = core_.getStatus().environment_status;
    environment_pub_.publish(mapping_manager_ros::toRos(initial_env, ros::Time::now()));

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.runtime.status_hz)),
                             &MappingManagerNode::onTimer, this);
    record_active_since_ = ros::Time::now();
    ROS_INFO("[mapping_manager] started backend_enabled=%s", config_.backend.enabled ? "true" : "false");
  }

  ~MappingManagerNode() {
    if (recorder_.running()) {
      ROS_WARN("[mapping_manager] node shutting down with active bag recording, stopping rosbag pid=%d",
               static_cast<int>(recorder_.pid()));
    }
    recorder_.shutdown();
  }

 private:
  static double nowSec() { return ros::Time::now().toSec(); }

  bool spawnBagRecorder(const std::string& bag_path, const std::vector<std::string>& topics) {
    if (!recorder_.start(bag_path, topics)) {
      ROS_ERROR("[mapping_manager] failed to spawn rosbag record path=%s", bag_path.c_str());
      return false;
    }
    record_active_since_ = ros::Time::now();
    ROS_INFO("[mapping_manager] bag recording started path=%s topics=%zu",
             bag_path.c_str(), topics.size());
    return true;
  }

  void startAutoBagRecording() {
    record_active_since_ = ros::Time::now();
    const auto snapshot = core_.getStatus();
    if (!snapshot.record_status.active || recorder_.running()) return;
    std::vector<std::string> topics = config_.defaults.bag_topics;
    if (topics.empty()) {
      core_.failBagRecording("自动录包未配置录制话题", nowSec());
      ROS_WARN("[mapping_manager] auto bag recording skipped: no topics configured");
      return;
    }
    if (!spawnBagRecorder(snapshot.record_status.bag_path, topics)) {
      core_.failBagRecording("无法启动 rosbag record 进程", nowSec());
    }
  }

  bool backendServiceReady(ros::ServiceClient& client, const std::string& name, double timeout_sec = 1.0) {
    if (!config_.backend.enabled) return false;
    if (client.exists()) return true;
    if (client.waitForExistence(ros::Duration(timeout_sec))) return true;
    ROS_WARN_THROTTLE(2.0, "[mapping_manager] backend service not ready: %s", name.c_str());
    return false;
  }

  void syncBackendStatus() {
    if (!config_.backend.enabled) return;
    if (!backendServiceReady(backend_get_status_client_, config_.backend.get_status_service, 0.1)) return;
    deeplumin_msgs::GetMappingRuntimeStatus srv;
    if (!backend_get_status_client_.call(srv)) {
      ROS_WARN_THROTTLE(2.0, "[mapping_manager] failed to call backend get_status service");
      return;
    }
    core_.updateFromBackend(mapping_manager_ros::fromRos(srv.response.status), nowSec());
  }

  void onBackendStatus(const deeplumin_msgs::MappingStatus::ConstPtr& msg) {
    ROS_INFO_THROTTLE(2.0,
                      "[mapping_manager] backend status topic state=%s task_id=%s output_valid=%s",
                      msg->state.c_str(), msg->task_id.c_str(), msg->output_valid ? "true" : "false");
    core_.updateFromBackend(mapping_manager_ros::fromRos(*msg), nowSec());
  }

  bool handleCheckEnvironment(deeplumin_msgs::CheckMappingEnvironment::Request& req,
                              deeplumin_msgs::CheckMappingEnvironment::Response& res) {
    auto env = core_.checkEnvironment(req.include_runtime_checks, req.config_path, nowSec());
    if (config_.backend.enabled && req.include_runtime_checks) {
      const bool ready = backendServiceReady(backend_get_status_client_, config_.backend.get_status_service, 0.2) &&
                         backendServiceReady(backend_start_client_, config_.backend.start_service, 0.2) &&
                         backendServiceReady(backend_stop_client_, config_.backend.stop_service, 0.2);
      mapping_manager::EnvironmentItem item;
      item.name = "slam_mapping_backend";
      item.ok = ready;
      item.level = ready ? "PASS" : "FAIL";
      item.detail = ready ? "slam_mapping backend 服务可用" : "slam_mapping backend 服务不可用";
      env.items.push_back(item);
      env.overall_ok = env.overall_ok && ready;
      env.summary = ready ? env.summary : (env.summary + "; backend unavailable");
    }
    mapping_manager_ros::fillRos(env, &res);
    environment_pub_.publish(mapping_manager_ros::toRos(env, ros::Time::now()));
    return true;
  }

  bool handleGetStatus(deeplumin_msgs::GetMappingStatus::Request&,
                       deeplumin_msgs::GetMappingStatus::Response& res) {
    if (config_.backend.enabled) syncBackendStatus();
    mapping_manager_ros::fillRos(core_.getStatus(), &res, ros::Time::now());
    return true;
  }

  bool handleStartMapping(deeplumin_msgs::StartMapping::Request& req,
                          deeplumin_msgs::StartMapping::Response& res) {
    ROS_INFO("[mapping_manager] /mapping/start task_name=%s loop=%s viz=%s auto_record=%s",
             req.task_name.c_str(),
             req.enable_loop_closure ? "true" : "false",
             req.enable_visualization ? "true" : "false",
             req.auto_start_recording ? "true" : "false");
    const auto out = core_.startMapping(mapping_manager_ros::fromRos(req), nowSec());
    if (!out.accepted) {
      mapping_manager_ros::fillRos(out, &res);
      return true;
    }

    if (config_.backend.enabled) {
      if (!backendServiceReady(backend_start_client_, config_.backend.start_service)) {
        core_.markFailed("slam_mapping start service unavailable", nowSec());
        core_.failBagRecording("slam_mapping 启动失败，自动录包已取消", nowSec());
        mapping_manager::StartMappingResponseData failed = out;
        failed.accepted = false;
        failed.state = "FAILED";
        failed.reason = "slam_mapping start service unavailable";
        mapping_manager_ros::fillRos(failed, &res);
        return true;
      }
      const auto snapshot = core_.getStatus();
      deeplumin_msgs::StartMappingRuntime srv;
      srv.request.task_id = snapshot.status.task_id;
      srv.request.task_name = snapshot.status.task_name;
      srv.request.config_path = req.config_path;
      srv.request.output_dir = snapshot.status.output_dir;
      srv.request.enable_loop_closure = req.enable_loop_closure;
      srv.request.enable_visualization = req.enable_visualization;
      srv.request.reset_before_start = true;
      ROS_INFO("[mapping_manager] calling backend start service=%s task_id=%s output_dir=%s",
               config_.backend.start_service.c_str(), srv.request.task_id.c_str(), srv.request.output_dir.c_str());
      if (!backend_start_client_.call(srv) || !srv.response.accepted) {
        ROS_ERROR("[mapping_manager] backend start failed state=%s reason=%s",
                  srv.response.state.c_str(), srv.response.reason.c_str());
        core_.markFailed(srv.response.reason.empty() ? "backend start failed" : srv.response.reason, nowSec());
        core_.failBagRecording("slam_mapping 启动失败，自动录包已取消", nowSec());
        mapping_manager::StartMappingResponseData failed = out;
        failed.accepted = false;
        failed.state = srv.response.state.empty() ? "FAILED" : srv.response.state;
        failed.reason = srv.response.reason.empty() ? "backend start failed" : srv.response.reason;
        mapping_manager_ros::fillRos(failed, &res);
        return true;
      }
      ROS_INFO("[mapping_manager] backend start accepted state=%s reason=%s",
               srv.response.state.c_str(), srv.response.reason.c_str());
      // 立即同步后端状态，让 GUI 尽快看到 RUNNING 状态
      syncBackendStatus();
    }

    if (req.auto_start_recording) startAutoBagRecording();

    // 使用同步后的最新状态填充响应
    const auto post_start = core_.getStatus().status;
    res.accepted = out.accepted;
    res.task_id = out.task_id;
    res.state = mapping_manager::toString(post_start.state);
    res.reason = post_start.status_message.empty() ? "建图已启动" : post_start.status_message;
    return true;
  }

  bool handleStopMapping(deeplumin_msgs::StopMapping::Request& req,
                         deeplumin_msgs::StopMapping::Response& res) {
    ROS_INFO("[mapping_manager] /mapping/stop task_id=%s save_map=%s save_keyframes=%s save_keyframe_poses=%s save_trajectory=%s",
             req.task_id.c_str(),
             req.save_map ? "true" : "false",
             req.save_keyframes ? "true" : "false",
             req.save_keyframe_poses ? "true" : "false",
             req.save_trajectory ? "true" : "false");
    const auto out = core_.stopMapping(mapping_manager_ros::fromRos(req), nowSec());
    if (!out.accepted) {
      mapping_manager_ros::fillRos(out, &res);
      return true;
    }

    if (config_.backend.enabled) {
      if (!backendServiceReady(backend_stop_client_, config_.backend.stop_service)) {
        core_.markFailed("slam_mapping stop service unavailable", nowSec());
        mapping_manager::StopMappingResponseData failed = out;
        failed.accepted = false;
        failed.state = "FAILED";
        failed.reason = "slam_mapping stop service unavailable";
        mapping_manager_ros::fillRos(failed, &res);
        return true;
      }
      deeplumin_msgs::StopMappingRuntime srv;
      srv.request.task_id = req.task_id;
      srv.request.save_map = req.save_map;
      srv.request.save_keyframes = req.save_keyframes;
      srv.request.save_keyframe_poses = req.save_keyframe_poses;
      srv.request.save_trajectory = req.save_trajectory;
      ROS_INFO("[mapping_manager] calling backend stop service=%s task_id=%s",
               config_.backend.stop_service.c_str(), srv.request.task_id.c_str());
      if (!backend_stop_client_.call(srv) || !srv.response.accepted) {
        ROS_ERROR("[mapping_manager] backend stop failed state=%s reason=%s",
                  srv.response.state.c_str(), srv.response.reason.c_str());
        core_.markFailed(srv.response.reason.empty() ? "backend stop failed" : srv.response.reason, nowSec());
        mapping_manager::StopMappingResponseData failed = out;
        failed.accepted = false;
        failed.state = srv.response.state.empty() ? "FAILED" : srv.response.state;
        failed.reason = srv.response.reason.empty() ? "backend stop failed" : srv.response.reason;
        mapping_manager_ros::fillRos(failed, &res);
        return true;
      }
      ROS_INFO("[mapping_manager] backend stop accepted state=%s reason=%s map_path=%s output_dir=%s",
               srv.response.state.c_str(), srv.response.reason.c_str(),
               srv.response.map_path.c_str(), srv.response.output_dir.c_str());
      syncBackendStatus();
      // 使用同步后的最新状态填充响应，让 GUI 立即看到最终状态（而非中间 STOPPING）
      const auto post_status = core_.getStatus().status;
      res.accepted = true;
      res.state = mapping_manager::toString(post_status.state);
      res.reason = post_status.status_message.empty() ? "建图已停止" : post_status.status_message;
      return true;
    }

    mapping_manager_ros::fillRos(out, &res);
    return true;
  }

  bool handleStartBag(deeplumin_msgs::StartBagRecording::Request& req,
                      deeplumin_msgs::StartBagRecording::Response& res) {
    record_active_since_ = ros::Time::now();
    const auto out = core_.startBagRecording(mapping_manager_ros::fromRos(req), nowSec());
    if (!out.accepted) {
      mapping_manager_ros::fillRos(out, &res);
      return true;
    }

    std::vector<std::string> topics(req.topics.begin(), req.topics.end());
    if (topics.empty()) topics = config_.defaults.bag_topics;
    if (topics.empty() || !spawnBagRecorder(out.bag_path, topics)) {
      const std::string reason = topics.empty() ? "未指定录制话题" : "无法启动 rosbag record 进程";
      core_.failBagRecording(reason, nowSec());
      const auto snapshot = core_.getStatus();
      res.accepted = false;
      res.state = mapping_manager::toString(snapshot.record_status.state);
      res.reason = reason;
      res.bag_path = out.bag_path;
      ROS_ERROR("[mapping_manager] start bag rejected: %s", reason.c_str());
      return true;
    }

    mapping_manager_ros::fillRos(out, &res);
    return true;
  }

  bool handleStopBag(deeplumin_msgs::StopBagRecording::Request& req,
                     deeplumin_msgs::StopBagRecording::Response& res) {
    const auto out = core_.stopBagRecording(mapping_manager_ros::fromRos(req), nowSec());
    if (!out.accepted) {
      mapping_manager_ros::fillRos(out, &res);
      return true;
    }

    if (!recorder_.stop()) {
      ROS_WARN("[mapping_manager] bag recorder did not exit gracefully, force killed");
    } else {
      ROS_INFO("[mapping_manager] bag recording stopped path=%s", out.bag_path.c_str());
    }
    struct stat st{};
    if (stat(out.bag_path.c_str(), &st) == 0) {
      core_.setRecordBytes(static_cast<unsigned long long>(st.st_size), nowSec());
    }
    mapping_manager_ros::fillRos(out, &res);
    return true;
  }

  void onTimer(const ros::TimerEvent&) {
    core_.update(nowSec());
    recorder_.poll();

    // 定期同步后端状态（每 2 次 ≈ 每 0.4 秒 @5Hz），确保关键帧/回环计数等字段保持最新
    {
      static int sync_counter = 0;
      if (++sync_counter >= 2) {
        sync_counter = 0;
        if (config_.backend.enabled) syncBackendStatus();
      }
    }

    auto snapshot = core_.getStatus();
    if (snapshot.record_status.active && !recorder_.running() && !recorder_.stopping() &&
        (ros::Time::now() - record_active_since_).toSec() > 2.0) {
      core_.failBagRecording("rosbag record 进程意外退出", nowSec());
      snapshot = core_.getStatus();
    } else if (snapshot.record_status.active && !snapshot.record_status.bag_path.empty()) {
      struct stat st{};
      if (stat(snapshot.record_status.bag_path.c_str(), &st) == 0) {
        core_.setRecordBytes(static_cast<unsigned long long>(st.st_size), nowSec());
        snapshot = core_.getStatus();
      }
    }

    const ros::Time stamp = ros::Time::now();
    status_pub_.publish(mapping_manager_ros::toRos(snapshot.status, stamp));
    record_status_pub_.publish(mapping_manager_ros::toRos(snapshot.record_status, stamp));
    environment_pub_.publish(mapping_manager_ros::toRos(snapshot.environment_status, stamp));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  mapping_manager::ManagerConfig config_;
  mapping_manager::ManagerCore core_;
  mapping_manager::BagRecorder recorder_;
  ros::Time record_active_since_;  // guard against false "unexpected exit" during startup

  ros::Publisher status_pub_;
  ros::Publisher record_status_pub_;
  ros::Publisher environment_pub_;

  ros::ServiceServer check_environment_srv_;
  ros::ServiceServer get_status_srv_;
  ros::ServiceServer start_mapping_srv_;
  ros::ServiceServer stop_mapping_srv_;
  ros::ServiceServer start_bag_srv_;
  ros::ServiceServer stop_bag_srv_;
  ros::Timer timer_;

  ros::ServiceClient backend_start_client_;
  ros::ServiceClient backend_stop_client_;
  ros::ServiceClient backend_get_status_client_;
  ros::Subscriber backend_status_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_manager");
  MappingManagerNode node;
  ros::spin();
  return 0;
}