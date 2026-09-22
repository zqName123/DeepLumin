#include "localization_v2/ros/relocalization_ros_adapter.hpp"


#include <utility>

namespace localization_v2 {

RelocalizationRosAdapter::RelocalizationRosAdapter(ros::NodeHandle& nh,
                                                   const std::string& result_topic,
                                                   const std::string& command_topic)
    : nh_(nh), result_topic_(result_topic), command_topic_(command_topic) {
  result_sub_ = nh_.subscribe(result_topic_, 2, &RelocalizationRosAdapter::onResult, this);
  command_pub_ = nh_.advertise<std_msgs::String>(command_topic_, 1, true);
  // ROS_INFO("[localization_v2] relocalization adapter: result=%s command=%s",
  //          result_topic_.c_str(), command_topic_.c_str());
  SYS_LOG(syslog::LogSeverity::INFO, true)
          << "relocalization adapter: result=" << result_topic_ << " command=" << command_topic_;


}

void RelocalizationRosAdapter::setResultCallback(RelocalizationResultCallback callback) {
  callback_ = std::move(callback);
}

void RelocalizationRosAdapter::start() {
  publishCommand("start");
  // ROS_INFO("[localization_v2] relocalization start requested");
  SYS_LOG(syslog::LogSeverity::INFO, true) 
          << "relocalization start requested";
}

void RelocalizationRosAdapter::stop() {
  publishCommand("stop");
  // ROS_INFO("[localization_v2] relocalization stop requested");
  SYS_LOG(syslog::LogSeverity::INFO, true) 
          << "relocalization stop requested";
}

void RelocalizationRosAdapter::publishCommand(const std::string& command) {
  std_msgs::String msg;
  msg.data = command;
  command_pub_.publish(msg);
}

void RelocalizationRosAdapter::onResult(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg) {
  if (!callback_) {
    return;
  }

  RelocalizationSignal signal;
  signal.success = msg->success;
  signal.accepted = msg->accepted;
  signal.map_base.position << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  signal.map_base.orientation =
      Quatd(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (!isFinite(signal.map_base.orientation)) {
    signal.map_base.orientation = Quatd::Identity();
  } else {
    signal.map_base.orientation.normalize();
  }
  signal.fitness_score = msg->fitness_score;
  signal.inlier_ratio = msg->inlier_ratio;
  signal.inlier_rmse = msg->inlier_rmse;
  signal.map_id = msg->map_id;
  signal.method = msg->method;

  callback_(signal);
}

}  // namespace localization_v2
