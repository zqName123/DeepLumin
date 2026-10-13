#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/ros/param_loader.hpp>

#include <deeplumin_msgs/RelocalizationResult.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <atomic>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <syslog/logger.h>
#include <deeplumin_health_checker/health_checker/health_checker.h>

namespace {

geometry_msgs::Pose matrixToPose(const Eigen::Matrix4d& tform) {
  geometry_msgs::Pose pose;
  pose.position.x = tform(0, 3);
  pose.position.y = tform(1, 3);
  pose.position.z = tform(2, 3);
  Eigen::Quaterniond q(tform.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::PoseStamped toPoseMsg(const Eigen::Matrix4d& tform,
                                     const std::string& frame_id,
                                     const ros::Time& stamp) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose = matrixToPose(tform);
  return msg;
}

void fillPoseCovariance(geometry_msgs::PoseWithCovariance& pose,
                        const relocalization::MatchResult& result) {
  for (double& v : pose.covariance) {
    v = 0.0;
  }
  const double rmse = std::isfinite(result.inlier_rmse) ? result.inlier_rmse : 10.0;
  const double cov_pos = std::max(0.01, rmse * rmse);
  const double yaw_std = std::max(0.5 * M_PI / 180.0, std::abs(result.delta_yaw));
  pose.covariance[0] = cov_pos;
  pose.covariance[7] = cov_pos;
  pose.covariance[14] = std::max(0.04, cov_pos);
  pose.covariance[21] = 0.05;
  pose.covariance[28] = 0.05;
  pose.covariance[35] = yaw_std * yaw_std;
}

deeplumin_msgs::RelocalizationResult toResultMsg(
    const relocalization::MatchResult& result,
    const relocalization_ros::OnlineRelocalizationConfig& config,
    const ros::Time& stamp,
    const std::string& reject_reason) {
  deeplumin_msgs::RelocalizationResult msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = config.frame_id;
  msg.child_frame_id = config.child_frame_id;
  msg.pose.pose = matrixToPose(result.transform);
  fillPoseCovariance(msg.pose, result);
  msg.success = result.success;
  msg.accepted = false;
  msg.map_id = config.map_id;
  msg.method = config.method;
  msg.reject_reason = reject_reason;
  msg.score = result.fitness;
  msg.descriptor_score = result.scan_context_score;
  msg.fitness_score = result.fitness;
  msg.inlier_ratio = result.inlier_ratio;
  msg.inlier_rmse = result.inlier_rmse;
  msg.inlier_count = static_cast<uint32_t>(std::max(0, result.inlier_count));
  msg.candidate_index = -1;
  msg.keyframe_id = result.candidate_id;
  msg.yaw_diff = result.delta_yaw;
  msg.translation_diff = result.delta_translation;
  msg.elapsed_ms = result.elapsed_ms;
  return msg;
}

std::string rejectReason(const relocalization::MatchResult& result) {
  if (result.success) {
    return "";
  }
  if (result.candidate_id < 0) {
    return "no_candidate";
  }
  if (!result.converged) {
    return "gicp_not_converged";
  }
  return "quality_gate_failed";
}

}  // namespace

class GlobalRelocalizationNode {
 public:
  GlobalRelocalizationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // 节点健康监测
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh_, pnh_);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();
    
    config_ = relocalization_ros::loadOnlineRelocalizationConfig(pnh_);
    if (!core_.initialize(config_.core)) {
      throw std::runtime_error("failed to initialize relocalization core");
    }

    lidar_sub_ = nh_.subscribe(config_.lidar_topic, 1, &GlobalRelocalizationNode::onCloud, this);
    command_sub_ =
        nh_.subscribe(config_.command_topic, 5, &GlobalRelocalizationNode::onCommand, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pc_reloc/pose", 1, true);
    result_pub_ = nh_.advertise<deeplumin_msgs::RelocalizationResult>(config_.result_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/pc_reloc/status", 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_reloc/aligned_scan", 1, true);
    target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_reloc/target_submap", 1, true);

    // ROS_INFO_STREAM("Loaded " << core_.descriptor().size() << " descriptors");
    SYS_LOG(syslog::LogSeverity::INFO, false) 
            << "Loaded " << core_.descriptor().size() << " descriptors";
    // ROS_INFO_STREAM("result_topic=" << config_.result_topic
    //                 << " frame_id=" << config_.frame_id
    //                 << " child_frame_id=" << config_.child_frame_id
    //                 << " lidar_frame_id=" << config_.lidar_frame_id
    //                 << " query_cloud_frame=" << config_.query_cloud_frame
    //                 << " map_id=" << config_.map_id
    //                 << " method=" << config_.method
    //                 << " command_topic=" << config_.command_topic);
    SYS_LOG(syslog::LogSeverity::INFO, false) 
            << "result_topic=" << config_.result_topic
            << " frame_id=" << config_.frame_id
            << " child_frame_id=" << config_.child_frame_id
            << " lidar_frame_id=" << config_.lidar_frame_id
            << " query_cloud_frame=" << config_.query_cloud_frame
            << " map_id=" << config_.map_id
            << " method=" << config_.method
            << " command_topic=" << config_.command_topic;
    // ROS_INFO_STREAM("query_frame.rotate_xy_y_negx=" << (config_.query_rotate_xy_y_negx ? "true" : "false")
    //                 << " yaw_deg=" << config_.query_frame_yaw_deg);
    SYS_LOG(syslog::LogSeverity::INFO, false) 
            << "query_frame.rotate_xy_y_negx=" << (config_.query_rotate_xy_y_negx ? "true" : "false")
            << " yaw_deg=" << config_.query_frame_yaw_deg;
  }

 private:
  // ===================================================================
  // 启停命令
  // ===================================================================

  /// start/stop 命令：控制是否执行匹配流程（默认开启，兼容无命令场景）
  void onCommand(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "start") {
      matching_enabled_.store(true);
      // ROS_INFO("[global_relocalization] matching enabled (start)");
      SYS_LOG(syslog::LogSeverity::INFO, true) 
              << "matching enabled (start)";
    } else if (msg->data == "stop") {
      matching_enabled_.store(false);
      // ROS_INFO("[global_relocalization] matching disabled (stop), skip matching flow");
      SYS_LOG(syslog::LogSeverity::INFO, true) 
              << "matching disabled (stop), skip matching flow";
    } else {
      // ROS_WARN_THROTTLE(2.0, "[global_relocalization] unknown command %s (expect start/stop)",
      //                   msg->data.c_str());
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
              << "unknown command " << msg->data << " (expect start/stop)";
    }
  }


  void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // stop 命令期间跳过匹配流程（节点后台常驻，几乎零消耗）
    if (!matching_enabled_.load()) {
      return;
    }
    relocalization::CloudPtr raw(new relocalization::CloudT);
    pcl::fromROSMsg(*msg, *raw);
    double query_frame_yaw_rad = config_.query_frame_yaw_deg * M_PI / 180.0;
    if (config_.query_rotate_xy_y_negx) {
      query_frame_yaw_rad += -M_PI / 2.0;
    }
    relocalization::CloudPtr query_local = std::abs(query_frame_yaw_rad) > 1e-9
        ? relocalization::rotateCloudYaw(raw, query_frame_yaw_rad)
        : raw;
    const std::string source_mode = (config_.query_cloud_frame == "auto")
        ? msg->header.frame_id
        : config_.query_cloud_frame;
    if (source_mode == "lidar" || source_mode == config_.lidar_frame_id) {
      query_local = relocalization::transformCloud(query_local, config_.base_to_lidar);
    }
    relocalization::CloudPtr source = relocalization::preprocessCloud(query_local,
                                                                      config_.core.database.preprocess);
    const auto query = core_.relocalize(source);
    const auto& result = query.match;
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const std::string reject_reason = rejectReason(result);

    std_msgs::String status;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4)
       << "success=" << result.success
       << " candidate=" << result.candidate_id
       << " sc_score=" << result.scan_context_score
       << " fitness=" << result.fitness
       << " inlier_ratio=" << result.inlier_ratio
       << " inlier_rmse=" << result.inlier_rmse
       << " inliers=" << result.inlier_count << "/" << result.correspondence_count
       << " gicp_pts=" << result.gicp_source_points << "/" << result.gicp_target_points
       << " elapsed_ms=" << result.elapsed_ms;
    status.data = ss.str();
    status_pub_.publish(status);
    result_pub_.publish(toResultMsg(result, config_, stamp, reject_reason));

    if (!result.success) {
      // ROS_WARN_STREAM(status.data);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
              << status.data;
      return;
    }

    pose_pub_.publish(toPoseMsg(result.transform, config_.frame_id, stamp));
    if (result.aligned && aligned_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*result.aligned, out);
      out.header.stamp = stamp;
      out.header.frame_id = config_.frame_id;
      aligned_pub_.publish(out);
    }
    if (result.target && target_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*result.target, out);
      out.header.stamp = stamp;
      out.header.frame_id = config_.frame_id;
      target_pub_.publish(out);
    }
    // ROS_INFO_STREAM(status.data);
    SYS_LOG(syslog::LogSeverity::INFO, true) 
              << status.data;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher result_pub_;
  ros::Publisher status_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher target_pub_;
  relocalization_ros::OnlineRelocalizationConfig config_;
  relocalization::RelocalizationCore core_;
  std::atomic<bool> matching_enabled_{true};  ///< start 后执行匹配，stop 后跳过（后台常驻）

  std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_relocalization_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir, "");
  SYS_LOG_INIT_FULL(log_dir,
                  syslog::LogSeverity::INFO,
                  50,
                  "relocalization");

  try {
    GlobalRelocalizationNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    // ROS_ERROR_STREAM(e.what());
    SYS_LOG(syslog::LogSeverity::ERROR, true) 
              << "exception: " << e.what();
    return 1;
  }
  return 0;
}