#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/ros/param_loader.hpp>

#include <deeplumin_msgs/RelocalizationResult.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <cmath>
#include <iomanip>
#include <sstream>

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
  GlobalRelocalizationNode() : pnh_("~") {
    config_ = relocalization_ros::loadOnlineRelocalizationConfig(pnh_);
    if (!core_.initialize(config_.core)) {
      throw std::runtime_error("failed to initialize relocalization core");
    }

    lidar_sub_ = nh_.subscribe(config_.lidar_topic, 1, &GlobalRelocalizationNode::onCloud, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reloc_pose", 1, true);
    result_pub_ = nh_.advertise<deeplumin_msgs::RelocalizationResult>(config_.result_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("reloc_status", 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reloc_aligned_scan", 1, true);
    target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reloc_target_submap", 1, true);

    ROS_INFO_STREAM("Loaded " << core_.descriptor().size() << " descriptors");
    ROS_INFO_STREAM("result_topic=" << config_.result_topic
                    << " frame_id=" << config_.frame_id
                    << " child_frame_id=" << config_.child_frame_id
                    << " map_id=" << config_.map_id
                    << " method=" << config_.method);
    ROS_INFO_STREAM("query_frame.rotate_xy_y_negx=" << (config_.query_rotate_xy_y_negx ? "true" : "false")
                    << " yaw_deg=" << config_.query_frame_yaw_deg);
  }

 private:
  void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    relocalization::CloudPtr raw = relocalization::cloudFromRosMsg(*msg);
    double query_frame_yaw_rad = config_.query_frame_yaw_deg * M_PI / 180.0;
    if (config_.query_rotate_xy_y_negx) {
      query_frame_yaw_rad += -M_PI / 2.0;
    }
    relocalization::CloudPtr query_local = std::abs(query_frame_yaw_rad) > 1e-9
        ? relocalization::rotateCloudYaw(raw, query_frame_yaw_rad)
        : raw;
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
      ROS_WARN_STREAM(status.data);
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
    ROS_INFO_STREAM(status.data);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lidar_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher result_pub_;
  ros::Publisher status_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher target_pub_;
  relocalization_ros::OnlineRelocalizationConfig config_;
  relocalization::RelocalizationCore core_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_relocalization_node");
  try {
    GlobalRelocalizationNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return 1;
  }
  return 0;
}
