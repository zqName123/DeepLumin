#include "pose_fusion/ros/ros_adapter.hpp"

namespace pose_fusion_ros {
namespace {

pose_fusion::Pose3d poseFromRos(const geometry_msgs::Pose& msg) {
  pose_fusion::Pose3d pose;
  pose.position = pose_fusion::Vec3d(msg.position.x, msg.position.y, msg.position.z);
  pose.orientation = pose_fusion::Quatd(msg.orientation.w, msg.orientation.x,
                                        msg.orientation.y, msg.orientation.z);
  if (pose.orientation.norm() < 1e-9) {
    pose.orientation = pose_fusion::Quatd::Identity();
  } else {
    pose.orientation.normalize();
  }
  return pose;
}

void fillRosPose(const pose_fusion::Pose3d& pose, geometry_msgs::Pose* msg) {
  msg->position.x = pose.position.x();
  msg->position.y = pose.position.y();
  msg->position.z = pose.position.z();
  const auto q = pose.orientation.normalized();
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
  msg->orientation.w = q.w();
}

pose_fusion::Mat6d covarianceFromRos(const boost::array<double, 36>& cov) {
  pose_fusion::Mat6d out;
  for (int i = 0; i < 36; ++i) {
    out(i / 6, i % 6) = cov[i];
  }
  return out;
}

void fillRosCovariance(const pose_fusion::Mat6d& cov, boost::array<double, 36>* out) {
  for (int i = 0; i < 36; ++i) {
    (*out)[i] = cov(i / 6, i % 6);
  }
}

}  // namespace

pose_fusion::PoseObservation fromOdom(const nav_msgs::Odometry& msg,
                                      pose_fusion::ObservationSource source) {
  pose_fusion::PoseObservation obs;
  obs.source = source;
  obs.timestamp = msg.header.stamp.toSec();
  obs.pose = poseFromRos(msg.pose.pose);
  obs.velocity = pose_fusion::Vec3d(msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y,
                                    msg.twist.twist.linear.z);
  obs.covariance = covarianceFromRos(msg.pose.covariance);
  obs.frame_id = msg.header.frame_id;
  obs.child_frame_id = msg.child_frame_id;
  obs.valid = true;
  return obs;
}

pose_fusion::PoseObservation fromGlobalMatch(const deeplumin_msgs::GlobalMatchResult& msg) {
  pose_fusion::PoseObservation obs;
  obs.source = pose_fusion::ObservationSource::GLOBAL_MATCHER;
  obs.timestamp = msg.header.stamp.toSec();
  obs.pose = poseFromRos(msg.pose.pose);
  obs.covariance = covarianceFromRos(msg.pose.covariance);
  obs.frame_id = msg.header.frame_id;
  obs.child_frame_id = msg.child_frame_id;
  obs.map_id = msg.map_id;
  obs.valid = true;
  obs.success = msg.success;
  obs.converged = msg.converged;
  obs.reject_reason = msg.reject_reason;
  obs.fitness_score = msg.fitness_score;
  obs.inlier_ratio = msg.inlier_ratio;
  obs.inlier_rmse = msg.inlier_rmse;
  obs.initial_translation_error = msg.initial_translation_error;
  obs.initial_yaw_error = msg.initial_yaw_error;
  return obs;
}

pose_fusion::ResetObservation fromInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg,
                                              const std::string& map_id) {
  pose_fusion::ResetObservation obs;
  obs.source = pose_fusion::ObservationSource::MANUAL;
  obs.timestamp = msg.header.stamp.toSec();
  if (obs.timestamp <= 0.0) {
    obs.timestamp = ros::Time::now().toSec();
  }
  obs.map_base = poseFromRos(msg.pose.pose);
  obs.covariance = covarianceFromRos(msg.pose.covariance);
  obs.frame_id = msg.header.frame_id;
  obs.child_frame_id = "";
  obs.map_id = map_id;
  obs.reason = "initialpose";
  obs.approved_by = "manual";
  obs.valid = true;
  return obs;
}

pose_fusion::ResetObservation fromRelocalization(const deeplumin_msgs::RelocalizationResult& msg) {
  pose_fusion::ResetObservation obs;
  obs.source = pose_fusion::ObservationSource::RELOCALIZATION;
  obs.timestamp = msg.header.stamp.toSec();
  obs.map_base = poseFromRos(msg.pose.pose);
  obs.covariance = covarianceFromRos(msg.pose.covariance);
  obs.frame_id = msg.header.frame_id;
  obs.child_frame_id = msg.child_frame_id;
  obs.map_id = msg.map_id;
  obs.reason = "accepted_relocalization";
  obs.approved_by = "localization_manager";
  obs.valid = msg.success && msg.accepted;
  return obs;
}

pose_fusion::ObserverPolicyData fromObserverPolicy(const deeplumin_msgs::ObserverPolicy& msg) {
  pose_fusion::ObserverPolicyData policy;
  policy.scene_type = msg.scene_type;
  policy.reason = msg.reason;
  policy.use_dr = msg.use_dr;
  policy.use_slam = msg.use_slam;
  policy.use_global_matcher = msg.use_global_matcher;
  policy.use_relocalization = msg.use_relocalization;
  policy.use_gnss = msg.use_gnss;
  policy.allow_fusion_reset = msg.allow_fusion_reset;
  policy.dr_weight = msg.dr_weight;
  policy.slam_weight = msg.slam_weight;
  policy.global_matcher_weight = msg.global_matcher_weight;
  policy.relocalization_weight = msg.relocalization_weight;
  return policy;
}

nav_msgs::Odometry toOdomMsg(const pose_fusion::FusionOutput& output,
                             const std::string& map_frame,
                             const std::string& base_frame) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(output.timestamp);
  msg.header.frame_id = map_frame;
  msg.child_frame_id = base_frame;
  fillRosPose(output.map_base, &msg.pose.pose);
  fillRosCovariance(output.covariance, &msg.pose.covariance);
  msg.twist.twist.linear.x = output.velocity_map.x();
  msg.twist.twist.linear.y = output.velocity_map.y();
  msg.twist.twist.linear.z = output.velocity_map.z();
  return msg;
}

geometry_msgs::PoseWithCovarianceStamped toPoseMsg(const pose_fusion::FusionOutput& output,
                                                   const std::string& map_frame) {
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.stamp = ros::Time(output.timestamp);
  msg.header.frame_id = map_frame;
  fillRosPose(output.map_base, &msg.pose.pose);
  fillRosCovariance(output.covariance, &msg.pose.covariance);
  return msg;
}

deeplumin_msgs::FusionStatus toStatusMsg(const pose_fusion::FusionStatusData& status,
                                          const std::string& map_frame) {
  deeplumin_msgs::FusionStatus msg;
  msg.header.stamp = ros::Time(status.timestamp);
  msg.header.frame_id = map_frame;
  msg.level = status.output_valid ? deeplumin_msgs::FusionStatus::LEVEL_NORMAL
                                  : deeplumin_msgs::FusionStatus::LEVEL_FAILURE;
  if (status.state == pose_fusion::FusionStateName::DEGRADED) {
    msg.level = deeplumin_msgs::FusionStatus::LEVEL_DEGRADED;
  }
  msg.state = pose_fusion::toString(status.state);
  msg.initialized = status.initialized;
  msg.output_valid = status.output_valid;
  msg.failure_reason = status.failure_reason;
  msg.current_map_id = status.current_map_id;
  msg.active_scene = status.active_scene;
  msg.last_dr_age = status.last_dr_age;
  msg.last_slam_age = status.last_slam_age;
  msg.last_global_matcher_age = status.last_global_matcher_age;
  msg.last_relocalization_age = status.last_relocalization_age;
  msg.last_gnss_age = 1e9;
  msg.last_can_age = 1e9;
  msg.last_imu_age = 1e9;
  msg.last_lidar_age = 1e9;
  msg.position_std = status.position_std;
  msg.velocity_std = status.velocity_std;
  msg.roll_std = 1.0;
  msg.pitch_std = 1.0;
  msg.yaw_std = status.yaw_std;
  msg.used_sources = status.used_sources;
  msg.rejected_sources = status.rejected_sources;
  msg.reject_reasons = status.reject_reasons;
  msg.is_dr_available = status.is_dr_available;
  msg.is_slam_available = status.is_slam_available;
  msg.is_global_matcher_available = status.is_global_matcher_available;
  msg.is_relocalization_available = status.is_relocalization_available;
  msg.is_gnss_available = false;
  msg.is_can_available = false;
  msg.is_imu_available = false;
  msg.is_lidar_available = status.is_slam_available;
  return msg;
}

geometry_msgs::PoseStamped toPathPose(const pose_fusion::FusionOutput& output,
                                      const std::string& map_frame) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time(output.timestamp);
  msg.header.frame_id = map_frame;
  fillRosPose(output.map_base, &msg.pose);
  return msg;
}

geometry_msgs::TransformStamped toTransformMsg(const pose_fusion::Pose3d& pose,
                                               const std::string& parent_frame,
                                               const std::string& child_frame,
                                               const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.transform.translation.x = pose.position.x();
  msg.transform.translation.y = pose.position.y();
  msg.transform.translation.z = pose.position.z();
  const auto q = pose.orientation.normalized();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

}  // namespace pose_fusion_ros
