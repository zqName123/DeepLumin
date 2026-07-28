#include "simple_odom_fusion/core/math_utils.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

namespace sof = simple_odom_fusion;

namespace {

sof::Pose3d poseFromRos(const geometry_msgs::Pose& msg) {
  sof::Pose3d pose;
  pose.position = sof::Vec3d(msg.position.x, msg.position.y, msg.position.z);
  pose.orientation = sof::Quatd(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (!sof::isFinite(pose.orientation)) {
    pose.orientation = sof::Quatd::Identity();
  } else {
    pose.orientation.normalize();
  }
  return pose;
}

void fillPose(const sof::Pose3d& pose, geometry_msgs::Pose* msg) {
  msg->position.x = pose.position.x();
  msg->position.y = pose.position.y();
  msg->position.z = pose.position.z();
  const auto q = pose.orientation.normalized();
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
  msg->orientation.w = q.w();
}

Eigen::Matrix4f toMatrix4f(const sof::Pose3d& pose) {
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix().cast<float>();
  mat.block<3, 1>(0, 3) = pose.position.cast<float>();
  return mat;
}

geometry_msgs::TransformStamped toTf(const sof::Pose3d& pose,
                                     const std::string& parent,
                                     const std::string& child,
                                     const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
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

bool loadBool(ros::NodeHandle& nh, const std::string& key, bool fallback) {
  bool value = fallback;
  nh.param<bool>(key, value, fallback);
  return value;
}

}  // namespace

class YswStyleFusionNode {
 public:
  YswStyleFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadConfig();

    slam_sub_ = nh_.subscribe(slam_odom_topic_, 100, &YswStyleFusionNode::onSlamOdom, this);
    gm_sub_ = nh_.subscribe(global_matcher_result_topic_, 20, &YswStyleFusionNode::onGlobalMatch, this);
    initial_pose_sub_ = nh_.subscribe(initial_pose_topic_, 5, &YswStyleFusionNode::onInitialPose, this);
    cloud_sub_ = nh_.subscribe(source_cloud_topic_, 5, &YswStyleFusionNode::onSourceCloud, this);

    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(fused_odom_topic_, 50);
    map_to_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(map_to_odom_topic_, 10);
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(fused_pose_topic_, 20);
    path_pub_ = nh_.advertise<nav_msgs::Path>(fused_path_topic_, 5);
    gm_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(gm_source_cloud_odom_topic_, 2);
    path_.header.frame_id = map_frame_;

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, output_rate_hz_)),
                             &YswStyleFusionNode::onTimer, this);

    ROS_INFO("[ysw_style_fusion] slam=%s gm=%s fused=%s map_to_odom=%s gm_cloud=%s",
             slam_odom_topic_.c_str(), global_matcher_result_topic_.c_str(), fused_odom_topic_.c_str(),
             map_to_odom_topic_.c_str(), gm_source_cloud_odom_topic_.c_str());
  }

 private:
  void loadConfig() {
    pnh_.param<std::string>("frames/map", map_frame_, map_frame_);
    pnh_.param<std::string>("frames/odom", odom_frame_, odom_frame_);
    pnh_.param<std::string>("frames/base_link", base_frame_, base_frame_);

    pnh_.param<std::string>("topics/slam_odom", slam_odom_topic_, slam_odom_topic_);
    pnh_.param<std::string>("topics/global_matcher_result", global_matcher_result_topic_,
                            global_matcher_result_topic_);
    pnh_.param<std::string>("topics/initial_pose", initial_pose_topic_, initial_pose_topic_);
    pnh_.param<std::string>("topics/source_cloud", source_cloud_topic_, source_cloud_topic_);
    pnh_.param<std::string>("topics/fused_odom", fused_odom_topic_, fused_odom_topic_);
    pnh_.param<std::string>("topics/map_to_odom", map_to_odom_topic_, map_to_odom_topic_);
    pnh_.param<std::string>("topics/fused_pose", fused_pose_topic_, fused_pose_topic_);
    pnh_.param<std::string>("topics/fused_path", fused_path_topic_, fused_path_topic_);
    pnh_.param<std::string>("topics/gm_source_cloud_odom", gm_source_cloud_odom_topic_,
                            gm_source_cloud_odom_topic_);

    pnh_.param<double>("runtime/output_rate_hz", output_rate_hz_, output_rate_hz_);
    pnh_.param<int>("runtime/max_path_size", max_path_size_, max_path_size_);
    publish_tf_ = loadBool(pnh_, "runtime/publish_tf", publish_tf_);
    publish_map_to_odom_tf_ = loadBool(pnh_, "runtime/publish_map_to_odom_tf", publish_map_to_odom_tf_);
    publish_odom_to_base_tf_ = loadBool(pnh_, "runtime/publish_odom_to_base_tf", publish_odom_to_base_tf_);
    publish_path_ = loadBool(pnh_, "runtime/publish_path", publish_path_);
    publish_gm_source_cloud_ = loadBool(pnh_, "global_matcher/publish_source_cloud_odom", publish_gm_source_cloud_);
    allow_initialpose_ = loadBool(pnh_, "initialization/allow_initialpose", allow_initialpose_);
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_slam_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    odom_base_ = poseFromRos(msg->pose.pose);
    latest_twist_ = msg->twist;
    have_slam_ = true;
    publishMatcherCloud(latest_slam_stamp_);
    publishOutput(latest_slam_stamp_);
  }

  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    if (!msg->success) {
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] ignore rejected global matcher result: %s",
                        msg->reject_reason.c_str());
      return;
    }
    if (!have_slam_) {
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] global matcher result arrived before slam odom");
      return;
    }

    map_odom_ = poseFromRos(msg->pose.pose);
    publishMapToOdom(msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
    ROS_INFO("[ysw_style_fusion] updated map->odom from global matcher map_id=%s child=%s fitness=%.4f inlier=%.3f",
             msg->map_id.c_str(), msg->child_frame_id.c_str(), msg->fitness_score, msg->inlier_ratio);
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!allow_initialpose_) {
      return;
    }
    const sof::Pose3d map_base = poseFromRos(msg->pose.pose);
    map_odom_ = have_slam_ ? sof::compose(map_base, sof::inverse(odom_base_)) : map_base;
    publishMapToOdom(msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
    ROS_INFO("[ysw_style_fusion] reset map->odom from initial pose");
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    latest_cloud_ = msg;
  }

  void onTimer(const ros::TimerEvent&) {
    if (!have_slam_) {
      publishIdentityMapOdom();
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] waiting for slam odom: %s", slam_odom_topic_.c_str());
      return;
    }
    const ros::Time stamp = latest_slam_stamp_.isZero() ? ros::Time::now() : latest_slam_stamp_;
    publishMatcherCloud(stamp);
    publishOutput(stamp);
  }

  void publishIdentityMapOdom() {
    if (!publish_tf_ || !publish_map_to_odom_tf_) {
      return;
    }
    tf_broadcaster_.sendTransform(toTf(sof::Pose3d(), map_frame_, odom_frame_, ros::Time::now()));
  }

  void publishMapToOdom(const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = odom_frame_;
    fillPose(map_odom_, &odom.pose.pose);
    map_to_odom_pub_.publish(odom);
  }

  void publishOutput(const ros::Time& stamp) {
    const sof::Pose3d map_base = sof::compose(map_odom_, odom_base_);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    fillPose(map_base, &odom.pose.pose);
    odom.twist = latest_twist_;
    fused_odom_pub_.publish(odom);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = odom.header;
    pose.pose.pose = odom.pose.pose;
    fused_pose_pub_.publish(pose);

    publishMapToOdom(stamp);

    if (publish_path_) {
      geometry_msgs::PoseStamped path_pose;
      path_pose.header = odom.header;
      path_pose.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(path_pose);
      while (static_cast<int>(path_.poses.size()) > max_path_size_) {
        path_.poses.erase(path_.poses.begin());
      }
      path_pub_.publish(path_);
    }

    if (publish_tf_) {
      std::vector<geometry_msgs::TransformStamped> transforms;
      if (publish_map_to_odom_tf_) {
        transforms.push_back(toTf(map_odom_, map_frame_, odom_frame_, stamp));
      }
      if (publish_odom_to_base_tf_) {
        transforms.push_back(toTf(odom_base_, odom_frame_, base_frame_, stamp));
      }
      if (!transforms.empty()) {
        tf_broadcaster_.sendTransform(transforms);
      }
    }
  }

  void publishMatcherCloud(const ros::Time& stamp) {
    if (!publish_gm_source_cloud_ || !latest_cloud_ || !have_slam_) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*latest_cloud_, in);
    if (in.empty()) {
      return;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(in, msg);
    msg.header.stamp = latest_cloud_->header.stamp.isZero() ? stamp : latest_cloud_->header.stamp;
    msg.header.frame_id = odom_frame_;
    gm_cloud_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  nav_msgs::Path path_;
  geometry_msgs::TwistWithCovariance latest_twist_;

  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string slam_odom_topic_ = "/localization/slam_odom";
  std::string global_matcher_result_topic_ = "/localization/global_matcher/result";
  std::string initial_pose_topic_ = "/initialpose";
  std::string source_cloud_topic_ = "/localization/cloud_registered";
  std::string fused_odom_topic_ = "/localization/ysw_style_odom";
  std::string map_to_odom_topic_ = "/localization/ysw_style_map_to_odom";
  std::string fused_pose_topic_ = "/localization/ysw_style_pose";
  std::string fused_path_topic_ = "/localization/ysw_style_path";
  std::string gm_source_cloud_odom_topic_ = "/localization/ysw_style/gm_source_cloud_odom";

  double output_rate_hz_ = 100.0;
  int max_path_size_ = 5000;
  bool publish_tf_ = true;
  bool publish_map_to_odom_tf_ = true;
  bool publish_odom_to_base_tf_ = true;
  bool publish_path_ = true;
  bool publish_gm_source_cloud_ = true;
  bool allow_initialpose_ = true;
  bool have_slam_ = false;
  ros::Time latest_slam_stamp_;
  sof::Pose3d odom_base_;
  sof::Pose3d map_odom_;

  ros::Subscriber slam_sub_;
  ros::Subscriber gm_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher map_to_odom_pub_;
  ros::Publisher fused_pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher gm_cloud_pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ysw_style_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  YswStyleFusionNode node(nh, pnh);
  ros::spin();
  return 0;
}
