#include "simple_odom_fusion/core/simple_fusion_core.hpp"

#include <deque>
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

class SimpleOdomFusionNode {
 public:
  SimpleOdomFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadConfig();
    core_.initialize(config_);

    dr_sub_ = nh_.subscribe(config_.topics.dr_odom, 200, &SimpleOdomFusionNode::onDrOdom, this);
    slam_sub_ = nh_.subscribe(config_.topics.slam_odom, 100, &SimpleOdomFusionNode::onSlamOdom, this);
    gm_sub_ = nh_.subscribe(config_.topics.global_matcher_result, 20, &SimpleOdomFusionNode::onGlobalMatch, this);
    initial_pose_sub_ = nh_.subscribe(config_.topics.initial_pose, 5, &SimpleOdomFusionNode::onInitialPose, this);
    cloud_sub_ = nh_.subscribe(config_.topics.source_cloud, 5, &SimpleOdomFusionNode::onSourceCloud, this);

    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.fused_odom, 50);
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.topics.fused_pose, 20);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.fused_path, 5);
    gm_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topics.gm_source_cloud_odom, 2);
    path_.header.frame_id = config_.frames.map;

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.output_rate_hz)),
                             &SimpleOdomFusionNode::onTimer, this);

    ROS_INFO("[simple_odom_fusion] dr=%s slam=%s gm=%s initial=%s out=%s gm_cloud=%s",
             config_.topics.dr_odom.c_str(), config_.topics.slam_odom.c_str(),
             config_.topics.global_matcher_result.c_str(), config_.topics.initial_pose.c_str(),
             config_.topics.fused_odom.c_str(), config_.topics.gm_source_cloud_odom.c_str());
  }

 private:
  void loadConfig() {
    pnh_.param<std::string>("frames/map", config_.frames.map, config_.frames.map);
    pnh_.param<std::string>("frames/odom", config_.frames.odom, config_.frames.odom);
    pnh_.param<std::string>("frames/base_link", config_.frames.base_link, config_.frames.base_link);
    pnh_.param<std::string>("frames/lidar", config_.frames.lidar, config_.frames.lidar);

    pnh_.param<std::string>("topics/dr_odom", config_.topics.dr_odom, config_.topics.dr_odom);
    pnh_.param<std::string>("topics/slam_odom", config_.topics.slam_odom, config_.topics.slam_odom);
    pnh_.param<std::string>("topics/global_matcher_result", config_.topics.global_matcher_result,
                            config_.topics.global_matcher_result);
    pnh_.param<std::string>("topics/initial_pose", config_.topics.initial_pose, config_.topics.initial_pose);
    pnh_.param<std::string>("topics/source_cloud", config_.topics.source_cloud, config_.topics.source_cloud);
    pnh_.param<std::string>("topics/fused_odom", config_.topics.fused_odom, config_.topics.fused_odom);
    pnh_.param<std::string>("topics/fused_pose", config_.topics.fused_pose, config_.topics.fused_pose);
    pnh_.param<std::string>("topics/fused_path", config_.topics.fused_path, config_.topics.fused_path);
    pnh_.param<std::string>("topics/gm_source_cloud_odom", config_.topics.gm_source_cloud_odom,
                            config_.topics.gm_source_cloud_odom);

    pnh_.param<double>("runtime/output_rate_hz", config_.output_rate_hz, config_.output_rate_hz);
    pnh_.param<int>("runtime/max_path_size", config_.max_path_size, config_.max_path_size);
    config_.publish_tf = loadBool(pnh_, "runtime/publish_tf", config_.publish_tf);
    config_.publish_map_to_odom_tf = loadBool(pnh_, "runtime/publish_map_to_odom_tf", config_.publish_map_to_odom_tf);
    config_.publish_odom_to_base_tf = loadBool(pnh_, "runtime/publish_odom_to_base_tf", config_.publish_odom_to_base_tf);
    config_.publish_path = loadBool(pnh_, "runtime/publish_path", config_.publish_path);

    int max_queue_size = static_cast<int>(config_.max_queue_size);
    pnh_.param<int>("sync/max_queue_size", max_queue_size, max_queue_size);
    config_.max_queue_size = static_cast<std::size_t>(std::max(10, max_queue_size));
    pnh_.param<double>("sync/tolerance", config_.sync_tolerance, config_.sync_tolerance);
    pnh_.param<double>("sync/max_pair_dt", config_.max_pair_dt, config_.max_pair_dt);

    pnh_.param<double>("fusion/max_delta_translation", config_.max_delta_translation, config_.max_delta_translation);
    pnh_.param<double>("fusion/max_delta_rotation_rad", config_.max_delta_rotation_rad, config_.max_delta_rotation_rad);
    pnh_.param<double>("fusion/dr_prediction_weight", config_.dr_prediction_weight, config_.dr_prediction_weight);
    pnh_.param<double>("fusion/slam_correction_gain", config_.slam_correction_gain, config_.slam_correction_gain);
    config_.use_slam_increment = loadBool(pnh_, "fusion/use_slam_increment", config_.use_slam_increment);

    config_.allow_initialpose = loadBool(pnh_, "initialization/allow_initialpose", config_.allow_initialpose);
    config_.allow_global_matcher_initialization =
        loadBool(pnh_, "initialization/allow_global_matcher", config_.allow_global_matcher_initialization);
    config_.allow_local_auto_initialization =
        loadBool(pnh_, "initialization/allow_local_auto", config_.allow_local_auto_initialization);

    config_.enable_global_matcher_correction =
        loadBool(pnh_, "global_matcher/enable_correction", config_.enable_global_matcher_correction);
    pnh_.param<double>("global_matcher/correction_gain", config_.global_matcher_correction_gain,
                       config_.global_matcher_correction_gain);
    pnh_.param<double>("global_matcher/min_interval", config_.global_matcher_min_interval,
                       config_.global_matcher_min_interval);
    config_.publish_gm_source_cloud =
        loadBool(pnh_, "global_matcher/publish_source_cloud_odom", config_.publish_gm_source_cloud);
    config_.gm_cloud_use_fused_pose =
        loadBool(pnh_, "global_matcher/cloud_use_fused_pose", config_.gm_cloud_use_fused_pose);
  }

  sof::OdomSample sampleFromOdom(const nav_msgs::Odometry& msg) const {
    sof::OdomSample sample;
    sample.stamp = msg.header.stamp.isZero() ? ros::Time::now().toSec() : msg.header.stamp.toSec();
    sample.pose = poseFromRos(msg.pose.pose);
    sample.linear_velocity = sof::Vec3d(msg.twist.twist.linear.x,
                                        msg.twist.twist.linear.y,
                                        msg.twist.twist.linear.z);
    sample.frame_id = msg.header.frame_id;
    sample.child_frame_id = msg.child_frame_id;
    return sample;
  }

  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedDr(sampleFromOdom(*msg));
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedSlam(sampleFromOdom(*msg));
  }

  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    sof::GlobalObservation obs;
    obs.stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    obs.map_base = poseFromRos(msg->pose.pose);
    obs.success = msg->success;
    if (!core_.initialized() && config_.allow_global_matcher_initialization) {
      core_.resetWithMapBase(obs);
      return;
    }
    core_.correctWithGlobalMatcher(obs);
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.allow_initialpose) {
      return;
    }
    sof::GlobalObservation obs;
    obs.stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    obs.map_base = poseFromRos(msg->pose.pose);
    obs.success = true;
    core_.resetWithMapBase(obs);
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    latest_cloud_ = msg;
  }

  void onTimer(const ros::TimerEvent&) {
    auto maybe_output = core_.update();
    const sof::FusionOutput output = maybe_output ? *maybe_output : core_.output(ros::Time::now().toSec());
    if (!output.valid) {
      publishIdentityMapOdom();
      publishBootstrapMatcherCloud();
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] waiting for synchronized dr/slam odometry; publishing identity map->odom for RViz");
      return;
    }
    publishOutput(output);
    publishMatcherCloud(output);
  }


  void publishIdentityMapOdom() {
    if (!config_.publish_tf || !config_.publish_map_to_odom_tf) {
      return;
    }
    sof::Pose3d identity;
    tf_broadcaster_.sendTransform(toTf(identity, config_.frames.map, config_.frames.odom, ros::Time::now()));
  }


  void publishBootstrapMatcherCloud() {
    if (!config_.publish_gm_source_cloud || !latest_cloud_) {
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] waiting for source cloud topic: %s", config_.topics.source_cloud.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*latest_cloud_, cloud);
    if (cloud.empty()) {
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] latest source cloud is empty");
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = latest_cloud_->header.stamp.isZero() ? ros::Time::now() : latest_cloud_->header.stamp;
    msg.header.frame_id = config_.frames.odom;
    gm_cloud_pub_.publish(msg);
  }

  void publishOutput(const sof::FusionOutput& output) {
    const ros::Time stamp(output.stamp);
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = config_.frames.map;
    odom.child_frame_id = config_.frames.base_link;
    fillPose(output.map_base, &odom.pose.pose);
    odom.twist.twist.linear.x = output.velocity_map.x();
    odom.twist.twist.linear.y = output.velocity_map.y();
    odom.twist.twist.linear.z = output.velocity_map.z();
    fused_odom_pub_.publish(odom);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = odom.header;
    pose.pose.pose = odom.pose.pose;
    fused_pose_pub_.publish(pose);

    if (config_.publish_path) {
      geometry_msgs::PoseStamped path_pose;
      path_pose.header = odom.header;
      path_pose.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(path_pose);
      while (static_cast<int>(path_.poses.size()) > config_.max_path_size) {
        path_.poses.erase(path_.poses.begin());
      }
      path_pub_.publish(path_);
    }

    if (config_.publish_tf) {
      std::vector<geometry_msgs::TransformStamped> transforms;
      if (config_.publish_map_to_odom_tf) {
        transforms.push_back(toTf(output.map_odom, config_.frames.map, config_.frames.odom, stamp));
      }
      if (config_.publish_odom_to_base_tf) {
        transforms.push_back(toTf(output.odom_base, config_.frames.odom, config_.frames.base_link, stamp));
      }
      if (!transforms.empty()) {
        tf_broadcaster_.sendTransform(transforms);
      }
    }
  }

  void publishMatcherCloud(const sof::FusionOutput& output) {
    if (!config_.publish_gm_source_cloud || !latest_cloud_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*latest_cloud_, in);
    if (in.empty()) {
      return;
    }

    const sof::Pose3d transform_pose = config_.gm_cloud_use_fused_pose ? output.odom_base : output.odom_base;
    pcl::PointCloud<pcl::PointXYZI> out;
    pcl::transformPointCloud(in, out, toMatrix4f(transform_pose));

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header.stamp = latest_cloud_->header.stamp.isZero() ? ros::Time(output.stamp) : latest_cloud_->header.stamp;
    msg.header.frame_id = config_.frames.odom;
    gm_cloud_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  sof::FusionConfig config_;
  sof::SimpleFusionCore core_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  nav_msgs::Path path_;

  ros::Subscriber dr_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber gm_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher fused_pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher gm_cloud_pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_odom_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SimpleOdomFusionNode node(nh, pnh);
  ros::spin();
  return 0;
}
