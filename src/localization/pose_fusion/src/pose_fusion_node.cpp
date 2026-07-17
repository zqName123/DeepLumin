#include "pose_fusion/core/pose_fusion_core.hpp"
#include "pose_fusion/ros/param_loader.hpp"
#include "pose_fusion/ros/ros_adapter.hpp"
#include "pose_fusion/ros/tf_manager.hpp"

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/ObserverPolicy.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <algorithm>

class PoseFusionNode {
 public:
  PoseFusionNode() : nh_(), pnh_("~") {
    config_ = pose_fusion_ros::loadFusionConfig(pnh_);
    core_.initialize(config_);

    dr_sub_ = nh_.subscribe(config_.topics.dr_odom, 200, &PoseFusionNode::onDrOdom, this);
    slam_sub_ = nh_.subscribe(config_.topics.slam_odom, 50, &PoseFusionNode::onSlamOdom, this);
    global_match_sub_ = nh_.subscribe(config_.topics.global_matcher_result, 10,
                                      &PoseFusionNode::onGlobalMatch, this);
    relocalization_sub_ = nh_.subscribe(config_.topics.accepted_relocalization, 5,
                                        &PoseFusionNode::onRelocalization, this);
    policy_sub_ = nh_.subscribe(config_.topics.observer_policy, 5, &PoseFusionNode::onPolicy, this);
    initial_pose_sub_ = nh_.subscribe(config_.topics.initial_pose, 2, &PoseFusionNode::onInitialPose, this);

    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.fused_odom, 50);
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.topics.fused_pose, 50);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.fused_path, 5);
    status_pub_ = nh_.advertise<deeplumin_msgs::FusionStatus>(config_.topics.fusion_status, 10);

    path_.header.frame_id = config_.frames.map;
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.runtime.output_rate_hz)),
                            &PoseFusionNode::onTimer, this);

    ROS_INFO("[pose_fusion] started dr=%s slam=%s global_matcher=%s fused=%s map=%s odom=%s base=%s",
             config_.topics.dr_odom.c_str(), config_.topics.slam_odom.c_str(),
             config_.topics.global_matcher_result.c_str(), config_.topics.fused_odom.c_str(),
             config_.frames.map.c_str(), config_.frames.odom.c_str(), config_.frames.base_link.c_str());
  }

 private:
  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedDrOdom(pose_fusion_ros::fromOdom(*msg, pose_fusion::ObservationSource::DR));
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedSlamOdom(pose_fusion_ros::fromOdom(*msg, pose_fusion::ObservationSource::SLAM));
  }

  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    core_.feedGlobalMatch(pose_fusion_ros::fromGlobalMatch(*msg));
  }

  void onRelocalization(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg) {
    core_.feedRelocalizationReset(pose_fusion_ros::fromRelocalization(*msg));
  }

  void onPolicy(const deeplumin_msgs::ObserverPolicy::ConstPtr& msg) {
    core_.setObserverPolicy(pose_fusion_ros::fromObserverPolicy(*msg));
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.initialization.allow_initialpose) {
      ROS_WARN_THROTTLE(2.0, "[pose_fusion] ignore /initialpose because initialization/allow_initialpose=false");
      return;
    }
    core_.reset(pose_fusion_ros::fromInitialPose(*msg, config_.default_map_id));
  }

  void onTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const auto output = core_.tick(now.toSec());
    const auto status = core_.status(now.toSec());

    status_pub_.publish(pose_fusion_ros::toStatusMsg(status, config_.frames.map));
    if (!output.valid) {
      return;
    }

    fused_odom_pub_.publish(pose_fusion_ros::toOdomMsg(output, config_.frames.map, config_.frames.base_link));
    fused_pose_pub_.publish(pose_fusion_ros::toPoseMsg(output, config_.frames.map));
    tf_manager_.publish(output, config_, now);

    if (config_.runtime.publish_path) {
      path_.header.stamp = now;
      path_.poses.push_back(pose_fusion_ros::toPathPose(output, config_.frames.map));
      if (static_cast<int>(path_.poses.size()) > config_.runtime.max_path_size) {
        const int remove_count = static_cast<int>(path_.poses.size()) - config_.runtime.max_path_size;
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + remove_count);
      }
      path_pub_.publish(path_);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  pose_fusion::FusionConfig config_;
  pose_fusion::PoseFusionCore core_;
  pose_fusion_ros::TfManager tf_manager_;

  ros::Subscriber dr_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber global_match_sub_;
  ros::Subscriber relocalization_sub_;
  ros::Subscriber policy_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher fused_pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;
  nav_msgs::Path path_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_fusion");
  PoseFusionNode node;
  ros::spin();
  return 0;
}
