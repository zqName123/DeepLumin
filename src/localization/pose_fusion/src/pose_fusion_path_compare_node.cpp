#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <algorithm>
#include <string>

class PoseFusionPathCompareNode {
 public:
  PoseFusionPathCompareNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<int>("max_path_size", max_path_size_, 5000);
    pnh_.param<bool>("clear_on_time_jump", clear_on_time_jump_, true);
    pnh_.param<bool>("enable_dr", enable_dr_, true);
    pnh_.param<bool>("enable_slam", enable_slam_, true);
    pnh_.param<bool>("enable_fused", enable_fused_, true);

    pnh_.param<std::string>("topics/dr_odom", dr_odom_topic_, "/localization/dr_odom");
    pnh_.param<std::string>("topics/slam_odom", slam_odom_topic_, "/localization/slam_odom");
    pnh_.param<std::string>("topics/fused_odom", fused_odom_topic_, "/localization/fused_odom");
    pnh_.param<std::string>("topics/dr_path_map", dr_path_topic_, "/localization/compare/dr_path_map");
    pnh_.param<std::string>("topics/slam_path_map", slam_path_topic_, "/localization/compare/slam_path_map");
    pnh_.param<std::string>("topics/fused_path_map", fused_path_topic_, "/localization/compare/fused_path_map");

    if (enable_dr_) {
      dr_path_.header.frame_id = map_frame_;
      dr_path_pub_ = nh_.advertise<nav_msgs::Path>(dr_path_topic_, 5, true);
      dr_sub_ = nh_.subscribe(dr_odom_topic_, 100, &PoseFusionPathCompareNode::onDrOdom, this);
    }
    if (enable_slam_) {
      slam_path_.header.frame_id = map_frame_;
      slam_path_pub_ = nh_.advertise<nav_msgs::Path>(slam_path_topic_, 5, true);
      slam_sub_ = nh_.subscribe(slam_odom_topic_, 100, &PoseFusionPathCompareNode::onSlamOdom, this);
    }
    if (enable_fused_) {
      fused_path_.header.frame_id = map_frame_;
      fused_path_pub_ = nh_.advertise<nav_msgs::Path>(fused_path_topic_, 5, true);
      fused_sub_ = nh_.subscribe(fused_odom_topic_, 100, &PoseFusionPathCompareNode::onFusedOdom, this);
    }

    ROS_INFO("[pose_fusion_path_compare] map_frame=%s enable_dr=%s enable_slam=%s enable_fused=%s",
             map_frame_.c_str(), enable_dr_ ? "true" : "false", enable_slam_ ? "true" : "false",
             enable_fused_ ? "true" : "false");
    ROS_INFO("[pose_fusion_path_compare] identity map_odom assumption: odom pose values are republished as map-frame paths");
  }

 private:
  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    appendPose(*msg, &dr_path_, &last_dr_stamp_, dr_path_pub_);
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    appendPose(*msg, &slam_path_, &last_slam_stamp_, slam_path_pub_);
  }

  void onFusedOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    appendPose(*msg, &fused_path_, &last_fused_stamp_, fused_path_pub_);
  }

  void appendPose(const nav_msgs::Odometry& odom, nav_msgs::Path* path,
                  ros::Time* last_stamp, const ros::Publisher& pub) {
    const ros::Time stamp = odom.header.stamp.isZero() ? ros::Time::now() : odom.header.stamp;
    if (clear_on_time_jump_ && !last_stamp->isZero() && stamp < *last_stamp) {
      path->poses.clear();
    }
    *last_stamp = stamp;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose = odom.pose.pose;

    path->header.stamp = stamp;
    path->header.frame_id = map_frame_;
    path->poses.push_back(pose);
    if (static_cast<int>(path->poses.size()) > max_path_size_) {
      const int remove_count = static_cast<int>(path->poses.size()) - max_path_size_;
      path->poses.erase(path->poses.begin(), path->poses.begin() + remove_count);
    }
    pub.publish(*path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string map_frame_ = "map";
  int max_path_size_ = 5000;
  bool clear_on_time_jump_ = true;
  bool enable_dr_ = true;
  bool enable_slam_ = true;
  bool enable_fused_ = true;

  std::string dr_odom_topic_;
  std::string slam_odom_topic_;
  std::string fused_odom_topic_;
  std::string dr_path_topic_;
  std::string slam_path_topic_;
  std::string fused_path_topic_;

  ros::Subscriber dr_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber fused_sub_;
  ros::Publisher dr_path_pub_;
  ros::Publisher slam_path_pub_;
  ros::Publisher fused_path_pub_;
  nav_msgs::Path dr_path_;
  nav_msgs::Path slam_path_;
  nav_msgs::Path fused_path_;
  ros::Time last_dr_stamp_;
  ros::Time last_slam_stamp_;
  ros::Time last_fused_stamp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_fusion_path_compare");
  PoseFusionPathCompareNode node;
  ros::spin();
  return 0;
}
