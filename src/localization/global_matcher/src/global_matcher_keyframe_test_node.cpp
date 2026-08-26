#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct TumPose {
  double stamp = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

bool loadTumPoses(const std::string& path, std::vector<TumPose>* poses) {
  poses->clear();
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    ROS_ERROR("[global_matcher_keyframe_test] failed to open pose file: %s", path.c_str());
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    TumPose pose;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(iss >> pose.stamp >> pose.p.x() >> pose.p.y() >> pose.p.z() >> qx >> qy >> qz >> qw)) {
      continue;
    }
    pose.q = Eigen::Quaterniond(qw, qx, qy, qz);
    if (pose.q.norm() < 1e-9) {
      pose.q = Eigen::Quaterniond::Identity();
    } else {
      pose.q.normalize();
    }
    poses->push_back(pose);
  }
  ROS_INFO("[global_matcher_keyframe_test] loaded %zu TUM poses from %s", poses->size(), path.c_str());
  return !poses->empty();
}

geometry_msgs::Pose toRosPose(const TumPose& tum,
                              double xyz_noise,
                              double yaw_noise_deg) {
  TumPose noisy = tum;
  noisy.p.x() += xyz_noise;
  noisy.p.y() += xyz_noise;

  const double yaw = yaw_noise_deg * M_PI / 180.0;
  if (std::abs(yaw) > 1e-12) {
    noisy.q = noisy.q * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    noisy.q.normalize();
  }

  geometry_msgs::Pose pose;
  pose.position.x = noisy.p.x();
  pose.position.y = noisy.p.y();
  pose.position.z = noisy.p.z();
  pose.orientation.x = noisy.q.x();
  pose.orientation.y = noisy.q.y();
  pose.orientation.z = noisy.q.z();
  pose.orientation.w = noisy.q.w();
  return pose;
}

}  // namespace

class GlobalMatcherKeyframeTestNode {
 public:
  GlobalMatcherKeyframeTestNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("keyframe_dir", keyframe_dir_, "/home/hhy/2004ros/relocalization_ws/key_point_frame");
    pnh_.param<std::string>("pose_file", pose_file_, "/home/hhy/2004ros/relocalization_ws/optimized_poses_tum.txt");
    pnh_.param<int>("frame_index", frame_index_, 100);
    pnh_.param<double>("publish_rate", publish_rate_, 1.0);
    pnh_.param<double>("initial_xyz_noise", initial_xyz_noise_, 0.0);
    pnh_.param<double>("initial_yaw_noise_deg", initial_yaw_noise_deg_, 0.0);
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("source_frame", source_frame_, "base_link");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/localization/cloud_registered");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/localization/fused_odom");
    pnh_.param<std::string>("source_odom_topic", source_odom_topic_, "/localization/slam_odom");
    pnh_.param<std::string>("truth_pose_topic", truth_pose_topic_, "/localization/global_matcher/test_truth_pose");
    pnh_.param<std::string>("frame_index_topic", frame_index_topic_, "/localization/global_matcher/test_frame_index");

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 2, true);
    initial_pose_pub_ = nh_.advertise<nav_msgs::Odometry>(initial_pose_topic_, 2, true);
    source_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(source_odom_topic_, 2, true);
    truth_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(truth_pose_topic_, 2, true);
    frame_index_pub_ = nh_.advertise<std_msgs::Int32>(frame_index_topic_, 2, true);

    if (!loadTumPoses(pose_file_, &poses_)) {
      ros::shutdown();
      return;
    }
    if (!loadFrame(frame_index_)) {
      ros::shutdown();
      return;
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, publish_rate_)),
                            &GlobalMatcherKeyframeTestNode::onTimer, this);
    publishOnce();
  }

 private:
  bool loadFrame(int index) {
    if (index < 0 || static_cast<size_t>(index) >= poses_.size()) {
      ROS_ERROR("[global_matcher_keyframe_test] frame_index=%d outside pose range [0,%zu)", index, poses_.size());
      return false;
    }

    const std::string pcd_path = keyframe_dir_ + "/" + std::to_string(index) + ".pcd";
    CloudT::Ptr cloud(new CloudT());
    if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud) != 0 || cloud->empty()) {
      ROS_ERROR("[global_matcher_keyframe_test] failed to load keyframe cloud: %s", pcd_path.c_str());
      return false;
    }
    std::vector<int> indices;
    cloud_.reset(new CloudT());
    pcl::removeNaNFromPointCloud(*cloud, *cloud_, indices);
    pose_ = poses_[index];
    frame_index_ = index;

    ROS_INFO("[global_matcher_keyframe_test] loaded frame=%d cloud=%s points=%zu pose=(%.3f %.3f %.3f)",
             frame_index_, pcd_path.c_str(), cloud_->size(), pose_.p.x(), pose_.p.y(), pose_.p.z());
    return !cloud_->empty();
  }

  void onTimer(const ros::TimerEvent&) {
    publishOnce();
  }

  void publishOnce() {
    if (!cloud_ || cloud_->empty()) {
      return;
    }
    const ros::Time now = ros::Time::now();

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_, cloud_msg);
    cloud_msg.header.stamp = now;
    cloud_msg.header.frame_id = source_frame_;
    cloud_pub_.publish(cloud_msg);

    nav_msgs::Odometry initial;
    initial.header.stamp = now;
    initial.header.frame_id = map_frame_;
    initial.child_frame_id = base_frame_;
    initial.pose.pose = toRosPose(pose_, initial_xyz_noise_, initial_yaw_noise_deg_);
    initial.pose.covariance[0] = 0.25;
    initial.pose.covariance[7] = 0.25;
    initial.pose.covariance[14] = 0.25;
    initial.pose.covariance[35] = 0.03;
    initial_pose_pub_.publish(initial);

    nav_msgs::Odometry source_odom;
    source_odom.header.stamp = now;
    source_odom.header.frame_id = "odom";
    source_odom.child_frame_id = base_frame_;
    source_odom.pose.pose.orientation.w = 1.0;
    source_odom_pub_.publish(source_odom);

    geometry_msgs::PoseStamped truth;
    truth.header.stamp = now;
    truth.header.frame_id = map_frame_;
    truth.pose = toRosPose(pose_, 0.0, 0.0);
    truth_pose_pub_.publish(truth);

    std_msgs::Int32 idx;
    idx.data = frame_index_;
    frame_index_pub_.publish(idx);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cloud_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Publisher source_odom_pub_;
  ros::Publisher truth_pose_pub_;
  ros::Publisher frame_index_pub_;
  ros::Timer timer_;

  std::string keyframe_dir_;
  std::string pose_file_;
  std::string map_frame_;
  std::string base_frame_;
  std::string source_frame_;
  std::string cloud_topic_;
  std::string initial_pose_topic_;
  std::string source_odom_topic_;
  std::string truth_pose_topic_;
  std::string frame_index_topic_;
  int frame_index_ = 100;
  double publish_rate_ = 1.0;
  double initial_xyz_noise_ = 0.0;
  double initial_yaw_noise_deg_ = 0.0;
  std::vector<TumPose> poses_;
  TumPose pose_;
  CloudT::Ptr cloud_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_matcher_keyframe_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GlobalMatcherKeyframeTestNode node(nh, pnh);
  ros::spin();
  return 0;
}
