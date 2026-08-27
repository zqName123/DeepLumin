/**
 * @file online_node.cpp
 * @brief ArUco 在线重定位节点
 */

#include <aruco_relocalization/detector/marker_detector.hpp>
#include <aruco_relocalization/map/marker_map_loader.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>
#include <aruco_relocalization/solver/relocalization_solver.hpp>
#include <aruco_relocalization/validator/relocalization_validator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <sstream>

class ArucoRelocNode {
 public:
  ArucoRelocNode() : pnh_("~"), tf_listener_(tf_buffer_) {
    config_ = aruco_reloc::loadOnlineNodeConfig(pnh_);

    // 加载 marker 地图
    if (!map_loader_.loadFromFile(config_.marker_map_path)) {
      throw std::runtime_error("failed to load marker map from " + config_.marker_map_path);
    }

    // 初始化外参
    T_base_cam_ = Eigen::Isometry3d::Identity();
    T_base_cam_.translation() = config_.extrinsic.translation;
    T_base_cam_.rotate(config_.extrinsic.rotation);

    // 订阅
    image_sub_ = nh_.subscribe(config_.image_topic, 5, &ArucoRelocNode::onImage, this);
    camera_info_sub_ = nh_.subscribe(config_.camera_info_topic, 1,
                                     &ArucoRelocNode::onCameraInfo, this);
    slam_odom_sub_ = nh_.subscribe(config_.slam_odom_topic, 5,
                                   &ArucoRelocNode::onSlamOdom, this);
    command_sub_ = nh_.subscribe(config_.command_topic, 5, &ArucoRelocNode::onCommand, this);

    // 发布
    result_pub_ = nh_.advertise<deeplumin_msgs::RelocalizationResult>(config_.result_topic, 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.pose_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(config_.status_topic, 1, true);

    ROS_INFO("[aruco_reloc] node started");
    ROS_INFO("[aruco_reloc] marker_map=%s (%zu markers)", config_.marker_map_path.c_str(),
             map_loader_.size());
    ROS_INFO("[aruco_reloc] result_topic=%s frame_id=%s child_frame_id=%s",
             config_.result_topic.c_str(), config_.frame_id.c_str(),
             config_.child_frame_id.c_str());
  }

 private:
  void onCommand(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "start") {
      enabled_.store(true);
      ROS_INFO("[aruco_reloc] relocalization enabled");
    } else if (msg->data == "stop") {
      enabled_.store(false);
      ROS_INFO("[aruco_reloc] relocalization disabled");
    }
  }

  void onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (camera_initialized_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        camera_matrix_.at<double>(i, j) = msg->K[i * 3 + j];
      }
    }

    if (msg->D.size() > 0) {
      dist_coeffs_ = cv::Mat(msg->D).reshape(1, 1);
    } else {
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    // 初始化各模块
    detector_.initialize(config_.detector, camera_matrix_, dist_coeffs_);
    solver_.initialize(config_.solver, camera_matrix_, dist_coeffs_, T_base_cam_);
    validator_.initialize(config_.validator, camera_matrix_, dist_coeffs_, T_base_cam_);

    camera_initialized_ = true;
    ROS_INFO("[aruco_reloc] camera initialized, ready for relocalization");
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_slam_pose_ = Eigen::Isometry3d::Identity();
    last_slam_pose_.translation() = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    if (std::abs(q.norm() - 1.0) < 0.1) {
      q.normalize();
      last_slam_pose_.rotate(q);
    }
    has_slam_pose_ = true;
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg) {
    if (!enabled_.load() || !camera_initialized_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // 1. 检测 marker
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[aruco_reloc] cv_bridge error: %s", e.what());
      return;
    }

    auto detection = detector_.detect(image, msg->header.stamp.toSec());
    if (detection.markers.empty()) {
      return;
    }

    // 2. 求解
    auto solve_result = solver_.solve(detection, map_loader_);
    if (!solve_result.success) {
      publishResult(msg->header.stamp, solve_result, "solve_failed");
      return;
    }

    // 3. 校验
    const Eigen::Isometry3d* last_pose_ptr = has_reloc_pose_ ? &last_reloc_pose_ : nullptr;
    auto validation = validator_.validate(solve_result, detection, map_loader_, last_pose_ptr);

    if (!validation.passed) {
      publishResult(msg->header.stamp, solve_result, validation.reject_reason);
      return;
    }

    // 4. 成功：更新位姿并发布
    last_reloc_pose_ = solve_result.T_map_base;
    has_reloc_pose_ = true;

    publishResult(msg->header.stamp, solve_result, "");
    publishPose(msg->header.stamp, solve_result.T_map_base);

    ROS_INFO("[aruco_reloc] relocalization success: %d markers, reproj=%.2f, time=%.1fms",
             solve_result.num_markers_used, validation.mean_reproj_error,
             solve_result.solve_time_ms);
  }

  void publishResult(const ros::Time& stamp, const aruco_reloc::SolveResult& solve_result,
                     const std::string& reject_reason) {
    deeplumin_msgs::RelocalizationResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.child_frame_id = config_.child_frame_id;

    // 设置位姿
    msg.pose.pose.position.x = solve_result.T_map_base.translation().x();
    msg.pose.pose.position.y = solve_result.T_map_base.translation().y();
    msg.pose.pose.position.z = solve_result.T_map_base.translation().z();
    Eigen::Quaterniond q(solve_result.T_map_base.rotation());
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();

    // 设置协方差（简化）
    for (auto& v : msg.pose.covariance) v = 0.0;
    msg.pose.covariance[0] = 0.01;   // x
    msg.pose.covariance[7] = 0.01;   // y
    msg.pose.covariance[14] = 0.01;  // z
    msg.pose.covariance[21] = 0.001; // roll
    msg.pose.covariance[28] = 0.001; // pitch
    msg.pose.covariance[35] = 0.001; // yaw

    msg.success = solve_result.success && reject_reason.empty();
    msg.accepted = false;  // 由 localization_v2 门控
    msg.method = "aruco";
    msg.reject_reason = reject_reason;
    msg.inlier_count = solve_result.num_markers_used;
    msg.elapsed_ms = solve_result.solve_time_ms;

    result_pub_.publish(msg);

    // 发布状态
    std_msgs::String status;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "success=" << msg.success
       << " markers=" << solve_result.num_markers_used
       << " time_ms=" << solve_result.solve_time_ms;
    if (!reject_reason.empty()) {
      ss << " reject=" << reject_reason;
    }
    status.data = ss.str();
    status_pub_.publish(status);
  }

  void publishPose(const ros::Time& stamp, const Eigen::Isometry3d& T_map_base) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.pose.position.x = T_map_base.translation().x();
    msg.pose.position.y = T_map_base.translation().y();
    msg.pose.position.z = T_map_base.translation().z();
    Eigen::Quaterniond q(T_map_base.rotation());
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    pose_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber slam_odom_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher result_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;

  aruco_reloc::OnlineNodeConfig config_;
  aruco_reloc::MarkerDetector detector_;
  aruco_reloc::MarkerMapLoader map_loader_;
  aruco_reloc::RelocalizationSolver solver_;
  aruco_reloc::RelocalizationValidator validator_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  Eigen::Isometry3d T_base_cam_ = Eigen::Isometry3d::Identity();

  mutable std::mutex mutex_;
  Eigen::Isometry3d last_slam_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_reloc_pose_ = Eigen::Isometry3d::Identity();
  bool has_slam_pose_ = false;
  bool has_reloc_pose_ = false;
  bool camera_initialized_ = false;
  std::atomic<bool> enabled_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_reloc_online_node");

  try {
    ArucoRelocNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[aruco_reloc] exception: " << e.what());
    return 1;
  }

  return 0;
}
