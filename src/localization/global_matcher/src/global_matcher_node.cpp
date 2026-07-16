#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <XmlRpcValue.h>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/LocalizationStatus.h>

namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

constexpr double kPi = 3.14159265358979323846;

struct MatchMetrics {
  uint32_t total_points = 0;
  uint32_t inlier_count = 0;
  double inlier_ratio = 0.0;
  double inlier_rmse = std::numeric_limits<double>::infinity();
};

struct MapSegment {
  std::string id;
  std::string path;
  CloudPtr full_map;
  CloudPtr viz_map;
};

Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                       static_cast<float>(pose.orientation.x),
                       static_cast<float>(pose.orientation.y),
                       static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }

  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  mat(0, 3) = static_cast<float>(pose.position.x);
  mat(1, 3) = static_cast<float>(pose.position.y);
  mat(2, 3) = static_cast<float>(pose.position.z);
  return mat;
}

geometry_msgs::Pose matrixToPose(const Eigen::Matrix4f& mat) {
  geometry_msgs::Pose pose;
  Eigen::Matrix3f rot = mat.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);
  q.normalize();
  pose.position.x = mat(0, 3);
  pose.position.y = mat(1, 3);
  pose.position.z = mat(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

double yawFromMatrix(const Eigen::Matrix4f& mat) {
  return std::atan2(mat(1, 0), mat(0, 0));
}

double normalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double yawDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return normalizeAngle(yawFromMatrix(a) - yawFromMatrix(b));
}

double translationDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Vector3f da = a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3);
  return da.norm();
}

CloudPtr downsample(const CloudConstPtr& input, double leaf) {
  CloudPtr output(new CloudT());
  if (!input || input->empty()) {
    return output;
  }
  if (leaf <= 1e-4) {
    *output = *input;
    return output;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}


bool loadVector3(ros::NodeHandle& nh, const std::string& key, Eigen::Vector3f* value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 3) {
    return false;
  }
  *value = Eigen::Vector3f(static_cast<float>(flat[0]),
                          static_cast<float>(flat[1]),
                          static_cast<float>(flat[2]));
  return true;
}

bool loadMatrix3(ros::NodeHandle& nh, const std::string& key, Eigen::Matrix3f* value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 9) {
    return false;
  }
  *value << static_cast<float>(flat[0]), static_cast<float>(flat[1]), static_cast<float>(flat[2]),
            static_cast<float>(flat[3]), static_cast<float>(flat[4]), static_cast<float>(flat[5]),
            static_cast<float>(flat[6]), static_cast<float>(flat[7]), static_cast<float>(flat[8]);
  return true;
}

Eigen::Matrix4f loadExtrinsic(ros::NodeHandle& nh, const std::string& key) {
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  loadVector3(nh, key + "/translation", &translation);
  loadMatrix3(nh, key + "/rotation", &rotation);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

CloudPtr removeInvalid(const CloudConstPtr& input) {
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *output, indices);
  return output;
}

}  // namespace

class GlobalMatcherNode {
 public:
  GlobalMatcherNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    loadParams();

    result_pub_ = nh_.advertise<deeplumin_msgs::GlobalMatchResult>(result_topic_, 10);
    status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(status_topic_, 10);
    source_initial_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(source_initial_scan_topic_, 2);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(aligned_scan_topic_, 2);
    submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(target_submap_topic_, 2);
    initial_guess_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(initial_guess_topic_, 2);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 2);
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 1, true);

    source_sub_ = nh_.subscribe(source_cloud_topic_, 2, &GlobalMatcherNode::onSourceCloud, this);
    initial_odom_sub_ = nh_.subscribe(initial_pose_topic_, 5, &GlobalMatcherNode::onInitialOdom, this);
    source_odom_sub_ = nh_.subscribe(source_odom_topic_, 20, &GlobalMatcherNode::onSourceOdom, this);
    initial_pose_sub_ = nh_.subscribe(manual_initial_pose_topic_, 2, &GlobalMatcherNode::onManualInitialPose, this);
    load_map_command_sub_ = nh_.subscribe(load_map_command_topic_, 2, &GlobalMatcherNode::onLoadMapCommand, this);
    set_map_segment_command_sub_ = nh_.subscribe(set_map_segment_command_topic_, 2, &GlobalMatcherNode::onSetMapSegmentCommand, this);

    if (!loadMap()) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0, "failed to load map");
    } else {
      publishGlobalMap();
    }

    match_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, match_rate_)),
                                  &GlobalMatcherNode::onMatchTimer, this);
    map_timer_ = nh_.createTimer(ros::Duration(std::max(0.5, global_map_publish_period_)),
                                &GlobalMatcherNode::onMapTimer, this);
  }

 private:
  void loadParams() {
    pnh_.param<std::string>("frames/map", map_frame_, "map");
    pnh_.param<std::string>("frames/odom", odom_frame_, "odom");
    pnh_.param<std::string>("frames/base_link", base_frame_, "base_link");
    pnh_.param<std::string>("frames/lidar", lidar_frame_, "lidar_link");
    base_to_lidar_ = loadExtrinsic(pnh_, "extrinsics/base_to_lidar");

    pnh_.param<std::string>("topics/source_cloud", source_cloud_topic_, "/localization/cloud_registered");
    pnh_.param<std::string>("topics/initial_pose", initial_pose_topic_, "/localization/fused_odom");
    pnh_.param<std::string>("topics/source_odom", source_odom_topic_, "/localization/slam_odom");
    pnh_.param<std::string>("topics/manual_initial_pose", manual_initial_pose_topic_, "/initialpose");
    pnh_.param<std::string>("topics/result", result_topic_, "/localization/global_matcher/result");
    pnh_.param<std::string>("topics/status", status_topic_, "/localization/global_matcher/status");
    pnh_.param<std::string>("topics/source_initial_scan", source_initial_scan_topic_, "/localization/global_matcher/source_scan_initial");
    pnh_.param<std::string>("topics/aligned_scan", aligned_scan_topic_, "/localization/global_matcher/aligned_scan");
    pnh_.param<std::string>("topics/target_submap", target_submap_topic_, "/localization/global_matcher/target_submap");
    pnh_.param<std::string>("topics/global_map", global_map_topic_, "/localization/global_matcher/global_map");
    pnh_.param<std::string>("topics/initial_guess", initial_guess_topic_, "/localization/global_matcher/initial_guess");
    pnh_.param<std::string>("topics/path", path_topic_, "/localization/global_matcher/path");
    pnh_.param<std::string>("topics/load_map_command", load_map_command_topic_, "/localization/global_matcher/load_map_command");
    pnh_.param<std::string>("topics/set_map_segment_command", set_map_segment_command_topic_, "/localization/global_matcher/set_map_segment_command");

    pnh_.param<std::string>("map/default_map_id", default_map_id_, "default");
    pnh_.param<std::string>("map/pcd_path", pcd_path_, "/home/hhy/2004ros/ysw_loc/src/map/0730map/yuhsuwan2_loc_rot_72.pcd");
    pnh_.param<double>("map/publish_voxel_leaf", global_map_publish_voxel_leaf_, 5.0);
    pnh_.param<double>("map/global_publish_period", global_map_publish_period_, 5.0);

    pnh_.param<bool>("runtime/auto_match", auto_match_, true);
    pnh_.param<bool>("runtime/viz_test_mode", viz_test_mode_, false);
    pnh_.param<bool>("runtime/source_cloud_is_odom_frame", source_cloud_is_odom_frame_, true);
    pnh_.param<std::string>("runtime/source_cloud_frame", source_cloud_frame_, "auto");
    pnh_.param<double>("runtime/match_rate", match_rate_, 1.0);
    pnh_.param<double>("runtime/max_cloud_age", max_cloud_age_, 1.0);
    pnh_.param<int>("runtime/max_path_size", max_path_size_, 3000);

    pnh_.param<double>("crop/forward", crop_forward_, 70.0);
    pnh_.param<double>("crop/backward", crop_backward_, 7.0);
    pnh_.param<double>("crop/left", crop_left_, 35.0);
    pnh_.param<double>("crop/right", crop_right_, 35.0);
    pnh_.param<double>("crop/down", crop_down_, 10.0);
    pnh_.param<double>("crop/up", crop_up_, 10.0);

    pnh_.param<std::string>("matcher/type", matcher_type_, "fast_gicp");
    pnh_.param<double>("matcher/source_voxel_leaf", source_voxel_leaf_, 0.1);
    pnh_.param<double>("matcher/target_voxel_leaf", target_voxel_leaf_, 0.1);
    pnh_.param<double>("matcher/coarse_scale", coarse_scale_, 6.0);
    pnh_.param<double>("matcher/fine_scale", fine_scale_, 3.0);
    pnh_.param<int>("matcher/max_iterations", max_iterations_, 20);
    pnh_.param<double>("matcher/transformation_epsilon", transformation_epsilon_, 0.01);
    pnh_.param<double>("matcher/max_correspondence_distance", max_correspondence_distance_, 2.0);
    pnh_.param<int>("matcher/fast_gicp_num_threads", fast_gicp_num_threads_, 2);
    pnh_.param<int>("matcher/fast_gicp_correspondence_randomness", fast_gicp_correspondence_randomness_, 20);

    pnh_.param<int>("gate/min_source_points", min_source_points_, 200);
    pnh_.param<int>("gate/min_target_points", min_target_points_, 1000);
    pnh_.param<double>("gate/max_fitness_score", max_fitness_score_, 1.0);
    pnh_.param<double>("gate/min_inlier_ratio", min_inlier_ratio_, 0.5);
    pnh_.param<double>("gate/max_inlier_rmse", max_inlier_rmse_, 0.8);
    pnh_.param<double>("gate/inlier_distance", inlier_distance_, 0.5);
    pnh_.param<double>("gate/max_delta_translation", max_delta_translation_, 5.0);
    pnh_.param<double>("gate/max_delta_yaw_deg", max_delta_yaw_deg_, 20.0);

#ifdef GLOBAL_MATCHER_VIZ_TEST
    viz_test_mode_ = true;
#endif
    if (viz_test_mode_) {
      auto_match_ = true;
    }
  }

  bool loadMap() {
    if (!loadMapSegment(default_map_id_, pcd_path_, true)) {
      return false;
    }
    loadConfiguredMapSegments();
    return true;
  }

  bool loadMapSegment(const std::string& map_id, const std::string& pcd_path, bool activate) {
    if (map_id.empty() || pcd_path.empty()) {
      ROS_ERROR("[global_matcher] map_id/path is empty, map_id='%s' path='%s'", map_id.c_str(), pcd_path.c_str());
      return false;
    }

    CloudPtr raw(new CloudT());
    const int ret = pcl::io::loadPCDFile<PointT>(pcd_path, *raw);
    if (ret != 0 || raw->empty()) {
      ROS_ERROR("[global_matcher] failed to load pcd map_id=%s path=%s", map_id.c_str(), pcd_path.c_str());
      return false;
    }
    raw = removeInvalid(raw);
    if (!raw || raw->empty()) {
      ROS_ERROR("[global_matcher] pcd map is empty after invalid-point filtering, map_id=%s path=%s",
                map_id.c_str(), pcd_path.c_str());
      return false;
    }

    MapSegment segment;
    segment.id = map_id;
    segment.path = pcd_path;
    segment.full_map = raw;
    segment.viz_map = downsample(segment.full_map, global_map_publish_voxel_leaf_);
    if (!segment.viz_map || segment.viz_map->empty()) {
      segment.viz_map = segment.full_map;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_segments_[map_id] = segment;
      if (activate) {
        activateSegmentLocked(map_id);
      }
    }

    ROS_INFO("[global_matcher] loaded map segment id=%s path=%s points=%zu viz_points=%zu activate=%d",
             map_id.c_str(), pcd_path.c_str(), segment.full_map->size(), segment.viz_map->size(), activate);
    if (activate) {
      publishGlobalMap();
    }
    return true;
  }

  void loadConfiguredMapSegments() {
    XmlRpc::XmlRpcValue segments;
    if (!pnh_.getParam("map/segments", segments)) {
      return;
    }
    if (segments.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[global_matcher] map/segments must be a list of {id, path}; ignore it");
      return;
    }

    for (int i = 0; i < segments.size(); ++i) {
      if (segments[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !segments[i].hasMember("id") || !segments[i].hasMember("path")) {
        ROS_WARN("[global_matcher] invalid map/segments[%d], require id and path", i);
        continue;
      }
      const std::string id = static_cast<std::string>(segments[i]["id"]);
      const std::string path = static_cast<std::string>(segments[i]["path"]);
      if (id == default_map_id_ && path == pcd_path_) {
        continue;
      }
      loadMapSegment(id, path, false);
    }
  }

  bool activateSegmentLocked(const std::string& map_id) {
    auto it = map_segments_.find(map_id);
    if (it == map_segments_.end() || !it->second.full_map || it->second.full_map->empty()) {
      return false;
    }
    active_map_id_ = map_id;
    global_map_ = it->second.full_map;
    global_map_viz_ = it->second.viz_map;
    path_.poses.clear();
    return true;
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    CloudPtr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);
    cloud = removeInvalid(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_ = cloud;
    latest_source_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    latest_source_frame_ = msg->header.frame_id;
  }

  void onInitialOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_initial_map_base_ = poseToMatrix(msg->pose.pose);
    latest_initial_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_initial_map_base_ = true;
  }

  void onSourceOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_base_ = poseToMatrix(msg->pose.pose);
    latest_source_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_source_odom_ = true;
  }

  void onManualInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_initial_map_base_ = poseToMatrix(msg->pose.pose);
    latest_initial_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_initial_map_base_ = true;
    ROS_INFO("[global_matcher] received manual initial pose from %s", manual_initial_pose_topic_.c_str());
  }

  void onLoadMapCommand(const std_msgs::String::ConstPtr& msg) {
    std::vector<std::string> fields;
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, '|')) {
      fields.push_back(item);
    }
    if (fields.size() < 2) {
      fields.clear();
      std::stringstream csv(msg->data);
      while (std::getline(csv, item, ',')) {
        fields.push_back(item);
      }
    }
    if (fields.size() < 2) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "load_map_command format: map_id|pcd_path[|activate]");
      return;
    }

    bool activate = true;
    if (fields.size() >= 3) {
      const std::string flag = fields[2];
      activate = !(flag == "0" || flag == "false" || flag == "False" || flag == "no");
    }
    if (!loadMapSegment(fields[0], fields[1], activate)) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0, "failed to load map segment");
      return;
    }
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL, 1.0,
                  activate ? "loaded and activated map segment" : "loaded map segment");
  }

  void onSetMapSegmentCommand(const std_msgs::String::ConstPtr& msg) {
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ok = activateSegmentLocked(msg->data);
    }
    if (!ok) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "requested map segment is not loaded: " + msg->data);
      return;
    }
    ROS_INFO("[global_matcher] activated map segment id=%s", msg->data.c_str());
    publishGlobalMap();
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL, 1.0, "activated map segment");
  }

  void onMatchTimer(const ros::TimerEvent&) {
    if (!auto_match_) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0, "auto_match disabled");
      return;
    }
    runMatch();
  }

  void onMapTimer(const ros::TimerEvent&) {
    if (viz_test_mode_) {
      publishGlobalMap();
    }
  }

  void runMatch() {
    CloudPtr source;
    ros::Time source_stamp;
    Eigen::Matrix4f initial_map_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f source_base = Eigen::Matrix4f::Identity();
    bool have_initial = false;
    bool have_source_odom = false;
    std::string source_frame;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      source = latest_source_;
      source_stamp = latest_source_stamp_;
      initial_map_base = latest_initial_map_base_;
      source_base = latest_source_base_;
      have_initial = have_initial_map_base_;
      have_source_odom = have_source_odom_;
      source_frame = latest_source_frame_;
    }

    if (!global_map_ || global_map_->empty()) {
      publishRejected(source_stamp, "map not loaded");
      return;
    }
    if (!source || source->empty()) {
      publishRejected(ros::Time::now(), "waiting source cloud");
      return;
    }
    const double cloud_age = (ros::Time::now() - source_stamp).toSec();
    if (cloud_age > max_cloud_age_) {
      publishRejected(source_stamp, "source cloud timeout");
      return;
    }
    if (static_cast<int>(source->size()) < min_source_points_) {
      publishRejected(source_stamp, "source points below threshold");
      return;
    }

    Eigen::Matrix4f initial_map_source = Eigen::Matrix4f::Identity();
    const std::string source_mode = (source_cloud_frame_ == "auto") ? source_frame : source_cloud_frame_;
    const bool source_is_odom = source_cloud_is_odom_frame_ || source_mode == "odom" || source_mode == odom_frame_;
    const bool source_is_lidar = source_mode == "lidar" || source_mode == lidar_frame_;
    if (source_is_odom) {
      if (have_initial && have_source_odom) {
        initial_map_source = initial_map_base * source_base.inverse();
      } else if (have_initial) {
        initial_map_source = initial_map_base;
        ROS_WARN_THROTTLE(2.0, "[global_matcher] source_odom missing, using initial pose as map->source guess");
      } else {
        initial_map_source = Eigen::Matrix4f::Identity();
        ROS_WARN_THROTTLE(2.0, "[global_matcher] no initial pose, using identity map->odom guess");
      }
    } else if (source_is_lidar) {
      source_base = base_to_lidar_.inverse();
      initial_map_source = (have_initial ? initial_map_base : Eigen::Matrix4f::Identity()) * base_to_lidar_;
    } else {
      source_base = Eigen::Matrix4f::Identity();
      initial_map_source = have_initial ? initial_map_base : Eigen::Matrix4f::Identity();
    }
    const Eigen::Matrix4f initial_map_base_for_crop = initial_map_source * source_base;

    publishInitialGuess(source_stamp, initial_map_base_for_crop);
    publishSourceInitial(source_stamp, source, initial_map_source);

    CloudPtr target_submap = cropMap(initial_map_base_for_crop);
    if (!target_submap || static_cast<int>(target_submap->size()) < min_target_points_) {
      publishSubmap(source_stamp, target_submap);
      publishRejected(source_stamp, "target submap points below threshold");
      return;
    }
    publishSubmap(source_stamp, target_submap);

    const ros::Time start = ros::Time::now();
    double coarse_fitness = std::numeric_limits<double>::infinity();
    bool coarse_converged = false;
    const Eigen::Matrix4f coarse = registerAtScale(source, target_submap, initial_map_source,
                                                   coarse_scale_, &coarse_fitness, &coarse_converged);
    double fine_fitness = std::numeric_limits<double>::infinity();
    bool fine_converged = false;
    const Eigen::Matrix4f final_map_source = registerAtScale(source, target_submap, coarse,
                                                            fine_scale_, &fine_fitness, &fine_converged);
    const double elapsed_ms = (ros::Time::now() - start).toSec() * 1000.0;
    const Eigen::Matrix4f final_map_base = final_map_source * source_base;

    MatchMetrics metrics = computeInliers(source, target_submap, final_map_source, inlier_distance_);
    const double delta_translation = translationDiff(final_map_base, initial_map_base_for_crop);
    const double delta_yaw = std::abs(yawDiff(final_map_base, initial_map_base_for_crop));

    // Gate by match quality only. hasConverged() means the last ICP step was small enough,
    // not that the alignment is bad — treat it as a soft warning.
    std::string reject_reason;
    bool success = true;
    if (fine_fitness > max_fitness_score_) {
      reject_reason = "fitness score above threshold";
      success = false;
    } else if (metrics.inlier_ratio < min_inlier_ratio_) {
      reject_reason = "inlier ratio below threshold";
      success = false;
    } else if (metrics.inlier_rmse > max_inlier_rmse_) {
      reject_reason = "inlier rmse above threshold";
      success = false;
    } else if (delta_translation > max_delta_translation_) {
      reject_reason = "translation delta above threshold";
      success = false;
    } else if (delta_yaw > max_delta_yaw_deg_ * kPi / 180.0) {
      reject_reason = "yaw delta above threshold";
      success = false;
    }

    if (success && !fine_converged) {
      ROS_WARN_THROTTLE(2.0,
                        "[global_matcher] matcher hasConverged=false but quality gates passed "
                        "(fitness=%.4f inlier=%.3f rmse=%.3f); accepting result",
                        fine_fitness, metrics.inlier_ratio, metrics.inlier_rmse);
    }

    publishAligned(source_stamp, source, final_map_source);
    publishResult(source_stamp, success, fine_converged, final_map_base, fine_fitness, metrics,
                  elapsed_ms, static_cast<uint32_t>(target_submap->size()),
                  delta_translation, delta_yaw, reject_reason);
    publishStatus(success ? deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL
                          : deeplumin_msgs::LocalizationStatus::LEVEL_WARNING,
                  success ? 1.0 : 0.2,
                  success ? "" : reject_reason);

    ROS_INFO("[global_matcher] success=%d converged=%d source=%zu target=%zu fitness=%.4f inlier=%.3f rmse=%.3f delta=%.3fm yaw=%.2fdeg cost=%.1fms reason=%s",
             success, fine_converged, source->size(), target_submap->size(), fine_fitness,
             metrics.inlier_ratio, metrics.inlier_rmse, delta_translation,
             delta_yaw * 180.0 / kPi, elapsed_ms, reject_reason.c_str());
  }

  CloudPtr cropMap(const Eigen::Matrix4f& map_base) const {
    CloudPtr cropped(new CloudT());
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(global_map_);
    crop.setMin(Eigen::Vector4f(static_cast<float>(-crop_backward_),
                                static_cast<float>(-crop_right_),
                                static_cast<float>(-crop_down_), 1.0f));
    crop.setMax(Eigen::Vector4f(static_cast<float>(crop_forward_),
                                static_cast<float>(crop_left_),
                                static_cast<float>(crop_up_), 1.0f));
    crop.setTransform(Eigen::Affine3f(map_base.inverse()));
    crop.filter(*cropped);
    return cropped;
  }

  Eigen::Matrix4f registerAtScale(const CloudConstPtr& source,
                                  const CloudConstPtr& target,
                                  const Eigen::Matrix4f& initial_guess,
                                  double scale,
                                  double* fitness,
                                  bool* converged) const {
    const double source_leaf = std::max(0.01, source_voxel_leaf_ * scale);
    const double target_leaf = std::max(0.01, target_voxel_leaf_ * scale);
    CloudPtr source_filtered = downsample(source, source_leaf);
    CloudPtr target_filtered = downsample(target, target_leaf);

    if (source_filtered->size() < 4 || target_filtered->size() < 4) {
      if (fitness) *fitness = std::numeric_limits<double>::infinity();
      if (converged) *converged = false;
      return initial_guess;
    }

    pcl::PointCloud<PointT> aligned;
    bool has_converged = false;
    double score = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f transform = initial_guess;

    if (matcher_type_ == "icp") {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setInputSource(source_filtered);
      icp.setInputTarget(target_filtered);
      icp.setMaximumIterations(max_iterations_);
      icp.setTransformationEpsilon(transformation_epsilon_);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance_ * scale);
      icp.align(aligned, initial_guess);
      has_converged = icp.hasConverged();
      score = icp.getFitnessScore();
      // Always take the last iterate; quality gates decide accept/reject.
      transform = icp.getFinalTransformation();
    } else if (matcher_type_ == "gicp") {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setInputSource(source_filtered);
      gicp.setInputTarget(target_filtered);
      gicp.setMaximumIterations(max_iterations_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_ * scale);
      gicp.align(aligned, initial_guess);
      has_converged = gicp.hasConverged();
      score = gicp.getFitnessScore();
      transform = gicp.getFinalTransformation();
    } else {
      fast_gicp::FastGICP<PointT, PointT> gicp;
      gicp.setNumThreads(std::max(1, fast_gicp_num_threads_));
      gicp.setCorrespondenceRandomness(std::max(1, fast_gicp_correspondence_randomness_));
      gicp.setInputSource(source_filtered);
      gicp.setInputTarget(target_filtered);
      gicp.setMaximumIterations(max_iterations_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_ * scale);
      gicp.align(aligned, initial_guess);
      has_converged = gicp.hasConverged();
      score = gicp.getFitnessScore();
      transform = gicp.getFinalTransformation();
    }

    if (fitness) *fitness = score;
    if (converged) *converged = has_converged;
    return transform;
  }

  MatchMetrics computeInliers(const CloudConstPtr& source,
                              const CloudConstPtr& target,
                              const Eigen::Matrix4f& map_source,
                              double threshold) const {
    MatchMetrics metrics;
    if (!source || !target || source->empty() || target->empty()) {
      return metrics;
    }

    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*source, *transformed, map_source);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);
    std::vector<int> indices(1);
    std::vector<float> sq_distances(1);
    const double threshold_sq = threshold * threshold;
    double sum_sq = 0.0;

    for (const auto& pt : transformed->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      ++metrics.total_points;
      if (kdtree.nearestKSearch(pt, 1, indices, sq_distances) > 0 && sq_distances[0] <= threshold_sq) {
        ++metrics.inlier_count;
        sum_sq += sq_distances[0];
      }
    }
    if (metrics.total_points > 0) {
      metrics.inlier_ratio = static_cast<double>(metrics.inlier_count) / metrics.total_points;
    }
    if (metrics.inlier_count > 0) {
      metrics.inlier_rmse = std::sqrt(sum_sq / metrics.inlier_count);
    }
    return metrics;
  }

  void publishGlobalMap() const {
    const CloudPtr cloud = (global_map_viz_ && !global_map_viz_->empty()) ? global_map_viz_ : global_map_;
    if (!cloud || cloud->empty() || global_map_pub_.getNumSubscribers() == 0) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    global_map_pub_.publish(msg);
  }

  void publishSubmap(const ros::Time& stamp, const CloudConstPtr& cloud) const {
    if (!cloud || submap_pub_.getNumSubscribers() == 0) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    submap_pub_.publish(msg);
  }

  void publishSourceInitial(const ros::Time& stamp,
                            const CloudConstPtr& source,
                            const Eigen::Matrix4f& map_source) const {
    if (!source || source_initial_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*source, *transformed, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    source_initial_pub_.publish(msg);
  }

  void publishAligned(const ros::Time& stamp, const CloudConstPtr& source, const Eigen::Matrix4f& map_source) const {
    if (!source || aligned_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr aligned(new CloudT());
    pcl::transformPointCloud(*source, *aligned, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    aligned_pub_.publish(msg);
  }

  void publishInitialGuess(const ros::Time& stamp, const Eigen::Matrix4f& map_base) const {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose = matrixToPose(map_base);
    initial_guess_pub_.publish(msg);
  }

  void publishResult(const ros::Time& stamp,
                     bool success,
                     bool converged,
                     const Eigen::Matrix4f& map_base,
                     double fitness,
                     const MatchMetrics& metrics,
                     double elapsed_ms,
                     uint32_t target_points,
                     double delta_translation,
                     double delta_yaw,
                     const std::string& reject_reason) {
    deeplumin_msgs::GlobalMatchResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose = matrixToPose(map_base);
    const double cov_pos = std::max(0.01, metrics.inlier_rmse * metrics.inlier_rmse);
    const double cov_yaw = std::pow(std::max(0.5, std::abs(delta_yaw) * 180.0 / kPi) * kPi / 180.0, 2.0);
    for (int i = 0; i < 36; ++i) {
      msg.pose.covariance[i] = 0.0;
    }
    msg.pose.covariance[0] = cov_pos;
    msg.pose.covariance[7] = cov_pos;
    msg.pose.covariance[14] = std::max(0.04, cov_pos);
    msg.pose.covariance[21] = 0.05;
    msg.pose.covariance[28] = 0.05;
    msg.pose.covariance[35] = cov_yaw;
    msg.success = success;
    msg.converged = converged;
    msg.map_id = active_map_id_;
    msg.reject_reason = reject_reason;
    msg.fitness_score = fitness;
    msg.inlier_ratio = metrics.inlier_ratio;
    msg.inlier_rmse = metrics.inlier_rmse;
    msg.inlier_count = metrics.inlier_count;
    msg.source_points = metrics.total_points;
    msg.target_points = target_points;
    msg.elapsed_ms = elapsed_ms;
    msg.initial_translation_error = delta_translation;
    msg.initial_yaw_error = delta_yaw;
    result_pub_.publish(msg);

    geometry_msgs::PoseStamped path_pose;
    path_pose.header = msg.header;
    path_pose.pose = msg.pose.pose;
    path_.header.stamp = stamp;
    path_.header.frame_id = map_frame_;
    path_.poses.push_back(path_pose);
    if (static_cast<int>(path_.poses.size()) > max_path_size_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - max_path_size_));
    }
    path_pub_.publish(path_);
  }

  void publishRejected(const ros::Time& stamp, const std::string& reason) {
    MatchMetrics empty;
    publishResult(stamp.isZero() ? ros::Time::now() : stamp, false, false,
                  Eigen::Matrix4f::Identity(), std::numeric_limits<double>::infinity(),
                  empty, 0.0, 0, 0.0, 0.0, reason);
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0, reason);
  }

  void publishStatus(uint8_t level, double quality, const std::string& reason) const {
    deeplumin_msgs::LocalizationStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.level = level;
    msg.failure_reason = reason;
    msg.quality_score = quality;
    msg.is_gnss_available = false;
    msg.is_slam_available = true;
    msg.is_dr_available = false;
    msg.current_mode = (viz_test_mode_ ? "GLOBAL_MATCHER_VIZ_TEST" : "GLOBAL_MATCHER") + std::string(":") + active_map_id_;
    status_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber source_sub_;
  ros::Subscriber initial_odom_sub_;
  ros::Subscriber source_odom_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber load_map_command_sub_;
  ros::Subscriber set_map_segment_command_sub_;
  ros::Publisher result_pub_;
  ros::Publisher status_pub_;
  ros::Publisher source_initial_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher global_map_pub_;
  ros::Publisher initial_guess_pub_;
  ros::Publisher path_pub_;
  ros::Timer match_timer_;
  ros::Timer map_timer_;

  mutable std::mutex mutex_;
  std::map<std::string, MapSegment> map_segments_;
  CloudPtr global_map_;
  CloudPtr global_map_viz_;
  CloudPtr latest_source_;
  ros::Time latest_source_stamp_;
  std::string latest_source_frame_;
  Eigen::Matrix4f latest_initial_map_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f latest_source_base_ = Eigen::Matrix4f::Identity();
  ros::Time latest_initial_stamp_;
  ros::Time latest_source_odom_stamp_;
  bool have_initial_map_base_ = false;
  bool have_source_odom_ = false;
  nav_msgs::Path path_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string source_cloud_topic_;
  std::string initial_pose_topic_;
  std::string source_odom_topic_;
  std::string manual_initial_pose_topic_;
  std::string result_topic_;
  std::string status_topic_;
  std::string source_initial_scan_topic_;
  std::string aligned_scan_topic_;
  std::string target_submap_topic_;
  std::string global_map_topic_;
  std::string initial_guess_topic_;
  std::string path_topic_;
  std::string pcd_path_;
  std::string default_map_id_;
  std::string active_map_id_;
  std::string load_map_command_topic_;
  std::string set_map_segment_command_topic_;
  std::string matcher_type_;
  std::string source_cloud_frame_ = "auto";

  bool auto_match_ = true;
  bool viz_test_mode_ = false;
  bool source_cloud_is_odom_frame_ = true;
  Eigen::Matrix4f base_to_lidar_ = Eigen::Matrix4f::Identity();
  double match_rate_ = 1.0;
  double max_cloud_age_ = 1.0;
  int max_path_size_ = 3000;
  double global_map_publish_voxel_leaf_ = 5.0;
  double global_map_publish_period_ = 5.0;
  double crop_forward_ = 70.0;
  double crop_backward_ = 7.0;
  double crop_left_ = 35.0;
  double crop_right_ = 35.0;
  double crop_down_ = 10.0;
  double crop_up_ = 10.0;
  double source_voxel_leaf_ = 0.1;
  double target_voxel_leaf_ = 0.1;
  double coarse_scale_ = 6.0;
  double fine_scale_ = 3.0;
  int max_iterations_ = 20;
  double transformation_epsilon_ = 0.01;
  double max_correspondence_distance_ = 2.0;
  int fast_gicp_num_threads_ = 2;
  int fast_gicp_correspondence_randomness_ = 20;
  int min_source_points_ = 200;
  int min_target_points_ = 1000;
  double max_fitness_score_ = 1.0;
  double min_inlier_ratio_ = 0.5;
  double max_inlier_rmse_ = 0.8;
  double inlier_distance_ = 0.5;
  double max_delta_translation_ = 5.0;
  double max_delta_yaw_deg_ = 20.0;
};

int main(int argc, char** argv) {
#ifdef GLOBAL_MATCHER_VIZ_TEST
  ros::init(argc, argv, "global_matcher_viz_test");
#else
  ros::init(argc, argv, "global_matcher");
#endif
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GlobalMatcherNode node(nh, pnh);
  ros::spin();
  return 0;
}
