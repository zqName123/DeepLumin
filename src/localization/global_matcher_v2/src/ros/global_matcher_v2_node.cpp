/**
 * @file global_matcher_v2_node.cpp
 * @brief global_matcher_v2 全局匹配定位节点
 *
 * 与原始 global_matcher 流程保持一致（初始猜测选择 -> 粗/细两阶段配准 ->
 * 质量门控 -> 结果发布），主要差异：
 * - 地图管理：单图全量 / 分片实时加载（MapManager，后台异步加载 + 预构建
 *   target 结构：协方差点云 / KDTree / VGICP 体素地图，按活动分片集合缓存）；
 * - 匹配器：icp / gicp / vgicp 统一基于 small_gicp（含 Huber/Cauchy 鲁棒核），
 *   并提供 fast_gicp 粗配准 + vgicp 精细配准混合分支；
 * - 发布开关：调试话题按 publish.* 开关创建，减少话题量。
 */

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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/LocalizationStatus.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include "global_matcher_v2/core/map_manager.hpp"
#include "global_matcher_v2/core/matcher.hpp"
#include "global_matcher_v2/ros/global_matcher_v2_config.hpp"

namespace global_matcher_v2 {

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
  double fitness = std::numeric_limits<double>::infinity();
};

enum class MatcherRunMode { BOOTSTRAP, TRACKING };

struct InitialGuess {
  Eigen::Matrix4f map_source = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f source_base = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_base = Eigen::Matrix4f::Identity();
  MatcherRunMode mode = MatcherRunMode::BOOTSTRAP;
  std::string reason;
  bool valid = true;
};

std::string modeToString(MatcherRunMode mode) {
  switch (mode) {
    case MatcherRunMode::BOOTSTRAP:
      return "bootstrap";
    case MatcherRunMode::TRACKING:
      return "tracking";
  }
  return "unknown";
}

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
  const Eigen::Quaternionf q(mat.block<3, 3>(0, 0));
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
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return normalizeAngle(yawFromMatrix(a) - yawFromMatrix(b));
}

double translationDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

Eigen::Matrix3f rpyDegToMatrix(const Eigen::Vector3f& rpy_deg) {
  const float deg_to_rad = static_cast<float>(kPi / 180.0);
  const Eigen::AngleAxisf roll(rpy_deg.x() * deg_to_rad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(rpy_deg.y() * deg_to_rad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(rpy_deg.z() * deg_to_rad, Eigen::Vector3f::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
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

class GlobalMatcherV2Node {
 public:
  GlobalMatcherV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    config_ = loadGlobalMatcherV2Config(pnh_);
    default_initial_map_base_.block<3, 3>(0, 0) =
        rpyDegToMatrix(config_.default_initial_rpy_deg);
    default_initial_map_base_.block<3, 1>(0, 3) = config_.default_initial_translation;

    // ---- 地图管理 ----
    MapManagerConfig mm_cfg = config_.mapManagerConfig();
    // 只有 vgicp 相关匹配类型才预构建体素地图，避免无谓开销
    const bool use_vgicp =
        config_.matcher.type == "vgicp" || config_.matcher.type == "fast_gicp_vgicp";
    mm_cfg.prebuild_voxel_maps = use_vgicp;
    if (!map_manager_.configure(mm_cfg) || !map_manager_.loadInitial()) {
      ROS_ERROR("[global_matcher_v2] failed to init map manager mode=%s dir=%s pcd=%s",
                config_.map_mode.c_str(), config_.map_dir.c_str(),
                config_.map_pcd_path.c_str());
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0,
                    "failed to load map");
    } else {
      publishGlobalMap();
    }

    // ---- 匹配器 ----
    matcher_.configure(config_.matcher);

    // ---- 发布器（按 publish.* 开关创建） ----
    result_pub_ = nh_.advertise<deeplumin_msgs::GlobalMatchResult>(config_.result_topic, 10);
    if (config_.publish_status) {
      status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(config_.status_topic, 10);
    }
    if (config_.publish_global_map) {
      global_map_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(config_.global_map_topic, 1, true);
      // 发布器创建后立即发布一次全局地图（latch，新订阅者自动收到），
      // 避免“启动后要等首个匹配帧/定时器才出图”
      publishGlobalMap();
    }
    if (config_.publish_source_initial_scan) {
      source_initial_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(config_.source_initial_scan_topic, 2);
    }
    if (config_.publish_aligned_scan) {
      aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.aligned_scan_topic, 2);
    }
    if (config_.publish_target_submap) {
      submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.target_submap_topic, 2);
    }
    if (config_.publish_initial_guess) {
      initial_guess_pub_ =
          nh_.advertise<geometry_msgs::PoseStamped>(config_.initial_guess_topic, 2);
    }
    if (config_.publish_path) {
      path_pub_ = nh_.advertise<nav_msgs::Path>(config_.path_topic, 2);
      path_.header.frame_id = config_.map_frame;
    }

    // ---- 订阅器 ----
    source_sub_ = nh_.subscribe(config_.source_cloud_topic, 2, &GlobalMatcherV2Node::onSourceCloud,
                                this);
    tracking_initial_sub_ =
        nh_.subscribe(config_.initial_pose_topic, 5, &GlobalMatcherV2Node::onTrackingInitialOdom,
                      this);
    source_odom_sub_ =
        nh_.subscribe(config_.source_odom_topic, 1, &GlobalMatcherV2Node::onSourceOdom, this);
    bootstrap_initial_pose_sub_ = nh_.subscribe(
        config_.bootstrap_initial_pose_topic, 2, &GlobalMatcherV2Node::onBootstrapInitialPose,
        this);
    manual_initial_pose_sub_ = nh_.subscribe(config_.manual_initial_pose_topic, 2,
                                             &GlobalMatcherV2Node::onManualInitialPose, this);
    load_map_command_sub_ = nh_.subscribe(config_.load_map_command_topic, 2,
                                          &GlobalMatcherV2Node::onLoadMapCommand, this);
    set_map_segment_command_sub_ =
        nh_.subscribe(config_.set_map_segment_command_topic, 2,
                      &GlobalMatcherV2Node::onSetMapSegmentCommand, this);

    // ---- 定时器 ----
    match_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, config_.match_rate)),
                                   &GlobalMatcherV2Node::onMatchTimer, this);
    if (config_.publish_global_map && config_.global_map_publish_period >= 0.5) {
      map_timer_ = nh_.createTimer(ros::Duration(config_.global_map_publish_period),
                                   &GlobalMatcherV2Node::onMapTimer, this);
    }

    ROS_INFO("[global_matcher_v2] map_mode=%s matcher=%s robust=%s result_pose_frame=%s",
             config_.map_mode.c_str(), config_.matcher.type.c_str(),
             config_.matcher.robust_kernel.c_str(), config_.result_pose_frame.c_str());
  }

 private:
  // ===================================================================
  // 订阅回调
  // ===================================================================

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    CloudPtr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);
    cloud = removeInvalid(cloud);
    const ros::Time receive_stamp = ros::Time::now();
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_ = cloud;
    latest_source_stamp_ = msg->header.stamp.isZero() ? receive_stamp : msg->header.stamp;
    latest_source_receive_stamp_ = receive_stamp;
    latest_source_frame_ = msg->header.frame_id;
  }

  void onTrackingInitialOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_tracking_map_base_ = poseToMatrix(msg->pose.pose);
    latest_tracking_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_tracking_map_base_ = true;
  }

  void onSourceOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_base_ = poseToMatrix(msg->pose.pose);
    latest_source_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_source_odom_ = true;
  }

  void onBootstrapInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bootstrap_map_base_ = poseToMatrix(msg->pose.pose);
    latest_bootstrap_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_bootstrap_map_base_ = true;
    ROS_INFO("[global_matcher_v2] received bootstrap initial pose from %s",
             config_.bootstrap_initial_pose_topic.c_str());
  }

  void onManualInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.use_manual_initial_pose) {
      return;
    }
    Eigen::Matrix4f manual_pose = poseToMatrix(msg->pose.pose);
    // 与 ysw_loc GM 一致：人工 2D 初值用地图补全 z（该 xy 附近点云 z 中值）
    double init_z = 0.0;
    if (map_manager_.estimateGroundZ(manual_pose(0, 3), manual_pose(1, 3),
                                     config_.manual_initial_pose_z_search_radius, &init_z)) {
      manual_pose(2, 3) = init_z;
      ROS_INFO("[global_matcher_v2] manual initial pose z completed to %.2f", init_z);
    } else {
      ROS_WARN_THROTTLE(2.0,
                        "[global_matcher_v2] cannot estimate z from map at manual pose "
                        "(x=%.2f y=%.2f), keep given z=%.2f",
                        manual_pose(0, 3), manual_pose(1, 3), manual_pose(2, 3));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bootstrap_map_base_ = manual_pose;
    latest_bootstrap_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_bootstrap_map_base_ = true;
    if (have_source_odom_) {
      last_map_source_ = latest_bootstrap_map_base_ * latest_source_base_.inverse();
      have_last_map_source_ = true;
    }
    ++manual_reset_seq_;
    ROS_INFO("[global_matcher_v2] received manual initial pose from %s",
             config_.manual_initial_pose_topic.c_str());
  }

  void onLoadMapCommand(const std_msgs::String::ConstPtr& msg) {
    if (config_.map_mode == "chunked") {
      ROS_WARN("[global_matcher_v2] chunked mode: chunks are auto-managed, ignore load_map_command");
      return;
    }
    std::vector<std::string> fields;
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, '|')) {
      fields.push_back(item);
    }
    if (fields.size() < 2) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "load_map_command format: map_id|pcd_path");
      return;
    }
    if (map_manager_.reloadSingleMap(fields[1])) {
      publishGlobalMap();
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL, 1.0,
                    "reloaded single map");
    } else {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0,
                    "failed to reload single map");
    }
  }

  void onSetMapSegmentCommand(const std_msgs::String::ConstPtr& msg) {
    ROS_WARN("[global_matcher_v2] map segments are auto-managed (mode=%s), ignore command '%s'",
             config_.map_mode.c_str(), msg->data.c_str());
  }

  // ===================================================================
  // 匹配
  // ===================================================================

  void onMatchTimer(const ros::TimerEvent&) {
    if (!config_.auto_match) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "auto_match disabled");
      return;
    }
    runMatch();
  }

  void onMapTimer(const ros::TimerEvent&) { publishGlobalMap(); }

  void runMatch() {
    CloudPtr source;
    ros::Time source_stamp;
    ros::Time source_receive_stamp;
    Eigen::Matrix4f tracking_map_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f bootstrap_map_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f source_odom_base = Eigen::Matrix4f::Identity();
    bool have_tracking = false;
    bool have_bootstrap = false;
    bool have_source_odom = false;
    std::string source_frame;
    uint64_t match_start_seq = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      source = latest_source_;
      source_stamp = latest_source_stamp_;
      source_receive_stamp = latest_source_receive_stamp_;
      tracking_map_base = latest_tracking_map_base_;
      bootstrap_map_base = latest_bootstrap_map_base_;
      source_odom_base = latest_source_base_;
      have_tracking = have_tracking_map_base_;
      have_bootstrap = have_bootstrap_map_base_;
      have_source_odom = have_source_odom_;
      source_frame = latest_source_frame_;
      match_start_seq = manual_reset_seq_;
    }

    if (!source || source->empty()) {
      publishRejected(ros::Time::now(), "waiting source cloud");
      return;
    }
    const ros::Time now = ros::Time::now();
    const ros::Time cloud_age_stamp =
        config_.use_receive_time_for_cloud_age ? source_receive_stamp : source_stamp;
    const double cloud_age = cloud_age_stamp.isZero()
                                 ? std::numeric_limits<double>::infinity()
                                 : (now - cloud_age_stamp).toSec();
    if (cloud_age > config_.max_cloud_age) {
      publishRejected(source_stamp, "source cloud timeout");
      return;
    }
    if (static_cast<int>(source->size()) < config_.min_source_points) {
      publishRejected(source_stamp, "source points below threshold");
      return;
    }

    const InitialGuess guess = selectInitialGuess(source_frame, have_tracking, tracking_map_base,
                                                  have_bootstrap, bootstrap_map_base,
                                                  have_source_odom, source_odom_base);
    current_mode_ = modeToString(guess.mode);
    if (!guess.valid) {
      publishRejected(source_stamp, guess.reason);
      return;
    }
    const Eigen::Matrix4f initial_map_source = guess.map_source;
    const Eigen::Matrix4f initial_map_base_for_crop = guess.map_base;
    last_crop_map_base_ = initial_map_base_for_crop;

    // ---- 分片地图：按当前位置确保所需分片已加载 ----
    const Eigen::Vector3d pos = guess.map_base.block<3, 1>(0, 3).cast<double>();
    if (!map_manager_.ensureActive(pos)) {
      publishRejected(source_stamp, "map chunks not ready");
      return;
    }
    if (map_manager_.mapChanged()) {
      map_manager_.consumeMapChanged();
      publishGlobalMap();
    }

    // ---- 匹配 target：use_crop=true 时按 FOV（crop.*）裁剪活动地图（v1 行为，fast_gicp 推荐）；
    //      否则使用 MapManager 缓存的整图/活动分片结构（gicp/vgicp 推荐）----
    const bool use_crop = config_.matcher.use_crop;
    const bool need_target_cov =
        config_.matcher.type != "fast_gicp" && config_.matcher.type != "icp";
    const bool need_voxelmap =
        config_.matcher.type == "vgicp" || config_.matcher.type == "fast_gicp_vgicp";

    small_gicp::PointCloud::Ptr crop_sg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_pcl;
    std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> crop_kdtree;
    small_gicp::GaussianVoxelMap::Ptr crop_voxel_coarse;
    small_gicp::GaussianVoxelMap::Ptr crop_voxel_fine;

    const small_gicp::PointCloud* target_sg_ptr = nullptr;
    const pcl::PointCloud<pcl::PointXYZ>* target_pcl_ptr = nullptr;
    const small_gicp::KdTree<small_gicp::PointCloud>* target_kdtree_ptr = nullptr;
    const small_gicp::GaussianVoxelMap* coarse_voxelmap_ptr = nullptr;
    const small_gicp::GaussianVoxelMap* fine_voxelmap_ptr = nullptr;

    const ros::Time start = ros::Time::now();
    if (use_crop) {
      crop_pcl = map_manager_.cropSubmap(initial_map_base_for_crop, config_.crop);
      if (!crop_pcl || crop_pcl->empty()) {
        publishRejected(source_stamp, "cropped target empty");
        return;
      }
      last_match_submap_ = crop_pcl;  // 供 target_submap 可视化复用
      crop_sg = std::make_shared<small_gicp::PointCloud>();
      crop_sg->resize(crop_pcl->size());
      for (size_t i = 0; i < crop_pcl->size(); ++i) {
        const auto& p = crop_pcl->points[i];
        crop_sg->point(i) << p.x, p.y, p.z, 1.0;
      }
      if (need_target_cov) {
        small_gicp::estimate_covariances_omp(*crop_sg, std::max(3, config_.cov_num_neighbors),
                                             std::max(1, config_.cov_num_threads));
      }
      crop_kdtree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
          crop_sg, small_gicp::KdTreeBuilderOMP(std::max(1, config_.cov_num_threads)));
      if (need_voxelmap) {
        crop_voxel_coarse = std::make_shared<small_gicp::GaussianVoxelMap>(
            config_.vgicp_voxel_resolution * config_.vgicp_coarse_scale);
        crop_voxel_coarse->set_search_offsets(27);
        crop_voxel_coarse->insert(*crop_sg);
        crop_voxel_fine = std::make_shared<small_gicp::GaussianVoxelMap>(
            config_.vgicp_voxel_resolution * config_.vgicp_fine_scale);
        crop_voxel_fine->set_search_offsets(27);
        crop_voxel_fine->insert(*crop_sg);
      }
      target_sg_ptr = crop_sg.get();
      target_pcl_ptr = crop_pcl.get();
      target_kdtree_ptr = crop_kdtree.get();
      coarse_voxelmap_ptr = crop_voxel_coarse.get();
      fine_voxelmap_ptr = crop_voxel_fine.get();
    } else {
      last_match_submap_.reset();
      const auto cached_sg = map_manager_.targetCloud();
      const auto cached_pcl = map_manager_.targetPcl();
      const auto cached_kdtree = map_manager_.targetKdTree();
      const auto cached_voxel_coarse = map_manager_.targetVoxelMapCoarse();
      const auto cached_voxel_fine = map_manager_.targetVoxelMapFine();
      if (!cached_sg || cached_sg->empty() || !cached_pcl || cached_pcl->empty() ||
          !cached_kdtree) {
        publishRejected(source_stamp, "target map not ready");
        return;
      }
      target_sg_ptr = cached_sg.get();
      target_pcl_ptr = cached_pcl.get();
      target_kdtree_ptr = cached_kdtree.get();
      coarse_voxelmap_ptr = cached_voxel_coarse.get();
      fine_voxelmap_ptr = cached_voxel_fine.get();
    }
    if (static_cast<int>(target_pcl_ptr->size()) < config_.min_target_points) {
      publishSubmap(source_stamp);
      publishRejected(source_stamp, "target map points below threshold");
      return;
    }

    publishInitialGuess(source_stamp, initial_map_base_for_crop);
    publishSourceInitial(source_stamp, source, initial_map_source);
    // const ros::Time start = ros::Time::now();

    const MatchOutput match_result =
        matcher_.match(*source, *target_sg_ptr, *target_pcl_ptr, *target_kdtree_ptr,
                       coarse_voxelmap_ptr, fine_voxelmap_ptr, initial_map_source);
    const Eigen::Matrix4f final_map_source = match_result.final;
    const Eigen::Matrix4f final_map_base = final_map_source * guess.source_base;

    MatchMetrics metrics =
        computeInliers(*source, final_map_source, *target_kdtree_ptr, config_.inlier_distance);
    const double delta_translation = translationDiff(final_map_base, initial_map_base_for_crop);
    const double delta_yaw = std::abs(yawDiff(final_map_base, initial_map_base_for_crop));
    const double elapsed_ms = (ros::Time::now() - start).toSec() * 1000.0;

    // ---- 质量门控 ----
    std::string reject_reason;
    bool success = true;
    // fitness 门限仅对 fast_gicp 生效（PCL 语义）；small_gicp error 量纲不同，
    // 统一以内点率 / RMSE / delta 门控
    if ((config_.matcher.type == "fast_gicp") &&
        match_result.fine_fitness > config_.max_fitness_score) {
      reject_reason = "fitness score above threshold";
      success = false;
    } else if (metrics.inlier_ratio < config_.min_inlier_ratio) {
      reject_reason = "inlier ratio below threshold";
      success = false;
    } else if (metrics.inlier_rmse > config_.max_inlier_rmse) {
      reject_reason = "inlier rmse above threshold";
      success = false;
    } else if (delta_translation > config_.max_delta_translation) {
      reject_reason = "translation delta above threshold";
      success = false;
    } else if (delta_yaw > config_.max_delta_yaw_deg * kPi / 180.0) {
      reject_reason = "yaw delta above threshold";
      success = false;
    }

    if (success && !match_result.fine_converged) {
      ROS_WARN_THROTTLE(2.0,
                        "[global_matcher_v2] matcher converged=false but quality gates passed "
                        "(fitness=%.4f inlier=%.3f rmse=%.3f)",
                        match_result.fine_fitness, metrics.inlier_ratio, metrics.inlier_rmse);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (manual_reset_seq_ != match_start_seq) {
        publishRejected(source_stamp, "manual pose reset during match");
        return;
      }
    }
    if (success) {
      last_map_source_ = final_map_source;
      have_last_map_source_ = true;
    } else if (have_last_map_source_) {
      // 匹配失败时，检查当前结果与 last_map_source_ 的一致性
      // 若偏差过大，说明 last_map_source_ 已不可靠，丢弃之
      const double diff_t = translationDiff(final_map_source, last_map_source_);
      const double diff_yaw = std::abs(yawDiff(final_map_source, last_map_source_));
      constexpr double kMaxDiffTranslation = 1.0;   // 1 m
      constexpr double kMaxDiffYawDeg = 5.0;        // 5°
      if (diff_t > kMaxDiffTranslation || diff_yaw > kMaxDiffYawDeg * kPi / 180.0) {
        ROS_WARN("[global_matcher_v2] match failed and result deviates from last_map_source_ "
                 "(diff_t=%.3fm diff_yaw=%.2fdeg). Discarding last_map_source_.",
                 diff_t, diff_yaw * 180.0 / kPi);
        have_last_map_source_ = false;
        last_map_source_ = Eigen::Matrix4f::Identity();
      }
    }
    updateModeAfterMatch(success);

    publishAligned(source_stamp, source, final_map_source);
    publishSubmap(source_stamp);
    const bool publish_map_source =
        config_.result_pose_frame == "source" || config_.result_pose_frame == "odom";
    const Eigen::Matrix4f result_pose = publish_map_source ? final_map_source : final_map_base;
    const std::string result_child_frame = publish_map_source ? config_.odom_frame : config_.base_frame;
    publishResult(source_stamp, success, match_result.fine_converged, result_pose,
                  result_child_frame, final_map_base, match_result.fine_fitness, metrics,
                  elapsed_ms, static_cast<uint32_t>(target_pcl_ptr->size()), delta_translation,
                  delta_yaw, reject_reason);
    publishStatus(success ? deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL
                          : deeplumin_msgs::LocalizationStatus::LEVEL_WARNING,
                  success ? 1.0 : 0.2, success ? "" : reject_reason);

    ROS_INFO("[global_matcher_v2] mode=%s success=%d converged=%d source=%zu target=%zu "
             "fitness=%.4f inlier=%.3f rmse=%.3f delta=%.3fm yaw=%.2fdeg cost=%.1fms reason=%s",
             current_mode_.c_str(), success, match_result.fine_converged, source->size(),
             target_pcl_ptr->size(), match_result.fine_fitness, metrics.inlier_ratio,
             metrics.inlier_rmse, delta_translation, delta_yaw * 180.0 / kPi, elapsed_ms,
             reject_reason.c_str());
  }

  InitialGuess selectInitialGuess(const std::string& source_frame, bool have_tracking,
                                  const Eigen::Matrix4f& tracking_map_base, bool have_bootstrap,
                                  const Eigen::Matrix4f& bootstrap_map_base,
                                  bool have_source_odom,
                                  const Eigen::Matrix4f& source_odom_base) const {
    const std::string source_mode =
        (config_.source_cloud_frame == "auto") ? source_frame : config_.source_cloud_frame;
    const bool source_is_odom =
        config_.source_cloud_is_odom_frame || source_mode == "odom" ||
        source_mode == config_.odom_frame;
    const bool source_is_lidar =
        source_mode == "lidar" || source_mode == config_.lidar_frame;

    InitialGuess guess;
    bool have_map_base = false;
    Eigen::Matrix4f map_base = Eigen::Matrix4f::Identity();

    if (config_.use_tracking_initial_pose && (have_tracking || have_last_map_source_)) {
      guess.mode = MatcherRunMode::TRACKING;
      if (have_last_map_source_) {
        guess.reason = "tracking:last_result";
        guess.map_source = last_map_source_;
        guess.source_base = have_source_odom ? source_odom_base : Eigen::Matrix4f::Identity();
        guess.map_base = guess.map_source * guess.source_base;
        return guess;
      }
      guess.reason = "tracking:fused_odom";
      map_base = tracking_map_base;
      have_map_base = true;
    } else {
      guess.mode = MatcherRunMode::BOOTSTRAP;
      if (config_.use_bootstrap_initial_pose && have_bootstrap) {
        guess.reason = "bootstrap:initial_pose";
        map_base = bootstrap_map_base;
        have_map_base = true;
      } else if (config_.use_default_initial_pose) {
        guess.reason = "bootstrap:default_initial_pose";
        map_base = default_initial_map_base_;
        have_map_base = true;
      } else if (config_.allow_identity_bootstrap) {
        guess.reason = "bootstrap:identity";
        map_base = Eigen::Matrix4f::Identity();
        have_map_base = true;
      }
    }
    if (!have_map_base) {
      guess.reason = "bootstrap:no_initial_available";
      guess.valid = false;
      return guess;
    }

    if (source_is_odom) {
      guess.source_base = have_source_odom ? source_odom_base : Eigen::Matrix4f::Identity();
      guess.map_source = have_source_odom ? map_base * source_odom_base.inverse() : map_base;
      if (!have_source_odom) {
        ROS_WARN_THROTTLE(2.0, "[global_matcher_v2] source_odom missing, use map_base as map_source");
      }
    } else if (source_is_lidar) {
      guess.source_base = config_.base_to_lidar.inverse();
      guess.map_source = map_base * config_.base_to_lidar;
    } else {
      guess.source_base = Eigen::Matrix4f::Identity();
      guess.map_source = map_base;
    }
    guess.map_base = guess.map_source * guess.source_base;
    return guess;
  }

  void updateModeAfterMatch(bool success) {
    if (success) {
      consecutive_failures_ = 0;
    } else {
      ++consecutive_failures_;
    }
  }

  MatchMetrics computeInliers(const CloudT& source, const Eigen::Matrix4f& map_source,
                              const small_gicp::KdTree<small_gicp::PointCloud>& kdtree,
                              double threshold) const {
    MatchMetrics metrics;
    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(source, *transformed, map_source);

    const double threshold_sq = threshold * threshold;
    double sum_sq = 0.0;
    double sum_all_sq = 0.0;
    size_t k_index = 0;
    double k_sq_dist = 0.0;
    for (const auto& pt : transformed->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const Eigen::Vector4d q(pt.x, pt.y, pt.z, 1.0);
      if (kdtree.nearest_neighbor_search(q, &k_index, &k_sq_dist) == 0) {
        continue;
      }
      ++metrics.total_points;
      sum_all_sq += k_sq_dist;
      if (k_sq_dist <= threshold_sq) {
        ++metrics.inlier_count;
        sum_sq += k_sq_dist;
      }
    }
    if (metrics.total_points > 0) {
      metrics.inlier_ratio =
          static_cast<double>(metrics.inlier_count) / metrics.total_points;
      metrics.fitness = sum_all_sq / metrics.total_points;
    }
    if (metrics.inlier_count > 0) {
      metrics.inlier_rmse = std::sqrt(sum_sq / metrics.inlier_count);
    }
    return metrics;
  }

  // ===================================================================
  // 发布
  // ===================================================================

  void publishGlobalMap() const {
    if (!config_.publish_global_map || !global_map_pub_) {
      return;
    }
    const auto target_pcl = map_manager_.targetPcl();
    if (!target_pcl || target_pcl->empty()) {
      return;
    }
    const CloudPtr viz = downsample(target_pcl, config_.global_map_publish_voxel_leaf);
    if (!viz || viz->empty()) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*viz, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.map_frame;
    global_map_pub_.publish(msg);
  }

  void publishSubmap(const ros::Time& stamp) const {
    if (!config_.publish_target_submap || !submap_pub_ ||
        submap_pub_.getNumSubscribers() == 0) {
      return;
    }
    // use_crop 模式下复用本次匹配已裁剪的子图（与 v1 一致：同一 submap 既做匹配又做可视化），避免重复裁剪
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap;
    if (last_match_submap_ && !last_match_submap_->empty()) {
      submap = last_match_submap_;
    } else {
      submap = map_manager_.cropSubmap(last_crop_map_base_, config_.crop);
    }
    if (!submap || submap->empty()) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*submap, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    submap_pub_.publish(msg);
  }

  void publishSourceInitial(const ros::Time& stamp, const CloudConstPtr& source,
                            const Eigen::Matrix4f& map_source) const {
    if (!config_.publish_source_initial_scan || !source_initial_pub_ ||
        source_initial_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*source, *transformed, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    source_initial_pub_.publish(msg);
  }

  void publishAligned(const ros::Time& stamp, const CloudConstPtr& source,
                      const Eigen::Matrix4f& map_source) const {
    if (!config_.publish_aligned_scan || !aligned_pub_ ||
        aligned_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr aligned(new CloudT());
    pcl::transformPointCloud(*source, *aligned, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    aligned_pub_.publish(msg);
  }

  void publishInitialGuess(const ros::Time& stamp, const Eigen::Matrix4f& map_base) const {
    if (!config_.publish_initial_guess || !initial_guess_pub_) {
      return;
    }
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    msg.pose = matrixToPose(map_base);
    initial_guess_pub_.publish(msg);
  }

  void publishResult(const ros::Time& stamp, bool success, bool converged,
                     const Eigen::Matrix4f& result_pose,
                     const std::string& result_child_frame,
                     const Eigen::Matrix4f& path_map_base, double fitness,
                     const MatchMetrics& metrics, double elapsed_ms, uint32_t target_points,
                     double delta_translation, double delta_yaw,
                     const std::string& reject_reason) {
    deeplumin_msgs::GlobalMatchResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    msg.child_frame_id = result_child_frame;
    msg.pose.pose = matrixToPose(result_pose);
    const double cov_pos = std::max(0.01, metrics.inlier_rmse * metrics.inlier_rmse);
    const double cov_yaw =
        std::pow(std::max(0.5, std::abs(delta_yaw) * 180.0 / kPi) * kPi / 180.0, 2.0);
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
    msg.map_id = config_.map_mode == "chunked" ? "chunked" : "default";
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

    if (config_.publish_path && path_pub_) {
      geometry_msgs::PoseStamped path_pose;
      path_pose.header = msg.header;
      path_pose.pose = matrixToPose(path_map_base);
      path_.header.stamp = stamp;
      path_.poses.push_back(path_pose);
      if (static_cast<int>(path_.poses.size()) > 30000) {
        path_.poses.erase(path_.poses.begin(),
                          path_.poses.begin() + (path_.poses.size() - 30000));
      }
      path_pub_.publish(path_);
    }
  }

  void publishRejected(const ros::Time& stamp, const std::string& reason) {
    MatchMetrics empty;
    publishResult(stamp.isZero() ? ros::Time::now() : stamp, false, false,
                  Eigen::Matrix4f::Identity(), config_.base_frame, Eigen::Matrix4f::Identity(),
                  std::numeric_limits<double>::infinity(), empty, 0.0, 0, 0.0, 0.0, reason);
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0, reason);
  }

  void publishStatus(uint8_t level, double quality, const std::string& reason) const {
    if (!config_.publish_status || !status_pub_) {
      return;
    }
    deeplumin_msgs::LocalizationStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.map_frame;
    msg.level = level;
    msg.failure_reason = reason;
    msg.quality_score = quality;
    msg.is_gnss_available = false;
    msg.is_slam_available = true;
    msg.is_dr_available = false;
    msg.current_mode = std::string("GLOBAL_MATCHER_V2:") + config_.map_mode + ":" +
                       current_mode_;
    status_pub_.publish(msg);
  }

  // ===================================================================
  // 成员
  // ===================================================================
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GlobalMatcherV2Config config_;

  MapManager map_manager_;
  PointCloudMatcher matcher_;

  ros::Subscriber source_sub_;
  ros::Subscriber tracking_initial_sub_;
  ros::Subscriber source_odom_sub_;
  ros::Subscriber bootstrap_initial_pose_sub_;
  ros::Subscriber manual_initial_pose_sub_;
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
  CloudPtr latest_source_;
  ros::Time latest_source_stamp_;
  ros::Time latest_source_receive_stamp_;
  std::string latest_source_frame_;
  ros::Time latest_tracking_stamp_;
  ros::Time latest_source_odom_stamp_;
  ros::Time latest_bootstrap_stamp_;
  Eigen::Matrix4f latest_tracking_map_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f latest_bootstrap_map_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f latest_source_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f default_initial_map_base_ = Eigen::Matrix4f::Identity();
  bool have_tracking_map_base_ = false;
  bool have_bootstrap_map_base_ = false;
  bool have_source_odom_ = false;
  bool have_last_map_source_ = false;
  Eigen::Matrix4f last_map_source_ = Eigen::Matrix4f::Identity();
  uint64_t manual_reset_seq_ = 0;
  int consecutive_failures_ = 0;
  std::string current_mode_ = "bootstrap";
  nav_msgs::Path path_;
  Eigen::Matrix4f last_crop_map_base_ = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_match_submap_;  ///< use_crop 模式下复用匹配已裁剪的子图（同时用于 target_submap 可视化）
};

}  // namespace global_matcher_v2

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_matcher_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  global_matcher_v2::GlobalMatcherV2Node node(nh, pnh);
  ros::spin();
  return 0;
}