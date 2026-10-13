/**
 * @file online_node.cpp
 * @brief ArUco 在线重定位节点
 *
 * 数据流：
 *   图像 → ArUco 检测 → 单 marker PnP 求解 → 校验 → [VGICP 精化] → 发布结果
 *
 * 模块开关：
 *   - solver.use_multi_marker: 是否使用多 marker 联合解算
 *   - pc_verify.enabled: 是否使用点云验证（VGICP + inlier 计算）
 *   - pc_verify.use_vgicp: 是否使用 VGICP 精化（false 则直接用 ArUco 位姿计算 inlier）
 *   - visualizer.enabled: 是否发布调试图像和点云
 *
 * 与 localization_v2 的对接：
 *   result_topic: /v_reloc/result（与 relocalization 共用）
 *   command_topic: /v_reloc/command
 */

#include <aruco_relocalization/detector/marker_detector.hpp>
#include <aruco_relocalization/map/marker_map_loader.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>
#include <aruco_relocalization/solver/relocalization_solver.hpp>
#include <aruco_relocalization/validator/relocalization_validator.hpp>
#include <aruco_relocalization/verifier/pointcloud_verifier.hpp>
#include <aruco_relocalization/visualizer/debug_visualizer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <sstream>
#include <deeplumin_health_checker/health_checker/health_checker.h>


class ArucoRelocNode {
 public:
  ArucoRelocNode() : pnh_("~"), tf_listener_(tf_buffer_) {
    config_ = aruco_reloc::loadOnlineNodeConfig(pnh_);

    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh_, pnh_);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();

    std::cout << "ArucoRelocNode: HealthChecker enabled and activated." << std::endl;

    // 加载 marker 地图
    if (!map_loader_.loadFromFile(config_.marker_map_path)) {
      throw std::runtime_error("failed to load marker map from " + config_.marker_map_path);
    }

    // 初始化外参：T_base_cam = T_base_lidar * T_lidar_camera
    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
    T_base_lidar.translation() = config_.base_to_lidar.translation;
    T_base_lidar.rotate(config_.base_to_lidar.rotation);

    Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();
    T_lidar_camera.translation() = config_.lidar_to_camera.translation;
    T_lidar_camera.rotate(config_.lidar_to_camera.rotation);

    T_base_cam_ = T_base_lidar * T_lidar_camera;

    // 从配置文件加载相机内参
    cv::Mat camera_matrix, dist_coeffs;
    aruco_reloc::loadCameraIntrinsics(pnh_, camera_matrix, dist_coeffs);

    // 初始化各模块
    detector_.initialize(config_.detector, camera_matrix, dist_coeffs);
    solver_.initialize(config_.solver, camera_matrix, dist_coeffs, T_base_cam_);
    validator_.initialize(config_.validator, camera_matrix, dist_coeffs, T_base_cam_);

    // 初始化点云验证器
    if (config_.pc_verify.enabled) {
      if (!verifier_.initialize(config_.pc_verify)) {
        throw std::runtime_error("failed to initialize pointcloud verifier");
      }
      cloud_sub_ = nh_.subscribe(config_.pc_verify.cloud_topic, 5,
                                 &ArucoRelocNode::onCloud, this);
    }

    // 初始化调试可视化器
    if (config_.visualizer.enabled) {
      if (!visualizer_.initialize(config_.visualizer, camera_matrix, dist_coeffs,
                                  config_.detector.dictionary)) {
        throw std::runtime_error("failed to initialize debug visualizer");
      }
    }

    // 订阅
    image_sub_ = nh_.subscribe(config_.image_topic, 5, &ArucoRelocNode::onImage, this);
    command_sub_ = nh_.subscribe(config_.command_topic, 5, &ArucoRelocNode::onCommand, this);

    // 发布
    result_pub_ = nh_.advertise<deeplumin_msgs::RelocalizationResult>(config_.result_topic, 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.pose_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(config_.status_topic, 1, true);

    ROS_INFO("[aruco_reloc] node started");
    ROS_INFO("[aruco_reloc] marker_map=%s (%zu markers)", config_.marker_map_path.c_str(),
             map_loader_.size());
    ROS_INFO("[aruco_reloc] result_topic=%s command_topic=%s",
             config_.result_topic.c_str(), config_.command_topic.c_str());
    ROS_INFO("[aruco_reloc] solver: use_multi_marker=%d", config_.solver.use_multi_marker);
    ROS_INFO("[aruco_reloc] pc_verify: enabled=%d use_vgicp=%d",
             config_.pc_verify.enabled, config_.pc_verify.use_vgicp);
    ROS_INFO("[aruco_reloc] visualizer: enabled=%d", config_.visualizer.enabled);
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

  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!enabled_.load()) {
      ROS_WARN_THROTTLE(5.0, "[aruco_reloc] disabled, skipping cloud");
      return;
    }
    if (config_.pc_verify.enabled) {
      verifier_.onCloud(msg);
    }
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg) {
    if (!enabled_.load()) {
      ROS_WARN_THROTTLE(5.0, "[aruco_reloc] disabled, skipping image");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    try {
    // 1. 检测 marker
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[aruco_reloc] cv_bridge error: %s", e.what());
      return;
    }

    // 图像旋转
    if (config_.image_rotation == 1) {
      cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
    } else if (config_.image_rotation == 2) {
      cv::rotate(image, image, cv::ROTATE_180);
    } else if (config_.image_rotation == 3) {
      cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    auto detection = detector_.detect(image, msg->header.stamp.toSec());
    if (detection.markers.empty()) {
      ROS_WARN_THROTTLE(5.0, "[aruco_reloc] no markers detected");
      return;
    }

    // 输出检测到的 marker ID
    std::ostringstream ids;
    for (const auto& m : detection.markers) {
      ids << m.id << " ";
    }
    ROS_INFO("[aruco_reloc] detected %zu markers, ids: %s", detection.markers.size(),
             ids.str().c_str());

    // 2. 求解（单 marker 为主）
    auto solve_result = solver_.solve(detection, map_loader_);

    // 打印解算位姿
    if (solve_result.success) {
      auto t = solve_result.T_map_base.translation();
      Eigen::Quaterniond q(solve_result.T_map_base.rotation());
      double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) * 180.0 / M_PI;
      ROS_INFO("[aruco_reloc] solve: marker_id=%d reproj=%.2f | "
               "pos=[%.3f, %.3f, %.3f] yaw=%.1f° | quat=[%.4f, %.4f, %.4f, %.4f]",
               solve_result.marker_id, solve_result.reproj_error,
               t.x(), t.y(), t.z(), yaw, q.w(), q.x(), q.y(), q.z());
    } else {
      ROS_INFO("[aruco_reloc] solve failed: %s", solve_result.failure_reason.c_str());
    }

    if (!solve_result.success) {
      // 发布调试图像（即使失败也显示检测结果）
      if (config_.visualizer.enabled) {
        visualizer_.publishDebugImage(image, detection, solve_result, 0);
      }
      publishResult(msg->header.stamp, solve_result, "solve_failed", {}, solve_result.T_map_base);
      return;
    }

    // 3. 校验（ArUco 几何校验）
    const Eigen::Isometry3d* last_pose_ptr = has_reloc_pose_ ? &last_reloc_pose_ : nullptr;
    auto validation = validator_.validate(solve_result, detection, map_loader_, last_pose_ptr);
    ROS_INFO("[aruco_reloc] validation: passed=%d, reason=%s, reproj=%.2f",
             validation.passed, validation.reject_reason.c_str(), validation.mean_reproj_error);

    if (!validation.passed) {
      if (config_.visualizer.enabled) {
        visualizer_.publishDebugImage(image, detection, solve_result, solve_result.reproj_error);
      }
      publishResult(msg->header.stamp, solve_result, validation.reject_reason, {}, solve_result.T_map_base);
      return;
    }

    // 4. 点云验证（可选，使用图像时间戳同步点云）
    aruco_reloc::PointCloudVerifyResult pc_result;
    Eigen::Isometry3d final_pose = solve_result.T_map_base;

    if (config_.pc_verify.enabled) {
      pc_result = verifier_.verify(solve_result.T_map_base, msg->header.stamp.toSec());
      ROS_INFO("[aruco_reloc] pc_verify: valid=%d, vgicp=%d, inlier_ratio=%.3f, inlier_rmse=%.3f",
               pc_result.valid, pc_result.vgicp_converged, pc_result.inlier_ratio, pc_result.inlier_rmse);

      // 发布点云可视化
      if (config_.visualizer.enabled) {
        visualizer_.publishPointClouds(pc_result);
      }

      if (!pc_result.valid) {
        if (config_.visualizer.enabled) {
          visualizer_.publishDebugImage(image, detection, solve_result, solve_result.reproj_error);
        }
        publishResult(msg->header.stamp, solve_result, pc_result.reject_reason, pc_result, solve_result.T_map_base);
        return;
      }

      // 使用 VGICP 精化后的位姿
      if (config_.pc_verify.use_vgicp && pc_result.vgicp_converged) {
        final_pose = pc_result.T_map_base_refined;
      }
    }

    // 5. 发布调试图像
    if (config_.visualizer.enabled) {
      visualizer_.publishDebugImage(image, detection, solve_result, solve_result.reproj_error);
    }

    // 6. 成功：更新位姿并发布
    last_reloc_pose_ = final_pose;
    has_reloc_pose_ = true;

    publishResult(msg->header.stamp, solve_result, "", pc_result, final_pose);
    publishPose(msg->header.stamp, final_pose);

    ROS_INFO("[aruco_reloc] relocalization success: marker_id=%d, reproj=%.2f, "
             "inlier_ratio=%.3f, inlier_rmse=%.3f, time=%.1fms",
             solve_result.marker_id, solve_result.reproj_error,
             pc_result.inlier_ratio, pc_result.inlier_rmse, solve_result.solve_time_ms);

    } catch (const std::exception& e) {
      ROS_ERROR("[aruco_reloc] onImage exception: %s", e.what());
    } catch (...) {
      ROS_ERROR("[aruco_reloc] onImage unknown exception");
    }
  }

  void publishResult(const ros::Time& stamp, const aruco_reloc::SolveResult& solve_result,
                     const std::string& reject_reason,
                     const aruco_reloc::PointCloudVerifyResult& pc_result,
                     const Eigen::Isometry3d& final_pose) {
    deeplumin_msgs::RelocalizationResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.child_frame_id = config_.child_frame_id;

    // 设置位姿
    msg.pose.pose.position.x = final_pose.translation().x();
    msg.pose.pose.position.y = final_pose.translation().y();
    msg.pose.pose.position.z = final_pose.translation().z();
    Eigen::Quaterniond q(final_pose.rotation());
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();

    // 设置协方差
    for (auto& v : msg.pose.covariance) v = 0.0;
    msg.pose.covariance[0] = 0.01;
    msg.pose.covariance[7] = 0.01;
    msg.pose.covariance[14] = 0.01;
    msg.pose.covariance[21] = 0.001;
    msg.pose.covariance[28] = 0.001;
    msg.pose.covariance[35] = 0.001;

    msg.success = solve_result.success && reject_reason.empty();
    msg.accepted = false;
    msg.method = "aruco";
    msg.reject_reason = reject_reason;
    msg.inlier_count = solve_result.num_markers_used;
    msg.elapsed_ms = solve_result.solve_time_ms;

    // 填充点云验证结果
    if (config_.pc_verify.enabled) {
      msg.inlier_ratio = pc_result.inlier_ratio;
      msg.inlier_rmse = pc_result.inlier_rmse;
      msg.fitness_score = pc_result.inlier_rmse;
    } else {
      msg.inlier_ratio = 0.0;
      msg.inlier_rmse = 0.0;
      msg.fitness_score = 0.0;
    }

    msg.map_id = config_.marker_map_path;
    result_pub_.publish(msg);

    // 发布状态
    std_msgs::String status;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "success=" << msg.success
       << " marker_id=" << solve_result.marker_id
       << " reproj=" << solve_result.reproj_error
       << " time_ms=" << solve_result.solve_time_ms;
    if (config_.pc_verify.enabled) {
      ss << " inlier_ratio=" << pc_result.inlier_ratio
         << " inlier_rmse=" << pc_result.inlier_rmse;
    }
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
  ros::Subscriber cloud_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher result_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  
  std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_;

  aruco_reloc::OnlineNodeConfig config_;
  aruco_reloc::MarkerDetector detector_;
  aruco_reloc::MarkerMapLoader map_loader_;
  aruco_reloc::RelocalizationSolver solver_;
  aruco_reloc::RelocalizationValidator validator_;
  aruco_reloc::PointCloudVerifier verifier_;
  aruco_reloc::DebugVisualizer visualizer_;

  Eigen::Isometry3d T_base_cam_ = Eigen::Isometry3d::Identity();

  mutable std::mutex mutex_;
  Eigen::Isometry3d last_reloc_pose_ = Eigen::Isometry3d::Identity();
  bool has_reloc_pose_ = false;
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
