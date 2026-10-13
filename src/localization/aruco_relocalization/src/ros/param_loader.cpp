#include <aruco_relocalization/ros/param_loader.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

namespace aruco_reloc {

namespace {

Eigen::Isometry3d loadTransform(ros::NodeHandle& pnh, const std::string& key) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  std::vector<double> translation;
  if (pnh.getParam(key + "/translation", translation) && translation.size() == 3) {
    T.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  }

  std::vector<double> rotation;
  if (pnh.getParam(key + "/rotation", rotation) && rotation.size() == 4) {
    Eigen::Quaterniond q(rotation[0], rotation[1], rotation[2], rotation[3]);
    if (std::abs(q.norm() - 1.0) < 0.1) {
      q.normalize();
      T.rotate(q);
    }
  }

  return T;
}

}  // namespace

OnlineNodeConfig loadOnlineNodeConfig(ros::NodeHandle& pnh) {
  OnlineNodeConfig cfg;

  // 话题
  pnh.param<std::string>("image_topic", cfg.image_topic, "/camera/image_raw");
  pnh.param<std::string>("result_topic", cfg.result_topic, "/v_reloc/result");
  pnh.param<std::string>("pose_topic", cfg.pose_topic, "/aruco_reloc/pose");
  pnh.param<std::string>("status_topic", cfg.status_topic, "/aruco_reloc/status");
  pnh.param<std::string>("command_topic", cfg.command_topic, "/v_reloc/command");
  pnh.param<std::string>("debug_image_topic", cfg.debug_image_topic, "/aruco_reloc/debug_image");

  // 坐标系
  pnh.param<std::string>("frame_id", cfg.frame_id, "map");
  pnh.param<std::string>("child_frame_id", cfg.child_frame_id, "base_link");

  // 图像旋转
  pnh.param<int>("image_rotation", cfg.image_rotation, 0);

  // 路径
  pnh.param<std::string>("marker_map_path", cfg.marker_map_path, "");

  // 外参（base_link -> lidar）
  cfg.base_to_lidar.translation = Eigen::Vector3d::Zero();
  cfg.base_to_lidar.rotation = Eigen::Quaterniond::Identity();
  if (pnh.hasParam("extrinsic/base_to_lidar")) {
    auto T = loadTransform(pnh, "extrinsic/base_to_lidar");
    cfg.base_to_lidar.translation = T.translation();
    cfg.base_to_lidar.rotation = Eigen::Quaterniond(T.rotation());
  }

  // 外参（lidar -> camera）
  cfg.lidar_to_camera.translation = Eigen::Vector3d::Zero();
  cfg.lidar_to_camera.rotation = Eigen::Quaterniond::Identity();
  if (pnh.hasParam("extrinsic/lidar_to_camera")) {
    auto T = loadTransform(pnh, "extrinsic/lidar_to_camera");
    cfg.lidar_to_camera.translation = T.translation();
    cfg.lidar_to_camera.rotation = Eigen::Quaterniond(T.rotation());
  }

  pnh.param<bool>("use_tf_extrinsic", cfg.use_tf_extrinsic, false);

  // 检测器配置
  pnh.param<std::string>("aruco/dictionary", cfg.detector.dictionary, "DICT_4X4_50");
  pnh.param<double>("aruco/marker_size", cfg.detector.marker_size, 0.2);
  pnh.param<bool>("aruco/corner_subpix", cfg.detector.corner_subpix, true);

  // 求解器配置
  pnh.param<int>("solver/min_markers", cfg.solver.min_markers, 1);
  pnh.param<bool>("solver/use_multi_marker", cfg.solver.use_multi_marker, false);
  pnh.param<bool>("solver/use_ransac", cfg.solver.use_ransac, true);
  pnh.param<int>("solver/ransac_iterations", cfg.solver.ransac_iterations, 100);
  pnh.param<double>("solver/ransac_reproj_error", cfg.solver.ransac_reproj_error, 8.0);
  pnh.param<double>("solver/ransac_confidence", cfg.solver.ransac_confidence, 0.99);

  // 校验器配置
  pnh.param<double>("validator/max_reproj_error", cfg.validator.max_reproj_error, 10.0);
  pnh.param<double>("validator/max_marker_distance_error", cfg.validator.max_marker_distance_error, 0.1);
  pnh.param<double>("validator/max_translation_jump", cfg.validator.max_translation_jump, 2.0);
  pnh.param<double>("validator/max_rotation_jump", cfg.validator.max_rotation_jump, 30.0);
  pnh.param<bool>("validator/use_motion_check", cfg.validator.use_motion_check, true);
  pnh.param<int>("validator/min_markers", cfg.validator.min_markers, 2);

  // 点云验证配置
  pnh.param<bool>("pointcloud_verify/enabled", cfg.pc_verify.enabled, false);
  pnh.param<bool>("pointcloud_verify/use_vgicp", cfg.pc_verify.use_vgicp, true);
  pnh.param<std::string>("pointcloud_verify/cloud_topic", cfg.pc_verify.cloud_topic, "/localization/points");
  pnh.param<bool>("pointcloud_verify/rotate_xy_y_negx", cfg.pc_verify.rotate_xy_y_negx, true);
  pnh.param<double>("pointcloud_verify/query_yaw_deg", cfg.pc_verify.query_yaw_deg, 0.0);
  if (pnh.hasParam("pointcloud_verify/base_to_lidar")) {
    auto T = loadTransform(pnh, "pointcloud_verify/base_to_lidar");
    cfg.pc_verify.base_to_lidar.translation = T.translation();
    cfg.pc_verify.base_to_lidar.rotation = Eigen::Quaterniond(T.rotation());
  }
  pnh.param<std::string>("pointcloud_verify/global_map_pcd", cfg.pc_verify.global_map_pcd, "");
  pnh.param<double>("pointcloud_verify/crop_forward", cfg.pc_verify.crop_forward, 30.0);
  pnh.param<double>("pointcloud_verify/crop_backward", cfg.pc_verify.crop_backward, 30.0);
  pnh.param<double>("pointcloud_verify/crop_left", cfg.pc_verify.crop_left, 30.0);
  pnh.param<double>("pointcloud_verify/crop_right", cfg.pc_verify.crop_right, 30.0);
  pnh.param<double>("pointcloud_verify/crop_down", cfg.pc_verify.crop_down, 5.0);
  pnh.param<double>("pointcloud_verify/crop_up", cfg.pc_verify.crop_up, 5.0);
  pnh.param<double>("pointcloud_verify/voxel_leaf", cfg.pc_verify.voxel_leaf, 0.5);
  pnh.param<double>("pointcloud_verify/min_range", cfg.pc_verify.min_range, 3.0);
  pnh.param<double>("pointcloud_verify/max_range", cfg.pc_verify.max_range, 50.0);
  pnh.param<bool>("pointcloud_verify/z_filter_enable", cfg.pc_verify.z_filter_enable, true);
  pnh.param<double>("pointcloud_verify/z_filter_min", cfg.pc_verify.z_filter_min, -1.0);
  pnh.param<double>("pointcloud_verify/z_filter_max", cfg.pc_verify.z_filter_max, 5.0);
  pnh.param<double>("pointcloud_verify/inlier_distance", cfg.pc_verify.inlier_distance, 0.5);
  pnh.param<double>("pointcloud_verify/min_inlier_ratio", cfg.pc_verify.min_inlier_ratio, 0.5);
  pnh.param<double>("pointcloud_verify/max_inlier_rmse", cfg.pc_verify.max_inlier_rmse, 0.5);

  // 可视化配置
  pnh.param<bool>("visualizer/enabled", cfg.visualizer.enabled, false);
  pnh.param<std::string>("visualizer/debug_image_topic", cfg.visualizer.debug_image_topic, "/aruco_reloc/debug_image");
  pnh.param<std::string>("visualizer/source_cloud_topic", cfg.visualizer.source_cloud_topic, "/aruco_reloc/source_cloud");
  pnh.param<std::string>("visualizer/submap_topic", cfg.visualizer.submap_topic, "/aruco_reloc/submap");
  pnh.param<std::string>("visualizer/aligned_cloud_topic", cfg.visualizer.aligned_cloud_topic, "/aruco_reloc/aligned_cloud");

  return cfg;
}

OfflineMappingConfig loadOfflineMappingConfig(ros::NodeHandle& pnh) {
  OfflineMappingConfig cfg;

  // 话题
  pnh.param<std::string>("path_topic", cfg.path_topic, "/slam_mapping_v2/path");
  pnh.param<std::string>("image_topic", cfg.image_topic, "/camera/image_raw");
  pnh.param<std::string>("camera_info_topic", cfg.camera_info_topic, "/camera/camera_info");

  // 输出
  pnh.param<std::string>("output_path", cfg.output_path, "");

  // 图像旋转
  pnh.param<int>("image_rotation", cfg.image_rotation, 3);

  // 外参（支持 lidar_to_camera 或 base_to_camera）
  cfg.extrinsic.translation = Eigen::Vector3d::Zero();
  cfg.extrinsic.rotation = Eigen::Quaterniond::Identity();
  if (pnh.hasParam("extrinsic/lidar_to_camera")) {
    auto T = loadTransform(pnh, "extrinsic/lidar_to_camera");
    cfg.extrinsic.translation = T.translation();
    cfg.extrinsic.rotation = Eigen::Quaterniond(T.rotation());
  } else if (pnh.hasParam("extrinsic/base_to_camera")) {
    auto T = loadTransform(pnh, "extrinsic/base_to_camera");
    cfg.extrinsic.translation = T.translation();
    cfg.extrinsic.rotation = Eigen::Quaterniond(T.rotation());
  }

  // lidar_base 到 base_link 的变换
  cfg.lidar_base_to_base.translation = Eigen::Vector3d::Zero();
  cfg.lidar_base_to_base.rotation = Eigen::Quaterniond::Identity();
  if (pnh.hasParam("lidar_base_to_base")) {
    auto T = loadTransform(pnh, "lidar_base_to_base");
    cfg.lidar_base_to_base.translation = T.translation();
    cfg.lidar_base_to_base.rotation = Eigen::Quaterniond(T.rotation());
  }

  // 同步参数
  pnh.param<double>("time_sync_tolerance", cfg.time_sync_tolerance, 0.5);

  // 检测器配置
  pnh.param<std::string>("aruco/dictionary", cfg.detector.dictionary, "DICT_4X4_50");
  pnh.param<double>("aruco/marker_size", cfg.detector.marker_size, 0.2);
  pnh.param<bool>("aruco/corner_subpix", cfg.detector.corner_subpix, true);

  // 聚合参数
  pnh.param<int>("min_observations", cfg.min_observations, 5);
  pnh.param<double>("max_reproj_error", cfg.max_reproj_error, 5.0);

  return cfg;
}

bool loadCameraIntrinsics(ros::NodeHandle& pnh, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
  std::vector<double> K, D;

  if (pnh.getParam("camera_matrix", K) && K.size() == 9) {
    camera_matrix = (cv::Mat_<double>(3, 3) <<
        K[0], K[1], K[2],
        K[3], K[4], K[5],
        K[6], K[7], K[8]);
  } else {
    // 默认值
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    ROS_WARN("[aruco_reloc] camera_matrix not set, using identity");
  }

  if (pnh.getParam("dist_coeffs", D)) {
    dist_coeffs = cv::Mat(D).reshape(1, 1);
  } else {
    dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  }

  return true;
}

Eigen::Isometry3d loadExtrinsic(ros::NodeHandle& pnh, const std::string& key) {
  return loadTransform(pnh, key);
}

}  // namespace aruco_reloc
