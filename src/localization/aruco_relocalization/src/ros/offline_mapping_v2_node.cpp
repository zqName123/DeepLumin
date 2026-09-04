/**
 * @file offline_mapping_v2_node.cpp
 * @brief 离线建图 V2 节点：读取关键帧图像和位姿，遍历检测 marker，输出 marker map
 */

#include <aruco_relocalization/offline_mapping_v2/offline_mapping_v2.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offline_mapping_v2_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 加载配置
  aruco_reloc::OfflineMappingV2Config config;

  // 输入路径（从 offline_mapping_v2 配置节读取）
  pnh.param<std::string>("offline_mapping_v2/keyframe_poses_csv", config.keyframe_poses_csv, "");
  pnh.param<std::string>("offline_mapping_v2/keyframe_images_dir", config.keyframe_images_dir, "");

  // 输出路径
  pnh.param<std::string>("offline_mapping_v2/output_marker_map", config.output_marker_map,
                          "$(find aruco_relocalization)/config/marker_map.yaml");

  // 图像处理
  pnh.param<int>("image_rotation", config.image_rotation, 0);

  // 校验参数
  pnh.param<double>("offline_mapping_v2/max_reproj_error", config.max_reproj_error, 5.0);
  pnh.param<int>("offline_mapping_v2/min_observations", config.min_observations, 3);

  // 检测器配置
  pnh.param<std::string>("aruco/dictionary", config.detector.dictionary, "DICT_4X4_50");
  pnh.param<double>("aruco/marker_size", config.detector.marker_size, 0.2);
  pnh.param<bool>("aruco/corner_subpix", config.detector.corner_subpix, true);

  // 外参配置（从公有命名空间读取，因为 launch 加载到公有空间）
  config.T_base_lidar = Eigen::Isometry3d::Identity();
  if (nh.hasParam("extrinsic/base_to_lidar")) {
    config.T_base_lidar = aruco_reloc::loadExtrinsic(nh, "extrinsic/base_to_lidar");
  }

  config.T_lidar_camera = Eigen::Isometry3d::Identity();
  if (nh.hasParam("extrinsic/lidar_to_camera")) {
    config.T_lidar_camera = aruco_reloc::loadExtrinsic(nh, "extrinsic/lidar_to_camera");
  }

  // 检查必需的参数
  if (config.keyframe_poses_csv.empty()) {
    ROS_ERROR("[offline_mapping_v2] keyframe_poses_csv parameter is required");
    return 1;
  }
  if (config.keyframe_images_dir.empty()) {
    ROS_ERROR("[offline_mapping_v2] keyframe_images_dir parameter is required");
    return 1;
  }

  // 加载相机内参（从公有空间读取）
  cv::Mat camera_matrix, dist_coeffs;
  std::vector<double> K, D;
  if (nh.getParam("camera_matrix", K) && K.size() == 9) {
    camera_matrix = (cv::Mat_<double>(3, 3) <<
        K[0], K[1], K[2],
        K[3], K[4], K[5],
        K[6], K[7], K[8]);
  } else {
    ROS_ERROR("[offline_mapping_v2] camera_matrix not found");
    return 1;
  }

  if (nh.getParam("dist_coeffs", D)) {
    dist_coeffs = cv::Mat(D).reshape(1, 1);
  } else {
    dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  }

  // 检查内参是否有效
  if (camera_matrix.at<double>(0, 0) == 1.0 && camera_matrix.at<double>(1, 1) == 1.0) {
    ROS_ERROR("[offline_mapping_v2] camera matrix is identity, please configure camera intrinsics");
    return 1;
  }

  ROS_INFO("[offline_mapping_v2] starting offline mapping");
  ROS_INFO("[offline_mapping_v2] keyframe_poses_csv: %s", config.keyframe_poses_csv.c_str());
  ROS_INFO("[offline_mapping_v2] keyframe_images_dir: %s", config.keyframe_images_dir.c_str());
  ROS_INFO("[offline_mapping_v2] output_marker_map: %s", config.output_marker_map.c_str());

  // 创建离线建图对象
  aruco_reloc::OfflineMappingV2 mapper;
  if (!mapper.initialize(config, camera_matrix, dist_coeffs)) {
    ROS_ERROR("[offline_mapping_v2] failed to initialize");
    return 1;
  }

  // 处理关键帧
  if (!mapper.process()) {
    ROS_ERROR("[offline_mapping_v2] failed to process");
    return 1;
  }

  // 保存 marker map
  if (!mapper.saveMarkerMap()) {
    ROS_ERROR("[offline_mapping_v2] failed to save marker map");
    return 1;
  }

  ROS_INFO("[offline_mapping_v2] completed successfully");
  ROS_INFO("[offline_mapping_v2] processed %d keyframes, found %d markers",
           mapper.getKeyframeCount(), mapper.getMarkerCount());

  return 0;
}
