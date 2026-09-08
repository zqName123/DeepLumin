#include <aruco_relocalization/mapping/offline_mapper.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>

namespace aruco_reloc {

bool OfflineMapper::initialize(const OfflineMappingConfig& config) {
  config_ = config;

  // 构造 T_base_cam = T_base_lidar * T_lidar_cam
  // T_base_lidar = (lidar_base_to_base)^(-1)
  Eigen::Isometry3d T_lidar_base_base = Eigen::Isometry3d::Identity();
  T_lidar_base_base.translation() = config_.lidar_base_to_base.translation;
  T_lidar_base_base.rotate(config_.lidar_base_to_base.rotation);

  // T_lidar_cam (lidar_to_camera)
  Eigen::Isometry3d T_lidar_cam = Eigen::Isometry3d::Identity();
  T_lidar_cam.translation() = config_.extrinsic.translation;
  T_lidar_cam.rotate(config_.extrinsic.rotation);

  // T_base_cam = T_base_lidar * T_lidar_cam
  T_base_cam_ = T_lidar_base_base.inverse() * T_lidar_cam;

  initialized_ = true;
  ROS_INFO("[aruco_reloc] offline mapper initialized");
  ROS_INFO("[aruco_reloc] T_base_cam: translation=[%.3f, %.3f, %.3f]",
           T_base_cam_.translation().x(),
           T_base_cam_.translation().y(),
           T_base_cam_.translation().z());
  return true;
}

void OfflineMapper::setCameraIntrinsics(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (camera_info_received_) {
    ROS_WARN("[offline_mapping] camera info already received, ignoring config file values");
    return;
  }

  DetectorConfig det_cfg = config_.detector;
  detector_.initialize(det_cfg, camera_matrix, dist_coeffs);

  camera_info_received_ = true;
  ROS_INFO("[offline_mapping] camera intrinsics loaded from config file, detector ready");
}

void OfflineMapper::onPath(const nav_msgs::Path::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  keyframe_poses_.clear();
  for (const auto& pose_stamped : msg->poses) {
    KeyframePose kf;
    kf.timestamp = pose_stamped.header.stamp.toSec();
    kf.T_map_base = Eigen::Isometry3d::Identity();
    kf.T_map_base.translation() = Eigen::Vector3d(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z);
    Eigen::Quaterniond q(pose_stamped.pose.orientation.w,
                         pose_stamped.pose.orientation.x,
                         pose_stamped.pose.orientation.y,
                         pose_stamped.pose.orientation.z);
    if (std::abs(q.norm() - 1.0) < 0.1) {
      q.normalize();
      kf.T_map_base.rotate(q);
    }
    keyframe_poses_.push_back(kf);
  }

  std::sort(keyframe_poses_.begin(), keyframe_poses_.end(),
            [](const auto& a, const auto& b) { return a.timestamp < b.timestamp; });

  ROS_INFO_THROTTLE(2.0, "[offline_mapping] received %zu keyframe poses", keyframe_poses_.size());
}

void OfflineMapper::onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  if (camera_info_received_) return;

  std::lock_guard<std::mutex> lock(mutex_);

  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      camera_matrix.at<double>(i, j) = msg->K[i * 3 + j];
    }
  }

  cv::Mat dist_coeffs;
  if (msg->D.size() > 0) {
    dist_coeffs = cv::Mat(msg->D).reshape(1, 1);
  } else {
    dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  }

  DetectorConfig det_cfg = config_.detector;
  detector_.initialize(det_cfg, camera_matrix, dist_coeffs);

  camera_info_received_ = true;
  ROS_INFO("[offline_mapping] camera info received, detector ready");
}

void OfflineMapper::onImage(const sensor_msgs::Image::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!camera_info_received_) {
    ROS_WARN_THROTTLE(5.0, "[offline_mapping] waiting for camera info");
    return;
  }

  // 如果没有 SLAM 轨迹，使用单位矩阵作为位姿（用于测试）
  if (keyframe_poses_.empty()) {
    ROS_WARN_THROTTLE(5.0, "[offline_mapping] no SLAM trajectory, using identity pose for testing");
  }

  // 1. 检测 marker
  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("[offline_mapping] cv_bridge error: %s", e.what());
    return;
  }

  // 图像旋转处理（与 slam_mapping_v2 保持一致）
  if (config_.image_rotation != 0) {
    cv::RotateFlags flag;
    switch (config_.image_rotation) {
      case 1:
        flag = cv::ROTATE_90_CLOCKWISE;
        break;
      case 2:
        flag = cv::ROTATE_180;
        break;
      case 3:
        flag = cv::ROTATE_90_COUNTERCLOCKWISE;
        break;
      default:
        flag = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    cv::rotate(image, image, flag);
  }

  double image_time = msg->header.stamp.toSec();
  auto detection = detector_.detect(image, image_time);

  if (detection.markers.empty()) {
    return;
  }

  // 2. 找最近的关键帧位姿（如果没有 SLAM 轨迹，使用单位矩阵）
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  if (!keyframe_poses_.empty()) {
    const KeyframePose* best_kf = findNearestKeyframe(image_time);
    if (!best_kf) {
      ROS_WARN_THROTTLE(2.0, "[offline_mapping] no suitable keyframe found for time %.3f",
                        image_time);
      return;
    }

    double time_diff = std::abs(best_kf->timestamp - image_time);
    if (time_diff > config_.time_sync_tolerance) {
      ROS_WARN_THROTTLE(2.0, "[offline_mapping] time diff too large: %.3f sec", time_diff);
      return;
    }
    T_map_base = best_kf->T_map_base;
  }

  // 3. 计算每个 marker 在 map 下的位姿并聚合
  for (const auto& obs : detection.markers) {
    // T_cam_marker 从检测结果获取
    Eigen::Isometry3d T_cam_marker = obs.T_cam_marker;

    // T_map_marker = T_map_base * T_base_cam * T_cam_marker
    Eigen::Isometry3d T_map_marker = T_map_base * T_base_cam_ * T_cam_marker;

    aggregateMarker(obs.id, T_map_marker, obs.reprojection_error);
  }

  ROS_INFO_THROTTLE(5.0, "[offline_mapping] processed %zu markers, %zu unique IDs",
                    detection.markers.size(), marker_observations_.size());
}

void OfflineMapper::aggregateMarker(int id, const Eigen::Isometry3d& T_map_marker,
                                    double reproj_error) {
  if (reproj_error > config_.max_reproj_error) {
    return;
  }

  auto& obs_list = marker_observations_[id];
  obs_list.push_back({T_map_marker, reproj_error});
}

const KeyframePose* OfflineMapper::findNearestKeyframe(double timestamp) const {
  if (keyframe_poses_.empty()) return nullptr;

  auto it = std::lower_bound(keyframe_poses_.begin(), keyframe_poses_.end(), timestamp,
                             [](const auto& kf, double t) { return kf.timestamp < t; });

  if (it == keyframe_poses_.end()) {
    return &keyframe_poses_.back();
  }
  if (it == keyframe_poses_.begin()) {
    return &keyframe_poses_.front();
  }

  auto prev = std::prev(it);
  if (std::abs(prev->timestamp - timestamp) < std::abs(it->timestamp - timestamp)) {
    return &(*prev);
  }
  return &(*it);
}

void OfflineMapper::saveMarkerMap() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (marker_observations_.empty()) {
    ROS_WARN("[offline_mapping] no marker observations, nothing to save");
    return;
  }

  YAML::Node root;
  root["frame_id"] = "map";
  root["source_pose_topic"] = config_.path_topic;
  root["generation_time"] = std::to_string(ros::Time::now().toSec());

  YAML::Node markers_node;
  int saved_count = 0;

  for (const auto& [id, obs_list] : marker_observations_) {
    if (static_cast<int>(obs_list.size()) < config_.min_observations) {
      ROS_WARN("[offline_mapping] marker %d has only %zu observations (min %d), skip",
               id, obs_list.size(), config_.min_observations);
      continue;
    }

    // 位置加权平均
    Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    for (const auto& obs : obs_list) {
      double weight = 1.0 / std::max(0.1, obs.reproj_error);
      pos_sum += obs.T_map_marker.translation() * weight;
      weight_sum += weight;
    }
    Eigen::Vector3d pos_avg = pos_sum / weight_sum;

    // 旋转加权平均（简单平均，假设误差较小）
    Eigen::Quaterniond q_sum(0, 0, 0, 0);
    Eigen::Quaterniond q_first = Eigen::Quaterniond(obs_list[0].T_map_marker.rotation());
    for (const auto& obs : obs_list) {
      Eigen::Quaterniond q(obs.T_map_marker.rotation());
      // 处理双覆盖：确保四元数在同一半球
      if (q.dot(q_first) < 0) {
        q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
      }
      double weight = 1.0 / std::max(0.1, obs.reproj_error);
      q_sum.w() += q.w() * weight;
      q_sum.x() += q.x() * weight;
      q_sum.y() += q.y() * weight;
      q_sum.z() += q.z() * weight;
    }
    q_sum.normalize();

    // 计算平均重投影误差
    double error_sum = 0.0;
    for (const auto& obs : obs_list) {
      error_sum += obs.reproj_error;
    }
    double mean_error = error_sum / obs_list.size();

    // 构建 YAML 节点
    YAML::Node marker_node;
    marker_node["id"] = id;
    marker_node["x"] = pos_avg.x();
    marker_node["y"] = pos_avg.y();
    marker_node["z"] = pos_avg.z();
    marker_node["qw"] = q_sum.w();
    marker_node["qx"] = q_sum.x();
    marker_node["qy"] = q_sum.y();
    marker_node["qz"] = q_sum.z();
    marker_node["size"] = config_.detector.marker_size;
    marker_node["observation_count"] = static_cast<int>(obs_list.size());
    marker_node["mean_reproj_error"] = mean_error;

    markers_node.push_back(marker_node);
    saved_count++;
  }

  root["markers"] = markers_node;

  // 写入文件
  std::string output_path = config_.output_path;
  if (output_path.empty()) {
    output_path = "marker_map.yaml";
  }

  try {
    std::ofstream fout(output_path);
    fout << root;
    fout.close();
    ROS_INFO("[offline_mapping] saved %d markers to %s", saved_count, output_path.c_str());
  } catch (const std::exception& e) {
    ROS_ERROR("[offline_mapping] failed to save marker map: %s", e.what());
  }
}

size_t OfflineMapper::getMarkerCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return marker_observations_.size();
}

size_t OfflineMapper::getObservationCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  size_t total = 0;
  for (const auto& [_, obs_list] : marker_observations_) {
    total += obs_list.size();
  }
  return total;
}

}  // namespace aruco_reloc
