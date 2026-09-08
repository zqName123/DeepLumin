#include <aruco_relocalization/offline_mapping_v2/offline_mapping_v2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace aruco_reloc {

bool OfflineMappingV2::initialize(const OfflineMappingV2Config& config,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& dist_coeffs) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();

  // 初始化检测器
  detector_.initialize(config_.detector, camera_matrix_, dist_coeffs_);

  // 调试：打印相机内参和 marker_size
  ROS_INFO("[offline_mapping_v2] camera_matrix: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
           camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
           camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
  ROS_INFO("[offline_mapping_v2] marker_size=%.3f dictionary=%s",
           config_.detector.marker_size, config_.detector.dictionary.c_str());

  // 读取关键帧位姿
  if (!loadKeyframePoses(config_.keyframe_poses_csv)) {
    ROS_ERROR("[offline_mapping_v2] failed to load keyframe poses from %s",
              config_.keyframe_poses_csv.c_str());
    return false;
  }

  ROS_INFO("[offline_mapping_v2] initialized with %zu keyframes", keyframes_.size());
  return true;
}

bool OfflineMappingV2::loadKeyframePoses(const std::string& csv_path) {
  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    ROS_ERROR("[offline_mapping_v2] failed to open %s", csv_path.c_str());
    return false;
  }

  keyframes_.clear();
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') continue;

    // 解析 CSV: id,timestamp,x,y,z,qx,qy,qz,qw
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() < 9) continue;

    try {
      KeyframeData kf;
      kf.id = std::stoi(tokens[0]);
      kf.timestamp = std::stod(tokens[1]);

      double x = std::stod(tokens[2]);
      double y = std::stod(tokens[3]);
      double z = std::stod(tokens[4]);
      double qx = std::stod(tokens[5]);
      double qy = std::stod(tokens[6]);
      double qz = std::stod(tokens[7]);
      double qw = std::stod(tokens[8]);

      kf.T_map_base = Eigen::Isometry3d::Identity();
      kf.T_map_base.translation() = Eigen::Vector3d(x, y, z);
      kf.T_map_base.rotate(Eigen::Quaterniond(qw, qx, qy, qz));

      keyframes_.push_back(kf);
    } catch (const std::exception& e) {
      ROS_WARN("[offline_mapping_v2] failed to parse line: %s", line.c_str());
      continue;
    }
  }

  ROS_INFO("[offline_mapping_v2] loaded %zu keyframes from %s", keyframes_.size(), csv_path.c_str());
  return !keyframes_.empty();
}

cv::Mat OfflineMappingV2::loadKeyframeImage(int keyframe_id) {
  std::string image_path = config_.keyframe_images_dir + "/" + std::to_string(keyframe_id) + ".png";

  if (!std::filesystem::exists(image_path)) {
    // 尝试其他格式
    image_path = config_.keyframe_images_dir + "/" + std::to_string(keyframe_id) + ".jpg";
    if (!std::filesystem::exists(image_path)) {
      return cv::Mat();
    }
  }
  // 读取图像, IMREAD_COLOR: 读取为 BGR 格式
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  return image;
}

bool OfflineMappingV2::process() {
  if (keyframes_.empty()) {
    ROS_ERROR("[offline_mapping_v2] no keyframes to process");
    return false;
  }

  ROS_INFO("[offline_mapping_v2] processing %zu keyframes...", keyframes_.size());

  int processed_count = 0;
  int markers_detected = 0;

  for (const auto& kf : keyframes_) {
    if (processKeyframe(kf)) {
      processed_count++;
    }
  }

  // 聚合 marker 观测
  aggregateMarkerObservations();

  ROS_INFO("[offline_mapping_v2] processed %d/%zu keyframes, found %zu markers",
           processed_count, keyframes_.size(), marker_aggregates_.size());

  return true;
}

bool OfflineMappingV2::processKeyframe(const KeyframeData& kf) {
  // 读取图像
  cv::Mat image = loadKeyframeImage(kf.id);
  if (image.empty()) {
    ROS_WARN_THROTTLE(5.0, "[offline_mapping_v2] image not found for keyframe %d", kf.id);
    return false;
  }

  // 图像旋转处理
  if (config_.image_rotation != 0) {
    cv::RotateFlags flag;
    switch (config_.image_rotation) {
      case 1: flag = cv::ROTATE_90_CLOCKWISE; break;
      case 2: flag = cv::ROTATE_180; break;
      case 3: flag = cv::ROTATE_90_COUNTERCLOCKWISE; break;
      default: flag = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    cv::rotate(image, image, flag);
  }

  // 检测 marker
  auto detection = detector_.detect(image, kf.timestamp);

  if (detection.markers.empty()) {
    return false;
  }

  ROS_INFO_THROTTLE(0.01, "[offline_mapping_v2] keyframe %d: detected %zu markers",
                    kf.id, detection.markers.size());

  // 调试：打印 T_map_base
  auto t = kf.T_map_base.translation();
  ROS_INFO_THROTTLE(0.01, "[offline_mapping_v2] kf %d: T_map_base pos=[%.3f, %.3f, %.3f]",
                    kf.id, t.x(), t.y(), t.z());

  // 对每个检测到的 marker，计算其在 map 坐标系下的位姿
  for (const auto& obs : detection.markers) {
    // 检查重投影误差
    if (obs.reprojection_error > config_.max_reproj_error) {
      ROS_WARN_THROTTLE(0.01, "[offline_mapping_v2] marker %d rejected: reproj_error=%.2f > %.2f",
                        obs.id, obs.reprojection_error, config_.max_reproj_error);
      continue;
    }

    // T_map_marker = T_map_base * T_base_lidar * T_lidar_camera * T_camera_marker
    Eigen::Isometry3d T_base_camera = config_.T_base_lidar * config_.T_lidar_camera;
    Eigen::Isometry3d T_map_marker = kf.T_map_base * T_base_camera * obs.T_cam_marker;

    // 调试：打印 T_camera_marker 和 T_map_marker
    auto t_cam = obs.T_cam_marker.translation();
    auto t_map = T_map_marker.translation();
    ROS_INFO("[offline_mapping_v2] kf %d marker %d: T_cam=[%.3f, %.3f, %.3f] T_map=[%.3f, %.3f, %.3f] reproj=%.2f",
             kf.id, obs.id, t_cam.x(), t_cam.y(), t_cam.z(), t_map.x(), t_map.y(), t_map.z(), obs.reprojection_error);

    // 保存观测
    MarkerObservationV2 observation;
    observation.keyframe_id = kf.id;
    observation.T_map_marker = T_map_marker;
    observation.reprojection_error = obs.reprojection_error;

    marker_aggregates_[obs.id].id = obs.id;
    marker_aggregates_[obs.id].observations.push_back(observation);
  }

  return true;
}

bool OfflineMappingV2::aggregateMarkerObservations() {
  for (auto& [id, aggregate] : marker_aggregates_) {
    if (aggregate.observations.size() < static_cast<size_t>(config_.min_observations)) {
      ROS_WARN("[offline_mapping_v2] marker %d: only %zu observations (min %d), skipping",
               id, aggregate.observations.size(), config_.min_observations);
      continue;
    }

    // 位置加权平均（权重 = 1 / reproj_error）
    Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    for (const auto& obs : aggregate.observations) {
      double weight = 1.0 / std::max(0.1, obs.reprojection_error);
      pos_sum += obs.T_map_marker.translation() * weight;
      weight_sum += weight;
    }
    Eigen::Vector3d pos_avg = pos_sum / weight_sum;

    // 旋转加权平均（四元数，处理双覆盖）
    Eigen::Quaterniond q_sum(0, 0, 0, 0);
    Eigen::Quaterniond q_first = Eigen::Quaterniond(
        aggregate.observations[0].T_map_marker.rotation());
    for (const auto& obs : aggregate.observations) {
      Eigen::Quaterniond q(obs.T_map_marker.rotation());
      // 处理双覆盖：确保四元数在同一半球
      if (q.dot(q_first) < 0) {
        q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
      }
      double weight = 1.0 / std::max(0.1, obs.reprojection_error);
      q_sum.w() += q.w() * weight;
      q_sum.x() += q.x() * weight;
      q_sum.y() += q.y() * weight;
      q_sum.z() += q.z() * weight;
    }
    q_sum.normalize();

    // 设置聚合结果
    aggregate.T_map_marker = Eigen::Isometry3d::Identity();
    aggregate.T_map_marker.translation() = pos_avg;
    aggregate.T_map_marker.rotate(q_sum);

    // 计算平均重投影误差
    double error_sum = 0.0;
    for (const auto& obs : aggregate.observations) {
      error_sum += obs.reprojection_error;
    }
    aggregate.mean_reproj_error = error_sum / aggregate.observations.size();
    aggregate.observation_count = aggregate.observations.size();

    ROS_INFO("[offline_mapping_v2] marker %d: %d observations, pos=[%.3f, %.3f, %.3f], reproj_error=%.2f",
             id, aggregate.observation_count,
             pos_avg.x(), pos_avg.y(), pos_avg.z(),
             aggregate.mean_reproj_error);
  }

  return true;
}

bool OfflineMappingV2::saveMarkerMap() {
  YAML::Node root;
  root["frame_id"] = "map";
  root["source"] = "offline_mapping_v2";
  root["generation_time"] = std::to_string(ros::Time::now().toSec());

  // 保存配置信息
  YAML::Node config_node;
  std::vector<double> K = {
    camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
    camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
    camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2)
  };
  config_node["camera_matrix"] = K;
  config_node["marker_size"] = config_.detector.marker_size;
  config_node["dictionary"] = config_.detector.dictionary;
  root["config"] = config_node;

  // 保存 markers
  YAML::Node markers_node;
  int saved_count = 0;
  for (const auto& [id, aggregate] : marker_aggregates_) {
    if (aggregate.observation_count < config_.min_observations) {
      continue;
    }

    YAML::Node marker_node;
    marker_node["id"] = id;
    marker_node["x"] = aggregate.T_map_marker.translation().x();
    marker_node["y"] = aggregate.T_map_marker.translation().y();
    marker_node["z"] = aggregate.T_map_marker.translation().z();

    Eigen::Quaterniond q(aggregate.T_map_marker.rotation());
    marker_node["qw"] = q.w();
    marker_node["qx"] = q.x();
    marker_node["qy"] = q.y();
    marker_node["qz"] = q.z();

    marker_node["size"] = config_.detector.marker_size;
    marker_node["observation_count"] = aggregate.observation_count;
    marker_node["mean_reproj_error"] = aggregate.mean_reproj_error;

    // 保存观测到该 marker 的关键帧 ID
    std::vector<int> keyframe_ids;
    for (const auto& obs : aggregate.observations) {
      keyframe_ids.push_back(obs.keyframe_id);
    }
    marker_node["keyframe_ids"] = keyframe_ids;

    markers_node.push_back(marker_node);
    saved_count++;
  }

  root["markers"] = markers_node;

  // 写入文件
  std::string output_path = config_.output_marker_map;
  if (output_path.empty()) {
    output_path = "marker_map.yaml";
  }

  try {
    std::ofstream fout(output_path);
    fout << root;
    fout.close();
    ROS_INFO("[offline_mapping_v2] saved %d markers to %s", saved_count, output_path.c_str());
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("[offline_mapping_v2] failed to save marker map: %s", e.what());
    return false;
  }
}

}  // namespace aruco_reloc
