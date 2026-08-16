#include "global_matcher_v2/ros/global_matcher_v2_config.hpp"

#include <vector>

namespace global_matcher_v2 {

namespace {

bool loadBool(ros::NodeHandle& pnh, const std::string& key, bool fallback) {
  bool value = fallback;
  pnh.param<bool>(key, value, fallback);
  return value;
}

double loadDouble(ros::NodeHandle& pnh, const std::string& key, double fallback) {
  double value = fallback;
  pnh.param<double>(key, value, fallback);
  return value;
}

int loadInt(ros::NodeHandle& pnh, const std::string& key, int fallback) {
  int value = fallback;
  pnh.param<int>(key, value, fallback);
  return value;
}

std::string loadString(ros::NodeHandle& pnh, const std::string& key,
                       const std::string& fallback) {
  std::string value = fallback;
  pnh.param<std::string>(key, value, fallback);
  return value;
}

Eigen::Vector3f loadVec3(ros::NodeHandle& pnh, const std::string& key,
                         const Eigen::Vector3f& fallback) {
  std::vector<double> flat;
  if (!pnh.getParam(key, flat) || flat.size() != 3) {
    return fallback;
  }
  return Eigen::Vector3f(static_cast<float>(flat[0]), static_cast<float>(flat[1]),
                         static_cast<float>(flat[2]));
}

Eigen::Matrix4f loadExtrinsic(ros::NodeHandle& pnh, const std::string& key) {
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  std::vector<double> flat_t;
  if (pnh.getParam(key + "/translation", flat_t) && flat_t.size() == 3) {
    translation << static_cast<float>(flat_t[0]), static_cast<float>(flat_t[1]),
        static_cast<float>(flat_t[2]);
  }
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  std::vector<double> flat_r;
  if (pnh.getParam(key + "/rotation", flat_r) && flat_r.size() == 9) {
    rotation << static_cast<float>(flat_r[0]), static_cast<float>(flat_r[1]),
        static_cast<float>(flat_r[2]), static_cast<float>(flat_r[3]),
        static_cast<float>(flat_r[4]), static_cast<float>(flat_r[5]),
        static_cast<float>(flat_r[6]), static_cast<float>(flat_r[7]),
        static_cast<float>(flat_r[8]);
  }
  Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
  t.block<3, 3>(0, 0) = rotation;
  t.block<3, 1>(0, 3) = translation;
  return t;
}

}  // namespace

MapManagerConfig GlobalMatcherV2Config::mapManagerConfig() const {
  MapManagerConfig cfg;
  cfg.mode = map_mode;
  cfg.pcd_path = map_pcd_path;
  cfg.map_dir = map_dir;
  cfg.chunk_size = chunk_size;
  cfg.load_radius_chunks = load_radius_chunks;
  cfg.load_voxel_leaf = load_voxel_leaf;
  cfg.max_loaded_chunks = max_loaded_chunks;
  cfg.cov_num_neighbors = cov_num_neighbors;
  cfg.cov_num_threads = cov_num_threads;
  cfg.vgicp_voxel_resolution = vgicp_voxel_resolution;
  cfg.vgicp_coarse_scale = vgicp_coarse_scale;
  cfg.vgicp_fine_scale = vgicp_fine_scale;
  return cfg;
}

GlobalMatcherV2Config loadGlobalMatcherV2Config(ros::NodeHandle& pnh) {
  GlobalMatcherV2Config cfg;

  // frames
  cfg.map_frame = loadString(pnh, "frames/map", cfg.map_frame);
  cfg.odom_frame = loadString(pnh, "frames/odom", cfg.odom_frame);
  cfg.base_frame = loadString(pnh, "frames/base_link", cfg.base_frame);
  cfg.lidar_frame = loadString(pnh, "frames/lidar", cfg.lidar_frame);
  cfg.base_to_lidar = loadExtrinsic(pnh, "extrinsics/base_to_lidar");

  // topics
  cfg.source_cloud_topic = loadString(pnh, "topics/source_cloud", cfg.source_cloud_topic);
  cfg.initial_pose_topic = loadString(pnh, "topics/initial_pose", cfg.initial_pose_topic);
  cfg.source_odom_topic = loadString(pnh, "topics/source_odom", cfg.source_odom_topic);
  cfg.bootstrap_initial_pose_topic =
      loadString(pnh, "topics/bootstrap_initial_pose", cfg.bootstrap_initial_pose_topic);
  cfg.manual_initial_pose_topic =
      loadString(pnh, "topics/manual_initial_pose", cfg.manual_initial_pose_topic);
  cfg.gm_anchor_topic = loadString(pnh, "topics/gm_anchor", cfg.gm_anchor_topic);
  cfg.result_topic = loadString(pnh, "topics/result", cfg.result_topic);
  cfg.status_topic = loadString(pnh, "topics/status", cfg.status_topic);
  cfg.source_initial_scan_topic =
      loadString(pnh, "topics/source_initial_scan", cfg.source_initial_scan_topic);
  cfg.aligned_scan_topic = loadString(pnh, "topics/aligned_scan", cfg.aligned_scan_topic);
  cfg.target_submap_topic = loadString(pnh, "topics/target_submap", cfg.target_submap_topic);
  cfg.global_map_topic = loadString(pnh, "topics/global_map", cfg.global_map_topic);
  cfg.initial_guess_topic = loadString(pnh, "topics/initial_guess", cfg.initial_guess_topic);
  cfg.path_topic = loadString(pnh, "topics/path", cfg.path_topic);
  cfg.load_map_command_topic =
      loadString(pnh, "topics/load_map_command", cfg.load_map_command_topic);
  cfg.set_map_segment_command_topic =
      loadString(pnh, "topics/set_map_segment_command", cfg.set_map_segment_command_topic);

  // map
  cfg.map_mode = loadString(pnh, "map/mode", cfg.map_mode);
  cfg.map_pcd_path = loadString(pnh, "map/pcd_path", cfg.map_pcd_path);
  cfg.map_dir = loadString(pnh, "map/map_dir", cfg.map_dir);
  cfg.chunk_size = loadDouble(pnh, "map/chunk_size", cfg.chunk_size);
  cfg.load_radius_chunks = loadInt(pnh, "map/load_radius_chunks", cfg.load_radius_chunks);
  cfg.load_voxel_leaf = loadDouble(pnh, "map/load_voxel_leaf", cfg.load_voxel_leaf);
  cfg.max_loaded_chunks = loadInt(pnh, "map/max_loaded_chunks", cfg.max_loaded_chunks);
  cfg.cov_num_neighbors = loadInt(pnh, "map/cov_num_neighbors", cfg.cov_num_neighbors);
  cfg.cov_num_threads = loadInt(pnh, "map/cov_num_threads", cfg.cov_num_threads);
  cfg.vgicp_voxel_resolution =
      loadDouble(pnh, "map/vgicp_voxel_resolution", cfg.vgicp_voxel_resolution);
  cfg.vgicp_coarse_scale = loadDouble(pnh, "map/vgicp_coarse_scale", cfg.vgicp_coarse_scale);
  cfg.vgicp_fine_scale = loadDouble(pnh, "map/vgicp_fine_scale", cfg.vgicp_fine_scale);
  cfg.global_map_publish_voxel_leaf =
      loadDouble(pnh, "map/global_map_publish_voxel_leaf", cfg.global_map_publish_voxel_leaf);
  cfg.global_map_publish_period =
      loadDouble(pnh, "map/global_map_publish_period", cfg.global_map_publish_period);

  // runtime
  cfg.auto_match = loadBool(pnh, "runtime/auto_match", cfg.auto_match);
  cfg.source_cloud_is_odom_frame =
      loadBool(pnh, "runtime/source_cloud_is_odom_frame", cfg.source_cloud_is_odom_frame);
  cfg.source_cloud_frame = loadString(pnh, "runtime/source_cloud_frame", cfg.source_cloud_frame);
  cfg.match_rate = loadDouble(pnh, "runtime/match_rate", cfg.match_rate);
  cfg.max_cloud_age = loadDouble(pnh, "runtime/max_cloud_age", cfg.max_cloud_age);
  cfg.use_receive_time_for_cloud_age =
      loadBool(pnh, "runtime/use_receive_time_for_cloud_age", cfg.use_receive_time_for_cloud_age);
  cfg.use_tracking_initial_pose =
      loadBool(pnh, "runtime/use_tracking_initial_pose", cfg.use_tracking_initial_pose);
  cfg.use_bootstrap_initial_pose =
      loadBool(pnh, "runtime/use_bootstrap_initial_pose", cfg.use_bootstrap_initial_pose);
  cfg.use_manual_initial_pose =
      loadBool(pnh, "runtime/use_manual_initial_pose", cfg.use_manual_initial_pose);
  cfg.use_default_initial_pose =
      loadBool(pnh, "runtime/use_default_initial_pose", cfg.use_default_initial_pose);
  cfg.allow_identity_bootstrap =
      loadBool(pnh, "runtime/allow_identity_bootstrap", cfg.allow_identity_bootstrap);
  cfg.manual_initial_pose_z_search_radius =
      loadDouble(pnh, "manual_initial_pose/z_search_radius",
                 cfg.manual_initial_pose_z_search_radius);
  cfg.default_initial_translation =
      loadVec3(pnh, "runtime/default_initial_pose/translation", cfg.default_initial_translation);
  cfg.default_initial_rpy_deg =
      loadVec3(pnh, "runtime/default_initial_pose/rpy_deg", cfg.default_initial_rpy_deg);

  // crop
  cfg.crop.forward = loadDouble(pnh, "crop/forward", cfg.crop.forward);
  cfg.crop.backward = loadDouble(pnh, "crop/backward", cfg.crop.backward);
  cfg.crop.left = loadDouble(pnh, "crop/left", cfg.crop.left);
  cfg.crop.right = loadDouble(pnh, "crop/right", cfg.crop.right);
  cfg.crop.down = loadDouble(pnh, "crop/down", cfg.crop.down);
  cfg.crop.up = loadDouble(pnh, "crop/up", cfg.crop.up);

  // matcher
  cfg.matcher.type = loadString(pnh, "matcher/type", cfg.matcher.type);
  cfg.matcher.use_crop = loadBool(pnh, "matcher/use_crop", cfg.matcher.use_crop);
  cfg.matcher.source_voxel_leaf =
      loadDouble(pnh, "matcher/source_voxel_leaf", cfg.matcher.source_voxel_leaf);
  cfg.matcher.target_voxel_leaf =
      loadDouble(pnh, "matcher/target_voxel_leaf", cfg.matcher.target_voxel_leaf);
  cfg.matcher.coarse_scale = loadDouble(pnh, "matcher/coarse_scale", cfg.matcher.coarse_scale);
  cfg.matcher.fine_scale = loadDouble(pnh, "matcher/fine_scale", cfg.matcher.fine_scale);
  cfg.matcher.max_iterations = loadInt(pnh, "matcher/max_iterations", cfg.matcher.max_iterations);
  cfg.matcher.transformation_epsilon =
      loadDouble(pnh, "matcher/transformation_epsilon", cfg.matcher.transformation_epsilon);
  cfg.matcher.max_correspondence_distance =
      loadDouble(pnh, "matcher/max_correspondence_distance",
                 cfg.matcher.max_correspondence_distance);
  cfg.matcher.coarse_max_correspondence_distance = loadDouble(
      pnh, "matcher/coarse_max_correspondence_distance",
      cfg.matcher.coarse_max_correspondence_distance);
  cfg.matcher.fine_max_correspondence_distance = loadDouble(
      pnh, "matcher/fine_max_correspondence_distance",
      cfg.matcher.fine_max_correspondence_distance);
  cfg.matcher.robust_kernel = loadString(pnh, "matcher/robust_kernel", cfg.matcher.robust_kernel);
  cfg.matcher.robust_kernel_scale =
      loadDouble(pnh, "matcher/robust_kernel_scale", cfg.matcher.robust_kernel_scale);
  cfg.matcher.num_threads = loadInt(pnh, "matcher/num_threads", cfg.matcher.num_threads);
  cfg.matcher.cov_num_neighbors =
      loadInt(pnh, "matcher/cov_num_neighbors", cfg.matcher.cov_num_neighbors);
  cfg.matcher.cov_num_threads =
      loadInt(pnh, "matcher/cov_num_threads", cfg.matcher.cov_num_threads);
  cfg.matcher.fast_gicp_num_threads =
      loadInt(pnh, "matcher/fast_gicp_num_threads", cfg.matcher.fast_gicp_num_threads);
  cfg.matcher.fast_gicp_correspondence_randomness = loadInt(
      pnh, "matcher/fast_gicp_correspondence_randomness",
      cfg.matcher.fast_gicp_correspondence_randomness);
  cfg.matcher.vgicp_voxel_resolution =
      loadDouble(pnh, "matcher/vgicp_voxel_resolution", cfg.matcher.vgicp_voxel_resolution);
  cfg.matcher.vgicp_coarse_scale =
      loadDouble(pnh, "matcher/vgicp_coarse_scale", cfg.matcher.vgicp_coarse_scale);
  cfg.matcher.vgicp_fine_scale =
      loadDouble(pnh, "matcher/vgicp_fine_scale", cfg.matcher.vgicp_fine_scale);

  // gate
  cfg.min_source_points = loadInt(pnh, "gate/min_source_points", cfg.min_source_points);
  cfg.min_target_points = loadInt(pnh, "gate/min_target_points", cfg.min_target_points);
  cfg.max_fitness_score = loadDouble(pnh, "gate/max_fitness_score", cfg.max_fitness_score);
  cfg.min_inlier_ratio = loadDouble(pnh, "gate/min_inlier_ratio", cfg.min_inlier_ratio);
  cfg.max_inlier_rmse = loadDouble(pnh, "gate/max_inlier_rmse", cfg.max_inlier_rmse);
  cfg.inlier_distance = loadDouble(pnh, "gate/inlier_distance", cfg.inlier_distance);
  cfg.max_delta_translation =
      loadDouble(pnh, "gate/max_delta_translation", cfg.max_delta_translation);
  cfg.max_delta_yaw_deg = loadDouble(pnh, "gate/max_delta_yaw_deg", cfg.max_delta_yaw_deg);

  // result
  cfg.result_pose_frame = loadString(pnh, "result/pose_frame", cfg.result_pose_frame);

  // publish
  cfg.publish_global_map = loadBool(pnh, "publish/global_map", cfg.publish_global_map);
  cfg.publish_target_submap =
      loadBool(pnh, "publish/target_submap", cfg.publish_target_submap);
  cfg.publish_aligned_scan =
      loadBool(pnh, "publish/aligned_scan", cfg.publish_aligned_scan);
  cfg.publish_source_initial_scan =
      loadBool(pnh, "publish/source_initial_scan", cfg.publish_source_initial_scan);
  cfg.publish_initial_guess =
      loadBool(pnh, "publish/initial_guess", cfg.publish_initial_guess);
  cfg.publish_path = loadBool(pnh, "publish/path", cfg.publish_path);
  cfg.publish_status = loadBool(pnh, "publish/status", cfg.publish_status);

  return cfg;
}

}  // namespace global_matcher_v2
