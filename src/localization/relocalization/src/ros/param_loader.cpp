#include <relocalization/ros/param_loader.hpp>

#include <vector>

namespace relocalization_ros {
namespace {


bool loadVector3(ros::NodeHandle& pnh, const std::string& key, Eigen::Vector3d* value) {
  std::vector<double> flat;
  if (!pnh.getParam(key, flat) || flat.size() != 3) {
    return false;
  }
  *value = Eigen::Vector3d(flat[0], flat[1], flat[2]);
  return true;
}

bool loadMatrix3(ros::NodeHandle& pnh, const std::string& key, Eigen::Matrix3d* value) {
  std::vector<double> flat;
  if (!pnh.getParam(key, flat) || flat.size() != 9) {
    return false;
  }
  *value << flat[0], flat[1], flat[2], flat[3], flat[4], flat[5], flat[6], flat[7], flat[8];
  return true;
}

Eigen::Matrix4d loadExtrinsic(ros::NodeHandle& pnh, const std::string& key) {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  loadVector3(pnh, key + "/translation", &translation);
  loadMatrix3(pnh, key + "/rotation", &rotation);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

void loadCommonCoreConfig(ros::NodeHandle& pnh, relocalization::RelocalizationCoreConfig& cfg) {
  pnh.param<std::string>("descriptor/type", cfg.descriptor_type, "scan_context");
  pnh.param<std::string>("matcher/type", cfg.matcher_type, "gicp");
  pnh.param<std::string>("keyframe_dir", cfg.database.keyframe_dir,
                         "/home/hhy/2004ros/relocalization_ws/key_point_frame");
  pnh.param<std::string>("pose_file", cfg.database.pose_file,
                         "/home/hhy/2004ros/relocalization_ws/optimized_poses_tum.txt");
  pnh.param<std::string>("global_map_pcd", cfg.database.global_map_pcd, "");
  pnh.param<std::string>("scan_context_db", cfg.descriptor_db,
                         "/home/hhy/2004ros/relocalization_ws/scan_context_submap_db_50.bin");

  pnh.param<int>("top_k", cfg.top_k, 5);
  pnh.param<int>("scan_context/ring_key_candidates", cfg.ring_key_candidates, 50);

  pnh.param<std::string>("submap/mode", cfg.database.submap_mode, "radius");
  pnh.param<double>("submap/radius", cfg.database.submap_radius, 50.0);
  pnh.param<int>("submap/neighbor_keyframes", cfg.database.neighbor_keyframes, 10);
  pnh.param<int>("submap/before_frames", cfg.database.submap_before_frames, 0);
  pnh.param<int>("submap/after_frames", cfg.database.submap_after_frames, 0);

  pnh.param<double>("preprocess/voxel_leaf", cfg.database.preprocess.voxel_leaf, 0.05);
  pnh.param<double>("preprocess/min_range", cfg.database.preprocess.min_range, 3.0);
  pnh.param<double>("preprocess/max_range", cfg.database.preprocess.max_range, 80.0);

  pnh.param<int>("gicp/max_iterations", cfg.matcher.max_iterations, 64);
  pnh.param<double>("gicp/max_correspondence_distance", cfg.matcher.max_correspondence_distance, 2.0);
  pnh.param<double>("gicp/transformation_epsilon", cfg.matcher.transformation_epsilon, 0.01);
  pnh.param<double>("gicp/euclidean_fitness_epsilon", cfg.matcher.euclidean_fitness_epsilon, 0.01);
  pnh.param<double>("gicp/accept_fitness", cfg.matcher.accept_fitness, 0.5);
  pnh.param<double>("gicp/inlier_distance", cfg.matcher.inlier_distance, 0.5);
  pnh.param<double>("gicp/min_inlier_ratio", cfg.matcher.min_inlier_ratio, 0.6);
  pnh.param<double>("gicp/max_inlier_rmse", cfg.matcher.max_inlier_rmse, 0.5);
  pnh.param<bool>("gicp/z_filter_enable", cfg.matcher.z_filter_enable, false);
  pnh.param<double>("gicp/z_filter_min", cfg.matcher.z_filter_min, -1000.0);
  pnh.param<double>("gicp/z_filter_max", cfg.matcher.z_filter_max, 1000.0);
  pnh.param<bool>("gicp/near_ceiling_filter_enable", cfg.matcher.near_ceiling_filter_enable, false);
  pnh.param<double>("gicp/near_ceiling_radius", cfg.matcher.near_ceiling_radius, 15.0);
}

}  // namespace

BagEvalConfig loadBagEvalConfig(ros::NodeHandle& pnh) {
  BagEvalConfig cfg;
  loadCommonCoreConfig(pnh, cfg.core);

  pnh.param<std::string>("report_csv", cfg.report_csv,
                         "/home/hhy/2004ros/relocalization_ws/bag_eval_report.csv");
  std::string bag_report_csv;
  pnh.param<std::string>("bag_report_csv", bag_report_csv, cfg.report_csv);
  cfg.report_csv = bag_report_csv;
  pnh.param<std::string>("bag_file", cfg.bag_file, "");
  pnh.param<std::string>("bag_lidar_topic", cfg.lidar_topic, "/ouster/points");
  if (cfg.lidar_topic.empty()) {
    pnh.param<std::string>("lidar_topic", cfg.lidar_topic, "/ouster/points");
  }

  pnh.param<int>("bag_cloud_stride", cfg.cloud_stride, 10);
  pnh.param<int>("max_queries", cfg.max_queries, 50);
  pnh.param<double>("success_translation", cfg.success_translation, 1.0);
  pnh.param<double>("success_yaw_deg", cfg.success_yaw_deg, 5.0);
  pnh.param<double>("place_success_radius", cfg.place_success_radius, 5.0);
  pnh.param<double>("max_pose_time_diff", cfg.max_pose_time_diff, 0.15);
  pnh.param<double>("bag_start_offset", cfg.bag_start_offset, 0.0);
  pnh.param<double>("bag_duration", cfg.bag_duration, 0.0);
  pnh.param<bool>("run_gicp", cfg.run_gicp, true);
  pnh.param<bool>("bag_use_truth_pose", cfg.use_truth_pose, false);
  pnh.param<bool>("log_topk_candidates", cfg.log_topk_candidates, true);
  pnh.param<bool>("query_frame/rotate_xy_y_negx", cfg.query_rotate_xy_y_negx, false);
  pnh.param<double>("query_frame/yaw_deg", cfg.query_frame_yaw_deg, 0.0);

  pnh.param<bool>("enable_visualization", cfg.viz.enable, false);
  pnh.param<std::string>("frame_id", cfg.viz.frame_id, "map");
  pnh.param<double>("visualization/crop_radius", cfg.viz.crop_radius, 80.0);
  pnh.param<double>("visualization/wait_sec", cfg.viz.wait_sec, 3.0);
  pnh.param<bool>("visualization/interactive", cfg.viz.interactive, false);
  pnh.param<bool>("visualization/publish_truth_aligned", cfg.viz.publish_truth_aligned, true);
  pnh.param<bool>("visualization/publish_reloc_aligned", cfg.viz.publish_reloc_aligned, true);
  pnh.param<bool>("visualization/publish_target_submap", cfg.viz.publish_target_submap, true);
  pnh.param<std::string>("visualization/global_map_source", cfg.viz.global_map_source, "pcd");
  pnh.param<int>("visualization/max_keyframes_for_map", cfg.viz.max_keyframes_for_map, 200);

  if (!cfg.use_truth_pose) {
    cfg.viz.publish_truth_aligned = false;
  }
  return cfg;
}

OnlineRelocalizationConfig loadOnlineRelocalizationConfig(ros::NodeHandle& pnh) {
  OnlineRelocalizationConfig cfg;
  loadCommonCoreConfig(pnh, cfg.core);
  pnh.param<std::string>("lidar_topic", cfg.lidar_topic, "/ouster/points");
  pnh.param<std::string>("frame_id", cfg.frame_id, "map");
  pnh.param<std::string>("child_frame_id", cfg.child_frame_id, "base_link");
  pnh.param<std::string>("lidar_frame_id", cfg.lidar_frame_id, "lidar_link");
  pnh.param<std::string>("query_cloud_frame", cfg.query_cloud_frame, "auto");
  cfg.base_to_lidar = loadExtrinsic(pnh, "extrinsics/base_to_lidar");
  pnh.param<std::string>("result_topic", cfg.result_topic, "/localization/relocalization/result");
  pnh.param<std::string>("map_id", cfg.map_id, "default");
  pnh.param<std::string>("method", cfg.method, "scan_context_gicp");
  pnh.param<bool>("query_frame/rotate_xy_y_negx", cfg.query_rotate_xy_y_negx, false);
  pnh.param<double>("query_frame/yaw_deg", cfg.query_frame_yaw_deg, 0.0);
  return cfg;
}

}  // namespace relocalization_ros
