#include "slam_mapping/ros/param_loader.hpp"

#include <localization/ros/ros_converters.hpp>

#include <XmlRpcValue.h>

namespace slam_mapping_ros {

slam_mapping::SlamMappingConfig loadConfig(ros::NodeHandle& pnh) {
  slam_mapping::SlamMappingConfig cfg;
  pnh.param<bool>("mapping/auto_start", cfg.mapping.auto_start, cfg.mapping.auto_start);
  pnh.param<std::string>("mapping/task_name", cfg.mapping.task_name, cfg.mapping.task_name);
  pnh.param<std::string>("mapping/output_root", cfg.mapping.output_root, cfg.mapping.output_root);
  pnh.param<bool>("mapping/publish_tf", cfg.mapping.publish_tf, cfg.mapping.publish_tf);
  pnh.param<double>("mapping/process_rate_hz", cfg.mapping.process_rate_hz, cfg.mapping.process_rate_hz);

  pnh.param<std::string>("frames/map", cfg.frames.map, cfg.frames.map);
  pnh.param<std::string>("frames/odom", cfg.frames.odom, cfg.frames.odom);
  pnh.param<std::string>("frames/base_link", cfg.frames.base_link, cfg.frames.base_link);
  pnh.param<std::string>("frames/lidar", cfg.frames.lidar, cfg.frames.lidar);
  pnh.param<std::string>("frames/imu", cfg.frames.imu, cfg.frames.imu);

  pnh.param<std::string>("sensors/imu_topic", cfg.sensors.imu_topic, cfg.sensors.imu_topic);
  pnh.param<std::string>("sensors/lidar_topic", cfg.sensors.lidar_topic, cfg.sensors.lidar_topic);
  pnh.param<std::string>("sensors/can_topic", cfg.sensors.can_topic, cfg.sensors.can_topic);
  pnh.param<std::string>("sensors/gnss_topic", cfg.sensors.gnss_topic, cfg.sensors.gnss_topic);

  pnh.param<double>("keyframe/distance_thresh", cfg.keyframe.distance_thresh, cfg.keyframe.distance_thresh);
  pnh.param<double>("keyframe/rotation_thresh_rad", cfg.keyframe.rotation_thresh_rad, cfg.keyframe.rotation_thresh_rad);
  pnh.param<double>("keyframe/time_thresh_sec", cfg.keyframe.time_thresh_sec, cfg.keyframe.time_thresh_sec);
  pnh.param<int>("keyframe/min_points", cfg.keyframe.min_points, cfg.keyframe.min_points);
  pnh.param<int>("keyframe/min_frame_gap", cfg.keyframe.min_frame_gap, cfg.keyframe.min_frame_gap);

  pnh.param<double>("map_saver/default_resolution", cfg.map_saver.default_resolution, cfg.map_saver.default_resolution);
  pnh.param<bool>("map_saver/save_on_stop", cfg.map_saver.save_on_stop, cfg.map_saver.save_on_stop);
  pnh.param<bool>("map_saver/save_keyframes", cfg.map_saver.save_keyframes, cfg.map_saver.save_keyframes);
  pnh.param<bool>("map_saver/save_keyframe_poses", cfg.map_saver.save_keyframe_poses, cfg.map_saver.save_keyframe_poses);
  pnh.param<bool>("map_saver/save_trajectory", cfg.map_saver.save_trajectory, cfg.map_saver.save_trajectory);
  pnh.param<bool>("map_saver/save_loop_results", cfg.map_saver.save_loop_results, cfg.map_saver.save_loop_results);
  pnh.param<std::string>("map_saver/map_filename", cfg.map_saver.map_filename, cfg.map_saver.map_filename);
  pnh.param<std::string>("map_saver/trajectory_filename", cfg.map_saver.trajectory_filename, cfg.map_saver.trajectory_filename);
  pnh.param<std::string>("map_saver/keyframe_pose_filename", cfg.map_saver.keyframe_pose_filename, cfg.map_saver.keyframe_pose_filename);
  pnh.param<std::string>("map_saver/keyframe_dir_name", cfg.map_saver.keyframe_dir_name, cfg.map_saver.keyframe_dir_name);
  pnh.param<std::string>("map_saver/loop_result_dir_name", cfg.map_saver.loop_result_dir_name, cfg.map_saver.loop_result_dir_name);
  pnh.param<std::string>("map_saver/loop_pairs_filename", cfg.map_saver.loop_pairs_filename, cfg.map_saver.loop_pairs_filename);
  pnh.param<std::string>("map_saver/loop_current_submap_filename", cfg.map_saver.loop_current_submap_filename, cfg.map_saver.loop_current_submap_filename);
  pnh.param<std::string>("map_saver/loop_history_submap_filename", cfg.map_saver.loop_history_submap_filename, cfg.map_saver.loop_history_submap_filename);

  pnh.param<bool>("loop_closure/enable", cfg.loop_closure.enable, cfg.loop_closure.enable);
  // [Task A] Load the full loop-closure config surface so YAML changes actually take effect.
  pnh.param<std::string>("loop_closure/type", cfg.loop_closure.type, cfg.loop_closure.type);
  pnh.param<double>("loop_closure/search_radius", cfg.loop_closure.search_radius, cfg.loop_closure.search_radius);
  pnh.param<int>("loop_closure/min_index_gap", cfg.loop_closure.min_index_gap, cfg.loop_closure.min_index_gap);
  pnh.param<double>("loop_closure/time_thresh_sec", cfg.loop_closure.time_thresh_sec, cfg.loop_closure.time_thresh_sec);
  pnh.param<int>("loop_closure/loop_kf_gap", cfg.loop_closure.loop_kf_gap, cfg.loop_closure.loop_kf_gap);
  pnh.param<int>("loop_closure/min_id_interval", cfg.loop_closure.min_id_interval, cfg.loop_closure.min_id_interval);
  pnh.param<int>("loop_closure/closest_id_th", cfg.loop_closure.closest_id_th, cfg.loop_closure.closest_id_th);
  pnh.param<double>("loop_closure/max_range", cfg.loop_closure.max_range, cfg.loop_closure.max_range);
  pnh.param<int>("loop_closure/submap_search_num", cfg.loop_closure.submap_search_num, cfg.loop_closure.submap_search_num);
  pnh.param<double>("loop_closure/submap_resolution", cfg.loop_closure.submap_resolution, cfg.loop_closure.submap_resolution);
  pnh.param<double>("loop_closure/icp_fitness_thresh", cfg.loop_closure.icp_fitness_thresh, cfg.loop_closure.icp_fitness_thresh);
  pnh.param<double>("loop_closure/odom_rot_noise", cfg.loop_closure.odom_rot_noise, cfg.loop_closure.odom_rot_noise);
  pnh.param<double>("loop_closure/odom_pos_noise", cfg.loop_closure.odom_pos_noise, cfg.loop_closure.odom_pos_noise);
  pnh.param<double>("loop_closure/loop_noise_scale", cfg.loop_closure.loop_noise_scale, cfg.loop_closure.loop_noise_scale);
  pnh.param<double>("loop_closure/loop_rot_noise", cfg.loop_closure.loop_rot_noise, cfg.loop_closure.loop_rot_noise);
  pnh.param<double>("loop_closure/loop_pos_noise", cfg.loop_closure.loop_pos_noise, cfg.loop_closure.loop_pos_noise);
  pnh.param<double>("loop_closure/loop_score_floor", cfg.loop_closure.loop_score_floor, cfg.loop_closure.loop_score_floor);

  // [Task GNSS] GNSS 后端优化参数（loop_closure/gnss/*）
  pnh.param<bool>("loop_closure/gnss/enable", cfg.loop_closure.gnss.enable, cfg.loop_closure.gnss.enable);
  pnh.param<int>("loop_closure/gnss/min_status", cfg.loop_closure.gnss.min_status, cfg.loop_closure.gnss.min_status);
  pnh.param<int>("loop_closure/gnss/min_nsv", cfg.loop_closure.gnss.min_nsv, cfg.loop_closure.gnss.min_nsv);
  pnh.param<double>("loop_closure/gnss/max_age_sec", cfg.loop_closure.gnss.max_age_sec, cfg.loop_closure.gnss.max_age_sec);
  pnh.param<double>("loop_closure/gnss/min_spacing_m", cfg.loop_closure.gnss.min_spacing_m, cfg.loop_closure.gnss.min_spacing_m);
  pnh.param<bool>("loop_closure/gnss/use_gps_elevation", cfg.loop_closure.gnss.use_gps_elevation, cfg.loop_closure.gnss.use_gps_elevation);
  pnh.param<double>("loop_closure/gnss/z_override_sigma", cfg.loop_closure.gnss.z_override_sigma, cfg.loop_closure.gnss.z_override_sigma);
  pnh.param<double>("loop_closure/gnss/sigma_fixed", cfg.loop_closure.gnss.sigma_fixed, cfg.loop_closure.gnss.sigma_fixed);
  pnh.param<double>("loop_closure/gnss/sigma_float", cfg.loop_closure.gnss.sigma_float, cfg.loop_closure.gnss.sigma_float);
  pnh.param<double>("loop_closure/gnss/sigma_unknown", cfg.loop_closure.gnss.sigma_unknown, cfg.loop_closure.gnss.sigma_unknown);
  pnh.param<double>("loop_closure/gnss/sigma_floor", cfg.loop_closure.gnss.sigma_floor, cfg.loop_closure.gnss.sigma_floor);
  pnh.param<double>("loop_closure/gnss/robust_delta", cfg.loop_closure.gnss.robust_delta, cfg.loop_closure.gnss.robust_delta);
  pnh.param<int>("loop_closure/gnss/alignment_frames", cfg.loop_closure.gnss.alignment_frames, cfg.loop_closure.gnss.alignment_frames);
  pnh.param<double>("loop_closure/gnss/consistency_thresh_m", cfg.loop_closure.gnss.consistency_thresh_m, cfg.loop_closure.gnss.consistency_thresh_m);
  {
    std::vector<double> lever_arm;
    if (pnh.getParam("loop_closure/gnss/lever_arm", lever_arm) && lever_arm.size() == 3) {
      cfg.loop_closure.gnss.lever_arm << lever_arm[0], lever_arm[1], lever_arm[2];
    }
  }
  {
    // status_sigma: YAML 二维数组 [[status, sigma], ...]，按车型配置的白名单 + 噪声表
    XmlRpc::XmlRpcValue value;
    if (pnh.getParam("loop_closure/gnss/status_sigma", value) &&
        value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      cfg.loop_closure.gnss.status_sigma.clear();
      for (int i = 0; i < value.size(); ++i) {
        if (value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() == 2) {
          const int status = static_cast<int>(value[i][0]);
          const double sigma = static_cast<double>(value[i][1]);
          cfg.loop_closure.gnss.status_sigma.emplace_back(status, sigma);
        }
      }
    }
  }

  pnh.param<bool>("visualization/publish_path", cfg.visualization.publish_path, cfg.visualization.publish_path);
  pnh.param<bool>("visualization/publish_keyframes", cfg.visualization.publish_keyframes, cfg.visualization.publish_keyframes);
  pnh.param<bool>("visualization/publish_local_map", cfg.visualization.publish_local_map, cfg.visualization.publish_local_map);
  pnh.param<bool>("visualization/publish_current_scan", cfg.visualization.publish_current_scan, cfg.visualization.publish_current_scan);
  pnh.param<bool>("visualization/publish_accumulated_map_preview", cfg.visualization.publish_accumulated_map_preview, cfg.visualization.publish_accumulated_map_preview);
  pnh.param<bool>("visualization/publish_loop_markers", cfg.visualization.publish_loop_markers, cfg.visualization.publish_loop_markers);
  pnh.param<bool>("visualization/publish_gnss_markers", cfg.visualization.publish_gnss_markers, cfg.visualization.publish_gnss_markers);
  pnh.param<bool>("visualization/publish_gnss_edges", cfg.visualization.publish_gnss_edges, cfg.visualization.publish_gnss_edges);
  pnh.param<double>("visualization/accumulated_map_voxel_leaf", cfg.visualization.accumulated_map_voxel_leaf, cfg.visualization.accumulated_map_voxel_leaf);

  pnh.param<std::string>("outputs/odom_topic", cfg.outputs.odom_topic, cfg.outputs.odom_topic);
  pnh.param<std::string>("outputs/path_topic", cfg.outputs.path_topic, cfg.outputs.path_topic);
  pnh.param<std::string>("outputs/current_scan_topic", cfg.outputs.current_scan_topic, cfg.outputs.current_scan_topic);
  pnh.param<std::string>("outputs/local_map_topic", cfg.outputs.local_map_topic, cfg.outputs.local_map_topic);
  pnh.param<std::string>("outputs/map_preview_topic", cfg.outputs.map_preview_topic, cfg.outputs.map_preview_topic);
  pnh.param<std::string>("outputs/loop_current_submap_topic", cfg.outputs.loop_current_submap_topic, cfg.outputs.loop_current_submap_topic);
  pnh.param<std::string>("outputs/loop_history_submap_topic", cfg.outputs.loop_history_submap_topic, cfg.outputs.loop_history_submap_topic);
  pnh.param<std::string>("outputs/keyframe_markers_topic", cfg.outputs.keyframe_markers_topic, cfg.outputs.keyframe_markers_topic);
  pnh.param<std::string>("outputs/loop_markers_topic", cfg.outputs.loop_markers_topic, cfg.outputs.loop_markers_topic);
  pnh.param<std::string>("outputs/gnss_markers_topic", cfg.outputs.gnss_markers_topic, cfg.outputs.gnss_markers_topic);
  pnh.param<std::string>("outputs/gnss_factor_edges_topic", cfg.outputs.gnss_factor_edges_topic, cfg.outputs.gnss_factor_edges_topic);
  pnh.param<std::string>("outputs/status_topic", cfg.outputs.status_topic, cfg.outputs.status_topic);

  cfg.slam_backend = localization_ros::loadSlamConfig(pnh);

  // [Task B] Mirror slam_odometry startup warmup options at the mapping runtime layer.
  pnh.param<bool>("slam/startup/enable_imu_warmup", cfg.startup.enable_imu_warmup, cfg.startup.enable_imu_warmup);
  pnh.param<double>("slam/startup/imu_warmup_duration", cfg.startup.imu_warmup_duration, cfg.startup.imu_warmup_duration);
  pnh.param<int>("slam/startup/min_imu_samples", cfg.startup.min_imu_samples, cfg.startup.min_imu_samples);
  return cfg;
}

}  // namespace slam_mapping_ros
