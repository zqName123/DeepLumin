#include "slam_mapping_v2/ros/config_loader.hpp"

#include <XmlRpcValue.h>

#include <algorithm>

namespace slam_mapping_v2_ros {
using namespace slam_mapping_v2;

namespace {

void loadGnssConfig(ros::NodeHandle& pnh, GnssConfig& g) {
  pnh.param<bool>("loop_closure/gnss/enable", g.enable, g.enable);
  pnh.param<int>("loop_closure/gnss/min_status", g.min_status, g.min_status);
  pnh.param<int>("loop_closure/gnss/min_nsv", g.min_nsv, g.min_nsv);
  pnh.param<double>("loop_closure/gnss/max_age_sec", g.max_age_sec, g.max_age_sec);
  pnh.param<double>("loop_closure/gnss/min_spacing_m", g.min_spacing_m, g.min_spacing_m);
  pnh.param<bool>("loop_closure/gnss/use_gps_elevation", g.use_gps_elevation, g.use_gps_elevation);
  pnh.param<double>("loop_closure/gnss/z_override_sigma", g.z_override_sigma, g.z_override_sigma);
  pnh.param<double>("loop_closure/gnss/sigma_fixed", g.sigma_fixed, g.sigma_fixed);
  pnh.param<double>("loop_closure/gnss/sigma_float", g.sigma_float, g.sigma_float);
  pnh.param<double>("loop_closure/gnss/sigma_unknown", g.sigma_unknown, g.sigma_unknown);
  pnh.param<double>("loop_closure/gnss/sigma_floor", g.sigma_floor, g.sigma_floor);
  pnh.param<double>("loop_closure/gnss/robust_delta", g.robust_delta, g.robust_delta);
  pnh.param<int>("loop_closure/gnss/alignment_frames", g.alignment_frames, g.alignment_frames);
  pnh.param<double>("loop_closure/gnss/consistency_thresh_m", g.consistency_thresh_m,
                    g.consistency_thresh_m);
  std::vector<double> lever_arm;
  if (pnh.getParam("loop_closure/gnss/lever_arm", lever_arm) && lever_arm.size() == 3) {
    g.lever_arm << lever_arm[0], lever_arm[1], lever_arm[2];
  }
  XmlRpc::XmlRpcValue value;
  if (pnh.getParam("loop_closure/gnss/status_sigma", value) &&
      value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    g.status_sigma.clear();
    for (int i = 0; i < value.size(); ++i) {
      if (value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() == 2) {
        g.status_sigma.emplace_back(static_cast<int>(value[i][0]),
                                    static_cast<double>(value[i][1]));
      }
    }
  }
}

}  // namespace

MappingConfig loadConfig(ros::NodeHandle& pnh) {
  MappingConfig cfg;

  // runtime
  pnh.param<bool>("runtime/auto_start", cfg.runtime.auto_start, cfg.runtime.auto_start);
  pnh.param<std::string>("runtime/task_name", cfg.runtime.task_name, cfg.runtime.task_name);
  pnh.param<std::string>("runtime/output_root", cfg.runtime.output_root, cfg.runtime.output_root);
  pnh.param<bool>("runtime/publish_tf", cfg.runtime.publish_tf, cfg.runtime.publish_tf);
  pnh.param<double>("runtime/process_rate_hz", cfg.runtime.process_rate_hz,
                    cfg.runtime.process_rate_hz);

  // frames
  pnh.param<std::string>("frames/map", cfg.frames.map, cfg.frames.map);
  pnh.param<std::string>("frames/odom", cfg.frames.odom, cfg.frames.odom);
  pnh.param<std::string>("frames/base_link", cfg.frames.base_link, cfg.frames.base_link);

  // input
  pnh.param<std::string>("input/odom_topic", cfg.input.odom_topic, cfg.input.odom_topic);
  pnh.param<std::string>("input/scan_topic", cfg.input.scan_topic, cfg.input.scan_topic);
  pnh.param<std::string>("input/gnss_topic", cfg.input.gnss_topic, cfg.input.gnss_topic);

  // output
  pnh.param<std::string>("output/status_topic", cfg.output.status_topic, cfg.output.status_topic);
  pnh.param<std::string>("output/path_topic", cfg.output.path_topic, cfg.output.path_topic);
  pnh.param<std::string>("output/map_topic", cfg.output.map_topic, cfg.output.map_topic);
  pnh.param<std::string>("output/loop_markers_topic", cfg.output.loop_markers_topic,
                         cfg.output.loop_markers_topic);
  pnh.param<std::string>("output/keyframe_markers_topic", cfg.output.keyframe_markers_topic,
                         cfg.output.keyframe_markers_topic);
  pnh.param<bool>("output/publish_odom", cfg.output.publish_odom, cfg.output.publish_odom);
  pnh.param<bool>("output/publish_current_scan", cfg.output.publish_current_scan,
                  cfg.output.publish_current_scan);
  pnh.param<std::string>("output/odom_topic", cfg.output.odom_topic, cfg.output.odom_topic);
  pnh.param<std::string>("output/current_scan_topic", cfg.output.current_scan_topic,
                         cfg.output.current_scan_topic);
  pnh.param<bool>("output/publish_debug_markers", cfg.output.publish_debug_markers,
                  cfg.output.publish_debug_markers);

  // services
  pnh.param<std::string>("services/start", cfg.services.start, cfg.services.start);
  pnh.param<std::string>("services/stop", cfg.services.stop, cfg.services.stop);
  pnh.param<std::string>("services/save", cfg.services.save, cfg.services.save);
  pnh.param<std::string>("services/reset", cfg.services.reset, cfg.services.reset);
  pnh.param<std::string>("services/get_status", cfg.services.get_status, cfg.services.get_status);

  // threading
  pnh.param<double>("threading/backend_interval_sec", cfg.threading.backend_interval_sec,
                    cfg.threading.backend_interval_sec);
  pnh.param<double>("threading/viz_interval_sec", cfg.threading.viz_interval_sec,
                    cfg.threading.viz_interval_sec);
  int max_input_queue = static_cast<int>(cfg.threading.max_input_queue);
  pnh.param<int>("threading/max_input_queue", max_input_queue, max_input_queue);
  cfg.threading.max_input_queue = static_cast<std::size_t>(std::max(1, max_input_queue));
  pnh.param<double>("threading/time_sync_tolerance_sec", cfg.threading.time_sync_tolerance_sec,
                    cfg.threading.time_sync_tolerance_sec);

  // keyframe
  pnh.param<double>("keyframe/distance_thresh", cfg.keyframe.distance_thresh,
                    cfg.keyframe.distance_thresh);
  pnh.param<double>("keyframe/rotation_thresh_rad", cfg.keyframe.rotation_thresh_rad,
                    cfg.keyframe.rotation_thresh_rad);
  pnh.param<double>("keyframe/time_thresh_sec", cfg.keyframe.time_thresh_sec,
                    cfg.keyframe.time_thresh_sec);
  pnh.param<int>("keyframe/min_points", cfg.keyframe.min_points, cfg.keyframe.min_points);
  pnh.param<int>("keyframe/min_frame_gap", cfg.keyframe.min_frame_gap,
                 cfg.keyframe.min_frame_gap);

  // loop_closure
  pnh.param<bool>("loop_closure/enable", cfg.loop_closure.enable, cfg.loop_closure.enable);
  pnh.param<int>("loop_closure/min_index_gap", cfg.loop_closure.min_index_gap,
                 cfg.loop_closure.min_index_gap);
  pnh.param<double>("loop_closure/time_thresh_sec", cfg.loop_closure.time_thresh_sec,
                    cfg.loop_closure.time_thresh_sec);
  pnh.param<int>("loop_closure/loop_kf_gap", cfg.loop_closure.loop_kf_gap,
                 cfg.loop_closure.loop_kf_gap);
  pnh.param<int>("loop_closure/min_id_interval", cfg.loop_closure.min_id_interval,
                 cfg.loop_closure.min_id_interval);
  pnh.param<int>("loop_closure/closest_id_th", cfg.loop_closure.closest_id_th,
                 cfg.loop_closure.closest_id_th);
  pnh.param<double>("loop_closure/max_range", cfg.loop_closure.max_range,
                    cfg.loop_closure.max_range);
  pnh.param<int>("loop_closure/submap_search_num", cfg.loop_closure.submap_search_num,
                 cfg.loop_closure.submap_search_num);
  pnh.param<double>("loop_closure/submap_resolution", cfg.loop_closure.submap_resolution,
                    cfg.loop_closure.submap_resolution);
  pnh.param<double>("loop_closure/icp_fitness_thresh", cfg.loop_closure.icp_fitness_thresh,
                    cfg.loop_closure.icp_fitness_thresh);
  pnh.param<double>("loop_closure/loop_noise_scale", cfg.loop_closure.loop_noise_scale,
                    cfg.loop_closure.loop_noise_scale);
  pnh.param<double>("loop_closure/loop_rot_noise", cfg.loop_closure.loop_rot_noise,
                    cfg.loop_closure.loop_rot_noise);
  pnh.param<double>("loop_closure/loop_pos_noise", cfg.loop_closure.loop_pos_noise,
                    cfg.loop_closure.loop_pos_noise);
  pnh.param<double>("loop_closure/loop_score_floor", cfg.loop_closure.loop_score_floor,
                    cfg.loop_closure.loop_score_floor);
  pnh.param<double>("loop_closure/odom_rot_noise", cfg.loop_closure.odom_rot_noise,
                    cfg.loop_closure.odom_rot_noise);
  pnh.param<double>("loop_closure/odom_pos_noise", cfg.loop_closure.odom_pos_noise,
                    cfg.loop_closure.odom_pos_noise);
  loadGnssConfig(pnh, cfg.loop_closure.gnss);

  // map_saver
  pnh.param<double>("map_saver/default_resolution", cfg.map_saver.default_resolution,
                    cfg.map_saver.default_resolution);
  pnh.param<bool>("map_saver/save_on_stop", cfg.map_saver.save_on_stop,
                  cfg.map_saver.save_on_stop);
  pnh.param<bool>("map_saver/save_keyframes", cfg.map_saver.save_keyframes,
                  cfg.map_saver.save_keyframes);
  pnh.param<bool>("map_saver/save_keyframe_poses", cfg.map_saver.save_keyframe_poses,
                  cfg.map_saver.save_keyframe_poses);
  pnh.param<bool>("map_saver/save_trajectory", cfg.map_saver.save_trajectory,
                  cfg.map_saver.save_trajectory);
  pnh.param<bool>("map_saver/save_loop_results", cfg.map_saver.save_loop_results,
                  cfg.map_saver.save_loop_results);
  pnh.param<std::string>("map_saver/map_filename", cfg.map_saver.map_filename,
                         cfg.map_saver.map_filename);
  pnh.param<std::string>("map_saver/trajectory_filename", cfg.map_saver.trajectory_filename,
                         cfg.map_saver.trajectory_filename);
  pnh.param<std::string>("map_saver/keyframe_pose_filename",
                         cfg.map_saver.keyframe_pose_filename,
                         cfg.map_saver.keyframe_pose_filename);
  pnh.param<std::string>("map_saver/keyframe_dir_name", cfg.map_saver.keyframe_dir_name,
                         cfg.map_saver.keyframe_dir_name);
  pnh.param<std::string>("map_saver/loop_result_dir_name", cfg.map_saver.loop_result_dir_name,
                         cfg.map_saver.loop_result_dir_name);
  pnh.param<std::string>("map_saver/loop_pairs_filename", cfg.map_saver.loop_pairs_filename,
                         cfg.map_saver.loop_pairs_filename);
  pnh.param<std::string>("map_saver/loop_current_submap_filename",
                         cfg.map_saver.loop_current_submap_filename,
                         cfg.map_saver.loop_current_submap_filename);
  pnh.param<std::string>("map_saver/loop_history_submap_filename",
                         cfg.map_saver.loop_history_submap_filename,
                         cfg.map_saver.loop_history_submap_filename);

  // map
  pnh.param<double>("map/voxel_resolution", cfg.map.voxel_resolution, cfg.map.voxel_resolution);
  pnh.param<double>("map/filter_size_map", cfg.map.filter_size_map, cfg.map.filter_size_map);
  pnh.param<double>("map/publish_voxel_leaf", cfg.map.publish_voxel_leaf,
                    cfg.map.publish_voxel_leaf);
  pnh.param<bool>("map/publish_global_map", cfg.map.publish_global_map, cfg.map.publish_global_map);

  return cfg;
}

}  // namespace slam_mapping_v2_ros
