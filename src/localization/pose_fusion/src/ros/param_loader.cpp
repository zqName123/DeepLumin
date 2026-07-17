#include "pose_fusion/ros/param_loader.hpp"

namespace pose_fusion_ros {

pose_fusion::FusionConfig loadFusionConfig(ros::NodeHandle& pnh) {
  pose_fusion::FusionConfig cfg;

  pnh.param<std::string>("frames/map", cfg.frames.map, cfg.frames.map);
  pnh.param<std::string>("frames/odom", cfg.frames.odom, cfg.frames.odom);
  pnh.param<std::string>("frames/base_link", cfg.frames.base_link, cfg.frames.base_link);
  pnh.param<std::string>("map/default_map_id", cfg.default_map_id, cfg.default_map_id);

  pnh.param<std::string>("topics/dr_odom", cfg.topics.dr_odom, cfg.topics.dr_odom);
  pnh.param<std::string>("topics/dr_status", cfg.topics.dr_status, cfg.topics.dr_status);
  pnh.param<std::string>("topics/slam_odom", cfg.topics.slam_odom, cfg.topics.slam_odom);
  pnh.param<std::string>("topics/slam_health", cfg.topics.slam_health, cfg.topics.slam_health);
  pnh.param<std::string>("topics/global_matcher_result", cfg.topics.global_matcher_result,
                         cfg.topics.global_matcher_result);
  pnh.param<std::string>("topics/global_matcher_status", cfg.topics.global_matcher_status,
                         cfg.topics.global_matcher_status);
  pnh.param<std::string>("topics/accepted_relocalization", cfg.topics.accepted_relocalization,
                         cfg.topics.accepted_relocalization);
  pnh.param<std::string>("topics/observer_policy", cfg.topics.observer_policy,
                         cfg.topics.observer_policy);
  pnh.param<std::string>("topics/initial_pose", cfg.topics.initial_pose, cfg.topics.initial_pose);
  pnh.param<std::string>("topics/fused_odom", cfg.topics.fused_odom, cfg.topics.fused_odom);
  pnh.param<std::string>("topics/fused_pose", cfg.topics.fused_pose, cfg.topics.fused_pose);
  pnh.param<std::string>("topics/fused_path", cfg.topics.fused_path, cfg.topics.fused_path);
  pnh.param<std::string>("topics/fusion_status", cfg.topics.fusion_status, cfg.topics.fusion_status);

  pnh.param<double>("runtime/output_rate_hz", cfg.runtime.output_rate_hz, cfg.runtime.output_rate_hz);
  pnh.param<bool>("runtime/publish_tf", cfg.runtime.publish_tf, cfg.runtime.publish_tf);
  pnh.param<bool>("runtime/publish_map_to_odom_tf", cfg.runtime.publish_map_to_odom_tf,
                  cfg.runtime.publish_map_to_odom_tf);
  pnh.param<bool>("runtime/publish_odom_to_base_tf", cfg.runtime.publish_odom_to_base_tf,
                  cfg.runtime.publish_odom_to_base_tf);
  pnh.param<bool>("runtime/publish_path", cfg.runtime.publish_path, cfg.runtime.publish_path);
  pnh.param<int>("runtime/max_path_size", cfg.runtime.max_path_size, cfg.runtime.max_path_size);
  pnh.param<bool>("runtime/allow_debug_relocalization_direct", cfg.runtime.allow_debug_relocalization_direct,
                  cfg.runtime.allow_debug_relocalization_direct);

  pnh.param<bool>("initialization/allow_initialpose", cfg.initialization.allow_initialpose,
                  cfg.initialization.allow_initialpose);
  pnh.param<bool>("initialization/allow_dr_initialization", cfg.initialization.allow_dr_initialization,
                  cfg.initialization.allow_dr_initialization);
  pnh.param<bool>("initialization/allow_slam_initialization", cfg.initialization.allow_slam_initialization,
                  cfg.initialization.allow_slam_initialization);
  pnh.param<double>("initialization/initial_cov_position", cfg.initialization.initial_cov_position,
                    cfg.initialization.initial_cov_position);
  pnh.param<double>("initialization/initial_cov_yaw_deg", cfg.initialization.initial_cov_yaw_deg,
                    cfg.initialization.initial_cov_yaw_deg);

  pnh.param<double>("prediction/max_dr_gap", cfg.prediction.max_dr_gap, cfg.prediction.max_dr_gap);
  pnh.param<double>("prediction/max_dr_delta_translation", cfg.prediction.max_dr_delta_translation,
                    cfg.prediction.max_dr_delta_translation);
  pnh.param<double>("prediction/max_dr_delta_yaw_deg", cfg.prediction.max_dr_delta_yaw_deg,
                    cfg.prediction.max_dr_delta_yaw_deg);
  pnh.param<double>("prediction/process_noise_position", cfg.prediction.process_noise_position,
                    cfg.prediction.process_noise_position);
  pnh.param<double>("prediction/process_noise_yaw_deg", cfg.prediction.process_noise_yaw_deg,
                    cfg.prediction.process_noise_yaw_deg);

  pnh.param<bool>("observers/slam/enabled", cfg.slam.enabled, cfg.slam.enabled);
  pnh.param<double>("observers/slam/smooth_gain", cfg.slam.smooth_gain, cfg.slam.smooth_gain);
  pnh.param<double>("observers/slam/max_position_residual", cfg.slam.max_position_residual,
                    cfg.slam.max_position_residual);
  pnh.param<double>("observers/slam/max_yaw_residual_deg", cfg.slam.max_yaw_residual_deg,
                    cfg.slam.max_yaw_residual_deg);

  pnh.param<bool>("observers/global_matcher/enabled", cfg.global_matcher.enabled,
                  cfg.global_matcher.enabled);
  pnh.param<bool>("observers/global_matcher/allow_converged_false",
                  cfg.global_matcher.allow_converged_false,
                  cfg.global_matcher.allow_converged_false);
  pnh.param<double>("observers/global_matcher/base_gain", cfg.global_matcher.base_gain,
                    cfg.global_matcher.base_gain);
  pnh.param<double>("observers/global_matcher/min_gain", cfg.global_matcher.min_gain,
                    cfg.global_matcher.min_gain);
  pnh.param<double>("observers/global_matcher/max_gain", cfg.global_matcher.max_gain,
                    cfg.global_matcher.max_gain);
  pnh.param<double>("observers/global_matcher/target_inlier_ratio",
                    cfg.global_matcher.target_inlier_ratio,
                    cfg.global_matcher.target_inlier_ratio);
  pnh.param<double>("observers/global_matcher/target_rmse", cfg.global_matcher.target_rmse,
                    cfg.global_matcher.target_rmse);
  pnh.param<double>("observers/global_matcher/min_inlier_ratio",
                    cfg.global_matcher.min_inlier_ratio,
                    cfg.global_matcher.min_inlier_ratio);
  pnh.param<double>("observers/global_matcher/max_inlier_rmse",
                    cfg.global_matcher.max_inlier_rmse,
                    cfg.global_matcher.max_inlier_rmse);
  pnh.param<double>("observers/global_matcher/max_fitness", cfg.global_matcher.max_fitness,
                    cfg.global_matcher.max_fitness);
  pnh.param<double>("observers/global_matcher/max_position_residual",
                    cfg.global_matcher.max_position_residual,
                    cfg.global_matcher.max_position_residual);
  pnh.param<double>("observers/global_matcher/max_yaw_residual_deg",
                    cfg.global_matcher.max_yaw_residual_deg,
                    cfg.global_matcher.max_yaw_residual_deg);

  pnh.param<double>("buffer/max_dr_age", cfg.buffer.max_dr_age, cfg.buffer.max_dr_age);
  pnh.param<double>("buffer/max_slam_age", cfg.buffer.max_slam_age, cfg.buffer.max_slam_age);
  pnh.param<double>("buffer/max_global_matcher_age", cfg.buffer.max_global_matcher_age,
                    cfg.buffer.max_global_matcher_age);
  pnh.param<double>("buffer/max_relocalization_age", cfg.buffer.max_relocalization_age,
                    cfg.buffer.max_relocalization_age);
  return cfg;
}

}  // namespace pose_fusion_ros
