#pragma once

/**
 * @file localization_v3_config.hpp
 * @brief localization_v3 节点配置结构体与加载函数
 *
 * 配置来源：节点私有命名空间（~），由 launch 通过 <rosparam> 注入
 * config/localization_v3.yaml。参数含义见该 yaml 内注释与优化方案文档 §6。
 */

#include <ros/ros.h>

#include <string>

#include "localization_v3/core/failure_detector.hpp"
#include "localization_v3/core/fusion_core.hpp"
#include "localization_v3/core/init_arbiter.hpp"
#include "localization_v3/core/quality_gate.hpp"
#include "localization_v3/core/state_machine.hpp"

namespace localization_v3 {

struct LocalizationV3Config {
  // ---- 坐标系 ----
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  // ---- 话题（默认值与 v2 一致，下游零改动）----
  std::string slam_odom_topic = "/localization/slam_odom";
  std::string global_matcher_result_topic = "/localization/global_matcher/result";
  std::string initial_pose_topic = "/initialpose";
  std::string source_cloud_topic = "/localization/cloud_registered_effect";
  std::string fused_odom_topic = "/localization_v2/odom";
  std::string map_to_odom_topic = "/localization_v2/map_to_odom";
  std::string fused_path_topic = "/localization_v2/path";
  std::string status_topic = "/localization_v2/status";
  std::string current_scan_in_map_topic = "/localization_v2/current_scan_in_map";
  std::string relocalization_result_topic = "/localization/relocalization/result";
  std::string relocalization_command_topic = "/localization_v2/relocalization/command";
  std::string gm_anchor_topic = "/localization_v2/gm_anchor";
  std::string loc_point_topic = "/localization_v2/loc_point";

  // ---- 运行时 ----
  double output_rate_hz = 100.0;
  int max_path_size = 5000;
  bool publish_tf = true;
  bool publish_map_to_odom_tf = true;
  bool publish_odom_to_base_tf = false;
  bool publish_path = true;
  bool publish_current_scan_in_map = true;
  double base_yaw_offset = 0.0;

  // ---- GM 结果语义 ----
  std::string gm_result_pose_frame = "odom";  ///< "odom": map->odom；"base": map->base_link

  // ---- 分模块配置 ----
  QualityThresholds quality;          ///< GM 质量三级门限
  FusionCore::Config fusion;          ///< jump_guard + manual_settle_sec
  FailureDetectorConfig failure;      ///< 快速失效阈值
  InitArbiterConfig init_arbiter;     ///< 初始化仲裁
  TrackingFailureConfig tracking;     ///< 状态机计数阈值 / 驻留
  InitConfig init;                    ///< 初始化策略
  RelocalizationPolicy reloc_policy;  ///< 重定位启停策略

  // ---- 人工位姿 ----
  double manual_verify_timeout_sec = 5.0; ///< 未验证成功 → 转恢复流程
  bool publish_gm_anchor = true;          ///< 人工位姿/验证后向 GM 发锚点
  bool initial_pose_z_completion = true;  ///< 人工位姿 z 补全
  double initial_pose_z_search_radius = 5.0;
  std::string initial_pose_map_pcd_path;

  // ---- 重定位结果门限 ----
  bool relocalization_require_accepted = false;
  double relocalization_min_inlier_ratio = 0.69;
  double relocalization_max_inlier_rmse = 0.4;

  // ---- 降级输出 ----
  bool allow_degraded_output = true;
};

/// 从节点私有命名空间（~）加载全部配置
LocalizationV3Config loadLocalizationV3Config(ros::NodeHandle& pnh);

}  // namespace localization_v3
