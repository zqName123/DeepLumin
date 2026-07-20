#pragma once

/**
 * @file types.hpp
 * @brief localization_manager 公共数据类型
 *
 * 核心概念：
 *   ManagerState  — 管理器运行状态（对外展示与策略生成依据）
 *   ModuleHealth  — 单个子模块的健康快照（由回调刷新 age/level/available）
 *   ManagerSnapshot — 对外状态消息的数据源
 *   ObserverPolicyData — 下发给 pose_fusion 等的观测策略
 */

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/LocalizationStatus.h>
#include <deeplumin_msgs/ObserverPolicy.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <deeplumin_msgs/SlamHealth.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <string>

namespace localization_manager {

/** 管理器高层状态机枚举 */
enum class ManagerState {
  INIT,
  WAIT_INITIAL_POSE,  ///< 等待 /initialpose 或 fusion initialized
  LOCALIZATION,       ///< 正常运行
  GLOBAL_MATCHING,    ///< GM 正在匹配（success 但未 converged）
  RELOCALIZING,       ///< 已接受重定位，待 pose_fusion 消费
  DEGRADED,           ///< 部分模块不可用或连续降级
  FAILURE,            ///< 融合超时或无任何可用定位源
  PURE_DR,            ///< 预留：纯 DR 模式
  PURE_SLAM,          ///< 预留：纯 SLAM 模式
  MAPPING             ///< 预留：建图模式
};

/** 子模块健康等级 */
enum class ModuleLevel {
  NORMAL,
  WARNING,
  DEGRADED,
  FAILURE
};

/** 单模块健康快照（由 ros_adapter 从各消息转换） */
struct ModuleHealth {
  std::string name;
  ModuleLevel level = ModuleLevel::FAILURE;
  ros::Time last_stamp;
  double age_sec = 1e9;       ///< now - last_stamp
  double quality_score = 0.0;
  int consecutive_ok = 0;
  int consecutive_warn = 0;
  int consecutive_fail = 0;
  bool available = false;
  bool timeout = true;
  std::string reason;
};

/** GNSS 简化视图（用于场景自动判定） */
struct GnssView {
  ros::Time stamp;
  int status = 0;
  int nsv1 = 0;
  int nsv2 = 0;
  double heading_rad = 0.0;
  double speed_mps = 0.0;
  bool heading_valid = false;
  bool position_valid = false;
  bool velocity_valid = false;
};

/** 管理器对外状态快照 → deeplumin_msgs/LocalizationStatus */
struct ManagerSnapshot {
  ros::Time stamp;
  ManagerState state = ManagerState::INIT;
  bool initialized = false;
  bool output_valid = false;
  std::string failure_reason;
  std::string current_map_id = "default";
  std::string active_scene = "unknown";
  double last_dr_age = 1e9;
  double last_slam_age = 1e9;
  double last_global_matcher_age = 1e9;
  double last_relocalization_age = 1e9;
  double last_gnss_age = 1e9;
  double position_std = 1.0;
  double velocity_std = 1.0;
  double roll_std = 1.0;
  double pitch_std = 1.0;
  double yaw_std = 1.0;
  std::string used_sources;
  std::string rejected_sources;
  std::string reject_reasons;
  bool is_dr_available = false;
  bool is_slam_available = false;
  bool is_global_matcher_available = false;
  bool is_relocalization_available = false;
  bool is_gnss_available = false;
  bool is_can_available = false;
  bool is_imu_available = false;
  bool is_lidar_available = false;
};

/** 观测策略（内部表示）→ deeplumin_msgs/ObserverPolicy → pose_fusion */
struct ObserverPolicyData {
  std::string scene_type = "unknown";
  std::string reason;
  bool use_dr = true;
  bool use_slam = true;
  bool use_global_matcher = true;
  bool use_relocalization = true;
  bool use_gnss = false;
  bool use_gnss_position = false;
  bool use_gnss_heading = false;
  double dr_weight = 1.0;
  double slam_weight = 1.0;
  double global_matcher_weight = 1.0;
  double relocalization_weight = 1.0;
  double gnss_position_weight = 0.0;
  double gnss_heading_weight = 0.0;
  bool allow_fusion_reset = true;
  bool allow_dr_reset = false;
  bool allow_global_matcher_trigger = true;
  bool allow_relocalization_trigger = true;
};

}  // namespace localization_manager
