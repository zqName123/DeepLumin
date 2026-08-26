#pragma once

/**
 * @file fusion_core.hpp
 * @brief 融合核心：slam 局部里程计 + 全局修正量（map->odom）-> map->base_link
 *
 * 与 ysw_style_fusion 完全相同的融合思路：
 *   map_odom = map_base ⊕ (odom_base)⁻¹   （全局定位 / 初始位姿 / 重定位 更新）
 *   map_base = map_odom ⊕ odom_base        （高频输出）
 *
 * 该类只做纯数学与门控逻辑，不接触 ROS，便于测试与解耦。
 */

#include "localization_v2/core/types.hpp"

namespace localization_v2 {

struct JumpGuardConfig {
  bool enabled = false;        ///< 是否启用跳变门限（默认关 = ysw_loc 原始行为）
  double max_translation = 5.0;  ///< 单次修正最大平移跳变（米）
  double max_yaw_deg = 50.0;     ///< 单次修正最大偏航跳变（度）
};

/**
 * @brief 全局定位结果（已由 ROS 层转换为纯数据结构）
 */
struct GlobalMatchUpdate {
  bool success = false;
  bool has_pose = false;              ///< 消息是否携带可用位姿
  Pose3d pose = Pose3d();             ///< 位姿：map->odom 或 map->base（见 pose_is_map_base）
  bool pose_is_map_base = false;      ///< true: pose 为 map->base；false: pose 为 map->odom
  std::string map_id;
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
};

/**
 * @brief 重定位结果（已由 ROS 层转换为纯数据结构）
 */
struct RelocalizationUpdate {
  bool success = false;
  bool accepted = false;
  Pose3d map_base = Pose3d();         ///< 重定位给出的 map->base_link 位姿
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  std::string map_id;
  std::string method;
};

class FusionCore {
 public:
  void setJumpGuard(const JumpGuardConfig& config) { jump_guard_ = config; }

  /// 人工初值/重定位成功后调用：下一次 GM 结果跳过跳变门限（作为锚点确认/修正），一次性生效
  void requestJumpGuardBypass() { jump_guard_bypass_ = true; }

  // ---- slam 局部里程计 ----
  void setOdomBase(const Pose3d& odom_base);
  bool hasSlam() const { return have_slam_; }
  const Pose3d& odomBase() const { return odom_base_; }

  // ---- 全局修正量（map->odom） ----
  bool hasMapOdom() const { return have_map_odom_; }
  const Pose3d& mapOdom() const { return map_odom_; }

  /**
   * @brief 用 global_matcher 结果更新 map->odom
   * @return true 表示已接受并更新
   */
  bool updateFromGlobalMatch(const GlobalMatchUpdate& update);
  /**
   * @brief 直接设置 map->odom（初始化仲裁选优后写入，不做跳变门限）
   */
  bool setMapOdom(const Pose3d& map_odom);

  /**
   * @brief 用 map->base_link 位姿反推并设置 map->odom（人工初值 / 重定位结果）
   * @return true 表示成功（需要已有 slam 数据或允许无 slam 时直接采用）
   */
  bool updateMapOdomFromMapBase(const Pose3d& map_base, bool require_slam);

  /**
   * @brief 输出融合位姿 map->base_link = map_odom ⊕ odom_base
   */
  Pose3d fusedPose() const;

  /// 供初始化状态机使用的内部状态
  bool isInitialized() const { return have_map_odom_; }

 private:
  /// 跳变门限检查（人工初值 / 重定位结果不受限，仅用于 GM 结果）
  bool passesJumpGuard(const Pose3d& candidate_map_odom) const;

  Pose3d odom_base_;
  Pose3d map_odom_;
  bool have_slam_ = false;
  bool have_map_odom_ = false;
  JumpGuardConfig jump_guard_;
  bool jump_guard_bypass_ = false;  ///< 人工初值/重定位后的首次 GM 结果绕过跳变门限
};

}  // namespace localization_v2
