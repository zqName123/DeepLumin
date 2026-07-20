#pragma once

/**
 * @file manager_core.hpp
 * @brief 定位管理器内核（无 ROS 依赖的决策逻辑）
 *
 * 职责：
 *   1) 缓存各子模块 ModuleHealth 与最新消息
 *   2) 定频 update() 中依次：场景判定 → 状态机 → 快照 → 观测策略
 *   3) 重定位结果门控（relocalizationTrusted）后暂存 accepted_relocalization_
 *
 * 与 pose_fusion 的接口：
 *   observerPolicy() → ObserverPolicy（各源 use_* / weight / allow_fusion_reset）
 *   takeAcceptedRelocalization() → RelocalizationResult（accepted=true）
 */

#include "localization_manager/common/config.hpp"
#include "localization_manager/common/types.hpp"

#include <optional>
#include <string>

namespace localization_manager {

class ManagerCore {
 public:
  bool initialize(const ManagerConfig& config);

  /**
   * 定频节拍入口（由节点 onTimer 调用）：
   *   updateSceneLocked → updateStateLocked → updateStatusLocked → updatePolicyLocked
   */
  void update(const ros::Time& now);

  // ---------- 异步输入：仅更新内部快照，不触发状态转移 ----------
  void onDrStatus(const ModuleHealth& health);
  void onDrOdom(const ModuleHealth& health);
  void onSlamStatus(const ModuleHealth& health);
  void onSlamOdom(const ModuleHealth& health);
  void onFusionStatus(const ModuleHealth& health, const deeplumin_msgs::FusionStatus& msg);
  void onGlobalMatchStatus(const ModuleHealth& health);
  void onGlobalMatchResult(const ModuleHealth& health, const deeplumin_msgs::GlobalMatchResult& msg);
  void onRelocalizationResult(const ModuleHealth& health, const deeplumin_msgs::RelocalizationResult& msg);
  void onGnss(const GnssView& gnss);
  void onInitialPose(const ros::Time& stamp, const std::string& frame_id);
  void onSceneState(const std::string& scene);

  const ManagerSnapshot& status() const { return status_; }
  const ObserverPolicyData& observerPolicy() const { return policy_; }

  bool hasAcceptedRelocalization() const { return accepted_relocalization_.has_value(); }
  /** 取出并清空已接受重定位（一次性发布语义） */
  deeplumin_msgs::RelocalizationResult takeAcceptedRelocalization();

 private:
  /** 根据 GNSS / 手动覆盖 / 配置判定 current_scene_ */
  void updateSceneLocked(const ros::Time& now);
  /** 根据各模块可用性与降级计数判定 state_ */
  void updateStateLocked(const ros::Time& now);
  /** 组装对外 ManagerSnapshot */
  void updateStatusLocked(const ros::Time& now);
  /** 根据 state_ 与场景生成 ObserverPolicyData */
  void updatePolicyLocked(const ros::Time& now);
  /** 重定位结果质量门控 */
  bool relocalizationTrusted(const deeplumin_msgs::RelocalizationResult& msg) const;

  ManagerConfig config_;
  bool initialized_ = false;
  ros::Time last_update_;
  ros::Time last_initial_pose_time_;
  ros::Time last_manual_scene_time_;
  ros::Time last_gnss_time_;
  bool has_initial_pose_ = false;
  bool has_manual_scene_ = false;
  bool has_gnss_ = false;
  bool gnss_heading_only_ = false;
  bool gnss_full_ = false;
  std::string current_scene_ = "unknown";
  std::string manual_scene_ = "unknown";
  std::string scene_reason_ = "startup";
  std::string current_map_id_ = "default";
  std::string failure_reason_ = "startup";
  ManagerState state_ = ManagerState::INIT;
  ManagerSnapshot status_;
  ObserverPolicyData policy_;

  // 各子模块健康快照（由回调更新 age_sec / available / level）
  ModuleHealth dr_status_;
  ModuleHealth dr_odom_;
  ModuleHealth slam_status_;
  ModuleHealth slam_odom_;
  ModuleHealth fusion_status_;
  ModuleHealth global_matcher_status_;
  ModuleHealth global_matcher_result_;
  ModuleHealth relocalization_status_;
  GnssView gnss_;

  // 各子模块最新消息体（用于状态判定中的细节字段）
  deeplumin_msgs::FusionStatus last_fusion_msg_;
  deeplumin_msgs::GlobalMatchResult last_global_match_msg_;
  deeplumin_msgs::RelocalizationResult last_relocalization_msg_;
  std::optional<deeplumin_msgs::RelocalizationResult> accepted_relocalization_;

  int gnss_confirm_count_ = 0;
  int gnss_lost_count_ = 0;
  int degraded_count_ = 0;
  int healthy_count_ = 0;
};

}  // namespace localization_manager
