#pragma once

/**
 * @file fusion_core.hpp
 * @brief 融合核心：map->odom 唯一写入口 + 位姿写入策略
 *
 * 融合公式（与 v2 一致）：
 *   map_odom = map_base ⊕ (odom_base)⁻¹
 *   map_base = map_odom ⊕ odom_base
 *
 * 写入策略（优先级从高到低，一个阶段一个机制）：
 *   1. applyManualPose    人工位姿：无条件写入，永不经 jump guard；
 *                         写入后开启 settle 窗口并进入"人工验证"流程
 *   2. applyGlobalMatch   GM 结果（TRACKING）：
 *                         - settle 窗口内丢弃（等 GM 以新锚点算出的新鲜结果）
 *                         - 人工验证待定 且 质量 good → 验证采纳（无门限，确立新基准）
 *                         - 其余 → jump guard 判定后写入（防几何相似跳变）
 *   3. applyRelocalization / applyRecoveryMatch  恢复/初始化：无门限写入
 *                         （恢复本来就是大跳变；防误匹配交给恢复确认窗口）
 *
 * 该类只做纯数学与写入策略，不接触 ROS，便于单元测试。
 */

#include "localization_v3/core/jump_guard.hpp"
#include "localization_v3/core/quality_gate.hpp"
#include "localization_v3/core/types.hpp"

namespace localization_v3 {

/// GM 修正处理结果（节点据此打日志 / 喂失效判定）
enum class PoseUpdateResult {
  kAccepted = 0,      ///< 常规接受（jump guard 通过）
  kAcceptedVerified,  ///< 人工验证采纳（首个质量 good 的结果，确立新基准）
  kRejectedStale,     ///< 人工 settle 窗口内丢弃（等待新鲜结果）
  kRejectedQuality,   ///< 质量 bad，不写入
  kRejectedGuard,     ///< jump guard 拒绝（超限 / 确认失败）
  kConfirmPending,    ///< 大跳变待确认（候选已入缓冲）
};

class FusionCore {
 public:
  struct Config {
    JumpGuardConfig jump_guard;
    double manual_settle_sec = 0.3;  ///< 人工位姿后丢弃 GM 结果的短窗口
  };

  void setConfig(const Config& config);
  const Config& config() const { return config_; }

  // ---- SLAM 局部里程计 ----
  void setOdomBase(const Pose3d& odom_base);
  bool hasSlam() const { return have_slam_; }
  const Pose3d& odomBase() const { return odom_base_; }

  // ---- map->odom 状态 ----
  bool hasMapOdom() const { return have_map_odom_; }
  const Pose3d& mapOdom() const { return map_odom_; }
  bool isInitialized() const { return have_map_odom_; }

  /// 输出融合位姿 map->base = map_odom ⊕ odom_base
  Pose3d fusedPose() const;

  // ---- 写入口（见文件头策略说明）----

  /// 人工位姿（map->base 反推 map->odom）：无条件写入，开启 settle + 人工验证
  bool applyManualPose(const Pose3d& map_base, bool require_slam, double now_sec);

  /// GM 结果（候选 map->odom）：settle/人工验证/jump guard 全流程判定
  PoseUpdateResult applyGlobalMatch(const Pose3d& candidate_map_odom, QualityLevel quality,
                                    double now_sec);

  /// 重定位结果（map->base）：初始化 / TRACKING_LOST 恢复，无门限（P5：统一用 map_base 反推）
  bool applyRelocalization(const Pose3d& map_base, bool require_slam, double now_sec);

  /// 恢复期 GM 结果 / 初始化 GM 候选（map->odom）：无门限
  bool applyRecoveryMatch(const Pose3d& candidate_map_odom, double now_sec);

  // ---- 人工验证状态 ----
  bool manualVerifyPending() const { return manual_verify_pending_; }
  bool inManualSettle(double now_sec) const;
  double manualPoseStamp() const { return manual_pose_stamp_; }
  /// 验证失败/超时：退出人工验证流程（位姿保持不变），恢复正常写入规则
  void cancelManualVerify() { manual_verify_pending_ = false; }

  // ---- 跳变门限（供测试 / 调试）----
  JumpGuard& jumpGuard() { return jump_guard_; }
  const JumpGuard& jumpGuard() const { return jump_guard_; }

  /// 最近一次被采纳的修正时间（无修正看门狗用）
  double lastCorrectionStamp() const { return last_correction_stamp_; }

 private:
  void commit(const Pose3d& map_odom, double now_sec);

  Config config_;
  JumpGuard jump_guard_;

  Pose3d odom_base_;
  Pose3d map_odom_;
  bool have_slam_ = false;
  bool have_map_odom_ = false;

  bool manual_verify_pending_ = false;
  double manual_pose_stamp_ = -1.0;
  double manual_settle_until_ = -1.0;
  double last_correction_stamp_ = -1.0;
};

}  // namespace localization_v3
