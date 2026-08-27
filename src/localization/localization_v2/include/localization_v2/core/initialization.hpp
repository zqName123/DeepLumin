#pragma once

/**
 * @file initialization.hpp
 * @brief 定位状态机（纯 C++，不依赖 ROS）
 *
 * 状态机管理整个定位生命周期：
 * 1. 未初始化 → 重定位初始化 → 正常跟踪
 * 2. 跟踪丢失 → 重定位恢复 → 正常跟踪
 * 3. 恢复失败 → 定位失败 → 人工干预
 *
 * 设计原则：
 * - 人工定位在任何状态下都有效
 * - 跟踪丢失时自动触发重定位恢复
 * - 降级输出比完全无输出更安全
 */

#include <string>

namespace localization_v2 {

/// 重定位命令
enum class RelocCommand {
  kNone = 0,  ///< 无动作
  kStart,     ///< 请求启动重定位
  kStop,      ///< 请求停止重定位
};

/// 初始化状态机配置
struct InitConfig {
  /// 是否必须获得初始位姿后才输出定位
  bool require_initial_pose = false;
  /// 是否允许使用 relocalization 作为初始位姿来源
  bool use_relocalization_init = true;
  /// 是否允许人工 initial pose 完成初始化 / 重置
  bool use_manual_initialpose = true;
  /// 是否允许 global_matcher 自身成功即视为初始化完成
  bool use_global_matcher_init = true;
};

/// 重定位启停策略
struct RelocalizationPolicy {
  bool enabled = true;
  /// 未初始化且需要初值时，是否自动向重定位模块发出 start 命令
  bool auto_start = true;
  /// 重定位成功（通过门限）后停止
  bool stop_on_success = true;
  /// 收到人工 initial pose 后停止
  bool stop_on_manual_pose = true;
  /// global_matcher 自身成功匹配后停止
  bool stop_on_global_match = true;
};

/// 跟踪失效检测配置
struct TrackingFailureConfig {
  /// GM 连续失败阈值，达到此值触发跟踪丢失
  int gm_failure_threshold = 10;
  /// 重定位连续失败阈值，达到此值触发定位失败
  int reloc_failure_threshold = 5;
  /// 重定位超时时间（秒）
  double reloc_timeout_sec = 30.0;
};

class LocalizationStateMachine {
 public:
  enum class State {
    kUninitialized = 0,      ///< 未初始化，等待初始位姿
    kRelocalizing,           ///< 重定位初始化中
    kTracking,               ///< 正常跟踪，定位有效
    kTrackingLost,           ///< 跟踪丢失，尝试恢复中
    kLocalizationFailure,    ///< 定位失败，需要人工干预
  };

  void configure(const InitConfig& init,
                 const RelocalizationPolicy& policy,
                 const TrackingFailureConfig& tracking_config);

  // ========== 状态查询 ==========
  State state() const { return state_; }
  bool initialized() const {
    return state_ == State::kTracking ||
           state_ == State::kTrackingLost ||
           state_ == State::kLocalizationFailure;
  }
  bool relocalizationRunning() const { return state_ == State::kRelocalizing; }
  bool trackingLost() const { return state_ == State::kTrackingLost; }
  bool localizationFailure() const { return state_ == State::kLocalizationFailure; }
  const std::string& stateName() const { return state_name_; }

  // ========== 命令取走 ==========
  /// 每次事件处理后返回本周期应下发的重定位命令（取走后自动清零）
  RelocCommand takeCommand();

  // ========== 事件处理 ==========

  /// slam 数据到达：未初始化且启用自动启动时触发重定位
  void onSlamOdom();

  /// 人工 initial pose：任何状态下都有效
  void onManualPose();

  /// 重定位结果：成功（且通过外部门限）时完成初始化或恢复
  void onRelocalizationResult(bool accepted);

  /// global_matcher 自身匹配成功：完成初始化
  void onGlobalMatchSuccess();

  /// 跟踪失败检测：GM 连续失败达到阈值
  void onTrackingFailure();

  /// 恢复失败检测：重定位超时或连续失败达到阈值
  void onRecoveryFailed();

  // ========== 计数器查询（用于状态发布） ==========
  int getGmConsecutiveFailures() const { return gm_consecutive_failures_; }
  int getRelocConsecutiveFailures() const { return reloc_consecutive_failures_; }

 private:
  void transitionTo(State state);
  void request(RelocCommand command) { pending_command_ = command; }
  void resetCounters();

  InitConfig init_;
  RelocalizationPolicy policy_;
  TrackingFailureConfig tracking_config_;

  State state_ = State::kUninitialized;
  std::string state_name_ = "UNINITIALIZED";
  RelocCommand pending_command_ = RelocCommand::kNone;

  // 失效检测计数器
  int gm_consecutive_failures_ = 0;
  int reloc_consecutive_failures_ = 0;
};

}  // namespace localization_v2
