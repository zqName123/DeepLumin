#pragma once

/**
 * @file initialization.hpp
 * @brief 定位初始化状态机（纯 C++，不依赖 ROS）
 *
 * 启动阶段策略（与需求对应）：
 * 1. 算法开始时若需要初始位姿，先启用 relocalization 获取初始位姿；
 * 2. 满足以下任一条件即视为初始化完成并停止 relocalization：
 *    - relocalization 成功（并可通过质量门限）；
 *    - 人工给出 initial pose；
 *    - 定位算法（global_matcher）自身匹配成功。
 *
 * 状态机只输出"建议的重定位命令"（start/stop）与当前状态，实际的重定位
 * 通信由 relocalization 适配层实现，二者通过 IRelocalizationClient 解耦。
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
  /// 是否必须获得初始位姿后才输出定位（false = ysw 原始行为：无初值也能以单位 map->odom 输出）
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

class InitStateMachine {
 public:
  enum class State {
    kUninitialized = 0,  ///< 等待初始位姿
    kRelocalizing,       ///< 重定位进行中（等待初始位姿）
    kTracking,           ///< 已初始化，正常跟踪
  };

  void configure(const InitConfig& init, const RelocalizationPolicy& policy);

  State state() const { return state_; }
  bool initialized() const { return state_ == State::kTracking; }
  bool relocalizationRunning() const { return state_ == State::kRelocalizing; }
  const std::string& stateName() const { return state_name_; }

  /// 每次事件处理后返回本周期应下发的重定位命令（取走后自动清零）
  RelocCommand takeCommand();

  /// slam 数据到达：未初始化且启用自动启动时触发重定位
  void onSlamOdom();

  /// 人工 initial pose：完成初始化并停止重定位
  void onManualPose();

  /// 重定位结果：成功（且通过外部门限）时完成初始化并停止重定位
  void onRelocalizationResult(bool accepted);

  /// global_matcher 自身匹配成功：完成初始化并停止重定位
  void onGlobalMatchSuccess();

 private:
  void transitionTo(State state);
  void request(RelocCommand command) { pending_command_ = command; }

  InitConfig init_;
  RelocalizationPolicy policy_;
  State state_ = State::kUninitialized;
  std::string state_name_ = "UNINITIALIZED";
  RelocCommand pending_command_ = RelocCommand::kNone;
};

}  // namespace localization_v2
