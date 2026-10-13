#pragma once

/**
 * @file status_publisher.hpp
 * @brief FusionStatus 组装（纯函数，便于单测/复用）
 *
 * 与 v2 的差异：
 * - 数据新鲜度使用真实年龄（由节点按最后一条消息时间戳计算）
 * - failure_reason 记录真实原因（出图 / 断流 / 质量 / 超时）
 */

#include <deeplumin_msgs/FusionStatus.h>
#include <ros/ros.h>

#include <string>

#include "localization_v3/core/state_machine.hpp"

namespace localization_v3 {

/// 节点填好的状态输入（纯数据）
struct StatusInput {
  LocalizationStateMachine::State state = LocalizationStateMachine::State::kUninitialized;

  bool have_slam = false;
  bool have_map_odom = false;
  std::string map_id;

  // ---- 真实数据年龄（秒），未知用 -1 ----
  double slam_age = -1.0;
  double gm_age = -1.0;
  double reloc_age = -1.0;
  double manual_age = -1.0;

  // ---- 状态机计数 ----
  int gm_failures = 0;
  int gm_degraded = 0;
  int reloc_failures = 0;

  // ---- 活性信号给出的真实失效原因（无失效为空串）----
  std::string live_failure_reason;

  bool reloc_running = false;
  bool allow_degraded_output = true;
};

/// 组装 FusionStatus 消息
deeplumin_msgs::FusionStatus makeFusionStatus(const StatusInput& in, const ros::Time& stamp,
                                              const std::string& map_frame);

}  // namespace localization_v3
