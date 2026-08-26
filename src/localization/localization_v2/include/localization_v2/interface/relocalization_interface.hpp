#pragma once

/**
 * @file relocalization_interface.hpp
 * @brief 重定位客户端抽象接口（localization_v2 与重定位模块的解耦点）
 *
 * 后续实现"算法开始时先重定位拿初始位姿、成功后停止"只需要：
 * - 实现 IRelocalizationClient（例如通过 service / action / 话题协议），
 * - 在节点构造时注入即可，融合与状态机逻辑无需改动。
 *
 * 当前默认实现 RelocalizationRosAdapter：
 * - 订阅 deeplumin_msgs/RelocalizationResult（map->base_link）；
 * - 发布 std_msgs/String 命令话题（"start" / "stop"），供支持启停的重定位模块消费。
 */

#include <functional>
#include <string>

#include "localization_v2/core/types.hpp"

namespace localization_v2 {

/// 重定位结果回调（纯数据，ROS 层转换为该结构后回调）
struct RelocalizationSignal {
  bool success = false;
  bool accepted = false;
  Pose3d map_base;
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  std::string map_id;
  std::string method;
};

using RelocalizationResultCallback = std::function<void(const RelocalizationSignal&)>;

class IRelocalizationClient {
 public:
  virtual ~IRelocalizationClient() = default;

  /// 请求启动重定位（幂等）
  virtual void start() = 0;

  /// 请求停止重定位（幂等）
  virtual void stop() = 0;

  /// 注册结果回调（节点通过该回调将结果喂给初始化状态机）
  virtual void setResultCallback(RelocalizationResultCallback callback) = 0;
};

}  // namespace localization_v2
