#pragma once

/**
 * @file relocalization_interface.hpp
 * @brief 重定位客户端抽象接口（localization_v3 与重定位模块的解耦点）
 *
 * 默认实现 RelocalizationRosAdapter：
 * - 订阅 deeplumin_msgs/RelocalizationResult（map->base_link）；
 * - 发布 std_msgs/String 命令话题（"start" / "stop"）。
 */

#include <functional>
#include <string>

#include "localization_v3/core/types.hpp"

namespace localization_v3 {

/// 重定位结果回调（ROS 层转换为该结构后回调）
struct RelocalizationSignal {
  bool success = false;
  bool accepted = false;
  Pose3d map_base;          ///< map->base_link
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  std::string map_id;
  std::string method;
  double stamp_sec = 0.0;   ///< 结果时间戳（秒，ros::Time::now()）
};

using RelocalizationResultCallback = std::function<void(const RelocalizationSignal&)>;

class IRelocalizationClient {
 public:
  virtual ~IRelocalizationClient() = default;

  /// 请求启动重定位（幂等）
  virtual void start() = 0;

  /// 请求停止重定位（幂等）
  virtual void stop() = 0;

  /// 注册结果回调
  virtual void setResultCallback(RelocalizationResultCallback callback) = 0;
};

}  // namespace localization_v3
