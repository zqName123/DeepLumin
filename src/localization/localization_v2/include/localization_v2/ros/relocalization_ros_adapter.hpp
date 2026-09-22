#pragma once

/**
 * @file relocalization_ros_adapter.hpp
 * @brief 重定位 ROS 适配层：IRelocalizationClient 的默认实现
 *
 * - 订阅 deeplumin_msgs/RelocalizationResult（结果话题）
 * - 发布 std_msgs/String 命令话题（"start" / "stop"）
 *
 * 该适配层是"重定位接口"的默认落点；后续若重定位模块改为 service/action
 * 协议，只需新增一个 IRelocalizationClient 实现并替换注入即可。
 */

#include <deeplumin_msgs/RelocalizationResult.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

#include "localization_v2/interface/relocalization_interface.hpp"
#include <syslog/logger.h>

namespace localization_v2 {

class RelocalizationRosAdapter : public IRelocalizationClient {
 public:
  RelocalizationRosAdapter(ros::NodeHandle& nh, const std::string& result_topic,
                           const std::string& command_topic);

  void start() override;
  void stop() override;
  void setResultCallback(RelocalizationResultCallback callback) override;

 private:
  void onResult(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg);
  void publishCommand(const std::string& command);

  ros::NodeHandle nh_;
  std::string result_topic_;
  std::string command_topic_;
  ros::Subscriber result_sub_;
  ros::Publisher command_pub_;
  RelocalizationResultCallback callback_;
};

}  // namespace localization_v2
