/**
 * @file offline_mapping_node.cpp
 * @brief ArUco 离线建图节点（订阅 slam_mapping_v2 位姿，一次回放完成建图）
 */

#include <aruco_relocalization/mapping/offline_mapper.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <atomic>
#include <csignal>

namespace {
aruco_reloc::OfflineMapper* g_mapper = nullptr;
std::atomic<bool> g_shutdown_requested{false};

void sigintHandler(int sig) {
  ROS_INFO("[offline_mapping] SIGINT received, saving marker map...");
  g_shutdown_requested.store(true);
}
}  // namespace

class OfflineMappingNode {
 public:
  OfflineMappingNode() : pnh_("~") {
    config_ = aruco_reloc::loadOfflineMappingConfig(pnh_);
    mapper_.initialize(config_);
    g_mapper = &mapper_;

    // 订阅
    path_sub_ = nh_.subscribe(config_.path_topic, 10, &OfflineMappingNode::onPath, this);
    image_sub_ = nh_.subscribe(config_.image_topic, 10, &OfflineMappingNode::onImage, this);
    camera_info_sub_ = nh_.subscribe(config_.camera_info_topic, 1,
                                     &OfflineMappingNode::onCameraInfo, this);

    ROS_INFO("[offline_mapping] node started");
    ROS_INFO("[offline_mapping] path_topic=%s image_topic=%s output=%s",
             config_.path_topic.c_str(), config_.image_topic.c_str(),
             config_.output_path.c_str());
  }

  ~OfflineMappingNode() {
    saveMap();
  }

  void saveMap() {
    if (!map_saved_) {
      mapper_.saveMarkerMap();
      map_saved_ = true;
    }
  }

 private:
  void onPath(const nav_msgs::Path::ConstPtr& msg) {
    mapper_.onPath(msg);
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg) {
    if (g_shutdown_requested.load()) return;
    mapper_.onImage(msg);
  }

  void onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    mapper_.onCameraInfo(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;

  aruco_reloc::OfflineMappingConfig config_;
  aruco_reloc::OfflineMapper mapper_;
  bool map_saved_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_offline_mapping", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // 注册 SIGINT 处理
  std::signal(SIGINT, sigintHandler);

  try {
    OfflineMappingNode node;

    ros::Rate rate(100);
    while (ros::ok() && !g_shutdown_requested.load()) {
      ros::spinOnce();
      rate.sleep();
    }

    // 退出前保存
    node.saveMap();

    ROS_INFO("[offline_mapping] node shutdown");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[offline_mapping] exception: " << e.what());
    return 1;
  }

  return 0;
}
