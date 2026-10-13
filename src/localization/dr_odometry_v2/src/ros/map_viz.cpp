#include "dr_odometry_v2/ros/map_viz.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <vector>

#include <syslog/logger.h>

namespace dr_odometry_v2_ros {

MapVizPublisher::~MapVizPublisher() {
  if (loader_.joinable()) {
    loader_.join();
  }
}

bool MapVizPublisher::setup(ros::NodeHandle& nh, const Config& config) {
  config_ = config;
  if (!config_.enabled) {
    return false;
  }
  if (config_.pcd_path.empty()) {
    SYS_LOG(syslog::LogSeverity::WARNING, true) << "[MapViz] enabled but pcd_path is empty";
    return false;
  }

  // 发布/定时器立即建立；地图加载放到后台线程（大 PCD 不阻塞节点启动与订阅）
  pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.topic, 1, true /* latched */);
  timer_ = nh.createTimer(ros::Duration(std::max(0.5, config_.publish_period)),
                          [this](const ros::TimerEvent&) {
                            if (ready_.load()) {
                              publishOnce();
                            }
                          });
  loader_ = std::thread([this]() { loadMap(); });

  SYS_LOG(syslog::LogSeverity::INFO, true)
      << "[MapViz] loading in background: " << config_.pcd_path
      << " (leaf=" << config_.voxel_leaf << ") topic=" << config_.topic;
  return true;
}

void MapVizPublisher::loadMap() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(config_.pcd_path, *raw) != 0 || raw->empty()) {
    SYS_LOG(syslog::LogSeverity::ERROR, true)
        << "[MapViz] failed to load global map: " << config_.pcd_path;
    return;
  }
  const std::size_t raw_size = raw->size();
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*raw, *raw, valid_indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (config_.voxel_leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(raw);
    const float leaf = static_cast<float>(config_.voxel_leaf);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*cloud);
  } else {
    *cloud = *raw;
  }

  // 手动对齐偏移（先旋转后平移；顺时针为负）
  const double yaw = config_.offset_yaw_deg * M_PI / 180.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (auto& p : cloud->points) {
    const double x = c * p.x - s * p.y + config_.offset_x;
    const double y = s * p.x + c * p.y + config_.offset_y;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(p.z + config_.offset_z);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_ = cloud;
  }
  ready_.store(true);
  publishOnce();

  SYS_LOG(syslog::LogSeverity::INFO, true)
      << "[MapViz] loaded " << config_.pcd_path << " raw=" << raw_size << " -> " << cloud->size()
      << " pts (leaf=" << config_.voxel_leaf << ") topic=" << config_.topic
      << " frame=" << config_.frame_id << " offset=(" << config_.offset_x << ", "
      << config_.offset_y << ", " << config_.offset_z << ", " << config_.offset_yaw_deg << "deg)";
}

void MapVizPublisher::publishOnce() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud = cloud_;
  }
  if (!ready_.load() || !cloud) {
    return;
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::isValid() ? ros::Time::now() : ros::Time(0);  // 时钟未到不抛异常
  msg.header.frame_id = config_.frame_id;
  pub_.publish(msg);
}

}  // namespace dr_odometry_v2_ros
