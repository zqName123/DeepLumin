#pragma once

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <string>
#include <vector>

namespace localization_ros {

class GlobalMapVizPublisher {
 public:
  void setup(ros::NodeHandle& nh, ros::NodeHandle& pnh,
             const std::string& default_topic,
             const std::string& default_frame,
             const std::string& logger_name) {
    logger_name_ = logger_name;
    pnh.param<bool>("map/publish_global_map", enabled_, false);
    pnh.param<std::string>("map/pcd_path", pcd_path_, std::string());
    pnh.param<std::string>("map/global_map_topic", topic_, default_topic);
    pnh.param<std::string>("map/frame_id", frame_id_, default_frame);
    pnh.param<double>("map/publish_voxel_leaf", voxel_leaf_, 5.0);
    pnh.param<double>("map/publish_period", publish_period_, 5.0);

    if (!enabled_) {
      return;
    }
    if (pcd_path_.empty()) {
      ROS_WARN("[%s][MapViz] map/publish_global_map is true but map/pcd_path is empty", logger_name_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
    const int ret = pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *raw);
    if (ret != 0 || raw->empty()) {
      ROS_ERROR("[%s][MapViz] failed to load global map: %s", logger_name_.c_str(), pcd_path_.c_str());
      return;
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw, *raw, valid_indices);
    if (raw->empty()) {
      ROS_ERROR("[%s][MapViz] global map is empty after NaN filtering: %s", logger_name_.c_str(), pcd_path_.c_str());
      return;
    }

    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    if (voxel_leaf_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(raw);
      const float leaf = static_cast<float>(voxel_leaf_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*cloud_);
    } else {
      *cloud_ = *raw;
    }

    if (!cloud_ || cloud_->empty()) {
      ROS_ERROR("[%s][MapViz] global map is empty after downsample: %s", logger_name_.c_str(), pcd_path_.c_str());
      return;
    }

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_, 1, true);
    const double period = std::max(0.5, publish_period_);
    timer_ = nh.createTimer(ros::Duration(period), &GlobalMapVizPublisher::onTimer, this);
    publish();

    ROS_INFO("[%s][MapViz] loaded global map path=%s raw_points=%zu viz_points=%zu topic=%s frame=%s voxel=%.3f period=%.3fs",
             logger_name_.c_str(), pcd_path_.c_str(), raw->size(), cloud_->size(),
             topic_.c_str(), frame_id_.c_str(), voxel_leaf_, period);
  }

  void publish() const {
    if (!enabled_ || !cloud_ || cloud_->empty() || !pub_) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    pub_.publish(msg);
  }

 private:
  void onTimer(const ros::TimerEvent&) { publish(); }

  bool enabled_ = false;
  std::string pcd_path_;
  std::string topic_;
  std::string frame_id_;
  std::string logger_name_;
  double voxel_leaf_ = 5.0;
  double publish_period_ = 5.0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  ros::Publisher pub_;
  ros::Timer timer_;
};

}  // namespace localization_ros
