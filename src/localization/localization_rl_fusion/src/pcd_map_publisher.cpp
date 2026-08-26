#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_map_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string pcd_path;
  std::string frame_id;
  std::string topic;
  double voxel_leaf = 2.0;
  double publish_rate = 0.2;

  pnh.param<std::string>("pcd_path", pcd_path, std::string());
  pnh.param<std::string>("frame_id", frame_id, std::string("odom"));
  pnh.param<std::string>("topic", topic, std::string("/localization/rl_fusion/global_map"));
  pnh.param<double>("voxel_leaf", voxel_leaf, 2.0);
  pnh.param<double>("publish_rate", publish_rate, 0.2);

  if (pcd_path.empty()) {
    ROS_ERROR("[pcd_map_publisher] ~pcd_path is empty");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *raw) != 0) {
    ROS_ERROR("[pcd_map_publisher] failed to load PCD: %s", pcd_path.c_str());
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (voxel_leaf > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(raw);
    voxel.setLeafSize(static_cast<float>(voxel_leaf),
                      static_cast<float>(voxel_leaf),
                      static_cast<float>(voxel_leaf));
    voxel.filter(*cloud);
  } else {
    cloud = raw;
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = frame_id;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
  ROS_INFO("[pcd_map_publisher] loaded %zu -> %zu points from %s, topic=%s frame=%s",
           raw->size(), cloud->size(), pcd_path.c_str(), topic.c_str(), frame_id.c_str());

  ros::Rate rate(std::max(0.01, publish_rate));
  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
