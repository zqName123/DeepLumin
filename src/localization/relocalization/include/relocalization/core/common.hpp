#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

namespace relocalization {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

struct Pose {
  int id = -1;
  double timestamp = 0.0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  Eigen::Matrix4d matrix() const;
};

struct PreprocessConfig {
  double voxel_leaf = 0.5;
  double min_range = 3.0;
  double max_range = 80.0;
};

struct PoseLookupResult {
  bool ok = false;
  Pose pose;
  double time_diff = 0.0;
};

std::vector<Pose> loadTumPoses(const std::string& path);
PoseLookupResult lookupPoseByTimestamp(const std::vector<Pose>& poses, double timestamp, double max_diff);
CloudPtr cloudFromRosMsg(const sensor_msgs::PointCloud2& msg);
std::string joinPath(const std::string& a, const std::string& b);
int parsePcdId(const std::string& path);
std::vector<std::string> listPcdFiles(const std::string& dir);
CloudPtr loadPcd(const std::string& path);
CloudPtr rotateCloudYaw(const CloudConstPtr& cloud, double yaw_rad);
CloudPtr rotateCloudXYToKeyframeFrame(const CloudConstPtr& cloud);
CloudPtr preprocessCloud(const CloudConstPtr& cloud, const PreprocessConfig& cfg);
CloudPtr voxelDownsampleCloud(const CloudConstPtr& cloud, double voxel_leaf);
CloudPtr transformCloud(const CloudConstPtr& cloud, const Eigen::Matrix4d& transform);
double yawFromQuaternion(const Eigen::Quaterniond& q);
Eigen::Matrix4d poseFromXYYaw(double x, double y, double z, double yaw);
double normalizeAngle(double angle);

}  // namespace relocalization
