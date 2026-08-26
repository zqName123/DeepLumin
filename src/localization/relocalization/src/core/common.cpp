/**
 * @file common.cpp
 * @brief 通用工具函数
 *
 * 提供点云 I/O、预处理、位姿读写、坐标变换等基础能力，
 * 被建库、在线重定位、离线评估三个节点共用。
 */

#include <relocalization/core/common.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace relocalization {

/// 将 Pose（四元数+平移）转换为 4x4 齐次变换矩阵 T
Eigen::Matrix4d Pose::matrix() const {
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  tform.block<3, 1>(0, 3) = t;
  return tform;
}

/**
 * @brief 读取 TUM 格式位姿文件
 *
 * 格式：timestamp x y z qx qy qz qw（每行一个关键帧）
 * keyframe_id 按行号从 0 递增分配，需与 key_point_frame/<id>.pcd 对齐。
 */
std::vector<Pose> loadTumPoses(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open pose file: " + path);
  }

  std::vector<Pose> poses;
  std::string line;
  int id = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    Pose pose;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(iss >> pose.timestamp >> pose.t.x() >> pose.t.y() >> pose.t.z() >> qx >> qy >> qz >> qw)) {
      continue;
    }
    pose.id = id++;
    pose.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    poses.push_back(pose);
  }
  return poses;
}

PoseLookupResult lookupPoseByTimestamp(const std::vector<Pose>& poses, double timestamp, double max_diff) {
  PoseLookupResult result;
  if (poses.empty()) {
    return result;
  }

  if (timestamp <= poses.front().timestamp) {
    result.time_diff = std::abs(timestamp - poses.front().timestamp);
    if (result.time_diff <= max_diff) {
      result.pose = poses.front();
      result.pose.timestamp = timestamp;
      result.ok = true;
    }
    return result;
  }
  if (timestamp >= poses.back().timestamp) {
    result.time_diff = std::abs(timestamp - poses.back().timestamp);
    if (result.time_diff <= max_diff) {
      result.pose = poses.back();
      result.pose.timestamp = timestamp;
      result.ok = true;
    }
    return result;
  }

  const auto it = std::upper_bound(
      poses.begin(), poses.end(), timestamp, [](double t, const Pose& pose) { return t < pose.timestamp; });
  const Pose& p0 = *(it - 1);
  const Pose& p1 = *it;
  const double dt = p1.timestamp - p0.timestamp;
  if (dt < 1e-9) {
    result.time_diff = std::abs(timestamp - p0.timestamp);
    if (result.time_diff <= max_diff) {
      result.pose = p0;
      result.pose.timestamp = timestamp;
      result.ok = true;
    }
    return result;
  }

  const double alpha = (timestamp - p0.timestamp) / dt;
  result.pose.timestamp = timestamp;
  result.pose.id = p0.id;
  result.pose.t = (1.0 - alpha) * p0.t + alpha * p1.t;
  result.pose.q = p0.q.slerp(alpha, p1.q).normalized();
  result.time_diff = 0.0;
  result.ok = true;
  return result;
}

CloudPtr cloudFromRosMsg(const sensor_msgs::PointCloud2& msg) {
  CloudPtr cloud(new CloudT);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) {
    return b;
  }
  if (a.back() == '/') {
    return a + b;
  }
  return a + "/" + b;
}

/// 从 PCD 文件路径解析数字 id，如 "key_point_frame/1000.pcd" -> 1000
int parsePcdId(const std::string& path) {
  const std::size_t slash = path.find_last_of('/');
  const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
  const std::size_t dot = name.find_last_of('.');
  const std::string stem = dot == std::string::npos ? name : name.substr(0, dot);
  try {
    return std::stoi(stem);
  } catch (...) {
    return -1;
  }
}

std::vector<std::string> listPcdFiles(const std::string& dir) {
  DIR* dp = opendir(dir.c_str());
  if (!dp) {
    throw std::runtime_error("failed to open pcd directory: " + dir);
  }

  std::vector<std::string> files;
  while (dirent* entry = readdir(dp)) {
    std::string name(entry->d_name);
    if (name.size() >= 4 && name.substr(name.size() - 4) == ".pcd") {
      files.push_back(joinPath(dir, name));
    }
  }
  closedir(dp);

  std::sort(files.begin(), files.end(), [](const std::string& lhs, const std::string& rhs) {
    return parsePcdId(lhs) < parsePcdId(rhs);
  });
  return files;
}

CloudPtr loadPcd(const std::string& path) {
  CloudPtr cloud(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(path, *cloud) != 0) {
    throw std::runtime_error("failed to load pcd: " + path);
  }
  return cloud;
}

CloudPtr rotateCloudYaw(const CloudConstPtr& cloud, double yaw_rad) {
  CloudPtr out(new CloudT);
  if (!cloud) {
    return out;
  }
  *out = *cloud;
  const float c = static_cast<float>(std::cos(yaw_rad));
  const float s = static_cast<float>(std::sin(yaw_rad));
  for (auto& pt : out->points) {
    const float x = pt.x;
    const float y = pt.y;
    pt.x = c * x - s * y;
    pt.y = s * x + c * y;
  }
  return out;
}

CloudPtr rotateCloudXYToKeyframeFrame(const CloudConstPtr& cloud) {
  return rotateCloudYaw(cloud, -M_PI / 2.0);
}

/**
 * @brief 点云预处理：距离裁剪 + 体素下采样
 *
 * 距离裁剪：保留 [min_range, max_range] 范围内的点（基于 XY 平面距离）
 *   - min_range 通常等于雷达 blind 距离，去除车体/近场噪声
 *   - max_range 控制 Scan Context 和 GICP 的有效感知范围
 *
 * 体素下采样：voxel_leaf 大小的立方体网格内只保留一个点，降低计算量
 *
 * 注意：建库和在线重定位必须使用相同的预处理参数，否则描述子不一致。
 */
CloudPtr preprocessCloud(const CloudConstPtr& cloud, const PreprocessConfig& cfg) {
  CloudPtr filtered(new CloudT);
  filtered->reserve(cloud ? cloud->size() : 0);
  if (!cloud) {
    return filtered;
  }

  const double min_sq = cfg.min_range * cfg.min_range;
  const double max_sq = cfg.max_range * cfg.max_range;
  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      continue;
    }
    const double r_sq = static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y;
    if (r_sq < min_sq || r_sq > max_sq) {
      continue;
    }
    filtered->push_back(pt);
  }

  if (cfg.voxel_leaf > 1e-6 && !filtered->empty()) {
    // VoxelGrid 整数索引溢出检测：当点云范围 / voxel_leaf 超过 INT_MAX^(1/3)≈1290 时溢出
    // 典型场景：Ouster max_range=100m 时 XY 范围=200m，0.05m leaf → 4000 格/轴 → 4000³ 溢出
    PointT mn, mx;
    pcl::getMinMax3D(*filtered, mn, mx);
    const float leaf_f = static_cast<float>(cfg.voxel_leaf);
    const float nx = (mx.x - mn.x) / leaf_f;
    const float ny = (mx.y - mn.y) / leaf_f;
    const float nz = (mx.z - mn.z) / leaf_f;
    // 乘积 > INT_MAX 时溢出（用 double 比较避免自身溢出）
    const bool will_overflow = (static_cast<double>(nx) * ny * nz) > 2.1e9;

    if (!will_overflow) {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setLeafSize(leaf_f, leaf_f, leaf_f);
      voxel.setInputCloud(filtered);
      CloudPtr downsampled(new CloudT);
      voxel.filter(*downsampled);
      if (!downsampled->empty()) {
        return downsampled;
      }
    } else {
      ROS_WARN_THROTTLE(10.0, "[preprocessCloud] VoxelGrid would overflow (leaf=%.3f, extent=%.1fx%.1fx%.1f). "
                        "Returning range-filtered cloud without downsampling. "
                        "Consider reducing max_range (<=50m) or increasing voxel_leaf (>=0.1m).",
                        cfg.voxel_leaf, mx.x - mn.x, mx.y - mn.y, mx.z - mn.z);
    }
  }

  return filtered;
}

CloudPtr voxelDownsampleCloud(const CloudConstPtr& cloud, double voxel_leaf) {
  if (!cloud || cloud->empty() || voxel_leaf <= 1e-6) {
    CloudPtr out(new CloudT);
    if (cloud) {
      *out = *cloud;
    }
    return out;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(voxel_leaf),
                    static_cast<float>(voxel_leaf),
                    static_cast<float>(voxel_leaf));
  voxel.setInputCloud(cloud);
  CloudPtr downsampled(new CloudT);
  voxel.filter(*downsampled);
  return downsampled;
}

/// 用 4x4 变换矩阵变换点云
CloudPtr transformCloud(const CloudConstPtr& cloud, const Eigen::Matrix4d& transform) {
  CloudPtr out(new CloudT);
  if (!cloud) {
    return out;
  }
  pcl::transformPointCloud(*cloud, *out, transform.cast<float>());
  return out;
}

/// 从四元数提取 yaw 角（绕 Z 轴旋转，适用于地面车辆）
double yawFromQuaternion(const Eigen::Quaterniond& q) {
  const Eigen::Quaterniond n = q.normalized();
  return std::atan2(2.0 * (n.w() * n.z() + n.x() * n.y()),
                    1.0 - 2.0 * (n.y() * n.y() + n.z() * n.z()));
}

/// 由 (x, y, z, yaw) 构造 4x4 位姿矩阵（仅绕 Z 轴旋转，roll=pitch=0）
Eigen::Matrix4d poseFromXYYaw(double x, double y, double z, double yaw) {
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  tform(0, 0) = c;
  tform(0, 1) = -s;
  tform(1, 0) = s;
  tform(1, 1) = c;
  tform(0, 3) = x;
  tform(1, 3) = y;
  tform(2, 3) = z;
  return tform;
}

/// 将角度归一化到 [-π, π]
double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace relocalization
