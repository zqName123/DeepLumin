#include "slam_mapping_v2/core/map_saver.hpp"

#include <opencv2/imgcodecs.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "syslog/logger.h"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace slam_mapping_v2 {

void MapSaver::configure(const MapSaverConfig& cfg) { config_ = cfg; }

bool MapSaver::saveAll(const SaveRequest& req, const std::vector<Keyframe>& keyframes,
                       const std::vector<std::pair<int, int>>& loop_pairs,
                       const PointCloudPtr& latest_loop_current_submap,
                       const PointCloudPtr& latest_loop_history_submap, SaveResult* result) {
  if (!result) return false;
  result->success = false;
  if (keyframes.empty()) {
    result->message = "no keyframes";
    return false;
  }
  std::filesystem::create_directories(req.output_dir);
  const auto map = buildMap(keyframes, req.resolution > 0.0 ? req.resolution
                                                             : config_.default_resolution);
  result->map_path = (std::filesystem::path(req.output_dir) / req.map_name).string();
  if (!map || map->empty() || !savePcd(result->map_path, *map)) {
    result->message = "failed to save map pcd";
    return false;
  }
  if (req.save_keyframes) {
    result->keyframe_dir =
        (std::filesystem::path(req.output_dir) / config_.keyframe_dir_name).string();
    const auto images_dir =
        (std::filesystem::path(req.output_dir) / config_.images_dir_name).string();
    saveKeyframes(result->keyframe_dir, images_dir, keyframes);
  }
  if (req.save_keyframe_poses) {
    result->keyframe_pose_path =
        (std::filesystem::path(req.output_dir) / config_.keyframe_pose_filename).string();
    saveKeyframePoses(result->keyframe_pose_path, keyframes);
  }
  if (req.save_trajectory) {
    result->trajectory_path =
        (std::filesystem::path(req.output_dir) / config_.trajectory_filename).string();
    saveTrajectory(result->trajectory_path, keyframes);
  }
  if (req.save_loop_results && config_.save_loop_results) {
    const auto loop_dir = std::filesystem::path(req.output_dir) / config_.loop_result_dir_name;
    std::filesystem::create_directories(loop_dir);
    result->loop_result_path = loop_dir.string();
    saveLoopPairs((loop_dir / config_.loop_pairs_filename).string(), loop_pairs);
    saveLoopSubmap((loop_dir / config_.loop_current_submap_filename).string(),
                   latest_loop_current_submap);
    saveLoopSubmap((loop_dir / config_.loop_history_submap_filename).string(),
                   latest_loop_history_submap);
  }
  result->success = true;
  result->message = "ok";
  return true;
}

PointCloudPtr MapSaver::buildMap(const std::vector<Keyframe>& keyframes,
                                 double resolution) const {
  PointCloudPtr out(new PointCloud());
  for (const auto& kf : keyframes) {
    if (!kf.cloud || kf.cloud->empty()) continue;
    transformCloud(*kf.cloud, kf.pose_map, kf.pose_odom, out.get());
  }
  if (resolution > 0.0 && !out->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(resolution, resolution, resolution);
    voxel.setInputCloud(out);
    PointCloudPtr filtered(new PointCloud());
    voxel.filter(*filtered);
    return filtered;
  }
  return out;
}

bool MapSaver::savePcd(const std::string& path, const PointCloud& cloud) const {
  return pcl::io::savePCDFileBinary(path, cloud) == 0;
}

bool MapSaver::saveKeyframes(const std::string& dir, const std::string& images_dir,
                             const std::vector<Keyframe>& keyframes) const {
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(images_dir);

  int saved_cloud = 0;
  int saved_image = 0;
  bool ok = true;

  for (const auto& kf : keyframes) {
    // 保存点云
    if (kf.cloud && !kf.cloud->empty()) {
      // 关键帧点云在 odom 系，保存时变换到 base_link 系
      // 拼接时只需 T(pose_map) 即可还原到 map 全局坐标
      Eigen::Matrix4d t_odom = Eigen::Matrix4d::Identity();
      t_odom.block<3, 3>(0, 0) = kf.pose_odom.orientation.normalized().toRotationMatrix();
      t_odom.block<3, 1>(0, 3) = kf.pose_odom.position;
      const Eigen::Matrix4f tf_odom_inv = t_odom.inverse().cast<float>();
      PointCloud cloud_base;
      transformCloud(*kf.cloud, tf_odom_inv, &cloud_base);
      const auto cloud_path =
          (std::filesystem::path(dir) / (std::to_string(kf.id) + ".pcd")).string();
      if (savePcd(cloud_path, cloud_base)) {
        saved_cloud++;
      } else {
        ok = false;
      }
    }

    // 保存图像
    if (kf.has_visual && !kf.image.empty()) {
      const auto image_path =
          (std::filesystem::path(images_dir) / (std::to_string(kf.id) + ".png")).string();
      if (cv::imwrite(image_path, kf.image)) {
        saved_image++;
      }
    }
  }

  // ROS_INFO("[slam_mapping_v2] saved %d keyframes (cloud: %d, image: %d) to %s",
  //          static_cast<int>(keyframes.size()), saved_cloud, saved_image, dir.c_str());
  SYS_LOG(syslog::LogSeverity::INFO, true) << "saved " << saved_image << " images to " << images_dir;
  return ok;
}

bool MapSaver::saveKeyframePoses(const std::string& path,
                                 const std::vector<Keyframe>& keyframes) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);
  for (const auto& kf : keyframes) {
    const auto& q = kf.pose_map.orientation;
    ofs << kf.id << ',' << kf.timestamp << ',' << kf.pose_map.position.x() << ','
        << kf.pose_map.position.y() << ',' << kf.pose_map.position.z() << ',' << q.x() << ','
        << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  }
  return true;
}

bool MapSaver::saveTrajectory(const std::string& path,
                              const std::vector<Keyframe>& keyframes) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);
  for (const auto& kf : keyframes) {
    const auto& q = kf.pose_map.orientation;
    ofs << kf.timestamp << ' ' << kf.pose_map.position.x() << ' ' << kf.pose_map.position.y()
        << ' ' << kf.pose_map.position.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
        << q.w() << '\n';
  }
  return true;
}

bool MapSaver::saveLoopPairs(const std::string& path,
                             const std::vector<std::pair<int, int>>& loop_pairs) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << "history_id,current_id\n";
  for (const auto& lp : loop_pairs) {
    ofs << lp.first << ',' << lp.second << '\n';
  }
  return true;
}

bool MapSaver::saveLoopSubmap(const std::string& path, const PointCloudPtr& cloud) const {
  if (!cloud || cloud->empty()) return false;
  return savePcd(path, *cloud);
}

bool MapSaver::saveGnssAlignment(const std::string& path, const Mat3d& rot_map_enu,
                                 const Vec3d& trans_map_enu, double origin_lat_deg,
                                 double origin_lon_deg, double origin_alt) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);
  const double yaw_deg = std::atan2(rot_map_enu(1, 0), rot_map_enu(0, 0)) * 180.0 / M_PI;
  ofs << "aligned: 1\n";
  ofs << "yaw_deg: " << yaw_deg << "\n";
  ofs << "tx: " << trans_map_enu.x() << "\n";
  ofs << "ty: " << trans_map_enu.y() << "\n";
  ofs << "tz: " << trans_map_enu.z() << "\n";
  ofs << "origin_lat: " << origin_lat_deg << "\n";
  ofs << "origin_lon: " << origin_lon_deg << "\n";
  ofs << "origin_alt: " << origin_alt << "\n";
  return true;
}

void MapSaver::transformCloud(const PointCloud& in, const Eigen::Matrix4f& tf,
                              PointCloud* out) const {
  if (!out) return;
  PointCloud temp;
  pcl::transformPointCloud(in, temp, tf);
  *out += temp;
}

void MapSaver::transformCloud(const PointCloud& in, const Pose& pose_map, const Pose& pose_odom,
                              PointCloud* out) const {
  if (!out) return;
  // 关键帧点云为 odom 系：map 系变换 = T(pose_map) * T(pose_odom)^-1（避免二次变换变形）
  Eigen::Matrix4d t_map = Eigen::Matrix4d::Identity();
  t_map.block<3, 3>(0, 0) = pose_map.orientation.normalized().toRotationMatrix();
  t_map.block<3, 1>(0, 3) = pose_map.position;
  Eigen::Matrix4d t_odom = Eigen::Matrix4d::Identity();
  t_odom.block<3, 3>(0, 0) = pose_odom.orientation.normalized().toRotationMatrix();
  t_odom.block<3, 1>(0, 3) = pose_odom.position;
  const Eigen::Matrix4f tf = (t_map * t_odom.inverse()).cast<float>();
  transformCloud(in, tf, out);
}

}  // namespace slam_mapping_v2
