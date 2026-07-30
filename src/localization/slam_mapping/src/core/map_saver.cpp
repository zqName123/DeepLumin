#include "slam_mapping/core/map_saver.hpp"

#include <filesystem>
#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace slam_mapping {

void MapSaver::configure(const MapSaverConfig& cfg) { config_ = cfg; }

bool MapSaver::saveAll(const SaveRequest& req,
                       const std::vector<MappingKeyframe>& keyframes,
                       const std::vector<std::pair<int, int>>& loop_pairs,
                       const localization::PointCloudPtr& latest_loop_current_submap,
                       const localization::PointCloudPtr& latest_loop_history_submap,
                       SaveResult* result) {
  if (!result) return false;
  result->success = false;
  if (keyframes.empty()) {
    result->message = "no keyframes";
    return false;
  }
  std::filesystem::create_directories(req.output_dir);
  const auto map = buildMap(keyframes, req.resolution > 0.0 ? req.resolution : config_.default_resolution);
  result->map_path = (std::filesystem::path(req.output_dir) / req.map_name).string();
  if (!map || map->empty() || !savePcd(result->map_path, *map)) {
    result->message = "failed to save map pcd";
    return false;
  }
  if (req.save_keyframes) {
    result->keyframe_dir = (std::filesystem::path(req.output_dir) / config_.keyframe_dir_name).string();
    saveKeyframes(result->keyframe_dir, keyframes);
  }
  if (req.save_keyframe_poses) {
    result->keyframe_pose_path = (std::filesystem::path(req.output_dir) / config_.keyframe_pose_filename).string();
    saveKeyframePoses(result->keyframe_pose_path, keyframes);
  }
  if (req.save_trajectory) {
    result->trajectory_path = (std::filesystem::path(req.output_dir) / config_.trajectory_filename).string();
    saveTrajectory(result->trajectory_path, keyframes);
  }
  if (req.save_loop_results && config_.save_loop_results) {
    const auto loop_dir = std::filesystem::path(req.output_dir) / config_.loop_result_dir_name;
    std::filesystem::create_directories(loop_dir);
    result->loop_result_path = loop_dir.string();
    const auto loop_pairs_path = (loop_dir / config_.loop_pairs_filename).string();
    saveLoopPairs(loop_pairs_path, loop_pairs);
    saveLoopSubmap((loop_dir / config_.loop_current_submap_filename).string(), latest_loop_current_submap);
    saveLoopSubmap((loop_dir / config_.loop_history_submap_filename).string(), latest_loop_history_submap);
  }
  result->success = true;
  result->message = "ok";
  return true;
}

localization::PointCloudPtr MapSaver::buildMap(const std::vector<MappingKeyframe>& keyframes,
                                               double resolution) const {
  localization::PointCloudPtr out(new localization::PointCloud());
  for (const auto& kf : keyframes) {
    if (!kf.cloud_body || kf.cloud_body->empty()) continue;
    transformCloud(*kf.cloud_body, kf.pose_map, out.get());
  }
  if (resolution > 0.0 && !out->empty()) {
    pcl::VoxelGrid<localization::PointType> voxel;
    voxel.setLeafSize(resolution, resolution, resolution);
    voxel.setInputCloud(out);
    localization::PointCloudPtr filtered(new localization::PointCloud());
    voxel.filter(*filtered);
    return filtered;
  }
  return out;
}

bool MapSaver::savePcd(const std::string& path, const localization::PointCloud& cloud) const {
  return pcl::io::savePCDFileBinary(path, cloud) == 0;
}

bool MapSaver::saveKeyframes(const std::string& dir,
                             const std::vector<MappingKeyframe>& keyframes) const {
  std::filesystem::create_directories(dir);
  bool ok = true;
  for (const auto& kf : keyframes) {
    if (!kf.cloud_body) continue;
    const auto path = (std::filesystem::path(dir) / (std::to_string(kf.id) + ".pcd")).string();
    ok = savePcd(path, *kf.cloud_body) && ok;
  }
  return ok;
}

bool MapSaver::saveKeyframePoses(const std::string& path,
                                 const std::vector<MappingKeyframe>& keyframes) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  for (const auto& kf : keyframes) {
    const auto& q = kf.pose_map.orientation;
    ofs << kf.id << ',' << kf.timestamp << ',' << kf.pose_map.position.x() << ',' << kf.pose_map.position.y() << ','
        << kf.pose_map.position.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  }
  return true;
}

bool MapSaver::saveTrajectory(const std::string& path,
                              const std::vector<MappingKeyframe>& keyframes) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  for (const auto& kf : keyframes) {
    const auto& q = kf.pose_map.orientation;
    ofs << kf.timestamp << ' ' << kf.pose_map.position.x() << ' ' << kf.pose_map.position.y() << ' '
        << kf.pose_map.position.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  return true;
}

bool MapSaver::saveLoopPairs(const std::string& path,
                             const std::vector<std::pair<int, int>>& loop_pairs) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << "history_id,current_id\n";
  for (const auto& loop_pair : loop_pairs) {
    ofs << loop_pair.first << ',' << loop_pair.second << '\n';
  }
  return true;
}

bool MapSaver::saveLoopSubmap(const std::string& path,
                              const localization::PointCloudPtr& cloud) const {
  if (!cloud || cloud->empty()) {
    return false;
  }
  return savePcd(path, *cloud);
}

void MapSaver::transformCloud(const localization::PointCloud& in,
                              const localization::Pose& pose_map,
                              localization::PointCloud* out) const {
  if (!out) return;
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  tf.block<3, 3>(0, 0) = pose_map.orientation.normalized().toRotationMatrix().cast<float>();
  tf.block<3, 1>(0, 3) = pose_map.position.cast<float>();
  localization::PointCloud temp;
  pcl::transformPointCloud(in, temp, tf);
  *out += temp;
}

}  // namespace slam_mapping
