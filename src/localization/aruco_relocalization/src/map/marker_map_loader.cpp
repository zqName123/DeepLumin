#include <aruco_relocalization/map/marker_map_loader.hpp>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace aruco_reloc {

bool MarkerMapLoader::loadFromFile(const std::string& yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (!root["markers"] || !root["markers"].IsSequence()) {
      ROS_ERROR("[aruco_reloc] invalid marker_map.yaml: missing 'markers' sequence");
      return false;
    }

    frame_id_ = root["frame_id"] ? root["frame_id"].as<std::string>() : "map";

    map_.clear();
    for (const auto& marker_node : root["markers"]) {
      MarkerItem item;
      item.id = marker_node["id"].as<int>();
      item.size = marker_node["size"] ? marker_node["size"].as<double>() : 0.2;
      item.observation_count = marker_node["observation_count"] ?
                               marker_node["observation_count"].as<int>() : 0;
      item.mean_reproj_error = marker_node["mean_reproj_error"] ?
                               marker_node["mean_reproj_error"].as<double>() : 0.0;

      if (item.size <= 0 || item.size > 10.0) {
        ROS_WARN("[aruco_reloc] marker %d has invalid size %.3f, skip", item.id, item.size);
        continue;
      }

      double x = marker_node["x"].as<double>();
      double y = marker_node["y"].as<double>();
      double z = marker_node["z"].as<double>();
      double qw = marker_node["qw"].as<double>();
      double qx = marker_node["qx"].as<double>();
      double qy = marker_node["qy"].as<double>();
      double qz = marker_node["qz"].as<double>();

      Eigen::Quaterniond q(qw, qx, qy, qz);
      if (std::abs(q.norm() - 1.0) > 0.1) {
        ROS_WARN("[aruco_reloc] marker %d has invalid quaternion, skip", item.id);
        continue;
      }
      q.normalize();

      item.T_map_marker = Eigen::Isometry3d::Identity();
      item.T_map_marker.translation() = Eigen::Vector3d(x, y, z);
      item.T_map_marker.rotate(q);

      map_[item.id] = item;
    }

    ROS_INFO("[aruco_reloc] loaded %zu markers from %s (frame_id=%s)",
             map_.size(), yaml_path.c_str(), frame_id_.c_str());
    return true;

  } catch (const std::exception& e) {
    ROS_ERROR("[aruco_reloc] failed to load marker_map.yaml: %s", e.what());
    return false;
  }
}

bool MarkerMapLoader::getMarker(int id, MarkerItem& item) const {
  auto it = map_.find(id);
  if (it == map_.end()) {
    return false;
  }
  item = it->second;
  return true;
}

std::vector<MarkerItem> MarkerMapLoader::getMarkers(const std::vector<int>& ids) const {
  std::vector<MarkerItem> result;
  for (int id : ids) {
    MarkerItem item;
    if (getMarker(id, item)) {
      result.push_back(item);
    }
  }
  return result;
}

std::vector<int> MarkerMapLoader::getAllIds() const {
  std::vector<int> ids;
  ids.reserve(map_.size());
  for (const auto& [id, _] : map_) {
    ids.push_back(id);
  }
  return ids;
}

}  // namespace aruco_reloc
