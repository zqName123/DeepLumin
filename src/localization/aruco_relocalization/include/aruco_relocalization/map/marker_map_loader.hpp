#pragma once

/**
 * @file marker_map_loader.hpp
 * @brief Marker 地图加载器
 */

#include <aruco_relocalization/common/types.hpp>

#include <string>
#include <vector>

namespace aruco_reloc {

struct MapLoaderConfig {
  std::string yaml_path;  // marker_map.yaml 路径
};

class MarkerMapLoader {
 public:
  bool loadFromFile(const std::string& yaml_path);

  /// 查询单个 marker
  bool getMarker(int id, MarkerItem& item) const;

  /// 查询多个 marker
  std::vector<MarkerItem> getMarkers(const std::vector<int>& ids) const;

  /// 获取所有 marker ID
  std::vector<int> getAllIds() const;

  /// 地图大小
  size_t size() const { return map_.size(); }

  /// 坐标系
  const std::string& frameId() const { return frame_id_; }

  /// 地图是否已加载
  bool isLoaded() const { return !map_.empty(); }

 private:
  MarkerMap map_;
  std::string frame_id_ = "map";
};

}  // namespace aruco_reloc
