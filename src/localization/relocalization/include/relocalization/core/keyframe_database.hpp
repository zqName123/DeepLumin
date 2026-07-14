#pragma once

#include <relocalization/core/common.hpp>

#include <unordered_map>

namespace relocalization {

struct KeyframeDatabaseConfig {
  std::string keyframe_dir;
  std::string pose_file;
  std::string global_map_pcd;
  std::string submap_mode = "radius";
  double submap_radius = 50.0;
  int neighbor_keyframes = 10;
  int submap_before_frames = 0;
  int submap_after_frames = 0;
  PreprocessConfig preprocess;
};

class KeyframeDatabase {
 public:
  bool load(const KeyframeDatabaseConfig& config);

  const std::vector<Pose>& poses() const { return poses_; }
  bool hasPose(int id) const;
  Pose pose(int id) const;

  CloudPtr loadKeyframeCloud(int id) const;
  CloudPtr buildSubmapAround(int center_id) const;
  CloudPtr buildSubmapAroundIndex(int center_id, int before_frames, int after_frames,
                                  double radius_gate = 0.0) const;
  CloudPtr buildLocalSubmapAround(int center_id, double radius, int max_keyframes) const;
  CloudPtr buildLocalSubmapAroundIndex(int center_id, int before_frames, int after_frames,
                                       double radius_gate = 0.0) const;
  CloudPtr cropGlobalMap(const Pose& center) const;
  CloudPtr cropGlobalMap(const Pose& center, double radius) const;
  CloudPtr buildKeyframeMapAround(const Pose& center, double radius, int max_keyframes) const;

 private:
  std::string keyframePath(int id) const;

  KeyframeDatabaseConfig config_;
  std::vector<Pose> poses_;
  std::unordered_map<int, std::size_t> pose_index_;
  CloudPtr global_map_;
};

}  // namespace relocalization
