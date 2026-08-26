#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

namespace slam_mapping {

class KeyframeManager {
 public:
  void configure(const KeyframeConfig& cfg);
  void reset();

  bool shouldAdd(const localization::OdomResult& odom,
                 const localization::TimestampedPointCloud& scan) const;
  bool add(const localization::OdomResult& odom,
           const localization::TimestampedPointCloud& scan);

  const std::vector<MappingKeyframe>& keyframes() const;
  std::vector<MappingKeyframe>& mutableKeyframes();
  const MappingKeyframe* latest() const;
  MappingKeyframe* latestMutable();
  int latestIndex() const;

 private:
  bool shouldAddAtFrame(const localization::OdomResult& odom,
                        const localization::TimestampedPointCloud& scan,
                        std::uint64_t frame_index) const;
  double relativeDistance(const localization::Pose& a,
                          const localization::Pose& b) const;
  double relativeRotationRad(const localization::Pose& a,
                             const localization::Pose& b) const;

  KeyframeConfig config_;
  std::vector<MappingKeyframe> keyframes_;
  std::uint64_t next_frame_index_ = 0;
  std::uint64_t last_keyframe_frame_index_ = 0;
};

}  // namespace slam_mapping
