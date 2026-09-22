#pragma once

/**
 * @file orb_feature_provider.hpp
 * @brief ORB 特征提供者（封装 ORB_SLAM3 的 ORBextractor）
 */

#include "slam_mapping_v2/core/i_visual_feature_provider.hpp"

// 前向声明 ORBextractor（避免暴露第三方头文件）
namespace slam_mapping_v2 {
class ORBextractor;
}

namespace slam_mapping_v2 {

/// ORB 特征提供者
class OrbFeatureProvider : public IVisualFeatureProvider {
 public:
  OrbFeatureProvider();
  ~OrbFeatureProvider() override;

  void configure(const VisualLoopConfig& cfg) override;

  VisualFrame extract(const cv::Mat& image,
                      double timestamp, int keyframe_id) override;

  std::vector<VisualFrame> extractBatch(
      const std::vector<std::pair<cv::Mat, double>>& images,
      const std::vector<int>& keyframe_ids) override;

 private:
  std::unique_ptr<ORBextractor> extractor_;
  int nfeatures_ = 2000;
};

}  // namespace slam_mapping_v2
