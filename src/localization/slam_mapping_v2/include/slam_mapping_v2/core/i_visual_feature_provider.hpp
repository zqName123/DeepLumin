#pragma once

/**
 * @file i_visual_feature_provider.hpp
 * @brief 视觉特征提供者接口（可插拔：ORB、SuperPoint 等）
 */

#include "slam_mapping_v2/common/types_visual.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace slam_mapping_v2 {

// 前向声明
struct VisualLoopConfig;

/// 视觉特征提供者接口
class IVisualFeatureProvider {
 public:
  virtual ~IVisualFeatureProvider() = default;

  /// 配置特征提取器
  virtual void configure(const VisualLoopConfig& cfg) = 0;

  /// 提取单帧特征
  virtual VisualFrame extract(const cv::Mat& image,
                              double timestamp, int keyframe_id) = 0;

  /// 批量提取特征
  virtual std::vector<VisualFrame> extractBatch(
      const std::vector<std::pair<cv::Mat, double>>& images,
      const std::vector<int>& keyframe_ids) = 0;
};

}  // namespace slam_mapping_v2
