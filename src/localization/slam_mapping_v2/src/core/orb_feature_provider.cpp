/**
 * @file orb_feature_provider.cpp
 * @brief ORB 特征提供者实现
 */

#include "slam_mapping_v2/core/orb_feature_provider.hpp"
#include "slam_mapping_v2/common/config.hpp"
#include "ORBextractor.h"

#include "syslog/logger.h"

namespace slam_mapping_v2 {

OrbFeatureProvider::OrbFeatureProvider() = default;

OrbFeatureProvider::~OrbFeatureProvider() = default;

void OrbFeatureProvider::configure(const VisualLoopConfig& cfg) {
  nfeatures_ = cfg.nfeatures;

  // 创建 ORBextractor（来自 ORB_SLAM3，命名空间已改为 slam_mapping_v2）
  extractor_ = std::make_unique<ORBextractor>(
      cfg.nfeatures,
      cfg.scale_factor,
      cfg.nlevels,
      cfg.ini_th_fast,
      cfg.min_th_fast);

  // ROS_INFO("[OrbFeatureProvider] Configured: nfeatures=%d, scale=%.1f, levels=%d",
  //          cfg.nfeatures, cfg.scale_factor, cfg.nlevels);
  SYS_LOG(syslog::LogSeverity::INFO, true) << "OrbFeatureProvider Configured: nfeatures=" << cfg.nfeatures << " scale=" << cfg.scale_factor << " levels=" << cfg.nlevels;
}

VisualFrame OrbFeatureProvider::extract(const cv::Mat& image,
                                        double timestamp, int keyframe_id) {
  VisualFrame frame;
  frame.timestamp = timestamp;
  frame.keyframe_id = keyframe_id;
  frame.image = image.clone();

  if (image.empty() || !extractor_) {
    // ROS_WARN("[OrbFeatureProvider] Empty image or extractor not initialized");
    SYS_LOG(syslog::LogSeverity::WARNING, true) << "Empty image or extractor not initialized";
    return frame;
  }

  // 确保图像是灰度图
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 1) {
    gray = image;
  } else {
    // ROS_WARN("[OrbFeatureProvider] Unsupported image channels: %d", image.channels());
    SYS_LOG(syslog::LogSeverity::WARNING, true) << "Unsupported image channels: " << image.channels();
    return frame;
  }

  // 提取 ORB 特征
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<int> vLappingArea = {0, static_cast<int>(gray.cols)};

  (*extractor_)(gray, cv::Mat(), keypoints, descriptors, vLappingArea);

  frame.keypoints = std::move(keypoints);
  frame.descriptors = std::move(descriptors);
  frame.features_ready = true;

  return frame;
}

std::vector<VisualFrame> OrbFeatureProvider::extractBatch(
    const std::vector<std::pair<cv::Mat, double>>& images,
    const std::vector<int>& keyframe_ids) {

  std::vector<VisualFrame> frames;
  frames.reserve(images.size());

  for (size_t i = 0; i < images.size(); ++i) {
    int kf_id = (i < keyframe_ids.size()) ? keyframe_ids[i] : static_cast<int>(i);
    frames.push_back(extract(images[i].first, images[i].second, kf_id));
  }

  return frames;
}

}  // namespace slam_mapping_v2
