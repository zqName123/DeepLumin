#pragma once

/**
 * @file types_visual.hpp
 * @brief 视觉回环检测相关数据类型
 */

#include <opencv2/core.hpp>
#include <vector>

namespace slam_mapping_v2 {

/// 视觉帧：包含图像和提取的特征
struct VisualFrame {
  double timestamp = 0.0;
  cv::Mat image;                           // 原始灰度图像
  std::vector<cv::KeyPoint> keypoints;     // 关键点
  cv::Mat descriptors;                     // 描述子 (ORB: N x 32 CV_8U, SuperPoint: N x 256 CV_32F)
  int keyframe_id = -1;                    // 关联的关键帧 ID
  bool features_ready = false;             // 特征是否已提取完成
};

/// BoW 检索结果
struct BowQueryResult {
  int keyframe_id = -1;
  double bow_score = 0.0;
  int shared_words = 0;
};

/// BoW 匹配中间结果
struct BoWMatchResult {
  std::vector<std::pair<int, int>> matches;  // (query_kp_idx, cand_kp_idx)
  int num_matches = 0;
};

}  // namespace slam_mapping_v2
